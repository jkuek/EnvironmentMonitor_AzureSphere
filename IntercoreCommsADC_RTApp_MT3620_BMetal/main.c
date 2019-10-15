/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#include <ctype.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>

#include "mt3620-baremetal.h"
#include "mt3620-intercore.h"
#include "mt3620-timer-poll.h"
#include "mt3620-uart-poll.h"
#include "mt3620-adc.h"

#define MAX_AMBIENT_BUFFER_SIZE (2000)

extern uint32_t StackTop; // &StackTop == end of TCM0

static _Noreturn void DefaultExceptionHandler(void);

static void PrintBytes(const uint8_t *buf, int start, int end);
static void PrintGuid(const uint8_t *guid);

static _Noreturn void RTCoreMain(void);

// ARM DDI0403E.d SB1.5.2-3
// From SB1.5.3, "The Vector table must be naturally aligned to a power of two whose alignment
// value is greater than or equal to (Number of Exceptions supported x 4), with a minimum alignment
// of 128 bytes.". The array is aligned in linker.ld, using the dedicated section ".vector_table".

// The exception vector table contains a stack pointer, 15 exception handlers, and an entry for
// each interrupt.
#define INTERRUPT_COUNT 100 // from datasheet
#define EXCEPTION_COUNT (16 + INTERRUPT_COUNT)
#define INT_TO_EXC(i_) (16 + (i_))
static const uintptr_t ExceptionVectorTable[EXCEPTION_COUNT]
    __attribute__((section(".vector_table"))) __attribute__((used)) = {
        [0] = (uintptr_t)&StackTop,                // Main Stack Pointer (MSP)
        [1] = (uintptr_t)RTCoreMain,               // Reset
        [2] = (uintptr_t)DefaultExceptionHandler,  // NMI
        [3] = (uintptr_t)DefaultExceptionHandler,  // HardFault
        [4] = (uintptr_t)DefaultExceptionHandler,  // MPU Fault
        [5] = (uintptr_t)DefaultExceptionHandler,  // Bus Fault
        [6] = (uintptr_t)DefaultExceptionHandler,  // Usage Fault
        [11] = (uintptr_t)DefaultExceptionHandler, // SVCall
        [12] = (uintptr_t)DefaultExceptionHandler, // Debug monitor
        [14] = (uintptr_t)DefaultExceptionHandler, // PendSV
        [15] = (uintptr_t)DefaultExceptionHandler, // SysTick

        [INT_TO_EXC(0)... INT_TO_EXC(INTERRUPT_COUNT - 1)] = (uintptr_t)DefaultExceptionHandler};

static _Noreturn void DefaultExceptionHandler(void)
{
    for (;;) {
        // empty.
    }
}


static uint32_t GetAmbientNoiseLevel(void)
{
	const uint32_t duration = 100000; //100000 us = 100 ms
	uint32_t count = 0;
	int32_t buffer[MAX_AMBIENT_BUFFER_SIZE];
	uint32_t start = Gpt3_CurrentTime();

	/* I am feeding a 0-2.2 V signal into the ADC, where 2.5V is full-scale (= 4095 at 12-bits)
		Calculate the DC offset to subtract
	*/
	const uint32_t analog_offset = (1.1 / 2.5) * 4096; 

	while (((Gpt3_CurrentTime() - start) < duration) && (count < MAX_AMBIENT_BUFFER_SIZE))
	{
		//Mic Click board is in socket #1 --> use analog channel 1
		buffer[count] = ReadAdc(1);
		count++;
	};

	uint32_t total = 0;
	//now process the samples
	for (int i = 0; i < count; i++)
	{
		//subtract DC offset (1.1 V)
		buffer[i] -= analog_offset;

		//sum of squares
		total += buffer[i] * buffer[i];
	}

	//calculate the average
	uint32_t average = total / count;

	return average;
}


static void PrintBytes(const uint8_t *buf, int start, int end)
{
    int step = (end >= start) ? +1 : -1;

    for (/* nop */; start != end; start += step) {
        Uart_WriteHexBytePoll(buf[start]);
    }
    Uart_WriteHexBytePoll(buf[end]);
}

static void PrintGuid(const uint8_t *guid)
{
    PrintBytes(guid, 3, 0); // 4-byte little-endian word
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 5, 4); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 7, 6); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 8, 9); // 2 bytes
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 10, 15); // 6 bytes
}

static _Noreturn void RTCoreMain(void)
{
	struct Payload
	{
		uint32_t ambient_light;
		uint32_t ambient_noise;
	} payload;

    // SCB->VTOR = ExceptionVectorTable
    WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

	Gpt3_Init();

    Uart_Init();
    Uart_WriteStringPoll("--------------------------------\r\n");
    Uart_WriteStringPoll("IntercoreCommsADC_RTApp_MT3620_BareMetal\r\n");
    Uart_WriteStringPoll("App built on: " __DATE__ ", " __TIME__ "\r\n");

	//// ADC Communication
	EnableAdc();

    BufferHeader *outbound, *inbound;
    uint32_t sharedBufSize = 0;
    if (GetIntercoreBuffers(&outbound, &inbound, &sharedBufSize) == -1) {
        for (;;) {
            // empty.
        }
    }

    static const size_t payloadStart = 20;

    for (;;)
	{
        uint8_t buf[256];
        uint32_t dataSize = sizeof(buf);		

		// update ambient noise sample, this will block for ~105 ms
		uint32_t ambient_noise = GetAmbientNoiseLevel();

        // On success, dataSize is set to the actual number of bytes which were read.
        int r = DequeueData(outbound, inbound, sharedBufSize, buf, &dataSize);

        if (r == -1 || dataSize < payloadStart) 
		{
            continue;
        }

		// For debug purposes 
        Uart_WriteStringPoll("Received message of ");
        Uart_WriteIntegerPoll(dataSize);
        Uart_WriteStringPoll("bytes:\r\n");

		// Print the Component Id (A7 Core)
        Uart_WriteStringPoll("  Component Id (16 bytes): ");
        PrintGuid(buf);
        Uart_WriteStringPoll("\r\n");

        // Print reserved field as little-endian 4-byte integer.
        Uart_WriteStringPoll("  Reserved (4 bytes): ");
        PrintBytes(buf, 19, 16);
        Uart_WriteStringPoll("\r\n");

        // Print message as hex.
        size_t payloadBytes = dataSize - payloadStart;
        Uart_WriteStringPoll("  Payload (");
        Uart_WriteIntegerPoll(payloadBytes);
        Uart_WriteStringPoll(" bytes as hex): ");

        for (size_t i = payloadStart; i < dataSize; ++i)
		{
            Uart_WriteHexBytePoll(buf[i]);
            if (i != dataSize - 1) {
                Uart_WriteStringPoll(":");
            }
        }
        Uart_WriteStringPoll("\r\n");

        // Print message as text.
        Uart_WriteStringPoll("  Payload (");
        Uart_WriteIntegerPoll(payloadBytes);
        Uart_WriteStringPoll(" bytes as text): ");
        for (size_t i = payloadStart; i < dataSize; ++i) 
		{
            char c[2];
            c[0] = isprint(buf[i]) ? buf[i] : '.';
            c[1] = '\0';
            Uart_WriteStringPoll(c);
        }
        Uart_WriteStringPoll("\r\n");

		// Read ADC channel 0 (ambient light)
		payload.ambient_light = ReadAdc(0);
		payload.ambient_noise = ambient_noise;
		
		uint32_t mV = (payload.ambient_light * 2500) / 0xFFF;
		Uart_WriteStringPoll("ADC channel 0: ");
		Uart_WriteIntegerPoll(mV / 1000);
		Uart_WriteStringPoll(".");
		Uart_WriteIntegerWidthPoll(mV % 1000, 3);
		Uart_WriteStringPoll(" V");
		Uart_WriteStringPoll("\r\n");
		
		memcpy(&buf[payloadStart], (uint8_t*)&payload, sizeof(payload));

		// Send buffer to A7 Core
        EnqueueData(inbound, outbound, sharedBufSize, buf, payloadStart + sizeof(payload));
    }
}
