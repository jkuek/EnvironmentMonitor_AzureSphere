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

#include <math.h>


#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "bsec_serialized_configurations_iaq.h"

#define MAX_AMBIENT_BUFFER_SIZE (2000)


struct bme680_raw_data
{
	int64_t timestamp_ns;
	uint32_t new_data; /* non-zero if following data is new and valid, else 0 */

	float temperature; /* degrees C */
	float pressure; /* kPa */
	float humidity; /*! Relative humidity in % */
	float gas_resistance; /*! Gas resistance in Ohms */

} bme680_data;

struct real_time_outputs
{
	uint32_t ambient_light;
	uint32_t ambient_noise;

	bsec_output_t iaq;
	bsec_output_t temperature;
	bsec_output_t humidity;
	bsec_output_t rawTemperature;
	bsec_output_t rawHumidity;
	bsec_output_t rawPressure;
	bsec_output_t rawGas;
	bsec_output_t co2Equivalent;

	bsec_bme_settings_t sensor_settings;
} outputPayload;



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

const uintptr_t ExceptionVectorTable[EXCEPTION_COUNT] __attribute__((section(".vector_table")))
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


//Initialise BSEC library
bool InitialiseBsecLibrary(void)
{
	bsec_library_return_t bsec_result; //used to check return values of BSEC library calls
	
	//working buffer needed for BSEC library
	uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
	uint32_t n_work_buffer = BSEC_MAX_PROPERTY_BLOB_SIZE;

	bsec_result = bsec_init();
	if (bsec_result != BSEC_OK)
	{
		return false;
	}

	/*
		Configure the BSEC library
		The default configuration is "generic_18v_300s_4d"
		I used "generic_33v_3s_4d", since we are running from 3.3V and I want data every 3s with a 4-day calibration interval
		Copy the appropriate file from the BSEC release ZIP.
	*/
	bsec_result = bsec_set_configuration(bsec_config_iaq, sizeof(bsec_config_iaq), work_buffer, n_work_buffer);
	if (bsec_result != BSEC_OK)
	{
		return false;
	}

	// TODO: load saved state
	/*
	bsec_result = bsec_set_state(bsec_state, bsec_state_len, work_buffer, sizeof(work_buffer));
	if (bsec_result != BSEC_OK)
	{
		return false;
	}
	*/

	{
		// Set up which processed outputs we want from BSEC
		bsec_sensor_configuration_t requested_virtual_sensors[8];
		uint8_t n_requested_virtual_sensors = 8;

		requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
		requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
		requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
		requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
		requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
		requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
		requested_virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
		requested_virtual_sensors[6].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
		requested_virtual_sensors[7].sample_rate = BSEC_SAMPLE_RATE_LP;

		// Allocate a struct for the returned physical sensor settings
		bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
		uint8_t  n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

		// Call bsec_update_subscription() to enable/disable the requested virtual sensors
		bsec_result = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);

		/* NOTE: according to the integration guide 4.1.2.10,
			the returned required_sensor_settings are "purely informational" so no need to use them for anything
		*/

		if (bsec_result != BSEC_OK)
		{
			return false;
		}
	}

	return true;
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
	const uint32_t analog_offset = 1801; // = (~1.1 / 2.5) * 4096; 

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
		//subtract midpoint (DC offset)
		buffer[i] -= analog_offset;

		//sum of squares
		total += buffer[i] * buffer[i];
	}

	//calculate the average
	uint32_t average = total / count;

	uint32_t rms = sqrtf(average);
	

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

static void ProcessBsec(struct bme680_raw_data * bme680_data)
{
	// Allocate input and output memory
	bsec_input_t inputs[4]; //up to 4 inputs
	uint8_t input_count = 0; //also used as index

	bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
	uint8_t  output_count = BSEC_NUMBER_OUTPUTS;

	/* find out which fields need to be processed */
	uint8_t process_data = bme680_data->new_data;

	//if (process_data & BSEC_PROCESS_TEMPERATURE)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_TEMPERATURE;
		inputs[input_count].time_stamp = bme680_data->timestamp_ns;
		inputs[input_count].signal = bme680_data->temperature;
		input_count++;
	}

	//if (process_data & BSEC_PROCESS_HUMIDITY)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_HUMIDITY;
		inputs[input_count].time_stamp = bme680_data->timestamp_ns;
		inputs[input_count].signal = bme680_data->humidity;
		input_count++;
	}

	//if (process_data & BSEC_PROCESS_PRESSURE)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_PRESSURE;
		inputs[input_count].time_stamp = bme680_data->timestamp_ns;
		inputs[input_count].signal = bme680_data->pressure;
		input_count++;
	}

	//if (process_data & BSEC_PROCESS_GAS)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_GASRESISTOR;
		inputs[input_count].time_stamp = bme680_data->timestamp_ns;
		inputs[input_count].signal = bme680_data->gas_resistance;
		input_count++;
	}

	// Invoke main processing BSEC function
	bsec_library_return_t status = bsec_do_steps(inputs, input_count, outputs, &output_count);

	// Iterate through the BSEC output data, if the call succeeded
	if (status == BSEC_OK)
	{
		for (int i = 0; i < output_count; i++)
		{
			switch (outputs[i].sensor_id)
			{
			case BSEC_OUTPUT_IAQ:
				outputPayload.iaq = outputs[i];
				break;
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
				outputPayload.temperature = outputs[i];
				break;
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
				outputPayload.humidity = outputs[i];
				break;
			case BSEC_OUTPUT_RAW_TEMPERATURE:
				outputPayload.rawTemperature = outputs[i];
				break;
			case BSEC_OUTPUT_RAW_PRESSURE:
				outputPayload.rawPressure = outputs[i];
				break;
			case BSEC_OUTPUT_RAW_HUMIDITY:
				outputPayload.rawHumidity = outputs[i];
				break;
			case BSEC_OUTPUT_RAW_GAS:
				outputPayload.rawGas = outputs[i];
				break;
			case BSEC_OUTPUT_CO2_EQUIVALENT:
				outputPayload.co2Equivalent = outputs[i];
				break;
			default:
				Uart_WriteStringPoll("ERROR: Unhandled sensor type\n");
				break;

			}
		}
	}


	// check to see if it's time to trigger the next measurement
	bsec_bme_settings_t sensor_settings;
	bsec_sensor_control(bme680_data->timestamp_ns, &sensor_settings);
	outputPayload.sensor_settings = sensor_settings;
}


static _Noreturn void RTCoreMain(void)
{
    // SCB->VTOR = ExceptionVectorTable
    WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

	Gpt3_Init();

	//delay before starting (seems to help when debugging all cores)
	Gpt3_WaitUs(2000000);


    Uart_Init();
    Uart_WriteStringPoll("--------------------------------\r\n");
    Uart_WriteStringPoll("Environmental Monitor RealTime App\r\n");
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

	InitialiseBsecLibrary();

    static const size_t payloadStart = 20;

    for (;;)
	{
        uint8_t buf[256];
        uint32_t dataSize = sizeof(buf);		

        // On success, dataSize is set to the actual number of bytes which were read.
        int r = DequeueData(outbound, inbound, sharedBufSize, buf, &dataSize);

        if (r == -1 || dataSize < payloadStart) 
		{
            continue;
        }

		struct bme680_raw_data* bme680_data = (struct bme680_raw_data *)(buf + payloadStart);


		// Print the Component Id (A7 Core)
        Uart_WriteStringPoll("  Component Id (16 bytes): ");
        PrintGuid(buf);
        Uart_WriteStringPoll("\r\n");

		//feed new BME680 values to BSEC library
		ProcessBsec(bme680_data);


		// Read ADC channel 0 (ambient light)
		outputPayload.ambient_light = ReadAdc(0);
		// update ambient noise sample, this will block for ~105 ms
		uint32_t ambient_noise = GetAmbientNoiseLevel();

		outputPayload.ambient_noise = ambient_noise;
		
		uint32_t mV = (outputPayload.ambient_light * 2500) / 0xFFF;
		
		memcpy(&buf[payloadStart], (uint8_t*)&outputPayload, sizeof(outputPayload));

		// Send buffer to A7 Core
        EnqueueData(inbound, outbound, sharedBufSize, buf, payloadStart + sizeof(outputPayload));
    }
}
