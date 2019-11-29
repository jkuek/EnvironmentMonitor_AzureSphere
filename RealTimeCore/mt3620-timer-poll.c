/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#include "mt3620-baremetal.h"
#include "mt3620-timer-poll.h"

static const uintptr_t GPT_BASE = 0x21030000;
static const uint32_t GPT3_EN = 0x00000001;
static const size_t GPT3_CNT = 0x58;

inline uint32_t Gpt3_CurrentTime(void)
{
	return ReadReg32(GPT_BASE, GPT3_CNT);
}

static inline void Gpt3_Enable(void)
{
	SetReg32(GPT_BASE, 0x50, GPT3_EN);
}

static inline void Gpt3_Disable(void)
{
	// GPT_CTRL -> disable timer
	ClearReg32(GPT_BASE, 0x50, GPT3_EN);
}


void Gpt3_WaitUs(int microseconds)
{
	uint32_t start = Gpt3_CurrentTime();
	while ((Gpt3_CurrentTime() - start) < microseconds) {};
}


void Gpt3_Init(void)
{
	// GPT3_INIT = initial counter value
	WriteReg32(GPT_BASE, 0x54, 0x0);

	// GPT3_CTRL
	uint32_t ctrlOn = 0x0;
	ctrlOn |= (0x19) << 16; // OSC_CNT_1US (default value)
	ctrlOn |= 0x1;          // GPT3_EN = 1 -> GPT3 enabled
	WriteReg32(GPT_BASE, 0x50, ctrlOn);

	Gpt3_Enable();
}