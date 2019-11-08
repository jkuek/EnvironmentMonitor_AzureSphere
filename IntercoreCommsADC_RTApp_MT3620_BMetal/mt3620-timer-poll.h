/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#ifndef MT3620_TIMER_POLL_H
#define MT3620_TIMER_POLL_H


uint32_t Gpt3_CurrentTime(void);

/// <summary>
/// Busy wait for the supplied number of microseconds.
/// </summary>
void Gpt3_WaitUs(int microseconds);

void Gpt3_Init(void);

#endif // #ifndef MT3620_TIMER_POLL_H
