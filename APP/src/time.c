/*
 * time.c
 *
 *  Created on: 13 feb. 2017
 *      Author: Anton Olsson
 */

#include "time.h"
#include "stm32f10x_systick.h"

vu32 msSystickCounter = 0;

void __ISR_SYSTICK(void)
{
	// tick at 1 ms.
	msSystickCounter++;
}

void mDelay(u32 nTime)
{
	u32 t = millis() + nTime;
	while(millis() < t);
}

void uDelay(u32 nTime)
{
	  u32 t = micros() + nTime;
	  while(micros() < t);
}

u32 millis() {
	return msSystickCounter;
}

u32 micros() {
	u32 ms = msSystickCounter;
	u32 us = SysTick_GetCounter(); // is counting DOWN from 9000 = 1 ms !
	// if systick interrupt between readings, return value will be up to 1000 us wrong.
	// so read ms again to check that. (interrupt again is impossible as next interrupt will be in about 1000 us)
	u32 ms2 = msSystickCounter;
	if (ms != ms2) {
		return ms2 * 1000;
	} else {
		return ms * 1000 + (9000 - us)/9; // TODO will overflow at 2^32 us ~= 71 minutes! [could separate us and ms in a struct]
	}
}
