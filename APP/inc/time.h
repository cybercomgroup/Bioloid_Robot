/*
 * delay.h
 *
 *  Created on: 3 feb. 2017
 *      Author: install
 */

#ifndef APP_INC_TIME_H_
#define APP_INC_TIME_H_

#include "global.h"

/* Replacement definitions for the AVR library's one. */
#define delay_ms mDelay
#define _delay_ms mDelay

void mDelay(uint32 	__ms);
void uDelay(uint32 	__ms);

u32 millis();
u32 micros();

#endif /* APP_INC_TIME_H_ */
