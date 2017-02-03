/*
 * delay.c
 *
 *  Created on: 3 feb. 2017
 *      Author: install
 */

#include "delay.h"

/* Replacement delay function for the AVR library's one. */
void _delay_ms	(	uint32 	__ms	) {
	mDelay(__ms);
}
