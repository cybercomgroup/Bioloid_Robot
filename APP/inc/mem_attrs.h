/*
 * mem_attrs.h
 *
 *  Created on: 7 feb. 2017
 *      Author: install
 */

#ifndef APP_INC_MEM_ATTRS_H_
#define APP_INC_MEM_ATTRS_H_

#include "typedefs.h"
// Declare the variable const and it should be in flash automatically
#define PROGMEM // __attribute__((section(".rodata")))

#define pgm_read_byte(x) *((char*)(x))
#define pgm_read_word(x) *((word*)(x))

#endif /* APP_INC_MEM_ATTRS_H_ */
