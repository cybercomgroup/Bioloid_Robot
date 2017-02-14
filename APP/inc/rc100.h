/*
 * rc100.h
 *
 *  Created on: 14 feb. 2017
 *      Author: Kalle Halvarsson
 */

#include "stm32f10x_map.h"
#include "stm32f10x_lib.h"

#ifndef APP_INC_RC100_H_
#define APP_INC_RC100_H_
#endif /* APP_INC_RC100_H_ */

/* Button definitions
 * Multiple buttons will return the sum of the pressed buttons! */
#define RC100_BTN_U		1
#define RC100_BTN_D		2
#define RC100_BTN_L		4
#define RC100_BTN_R		8
#define RC100_BTN_1		16
#define RC100_BTN_2		32
#define RC100_BTN_3		64
#define RC100_BTN_4		128
#define RC100_BTN_5		256
#define RC100_BTN_6		512

#define PACKET_LENGTH 			6

#define PORT_ZIGBEE_TXD			GPIOC
#define PORT_ZIGBEE_RXD			GPIOD
#define PORT_ZIGBEE_RESET		GPIOA


#define PIN_ZIGBEE_TXD			GPIO_Pin_12
#define PIN_ZIGBEE_RXD			GPIO_Pin_2
#define PIN_ZIGBEE_RESET		GPIO_Pin_12


void rc100_init(void);

int rc100_check(void);

int rc100_send_data(int data);

int rc100_read_data(void);