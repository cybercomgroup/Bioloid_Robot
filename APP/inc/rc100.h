/*
 * rc100.h
 *
 *  Created on: 14 feb. 2017
 *      Author: Kalle Halvarsson
 */

#include "stm32f10x_map.h"
#include "stm32f10x_lib.h"
#include "typedefs.h"

#ifndef APP_INC_RC100_H_
#define APP_INC_RC100_H_
#endif /* APP_INC_RC100_H_ */

/* Button definitions
 * Multiple buttons will return the sum of the pressed buttons! */

typedef enum {
	RC100_BTN_U = 1,
	RC100_BTN_D = 2,
	RC100_BTN_L = 4,
	RC100_BTN_R	= 8,
	RC100_BTN_1	= 16,
	RC100_BTN_2	= 32,
	RC100_BTN_3	= 64,
	RC100_BTN_4	= 128,
	RC100_BTN_5	= 256,
	RC100_BTN_6	= 512
} button;

/*
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
*/

#define PACKET_LENGTH 			6

#define PORT_ZIGBEE_TXD			GPIOC
#define PORT_ZIGBEE_RXD			GPIOD
#define PORT_ZIGBEE_RESET		GPIOA


#define PIN_ZIGBEE_TXD			GPIO_Pin_12
#define PIN_ZIGBEE_RXD			GPIO_Pin_2
#define PIN_ZIGBEE_RESET		GPIO_Pin_12

typedef enum {
	STATE_RELEASED,
	STATE_PRESSED,
	STATE_UNCHANGED
} button_state;

void RxD2Interrupt(void);

void rc100_init(void);

bool rc100_update();

int rc100_check(void);

int rc100_send_data(int data);

int rc100_read_data(void);

button_state rc100_get_btn_change_state(button btn);

button_state rc100_get_btn_state(button btn);
