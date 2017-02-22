/*
 * ir.h
 *
 *  Created on: 21 feb. 2017
 *      Author: install
 */

#ifndef APP_INC_IR_H_
#define APP_INC_IR_H_

#include "stm32f10x_gpio.h"
#include "typedefs.h"
#include "sensors.h"

#define PORT_IR_LEFT 			   GPIOA
#define PORT_IR_RIGHT 			   GPIOB

#define PIN_IR_LEFT_MOTP           GPIO_Pin_0
#define PIN_IR_LEFT_MOTM           GPIO_Pin_1
#define PIN_IR_RIGHT_MOTP		   GPIO_Pin_8
#define PIN_IR_RIGHT_MOTM		   GPIO_Pin_9

//IR
word read_ir_right(void);
word read_ir_left(void);

#endif /* APP_INC_IR_H_ */
