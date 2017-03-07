
#ifndef SENSORS_H_
#define SENSORS_H_

#include "time.h"
#include "hw_functions.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_map.h"
#include "stm32f10x_adc.h"


#define PORT_GYRO				   GPIOC
#define PORT_DMS			       GPIOA

#define PORT_IR_LEFT 			   GPIOA
#define PORT_IR_RIGHT 			   GPIOB

//ADC multiplexer
#define PORT_ADX_MUX GPIOC

#define PIN_IR_LEFT_MOTP           GPIO_Pin_0
#define PIN_IR_LEFT_MOTM           GPIO_Pin_1
#define PIN_IR_RIGHT_MOTP		   GPIO_Pin_8
#define PIN_IR_RIGHT_MOTM		   GPIO_Pin_9

#define PIN_GYRO_PITCH_MOTP		   GPIO_Pin_6
#define PIN_GYRO_PITCH_MOTM		   GPIO_Pin_7

#define PIN_GYRO_ROLL_MOTP	       GPIO_Pin_8
#define PIN_GYRO_ROLL_MOTM		   GPIO_Pin_9

#define PIN_DMS_MOTP		       GPIO_Pin_8
#define PIN_DMS_MOTM		       GPIO_Pin_11

//Used with the ADC multiplexer to select signal source for conversion
#define PIN_ADC_SELECT0            GPIO_Pin_1
#define PIN_ADC_SELECT1            GPIO_Pin_2

//IR
word read_ir_right(void);
word read_ir_left(void);

s32 gyro_get_pitch();
s32 gyro_get_roll();

void gyro_update();
void gyro_read();
void gyro_init();
void gyro_calibrate();
s16 gyro_get_x();
s16 gyro_get_y();
s16 gyro_get_center_x();
s16 gyro_get_center_y();

#endif
