
#ifndef SENSORS_H_
#define SENSORS_H_

#include "time.h"
#include "hw_functions.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_map.h"
#include "stm32f10x_adc.h"


#define PORT_GYRO				   GPIOC
#define PORT_DMS			       GPIOA

//ADC multiplexer
#define PORT_ADX_MUX GPIOC



#define PIN_DMS_MOTP		       GPIO_Pin_8
#define PIN_DMS_MOTM		       GPIO_Pin_11

//Used with the ADC multiplexer to select signal source for conversion
#define PIN_ADC_SELECT0            GPIO_Pin_1
#define PIN_ADC_SELECT1            GPIO_Pin_2


#endif




