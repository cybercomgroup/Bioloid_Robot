#include "hw_functions.h"
#include "stm32f10x_gpio.h"


#define PORT_IR_LEFT GPIOA
#define PORT_IR_RIGHT GPIOB

//ADC multiplexer
#define PORT_ADX_MUX GPIOC

//P for plus
#define PIN_IR_LEFT_MOTP           GPIO_Pin_0
//M for minus
#define PIN_IR_LEFT_MOTM           GPIO_Pin_1

#define PIN_IR_RIGHT_MOTP		   GPIO_Pin_8

#define PIN_IR_RIGHT_MOTM		   GPIO_Pin_9

//Used with the ADC multiplexer to select signal source for conversion
#define PIN_ADC_SELECT0            GPIO_Pin_1
#define PIN_ADC_SELECT1            GPIO_Pin_2

word read_ir_right(void);
word read_ir_left(void);
