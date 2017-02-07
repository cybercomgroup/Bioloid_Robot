#include "hw_functions.h"
#include "stm32f10x_gpio.h"

typedef enum {LEFT, RIGHT} ir;

/* THESE PORTS ARE CHOSEN ARBITRARILY AND ARE LIKELY INCORRECT!
 * PLEASE TEST ON THE HARDWARE TO VERIFY!
 *
 * Check stm32f10x_map.h for port definitions. */
#define PORT_IR_LEFT GPIOC
#define PORT_IR_RIGHT GPIOD

//ADC multiplexer
#define PORT_ADX_MUX GPIOA

//P for plus
#define PIN_SIG_MOT1P           GPIO_Pin_0
//M for minus
#define PIN_SIG_MOT1M           GPIO_Pin_1

//Used with the ADC multiplexer to select signal source for conversion
#define PIN_ADC_SELECT0         GPIO_Pin_1
#define PIN_ADC_SELECT1         GPIO_Pin_2

word read_ir_status(ir ir_id);
word read_ir_right(void);
word read_ir_left(void);
