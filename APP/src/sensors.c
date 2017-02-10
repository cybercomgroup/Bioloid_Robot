/*
 * sensors.c
 *
 *  Created on: 7 feb. 2017
 *      Author: kalle
 */

#include "printf.h"
#include "sensors.h"
#include "time.h"
#include "stm32f10x_map.h"
#include "stm32f10x_adc.h"

/* This tutorial (for another microcontroller) explains well how the
 * BSR and BSRR registers (used by SetBits and ResetBits) work:
 * http://hertaville.com/stm32f0-gpio-tutorial-part-1.html */

/* ADC5-6 = RIGHT
 * ADC1-2 = LEFT */

word read_ir_status(ir ir_id) {
	word result;

	GPIO_TypeDef *port = 0;

	//TODO: In this switch we might have to set the ADX MUX pins to read from the correct ADC
	switch(ir_id) {
	case RIGHT:
		port = PORT_IR_RIGHT;
		break;
	case LEFT:
		port = PORT_IR_LEFT;
		break;
	}

	//Do some weird wizard shit to enable the IR sensor
	GPIO_SetBits(port, PIN_SIG_MOT1P);
	GPIO_ResetBits(port, PIN_SIG_MOT1M);

	//Select which ADC pins to read from using the multiplexer
	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT0);
	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT1);

	//printf("done flag pre clear: %d \r\n", ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag
	//printf("done flag post clear: %d \r\n", ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	uDelay(30);
	//printf("Starting ADC conv.\r\n");
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); // ADC1 is on port 1 (GPIOA?)

	//printf("done flag pre delay: %d \r\n", ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	//Short delay to allow the ADC to process the signal
	uDelay(5);
	//printf("done flag post delay: %d \r\n", ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	//printf("Waiting for ADC to complete.\r\n");
	//while( ! ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //TODO test if this works.

	result = (ADC_GetConversionValue(ADC1));

	//printf("ADC to complete: %d\r\n", result);

	ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag

	//printf("done flag post clear2: %d \r\n", ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	//Disable the sensor again
	GPIO_ResetBits(port, PIN_SIG_MOT1P);
	GPIO_ResetBits(port, PIN_SIG_MOT1M);

	return result;
}

word read_ir_right(void) {
	return read_ir_status(RIGHT);
}

word read_ir_left(void) {
	return read_ir_status(LEFT);
}


