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

word read_ir_right(void) {
	word result;

	GPIO_SetBits(PORT_IR_RIGHT, PIN_IR_RIGHT_MOTP);
	GPIO_ResetBits(PORT_IR_RIGHT, PIN_IR_RIGHT_MOTM);

	//Select which ADC pins to read from using the multiplexer
	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT0);
	GPIO_SetBits(PORT_ADX_MUX,PIN_ADC_SELECT1);

	//ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag
	uDelay(30);
	//while( ! ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //TODO test if this works.

	ADC_SoftwareStartConvCmd(ADC2, ENABLE);

	//ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag

	uDelay(5);

	result = (ADC_GetConversionValue(ADC2));

	//Disable the sensor again
	GPIO_ResetBits(PORT_IR_RIGHT, PIN_IR_RIGHT_MOTP);
	GPIO_ResetBits(PORT_IR_RIGHT, PIN_IR_RIGHT_MOTM);

	return result;
}

word read_ir_left(void) {
	word result;

	GPIO_SetBits(PORT_IR_LEFT, PIN_IR_LEFT_MOTP);
	GPIO_ResetBits(PORT_IR_LEFT, PIN_IR_LEFT_MOTM);

	//Select which ADC pins to read from using the multiplexer
	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT0);
	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT1);

	uDelay(30);

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	uDelay(5);

	result = (ADC_GetConversionValue(ADC1));

	//ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag

	//Disable the sensor again
	GPIO_ResetBits(PORT_IR_LEFT, PIN_IR_LEFT_MOTP);
	GPIO_ResetBits(PORT_IR_LEFT, PIN_IR_LEFT_MOTM);

	return result;
}


