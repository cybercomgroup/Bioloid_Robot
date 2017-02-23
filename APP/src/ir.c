/*
 * sensors.c
 *
 *  Created on: 7 feb. 2017
 *      Author: kalle
 */

#include "printf.h"
#include "sensors.h"

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

	ADC_ClearFlag(ADC2, ADC_FLAG_EOC); //Clear EOC flag

	ADC_SoftwareStartConvCmd(ADC2, ENABLE);

	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == 0);

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

	ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);

	result = (ADC_GetConversionValue(ADC1));

	//Disable the sensor again
	GPIO_ResetBits(PORT_IR_LEFT, PIN_IR_LEFT_MOTP);
	GPIO_ResetBits(PORT_IR_LEFT, PIN_IR_LEFT_MOTM);

	return result;
}


