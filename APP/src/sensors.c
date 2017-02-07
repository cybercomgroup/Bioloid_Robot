/*
 * sensors.c
 *
 *  Created on: 7 feb. 2017
 *      Author: kalle
 */

#include "sensors.h"
#include "stm32f10x_map.h"

/* This tutorial (for another microcontroller) explains well how the
 * BSR and BSRR registers (used by SetBits and ResetBits) work:
 * http://hertaville.com/stm32f0-gpio-tutorial-part-1.html */

/* ADC5-6 = RIGHT
 * ADC1-2 = LEFT */

word read_ir_status(ir ir_id) {
	word result;

	GPIO_TypeDef port;

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

	uDelay(30);

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	//Short delay to allow the ADC to process the signal
	uDelay(5);
	result = (ADC_GetConversionValue(ADC1));

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

