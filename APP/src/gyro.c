/*
 * gyro.c
 *
 *  Created on: 21 feb. 2017
 *      Author: kalle
 */

#include "typedefs.h"
#include "sensors.h"
#include "printf.h"
#include "time.h"

/* Globals */

word adc_gyro_x = 0;
word adc_gyro_y = 0;

word adc_gyro_center_x;
word adc_gyro_center_y;

word gyro_pitch = 0;
word gyro_roll = 0;

void gyro_update() {
	gyro_read();

	//printf("GYro update: %d, %d", adc_gyro_x, adc_gyro_y);

	gyro_pitch += adc_gyro_x - adc_gyro_center_x;
	gyro_roll += adc_gyro_y - adc_gyro_center_y;
}

word gyro_get_pitch()
{
	return gyro_pitch;
}
word gyro_get_roll()
{
	return gyro_roll;
}

void gyro_init() {
	GPIO_SetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTP); //Power the pitch sensor
	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTM);

	GPIO_SetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTP); //Power the roll sensor
	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTM);

	const int samples = 10;
	for (int i = 0; i < samples; i++)
	{
		mDelay(50);
		gyro_read();

		adc_gyro_center_x += adc_gyro_x;
		adc_gyro_center_y += adc_gyro_y;
	}
	adc_gyro_center_x /= samples;
	adc_gyro_center_y /= samples;

	printf("Gyro calibration set to: x=%d, y=%d\n", adc_gyro_center_x, adc_gyro_center_y);
}

void gyro_read() {
	/* Start with pitch */

	//Port 3 has ADC 3

//	GPIO_SetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTP); //Power the pitch sensor
//	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTM);

	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT0);
	GPIO_SetBits(PORT_ADX_MUX,PIN_ADC_SELECT1); //0b10 = 2

	ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);

	adc_gyro_x = ADC_GetConversionValue(ADC1); //ADC1-3 are read from port ADC1

	//GPIO_ResetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTP); //Turn off power
	//GPIO_ResetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTM);

	/* Then roll */

	//Port 4 has ADC 4

//	GPIO_SetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTP); //Power the roll sensor
//	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTM);

	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT0);
	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT1); //0b00 = 0

	ADC_ClearFlag(ADC2, ADC_FLAG_EOC); //Clear EOC flag

	ADC_SoftwareStartConvCmd(ADC2, ENABLE);

	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == 0); //ADC4-6 are read from port ADC2

	adc_gyro_y = ADC_GetConversionValue(ADC2);

	//GPIO_ResetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTP); //Turn off power
	//GPIO_ResetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTM);

}
