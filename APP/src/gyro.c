/*
 * gyro.c
 *
 *  Created on: 21 feb. 2017
 *      Author: kalle
 */

#include "typedefs.h"
#include "gyro.h"
#include "printf.h"
#include "time.h"

/* Globals */
unsigned long last_read = 0;

word adc_gyro_x = 0;
word adc_gyro_y = 0;

word adc_gyro_center_x = 311;
word adc_gyro_center_y = 311;

word gyro_pitch = 0;
word gyro_roll = 0;

word pitch_angle = 0;
word roll_angle = 0;

gyro pitch_data;
gyro roll_data;

/* Function definitions */
void _gyro_init(gyro *g, const float q_angle, const float q_gyro, const float r_angle)
{
	g->q_angle = q_angle;
	g->q_gyro  = q_gyro;
	g->r_angle = r_angle;

	g->p_00 = 0;
	g->p_01 = 0;
	g->p_10 = 0;
	g->p_11 = 0;
}

void gyro_init() {
	_gyro_init(&roll_data, Q_ANGLE, Q_GYRO, R_ANGLE);
	_gyro_init(&pitch_data, Q_ANGLE, Q_GYRO, R_ANGLE);
	last_read = millis();
}

void gyro_predict(gyro *g, double dot_angle, double dt)
{
	g->x_angle += dt * (dot_angle - g->x_bias);
	g->p_00 += -1 * dt * (g->p_10 + g->p_01) + dt*dt * g->p_11 + g->q_angle;
	g->p_01 += -1 * dt * g->p_11;
	g->p_10 += -1 * dt * g->p_11;
	g->p_11 += g->q_gyro;
}

//Kalman update function
double gyro_update_kalman(gyro *g)
{
	const double y = -g->x_angle;
	const double s = g->p_00 + g->r_angle;
	const double k_0 = g->p_00 / s;
	const double k_1 = g->p_10 / s;

	g->x_angle += k_0 * y;
	g->x_bias  += k_1 * y;

	g->p_00 -= k_0 * g->p_00;
	g->p_01 -= k_0 * g->p_01;
	g->p_10 -= k_1 * g->p_00;
	g->p_11 -= k_1 * g->p_01;

	return g->x_angle;
}

//This is the funciton you run to get the gyro angles
void gyro_process()
{
	gyro_read();
	unsigned long now = millis();
	double dt = (now - last_read) / 1000;

	// Only process delta angles if at least 1/100 of a second has elapsed
	if (dt >= 0.01)
	{
		last_read = now;

 		// calculate gyro deviations from center values
		gyro_pitch = adc_gyro_x - (int16) adc_gyro_center_x;
		gyro_roll = adc_gyro_y - (int16) adc_gyro_center_y;

		gyro_predict(&pitch_data, gyro_pitch * DEG_TO_RAD, dt);
		gyro_predict(&roll_data, gyro_roll * DEG_TO_RAD, dt);

		pitch_angle = (word) (gyro_update_kalman(&pitch_data)*RAD_TO_DEG);
		roll_angle = (word) (gyro_update_kalman(&roll_data)*RAD_TO_DEG);

		// TEST:
		printf("\nPitch: %i, Roll: %i", pitch_angle, roll_angle);
	}
}

/* Reads gyroscope sensor data from the ADC.
 * Writes pitch and roll to adc_gyro_x and adc_gyro_y respectively. */
void gyro_read() {
	/* Start with pitch */

	//Port 3 has ADC 3

	GPIO_SetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTP); //Power the pitch sensor
	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTM);

	GPIO_SetBits(PORT_ADX_MUX,PIN_ADC_SELECT0);
	GPIO_SetBits(PORT_ADX_MUX,PIN_ADC_SELECT1); //0b11 = 3

	ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //Clear EOC flag

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);

	adc_gyro_x = ADC_GetConversionValue(ADC1); //ADC1-3 are read from port ADC1

	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTP); //Turn off power
	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_PITCH_MOTM);

	/* Then roll */

	//Port 4 has ADC 4

	GPIO_SetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTP); //Power the roll sensor
	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTM);

	GPIO_SetBits(PORT_ADX_MUX,PIN_ADC_SELECT0);
	GPIO_ResetBits(PORT_ADX_MUX,PIN_ADC_SELECT1); //0b01 = 1

	ADC_ClearFlag(ADC2, ADC_FLAG_EOC); //Clear EOC flag

	ADC_SoftwareStartConvCmd(ADC2, ENABLE);

	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == 0); //ADC4-6 are read from port ADC2

	adc_gyro_y = ADC_GetConversionValue(ADC2);

	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTP); //Turn off power
	GPIO_ResetBits(PORT_GYRO, PIN_GYRO_ROLL_MOTM);

}
