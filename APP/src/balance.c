/*
 * balance.c
 *
 *  Created on: 1 mars 2017
 *      Author: Johan
 */

#include "balance.h"
#include "pose.h"
#include "motion.h"
#include "sensors.h"

#define DEFAULT_ADJUSTMENT_TIME 20

s16 fbbaldata;
s16 rlbaldata;

s16 fbbalerror;
s16 rlbalerror;

s16 fbbalerrorscaled;
s16 rlbalerrorscaled;

s16 finalfbbal1;
s16 finalfbbal2;

s16 finalrlbal1;
s16 finalrlbal2;

u16 last_update;

void read_gyro()
{
	gyro_read();

	fbbaldata = get_adc_gyro_x();
	rlbaldata = get_adc_gyro_x();

	fbbalerror = fbbaldata - get_adc_gyro_center_x();
	rlbalerror = rlbaldata - get_adc_gyro_center_y();
}

u16 constrain_balancing() {

	return 1;
}

void scale_offsets()
{
	fbbalerrorscaled = fbbalerror * 4;
	rlbalerrorscaled = rlbalerror * 4;

	finalfbbal1 = fbbalerrorscaled / 54;
	finalfbbal2 = fbbalerrorscaled / 18;

	finalrlbal1 = rlbalerrorscaled / 20;
	finalrlbal2 = rlbalerrorscaled / 40;
}

void balance()
{

	if(millis() - last_update >= DEFAULT_ADJUSTMENT_TIME)
	{
		last_update = millis();
		read_gyro();

		if(constrain_balancing()) {
			scale_offsets();

			setJointOffsetById(13,finalfbbal1);
			setJointOffsetById(15,finalfbbal2);
			setJointOffsetById(14,finalfbbal1);
			setJointOffsetById(16,finalfbbal2);

			setJointOffsetById(9,finalrlbal1);
			setJointOffsetById(10,finalrlbal2);
			setJointOffsetById(17,finalrlbal1);
			setJointOffsetById(18,finalrlbal2);
		}

	}
}
