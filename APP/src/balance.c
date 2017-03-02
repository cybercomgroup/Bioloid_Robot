/*
 * balance.c
 *
 *  Created on: 1 mars 2017
 *      Author: Johan
 */

#include "balance.h"
#include "pose.h"
#include "motion.h"
#include "gyro.h"

#define DEFAULT_ADJUSTMENT_TIME 20

s16 pitch;
s16 roll;

s16 pitch_scaled;
s16 roll_scaled;

s16 finalfbbal1;
s16 finalfbbal2;

s16 finalrlbal1;
s16 finalrlbal2;

u16 last_update;

void read_gyro()
{
	gyro_read();

	pitch = gyro_get_pitch();
	roll = gyro_get_roll();
}

u16 constrain_balancing() {

	return 1;
}

void scale_offsets()
{
	pitch_scaled = pitch * 4;
	roll_scaled = roll * 4;

	finalfbbal1 = pitch_scaled / 54;
	finalfbbal2 = pitch_scaled / 18;

	finalrlbal1 = roll_scaled / 20;
	finalrlbal2 = roll_scaled / 40;
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
