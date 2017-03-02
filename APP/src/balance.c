/*
 * balance.c
 *
 *  Created on: 1 mars 2017
 *      Author: Johan
 */

#include "balance.h"
#include "pose.h"
#include "sensors.h"
#include "printf.h"

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

void update_balance_error()
{
	// gyro_read(); // already read in main loop

	fbbaldata = gyro_get_x();
	rlbaldata = gyro_get_y();

	fbbalerror = fbbaldata - gyro_get_center_x();
	rlbalerror = rlbaldata - gyro_get_center_y();
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
		update_balance_error();

		if(constrain_balancing()) {
			scale_offsets();

			printf("fbbalerror: %d\n", fbbalerror);
			printf("rlbalerror: %d\n", rlbalerror);
			printf("fbbalerrorscaled: %d\n", fbbalerrorscaled);
			printf("rlbalerrorscaled: %d\n", rlbalerrorscaled);
			printf("finalfbbal1: %d\n", finalfbbal1);
			printf("finalfbbal2: %d\n", finalfbbal2);
			printf("finalrlbal1: %d\n", finalrlbal1);
			printf("finalrlbal2: %d\n", finalrlbal2);
//			setJointOffsetById(13,finalfbbal1);
//			setJointOffsetById(15,finalfbbal2);
//			setJointOffsetById(14,finalfbbal1);
//			setJointOffsetById(16,finalfbbal2);
//
//			setJointOffsetById(9,finalrlbal1);
//			setJointOffsetById(10,finalrlbal2);
//			setJointOffsetById(17,finalrlbal1);
//			setJointOffsetById(18,finalrlbal2);
		}

	}
}
