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

#define DEFAULT_ADJUSTMENT_TIME 200

s16 fbbalerror;
s16 rlbalerror;

s16 fbbalerrorscaled;
s16 rlbalerrorscaled;

s16 front_back_balance_adjustment_1;
s16 front_back_balance_adjustment_2;

s16 left_right_balance_adjustment_1;
s16 left_right_balance_adjustment_2;

u16 last_update;

void update_balance_error()
{
	// gyro_read(); // already read in main loop

	// positive error: we are falling backwards
	// negative error: we are falling forewards
	fbbalerror = gyro_get_x() - gyro_get_center_x();

	// positive error: we are falling leftwards
	// negative error: we are falling rightwards
	rlbalerror = gyro_get_y() - gyro_get_center_y();

//	fbbalerror = 60; // FAKE ERROR!
//	rlbalerror = 0;
}

u16 constrain_balancing() {

	return 1;
}

void scale_offsets()
{
	fbbalerrorscaled = fbbalerror * 3;
	rlbalerrorscaled = rlbalerror * 3;

	front_back_balance_adjustment_1 = fbbalerrorscaled / 54;
	front_back_balance_adjustment_2 = fbbalerrorscaled / 18;

	left_right_balance_adjustment_1 = rlbalerrorscaled / 20;
	left_right_balance_adjustment_2 = rlbalerrorscaled / 40;
}

int iteration = 0;
void balance()
{
	if(millis() - last_update >= DEFAULT_ADJUSTMENT_TIME)
	{
		last_update = millis();
		update_balance_error();

		if(constrain_balancing()) {
			scale_offsets();

//			if (iteration % 100 == 0) {
//				printf("fbbalerror: %d\n", fbbalerror);
//				printf("rlbalerror: %d\n", rlbalerror);
//				printf("fbbalerrorscaled: %d\n", fbbalerrorscaled);
//				printf("rlbalerrorscaled: %d\n", rlbalerrorscaled);
//				printf("finalfbbal1: %d\n", front_back_balance_adjustment_1);
//				printf("finalfbbal2: %d\n", front_back_balance_adjustment_2);
//				printf("finalrlbal1: %d\n", left_right_balance_adjustment_1);
//				printf("finalrlbal2: %d\n", left_right_balance_adjustment_2);
//			}

			if (fbbalerrorscaled < 50) {
				if (front_back_balance_adjustment_1 != 0)
					printf("adjusting %s: %d\n", front_back_balance_adjustment_1>0?"forewards":"backwards", fbbalerror);
				/* Balance front/back */
				setJointOffsetById(13,front_back_balance_adjustment_1);
				setJointOffsetById(15,front_back_balance_adjustment_2);
				setJointOffsetById(14,-front_back_balance_adjustment_1);
				setJointOffsetById(16,-front_back_balance_adjustment_2);
			} else {
				printf("too much error fb: error scaled = %d\n", fbbalerrorscaled);
			}

			if (rlbalerrorscaled < 50) {
				/* Balance left/right */
				setJointOffsetById(9,left_right_balance_adjustment_2);
				setJointOffsetById(10,left_right_balance_adjustment_2);
				setJointOffsetById(17,left_right_balance_adjustment_1);
				setJointOffsetById(18,left_right_balance_adjustment_1);

			} else {
				printf("too much error rl: error scaled = %d\n", rlbalerrorscaled);
			}
		}

		iteration++;
	}
}
