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

#define DEFAULT_ADJUSTMENT_TIME 50
#define P_REGULATION 1
#define D_REGULATION 1

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

//	fbbalerror = -60; // FAKE ERROR!
//	rlbalerror = 0;
}

u16 constrain_balancing() {

	return 1;
}

void scale_offsets()
{
	front_back_balance_adjustment_1 = 0;
	front_back_balance_adjustment_2 = 0;
	left_right_balance_adjustment_1 = 0;
	left_right_balance_adjustment_2 = 0;

	if(D_REGULATION) {
		fbbalerrorscaled = fbbalerror*3;
		rlbalerrorscaled = rlbalerror*3;

		// limit FB output
		const int fbLimit = 250;
		fbbalerrorscaled = fbbalerrorscaled > fbLimit ? fbLimit : fbbalerrorscaled;
		fbbalerrorscaled = fbbalerrorscaled < -fbLimit ? -fbLimit : fbbalerrorscaled;
		// limit RL output
		const int rlLimit = 300;
		rlbalerrorscaled = rlbalerrorscaled > rlLimit ? rlLimit : rlbalerrorscaled;
		rlbalerrorscaled = rlbalerrorscaled < -rlLimit ? -rlLimit : rlbalerrorscaled;

		front_back_balance_adjustment_1 += fbbalerrorscaled / 60; // adjustment for the knees
		front_back_balance_adjustment_2 += fbbalerrorscaled / 25;

		left_right_balance_adjustment_1 += rlbalerrorscaled / 30;
		left_right_balance_adjustment_2 += rlbalerrorscaled / 52;
	}

	if (P_REGULATION) {

	}


//	 adjustment for the knees
	int delta_fb_knees = current_pose[12] - getCurrentGoalPose()[12]; // note 1 less than servo ID
	if (abs(delta_fb_knees) > 25) {
		if (delta_fb_knees < 0 && front_back_balance_adjustment_1 < 0) {
			front_back_balance_adjustment_1 /= (-delta_fb_knees/2);
		} else if (delta_fb_knees > 0 && front_back_balance_adjustment_1 > 0) {
			front_back_balance_adjustment_1 /= (delta_fb_knees/2);
		}
	}

//	// adjustment for the ankle fb
	int delta_fb_ankleFB = current_pose[14] - getCurrentGoalPose()[14]; // note 1 less than servo ID
	if (abs(delta_fb_ankleFB) > 25) {
		if (delta_fb_ankleFB < 0 && front_back_balance_adjustment_2 < 0) {
			front_back_balance_adjustment_2 /= -delta_fb_ankleFB;
		} else if (delta_fb_ankleFB > 0 && front_back_balance_adjustment_2 > 0) {
			front_back_balance_adjustment_2 /= delta_fb_ankleFB;
		}
	}
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

//			if (1) {
//				printf("\nfbbalerror: %d\n", fbbalerror);
////				printf("rlbalerror: %d\n", rlbalerror);
//				printf("fbbalerrorscaled: %d\n", fbbalerrorscaled);
////				printf("rlbalerrorscaled: %d\n", rlbalerrorscaled);
//				printf("finalfbbal1: %d\n", front_back_balance_adjustment_1);
//				printf("finalfbbal2: %d\n", front_back_balance_adjustment_2);
////				printf("finalrlbal1: %d\n", left_right_balance_adjustment_1);
////				printf("finalrlbal2: %d\n\n", left_right_balance_adjustment_2);
//			}



			if (front_back_balance_adjustment_2 != 0) {
				//printf("adjusting %s: %d\n", front_back_balance_adjustment_2>0?"forewards":"backwards", fbbalerror);
				//printf("offsets is: %d %d %d %d\n", get_offset(13), get_offset(15) ,get_offset(14), get_offset(16));
			}
			/* Balance front/back */
			setJointOffsetSpeedById(13,  front_back_balance_adjustment_1, 1023); // adjustment for the knees
			setJointOffsetSpeedById(15,  front_back_balance_adjustment_2, 1023);
			setJointOffsetSpeedById(14, -front_back_balance_adjustment_1, 1023); // adjustment for the knees
			setJointOffsetSpeedById(16, -front_back_balance_adjustment_2, 1023);


			if (left_right_balance_adjustment_1 != 0) {
				//printf("adjusting %s: %d\n", left_right_balance_adjustment_1>0?"rightwards":"leftwards", rlbalerror);
				//printf("offsets is: %d %d %d %d\n", get_offset(9), get_offset(10) ,get_offset(11), get_offset(11));
			}
			/* Balance left/right */
			setJointOffsetSpeedById(9,  left_right_balance_adjustment_2, 1023);
			setJointOffsetSpeedById(10, left_right_balance_adjustment_2, 1023);
			setJointOffsetSpeedById(17, left_right_balance_adjustment_1, 1023);
			setJointOffsetSpeedById(18, left_right_balance_adjustment_1, 1023);

		}

		iteration++;
	}
}
