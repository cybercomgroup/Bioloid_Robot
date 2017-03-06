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
#include "pid.h"

#define DEFAULT_ADJUSTMENT_TIME 30
#define P_REGULATION 1
#define D_REGULATION 1

#define LOW_PASS 1

s16 fbbalerror;
s16 rlbalerror;

s16 fbbalerrorscaled, fbbalerrorscaled_filtered;
s16 rlbalerrorscaled;

s16 front_back_balance_adjustment_1;
s16 front_back_balance_adjustment_2;

s16 left_right_balance_adjustment_1;
s16 left_right_balance_adjustment_2;

u16 last_update;

int balance_iteration = 0;

u32 dt;

#define N_SAMPLES 10
#define CUTOFF 6

int fb_scaled_values[N_SAMPLES];

void update_balance_error()
{
	// gyro_read(); // already read in main loop

	// positive error: we are falling backwards
	// negative error: we are falling forewards
	//fbbalerror = gyro_get_x() - gyro_get_center_x();

	pid_set_input(PID_CHANNEL_X, gyro_get_x() - gyro_get_center_x());

	// positive error: we are falling leftwards
	// negative error: we are falling rightwards
	//rlbalerror = gyro_get_y() - gyro_get_center_y();
	pid_set_input(PID_CHANNEL_Y, gyro_get_y() - gyro_get_center_y());


	pid_compute();

	fbbalerror = pid_get_output_unscaled(PID_CHANNEL_X);
	rlbalerror = pid_get_output_unscaled(PID_CHANNEL_Y);

//	fbbalerror = -60; // FAKE ERROR!
//	rlbalerror = 0;
}

u16 constrain_balancing() {

	return 1;
}

int lowPassFrequency(const int* input, /*int* output,*/ int dt, int points, int cutoff)
{
    long RC = (1000/cutoff) / 6;
    //output[0] = input[0];
    int output = input[0];
    for(int i = 1; i < points; ++i)
    {
    	output = output + ((input[i] - output) * dt / (RC+dt));
    }
    return output;
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

		if (LOW_PASS) {

			for (int i = 0; i < N_SAMPLES-1; i++) {
				fb_scaled_values[i] = fb_scaled_values[i+1];
			}
			fb_scaled_values[N_SAMPLES-1] = fbbalerrorscaled;

			//fbbalerrorscaled_filtered = lowPassFrequency(fb_scaled_values, dt, N_SAMPLES, 5);
			fbbalerrorscaled = lowPassFrequency(fb_scaled_values, dt, N_SAMPLES, CUTOFF);

		}

		// limit FB output
		const int fbLimit = 250;
		fbbalerrorscaled = fbbalerrorscaled > fbLimit ? fbLimit : fbbalerrorscaled;
		fbbalerrorscaled = fbbalerrorscaled < -fbLimit ? -fbLimit : fbbalerrorscaled;
		// limit RL output
		const int rlLimit = 300;
		rlbalerrorscaled = rlbalerrorscaled > rlLimit ? rlLimit : rlbalerrorscaled;
		rlbalerrorscaled = rlbalerrorscaled < -rlLimit ? -rlLimit : rlbalerrorscaled;

		front_back_balance_adjustment_1 += fbbalerrorscaled / 70; // adjustment for the knees
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



void balance()
{
	u32 t = millis();
	dt = t - last_update;
	if(dt >= DEFAULT_ADJUSTMENT_TIME)
	{
		last_update = t;
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
			int speed = 1023;
//			int speed = 1023 / (abs(fbbalerror)/8 +1);
//			if(speed < 500) speed = 500;
//			int speed = 800 + 1023 * abs(fbbalerror) / 50;
//			if(speed < 26) speed = 26;
//			else if(speed > 1023) speed = 1023;
//			printf("fberrror: %d   speed:%d,  is offset adjusting busy?: %d\n", fbbalerror, speed,
//					get_offset_adjustment_time(15) || get_offset_adjustment_time(16));

			//printf("error scaled: %d, error fitltered: %d\n", fbbalerrorscaled,  fbbalerrorscaled_filtered);

			setJointOffsetSpeedById(13,  front_back_balance_adjustment_1, speed); // adjustment for the knees
			setJointOffsetSpeedById(15,  front_back_balance_adjustment_2, speed);
			setJointOffsetSpeedById(14, -front_back_balance_adjustment_1, speed); // adjustment for the knees
			setJointOffsetSpeedById(16, -front_back_balance_adjustment_2, speed);


			if (left_right_balance_adjustment_1 != 0) {
				//printf("adjusting %s: %d\n", left_right_balance_adjustment_1>0?"rightwards":"leftwards", rlbalerror);
				//printf("offsets is: %d %d %d %d\n", get_offset(9), get_offset(10) ,get_offset(11), get_offset(11));
			}
			/* Balance left/right */
			setJointOffsetSpeedById(9,  left_right_balance_adjustment_2, speed);
			setJointOffsetSpeedById(10, left_right_balance_adjustment_2, speed);
			setJointOffsetSpeedById(17, left_right_balance_adjustment_1, speed);
			setJointOffsetSpeedById(18, left_right_balance_adjustment_1, speed);

		}

		balance_iteration++;
	}
}
