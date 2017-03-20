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
#include "filters.h"
#include "balance_motion_precorded.h"

#define DEFAULT_ADJUSTMENT_TIME 30
#define P_REGULATION 1
#define D_REGULATION 1

#define LOW_PASS 0

#define ADJUSTMENT_LIM_L 5

s16 fbbalerror;
s16 rlbalerror;

s16 fbbalerrorscaled, fbbalerrorscaled_filtered;
s16 rlbalerrorscaled;

s16 front_back_balance_adjustment_1;
s16 front_back_balance_adjustment_2;

s16 left_right_balance_adjustment_1;
s16 left_right_balance_adjustment_2;

u32 last_update;

int balance_iteration = 0;

extern volatile uint8 current_motion_page;

u32 dt;

#define N_SAMPLES	10
#define CUTOFF		6

int fb_scaled_values[N_SAMPLES];

/* Precorded motions stuff.
 * TODO dont just hardcode the walk forward motion like this. */
mtn_rot_vel_timings * last_mtn = wlk_fwd;
u32 last_walk_cycle_start_time = 0;

// Recieve the object velocities
void get_wanted_gyro_values(s16 * wanted_x, s16 * wanted_y) {
	*wanted_x = gyro_get_center_x();
	*wanted_y = gyro_get_center_y();

	/* Apply pre recorded motion rotational velocities. */
	if (current_motion_page >= 36 && current_motion_page <= 39) { // TODO not hard coded motion page range

		if (current_motion_page == 36) {
			if (getCurrentMotionStartTime() > last_walk_cycle_start_time) {
				// A new walk cycle has started.
				last_walk_cycle_start_time = getCurrentMotionStartTime();
				last_mtn = wlk_fwd;
			}
		}

		const u32 t = millis() - last_walk_cycle_start_time;

		mtn_rot_vel_timings * mtn = last_mtn;
		for (; (mtn+1)->time <= t; ) {
			mtn++;
		}

		s16 x = mtn->x + ((mtn+1)->x - mtn->x) * (t - mtn->time) / (s16)((mtn+1)->time - mtn->time);
		s16 y = mtn->y + ((mtn+1)->y - mtn->y) * (t - mtn->time) / (s16)((mtn+1)->time - mtn->time);

		//printf("x is %d + %d * %d / %d \n", mtn->x, ((mtn+1)->x - mtn->x),  (i - mtn->time), (s16)((mtn+1)->time - mtn->time));
		//printf("%d: %d %d %d %d %d\n", i, mtn->time, mtn->x, mtn->y, x, y);

		last_mtn = mtn;

		*wanted_x += x;
		*wanted_y += y;
	}

}

void update_balance_error()
{
	// gyro_read(); // already read in main loop

	s16 wanted_x, wanted_y;
	get_wanted_gyro_values(&wanted_x, &wanted_y);

	// positive error: we are falling backwards
	// negative error: we are falling forewards
	fbbalerror = gyro_get_x() - wanted_x;

	// positive error: we are falling leftwards
	// negative error: we are falling rightwards
	rlbalerror = gyro_get_y() - wanted_y;

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

//	pid_set_input(PID_CHANNEL_X, fbbalerror);
//	pid_set_input(PID_CHANNEL_Y, rlbalerror);
//	pid_compute(dt);
//
//	fbbalerror = -pid_get_output_unscaled(PID_CHANNEL_X);
//	rlbalerror = -pid_get_output_unscaled(PID_CHANNEL_Y);

//	printf("err: %d\n", fbbalerror);

	if(D_REGULATION) {
		fbbalerrorscaled = fbbalerror*3;
		rlbalerrorscaled = rlbalerror*3;

		if (LOW_PASS) {

			for (int i = 0; i < N_SAMPLES-1; i++) {
				fb_scaled_values[i] = fb_scaled_values[i+1];
			}
			fb_scaled_values[N_SAMPLES-1] = fbbalerrorscaled;

			//fbbalerrorscaled_filtered = lowPassFrequency(fb_scaled_values, dt, N_SAMPLES, 5);
			fbbalerrorscaled = lowPass(fb_scaled_values, dt, N_SAMPLES, CUTOFF);

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
	if (abs(delta_fb_knees) > 15) {
//		printf("d_knees %d adj %d\n", delta_fb_knees, front_back_balance_adjustment_1);

		if (delta_fb_knees < 0 && front_back_balance_adjustment_1 < 0) {
			front_back_balance_adjustment_1 /= (-delta_fb_knees/2);
		} else if (delta_fb_knees > 0 && front_back_balance_adjustment_1 > 0) {
			front_back_balance_adjustment_1 /= (delta_fb_knees/2);
		} else if (delta_fb_knees < 0 && front_back_balance_adjustment_1 > 0) {
			front_back_balance_adjustment_1 *= (-delta_fb_knees/2);
		} else if (delta_fb_knees > 0 && front_back_balance_adjustment_1 < 0) {
			front_back_balance_adjustment_1 *= (delta_fb_knees/2);
		}
	}

//	// adjustment for the ankle fb
	int delta_fb_ankleFB = current_pose[14] - getCurrentGoalPose()[14]; // note 1 less than servo ID
	if (abs(delta_fb_ankleFB) > 15) {
//		printf("d_andkle_fb %d adj %d\n", delta_fb_ankleFB, front_back_balance_adjustment_2);
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



			if (front_back_balance_adjustment_2 != 0 || front_back_balance_adjustment_2 != 0) {
//				printf("Adjusting FB: %d %d @ offs: %d %d\n", front_back_balance_adjustment_1, front_back_balance_adjustment_2, get_offset(13), get_offset(15) );
			}

			/* Balance front/back */
			int speed = 1023;

			// DEBUG prints
//			printf("fberrror: %d   speed:%d,  is offset adjusting busy?: %d\n", fbbalerror, speed,
//					get_offset_adjustment_time(15) || get_offset_adjustment_time(16));
//			printf("error scaled: %d, error fitltered: %d\n", fbbalerrorscaled,  fbbalerrorscaled_filtered);

			if (abs(front_back_balance_adjustment_1) < ADJUSTMENT_LIM_L) {
				 // adjustment for the knees
				setJointOffsetSpeedById(13, front_back_balance_adjustment_1, speed);
				setJointOffsetSpeedById(14, -front_back_balance_adjustment_1, speed);
			}
			if (abs(front_back_balance_adjustment_2) < ADJUSTMENT_LIM_L) {
				// adjustment for the ankles forwad/backward lean
				setJointOffsetSpeedById(15, front_back_balance_adjustment_2, speed);
				setJointOffsetSpeedById(16, -front_back_balance_adjustment_2, speed);
			}

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
