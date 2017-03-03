/*
 * pose.c - functions for assuming poses based on motion pages  
 *	also provides the interpolation to smooth out movement 
 * 
 * Modified by Anton Olsson / Cybercom, 2017
 * Based on code from C controller project by:
 * Version 0.5		31/10/2011
 * Written by Peter Lanius
 */


/*
 * You may freely modify and share this code, as long as you keep this
 * notice intact. Licensed under the Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, this work is provided
 * without any warranty. It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#include "printf.h"
#include "stdint.h"
#include "global.h"
#include "pose.h"
#include "dynamixel.h"
#include "time.h"
#include "walk.h"


// initial robot position (MotionPage 224 - Balance)
const uint16 InitialValues[NUM_AX12_SERVOS] = {235,788,279,744,462,561,358,666,507,516,341,682,240,783,647,376,507,516};
const uint16 InitialPlayTime = 400; // 0.4s is fast enough

// we keep shared variables for goal pose and speed
uint16 goal_pose[NUM_AX12_SERVOS];
uint16 goal_pose_shadow[NUM_AX12_SERVOS];
uint16 goal_speed[NUM_AX12_SERVOS];
uint16 goal_pose_adjusted[NUM_AX12_SERVOS]; // adjusted by offsets and bounds

u32 offset_timings[NUM_AX12_SERVOS];

s16 offsets[NUM_AX12_SERVOS];
s16 offsets_shadow[NUM_AX12_SERVOS];
s16 offsets_speeds [NUM_AX12_SERVOS];

u32 goal_time;



POSE_MODE pose_mode = POSE_MODE_DIRECT;

// read in current servo positions to determine current pose
// takes between 260us and 456us per servo (mainly 260us or 300us)
// all up takes 5-6ms
void readCurrentPose()
{
	int i;
	// loop over all possible actuators
	for(i=0; i<NUM_AX12_SERVOS; i++) {
		current_pose[i] = dxl_read_word( AX12_IDS[i], DXL_PRESENT_POSITION_L );
	}
}

// Function to wait out any existing servo movement
void waitForPoseFinish()
{
	uint8 still_moving[NUM_AX12_SERVOS];
	uint8 moving_flag = 0;
	uint8 first_loop = 0;
	int i;
	
	first_loop = 0;
	// keep looping over all possible actuators until done
	do 
	{
		// reset the flag
		moving_flag = 0;
		
		for (i=0; i<NUM_AX12_SERVOS; i++) {
			// keep reading the moving state of servos 
			if( first_loop == 0 || still_moving[i] == 1) {
				still_moving[i] = dxl_read_byte( AX12_IDS[i], DXL_MOVING );
				moving_flag += still_moving[i];
			}		
		}
		first_loop = 1;
	} while (moving_flag > 0);
}

// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// The Moving Speed control table entry states that 0x3FF = 114rpm
// and according to Robotis this means 0x212 = 59rpm and anything greater 0x212 is also 59rpm

void calculatePoseServoSpeeds(uint16 time, uint16 goal_pose[NUM_AX12_SERVOS], uint16 goal_speed[NUM_AX12_SERVOS])
{
    int i;
	uint16 travel;
	uint32 factor;

	// read the current pose only if we are not walking (no time)
	if( walk_getWalkState() == 0 ) {
		readCurrentPose();		// takes 6ms we do this in a global loop
	}
	
	// TEST:
	//printf("\nCalculate Pose Speeds. Time = %i \n", time);
	
	// determine travel for each servo 
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		// TEST:
		//printf("\nDXL%i Current, Goal, Travel, Speed:", i+1);
		


		// find the amount of travel for each servo
		if( goal_pose[i] > current_pose[i]) {
			travel = goal_pose[i] - current_pose[i];
		} else {
			travel = current_pose[i] - goal_pose[i];
		}
		
		// if we are walking we simply set the current pose as the goal pose to save time
		if( walk_getWalkState() != 0 ) {
			current_pose[i] = goal_pose[i];
		}
	
		// now we can calculate the desired moving speed
		// for 59pm the factor is 847.46 which we round to 848
		// we need to use a temporary 32bit integer to prevent overflow
		factor = (uint32) 848 * travel;
		goal_speed[i] = (uint16) ( factor / time );
		// if the desired speed exceeds the maximum, we need to adjust
		if (goal_speed[i] > 1023) goal_speed[i] = 1023;
		// we also use a minimum speed of 26 (5% of 530 the max value for 59RPM)
		if (goal_speed[i] < 26) goal_speed[i] = 26;
		
		// TEST:
		//printf(" %u, %u, %u, %u", current_pose[i], goal_pose[i], travel, goal_speed[i]);
	}
	
}

u16 calculate_speed(u16 goal_position, u16 current_position, u32 time) {
	u16 travel = abs(goal_position - current_position);

	// now we can calculate the desired moving speed
	// for 59pm the factor is 847.46 which we round to 848
	// we need to use a temporary 32bit integer to prevent overflow
	u16 goal_speed = (uint16) ( ((uint32) 848 * travel) / time );
	// if the desired speed exceeds the maximum, we need to adjust
	if (goal_speed > 1023) goal_speed = 1023;
	// we also use a minimum speed of 26 (5% of 530 the max value for 59RPM)
	if (goal_speed < 26) goal_speed = 26;

	return goal_speed;
}

void applyOffsetsBounds(uint16 goal_pose_adjusted[NUM_AX12_SERVOS]) {
	int16  temp_goal;
	int i;
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		// process the joint offset values bearing in mind the different variable types
		temp_goal = (int16) goal_pose[i] + offsets[i];
		if ( temp_goal < 0 ) {
			goal_pose_adjusted[i] = 0;		// can't go below 0
		}
		else if ( temp_goal > 1023 ) {
			goal_pose_adjusted[i] = 1023;	// or above 1023
		}
		else {
			goal_pose_adjusted[i] = (uint16) temp_goal;
		}
	}
}

// Moves from the current pose to the goal pose
// using calculated servo speeds and delay between steps
// to achieve the required step timing
// Inputs:  (uint16)  allocated step time in ms
//          (uint16)  array of goal positions for the actuators
//          (uint8)   flag = 0 don't wait for motion to finish
//					  flag = 1 wait for motion to finish and check alarms
// Returns	(int)	  -1  - communication error
//					   0  - all ok
//					   1  - alarm
int moveToGoalPose(uint16 time, const uint16 goal[], uint8 wait_flag)
{
	//printf("setting goal pose, mode=%d\n", pose_mode);

	if (pose_mode == POSE_MODE_DIRECT || wait_flag) {
		int i;
		int commStatus, errorStatus;

		for (i=0; i<NUM_AX12_SERVOS; i++) {
			goal_pose[i] = goal[i];
			goal_pose_adjusted[i] = goal[i];
		}


		applyOffsetsBounds(goal_pose_adjusted);

		//printf("calculate speeds\n");

		// do the setup and calculate speeds
		calculatePoseServoSpeeds(time, goal_pose_adjusted, goal_speed);

		//printf("setting goal pose done\n");

		// write out the goal positions via sync write
		commStatus = dxl_set_goal_speed(NUM_AX12_SERVOS, AX12_IDS, goal_pose_adjusted, goal_speed);
		// check for communication error or timeout
		if(commStatus != COMM_RXSUCCESS) {
			// there has been an error, print and break
			printf("\nmoveToGoalPose - ");
			dxl_printCommStatus(commStatus);
			return -1;
		}

		//printf("set speeds sent\n");

		// only wait for pose to finish if requested to do so
		if( wait_flag == 1 )
		{
			//printf("waiting for finish...");
			// wait for the movement to finish
			waitForPoseFinish();

			//printf(" done!\n");

			// check that we didn't cause any alarms
			for (i=0; i<NUM_AX12_SERVOS; i++) {
				// ping the servo and unpack error code (if any)
				errorStatus = dxl_ping(AX12_IDS[i]);
				if(errorStatus != 0) {
					// there has been an error, disable torque
					commStatus = dxl_write_byte(BROADCAST_ID, DXL_TORQUE_ENABLE, 0);
					printf("\nmoveToGoalPose Alarm ID%i - Error Code %i\n", AX12_IDS[i], errorStatus);
					return 1;
				}
			}
			//printf("moveToGoalPose: all ok, read back current pose.\n");
			// all ok, read back current pose
			//readCurrentPose();
		}

		return 0;
	} else { // pose_mode == POSE_MODE_SYNC
		for (int i=0; i<NUM_AX12_SERVOS; i++) {
			goal_pose[i] = goal[i];
		}

		goal_time = time;

		return 0;
	}
}

// move robot to default pose
void moveToDefaultPose()
{
	// assume default pose defined 
	moveToGoalPose(InitialPlayTime, InitialValues, WAIT_FOR_POSE_FINISH);
}

void resetJointOffsets(void) {
	for (int i = 0; i < NUM_AX12_SERVOS; i++) {
		offsets[i] = 0;
	}
}

void setJointOffsetById(u8 id, s16 offset)
{
	setJointOffsetSpeedById(id, offset, 1023);
	//printf("setting offset of %d to %d\n", id, offset);
}

void setJointOffsetSpeedById(u8 id, s16 offset, u16 speed)
{
	if (id == 0 || id > NUM_AX12_SERVOS) {
		printf("setJointOffsetById: invalid servo id, did you perhaps send the index?\n");
	}
	if (offset_timings[id-1] != 0)
		return;
	offsets[id-1] = offset;
	offsets_speeds[id-1] = speed;
	//printf("setting offset of %d to %d\n", id, offset);
}

u32 get_offset_adjustment_time(int id) {
	return offset_timings[id-1];
}

s16 get_offset(int id) {
	return offsets[id-1];
}


void set_pose_mode(POSE_MODE mode) {
	pose_mode = mode;
}


uint16 * getCurrentGoalPose(){
	return goal_pose;
}

// Checks the offsets array for new values, and sync writes the changes to the servos.
// This function will calculate the speeds of the servos automatically when a change has occured, based on the curent goal pose time setting.
void apply_new_pose_and_offsets()
{

	int num_changed = 0;
	u8 ids_changed[NUM_AX12_SERVOS];
	u16 goal_pose_changed[NUM_AX12_SERVOS];
	u16 goal_speeds_changed[NUM_AX12_SERVOS];

	const u32 now = micros();

	// check for offset changes, if not offset change, check for goal pose changes
	for (int i = 0; i < NUM_AX12_SERVOS; i++) {

		bool offset_adjustment_has_just_completed = 0;

		// check if offset adjustment is completed.
		if (offset_timings[i] != 0) {
			if (now < offset_timings[i]) {
				continue; // dont do anything with this servo as it is currently adjusting offset.
			} else {
				offset_timings[i] = 0;
				offset_adjustment_has_just_completed = 1;
			}
		}

		if (offsets[i] != offsets_shadow[i]) {
			s16 old_offset = offsets_shadow[i];
			offsets_shadow[i] = offsets[i];

			int speed = offsets_speeds[i];

			u32 est_time = (uint16) ( ((uint32) 848 * 1000 * abs(offsets[i])) / speed );
			//printf("est_time %d\n", est_time);
			offset_timings[i] = now + est_time;

			// apply offsets + bounds
			s16 goal_tmp = current_pose[i] - old_offset + offsets[i];
			if (goal_tmp < 0)
				goal_tmp = 0;
			else if (goal_tmp > 1023)
				goal_tmp = 1023;

			//printf("current pose + offset: %d\n", goal_tmp);

			goal_speeds_changed[num_changed] = speed; // full speed ahead!
			goal_pose_changed[num_changed] = goal_tmp;
			ids_changed[num_changed] = i+1;
			num_changed++;
		} else if (goal_pose[i] != goal_pose_shadow[i] || offset_adjustment_has_just_completed) {
			goal_pose_shadow[i] = goal_pose[i];

			// apply offsets and bounds
			s16 adjusted = (int16) goal_pose[i] + offsets[i];
			if ( adjusted < 0 ) {
				adjusted = 0;		// can't go below 0
			}
			else if ( adjusted > 1023 ) {
				adjusted = 1023;	// or above 1023
			}

			// calculate speeds
			u16 speed = calculate_speed(adjusted, current_pose[i], goal_time);

			goal_pose_adjusted[i] = adjusted;
			goal_pose_changed[num_changed] = adjusted;
			goal_speeds_changed[num_changed] = speed;
			ids_changed[num_changed] = i+1;
			num_changed++;
		}
	}

	if (num_changed > 0) {
		//printf("apply_new_pose_and_offsets: applying new positions to %d servos\n", num_changed);
		/* Apply only the changed servo settings. */
		int commStatus = dxl_set_goal_speed(num_changed, ids_changed, goal_pose_changed, goal_speeds_changed);
		// check for communication error or timeout
		if(commStatus != COMM_RXSUCCESS) {
			// there has been an error, print and break
			printf("ERROR in apply_new_pose_and_offsets - ");
			dxl_printCommStatus(commStatus);
		}
	}
}


