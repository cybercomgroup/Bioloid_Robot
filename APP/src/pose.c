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

#define SUB_GOALS_STACK_SIZE 64  // TODO decide a good stack size?


// initial robot position (MotionPage 224 - Balance)
const uint16 InitialValues[NUM_AX12_SERVOS] = {235,788,279,744,462,561,358,666,507,516,341,682,240,783,647,376,507,516};
const uint16 InitialPlayTime = 400; // 0.4s is fast enough


// current state of the robots pose and goal pose
struct pose_state_struct {
	// goal servo positions and speed etc.

	uint16 goal_pose_adjusted[NUM_AX12_SERVOS]; // adjusted by offsets and bounds
	uint16 goal_speed[NUM_AX12_SERVOS];

	uint16 sub_goal_poses[SUB_GOALS_STACK_SIZE][NUM_AX12_SERVOS];
	uint16 sub_goal_times[SUB_GOALS_STACK_SIZE];
	int nSubGoals;

	// robot's current "actual" positions and speeds etc.
	int16 current_pose[NUM_AX12_SERVOS];
	int16 joint_offset[NUM_AX12_SERVOS];
} state;





// read in current servo positions to determine current pose
// takes between 260us and 456us per servo (mainly 260us or 300us)
// all up takes 5-6ms
void readCurrentPose()
{
	int i;
	// loop over all possible actuators
	for(i=0; i<NUM_AX12_SERVOS; i++) {
		state.current_pose[i] = dxl_read_word( AX12_IDS[i], DXL_PRESENT_POSITION_L );
	}
}

// Poll servos to check if goal position is reached (close enough to) goal pose
bool isGoalPoseReached()
{
	const int eps = 5; // TODO decide value
	for (int i=0; i<NUM_AX12_SERVOS; i++) {
		u16 currentPosition = dxl_read_word( AX12_IDS[i], DXL_PRESENT_POSITION_L );
		u16 goalPositon = getCurrentGoalPose()[i] + state.joint_offset[i];
		if (goalPositon - currentPosition > eps) {
			return 0; // servo i not yet reached its goal position.
		}
	}
	return 1;
}

// Function to wait for servos to reach (close enough to) goal pose
void waitForPoseFinish()
{
	while (!isGoalPoseReached()) {
		mDelay(1);
	}
}
// Function to wait out any existing servo movement
//void waitForPoseFinish()
//{
//	uint8 still_moving[NUM_AX12_SERVOS];
//	uint8 moving_flag = 0;
//	uint8 first_loop = 0;
//	int i;
//
//	first_loop = 0;
//	// keep looping over all possible actuators until done
//	do
//	{
//		// reset the flag
//		moving_flag = 0;
//
//		for (i=0; i<NUM_AX12_SERVOS; i++) {
//			// keep reading the moving state of servos
//			if( first_loop == 0 || still_moving[i] == 1) {
//				still_moving[i] = dxl_read_byte( AX12_IDS[i], DXL_MOVING );
//				moving_flag += still_moving[i];
//			}
//		}
//		first_loop = 1;
//	} while (moving_flag > 0);
//}

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
		readCurrentPose();		// takes 6ms
	}
	
	// TEST:
	//printf("\nCalculate Pose Speeds. Time = %i \n", time);
	
	// determine travel for each servo 
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		// TEST:
		//printf("\nDXL%i Current, Goal, Travel, Speed:", i+1);
		
		

		// find the amount of travel for each servo
		if( goal_pose[i] > state.current_pose[i]) {
			travel = goal_pose[i] - state.current_pose[i];
		} else {
			travel = state.current_pose[i] - goal_pose[i];
		}
		
		// if we are walking we simply set the current pose as the goal pose to save time
		if( walk_getWalkState() != 0 ) {
			state.current_pose[i] = goal_pose[i];
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
		//printf(" %u, %u, %u, %u", current_pose[i], goal_pose[i], travel[i], goal_speed[i]);
	}
	
}

void applyOffsetsBounds(uint16 goal_pose_adjusted[NUM_AX12_SERVOS]) {
	int16  temp_goal;
	int i;
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		// process the joint offset values bearing in mind the different variable types
		temp_goal = (int16) getCurrentGoalPose()[i] + state.joint_offset[i];
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
    int i;
	int commStatus, errorStatus;

	//printf("setting goal pose\n");


	uint16 *goal_pose = getCurrentGoalPose();

	for (i=0; i<NUM_AX12_SERVOS; i++) {
		goal_pose[i] = goal[i];
		state.goal_pose_adjusted[i] = goal[i];
	}


	applyOffsetsBounds(state.goal_pose_adjusted);

	//printf("calculate speeds\n");

	// do the setup and calculate speeds
	calculatePoseServoSpeeds(time, state.goal_pose_adjusted, state.goal_speed);

	//printf("setting goal pose done\n");

	// write out the goal positions via sync write
	commStatus = dxl_set_goal_speed(NUM_AX12_SERVOS, AX12_IDS, state.goal_pose_adjusted, state.goal_speed);
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
		readCurrentPose();
	}	
	return 0;
}

// move robot to default pose
void moveToDefaultPose()
{
	// assume default pose defined 
	moveToGoalPose(InitialPlayTime, InitialValues, WAIT_FOR_POSE_FINISH);
}

void resetJointOffsets(void)
{
	for (int i = 0; i < NUM_AX12_SERVOS; i++) {
		state.joint_offset[i] = 0;
	}
}

void setJointOffsetById(u8 id, s16 offset)
{
	if (id == 0 || id > NUM_AX12_SERVOS) {
		printf("setJointOffsetById: invalid servo id, did you perhaps send the index?\n");
	}
	state.joint_offset[id-1] = offset;
}

uint16 * getCurrentGoalPose()
{
	return state.sub_goal_poses[0];
}

void poseAtTime(uint16 * ret_pose, uint16 time)
{
	for (int i = 0; i < NUM_AX12_SERVOS; i++) {
		ret_pose[i] = state.current_pose[i] + state.goal_speed[i] * time; // TODO using goal speeds here, may want to use actual speeds.
	}
}

int popSubGoal()
{
	if (state.nSubGoals >= 0) {
		state.nSubGoals--;
		return 0;
	}
	return -1;
}

//int pushSubGoal(u16 subPose[NUM_AX12_SERVOS], uint16 time)
//{
//	++state.nSubGoals;
//	if (state.nSubGoals < SUB_GOALS_STACK_SIZE) {
//		state.sub_goal_times[state.nSubGoals] = time;
//
//		for (int i = 0; i < NUM_AX12_SERVOS; i++) {
//			state.sub_goal_poses[state.nSubGoals][i] = subPose[i];
//		}
//		return 0;
//	} else {
//		// TODO handle stack overflow somehow
//		printf("Sub Goal pose stack overflow!\n");
//		state.nSubGoals--;
//		return 0;
//	}
//}

u16 * pushSubGoal()
{
	++state.nSubGoals;
	if (state.nSubGoals < SUB_GOALS_STACK_SIZE) {
		return state.sub_goal_poses[state.nSubGoals];
	} else {
		// TODO handle stack overflow somehow
		printf("Sub Goal pose stack overflow!\n");
		state.nSubGoals--;
		return 0;
	}
}

void applyOffsets(uint16 time)
{
	u16 * subPose = pushSubGoal();
	if (subPose != 0) {
		// TODO more effective read current pose?
		readCurrentPose();
		poseAtTime(subPose, time);
		state.sub_goal_times[state.nSubGoals] = time;
		moveToGoalPose(time, subPose, 0);
	}
}


