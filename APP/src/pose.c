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
	uint16 goal_start_pose[NUM_AX12_SERVOS]; // the pose of the robot at the time when the current movement started.

	u32 poseStartedMillis;

	uint16 sub_goal_poses[SUB_GOALS_STACK_SIZE][NUM_AX12_SERVOS];
	uint16 sub_goal_times[SUB_GOALS_STACK_SIZE];
	int current_sub_goal_index;
	unsigned int offset_adjustments_steps;

	// robot's current "actual" positions and speeds etc.
	u32 current_pose_last_read_ms;
	int16 current_pose[NUM_AX12_SERVOS];
	int16 new_joint_offset[NUM_AX12_SERVOS];
	int16 applied_joint_offset[NUM_AX12_SERVOS];
} state = {
		/* Any non-zero initialization of state goes here */
		.current_sub_goal_index = -1
};


void init_pose() {
	state.current_sub_goal_index = -1;
	state.offset_adjustments_steps = 0;
}

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

bool checkServosMoving()
{
	uint8 moving_flag;

	// reset the flag
	moving_flag = 0;

	for (int i=0; i<NUM_AX12_SERVOS; i++) {
		// keep reading the moving state of servos
		moving_flag += dxl_read_byte( AX12_IDS[i], DXL_MOVING );
		// if anything still moving - return
		if ( moving_flag == 1) {
			return moving_flag;
		}
	}
	return 0;
}

// Poll servos to check if goal position is reached (close enough to) goal pose
bool isGoalPoseReached()
{
	return !checkServosMoving();
//	const int eps = 5; // TODO decide value
//	for (int i=0; i<NUM_AX12_SERVOS; i++) {
//		u16 currentPosition = dxl_read_word( AX12_IDS[i], DXL_PRESENT_POSITION_L );
//		u16 goalPositon = getCurrentGoalPose()[i] + state.joint_offset[i];
//		if (goalPositon - currentPosition > eps) {
//			return 0; // servo i not yet reached its goal position.
//		}
//	}
//	return 1;
}

// Function to wait for servos to reach (close enough to) goal pose
void waitForPoseFinish()
{
	while (!isGoalPoseReached()) {
		__asm("nop");
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
		//printf(" %u, %u, %u, %u", state.current_pose[i], goal_pose[i], travel, goal_speed[i]);
	}
	
}

void applyOffsetsAndBounds(uint16 goal_pose_adjusted[NUM_AX12_SERVOS]) {
	int16  temp_goal;
	int i;
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		// process the joint offset values bearing in mind the different variable types
		temp_goal = (int16) getCurrentGoalPose()[i] + state.applied_joint_offset[i];
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

void setGoal(uint16 time, const uint16 goal[])
{
	//printf("Setting new goal pose! resetting stack etc.\n");
	state.current_sub_goal_index = 0;
	state.offset_adjustments_steps = 0;
	state.poseStartedMillis = 0xFFFFFFFF; // max unsigned int32
	for (int i=0; i<NUM_AX12_SERVOS; i++)
	{
		state.sub_goal_poses[0][i] = goal[i];
		state.sub_goal_times[0] = time;
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
int moveToGoalPose(uint8 wait_flag)
{
    int i;
	int commStatus, errorStatus;

	//printf("setting goal pose\n");

	if (state.current_sub_goal_index <= -1) {
		printf("Called moveToGoalPose without a goal pose!\n");
		return -1;
	}

	uint16 *goal_pose = getCurrentGoalPose();

	for (i=0; i<NUM_AX12_SERVOS; i++) {
		state.goal_pose_adjusted[i] = goal_pose[i];

		state.goal_start_pose[i] = state.current_pose[i];
	}


	applyOffsetsAndBounds(state.goal_pose_adjusted);

	//printf("calculate speeds: goal_time %d, stack index= %d\n", state.sub_goal_times[state.current_sub_goal_index], state.current_sub_goal_index);

	// do the setup and calculate speeds
	calculatePoseServoSpeeds(state.sub_goal_times[state.current_sub_goal_index], state.goal_pose_adjusted, state.goal_speed);

	//printf("setting goal pose done\n");

	state.poseStartedMillis = millis();

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
	setGoal(InitialPlayTime, InitialValues);
	moveToGoalPose(WAIT_FOR_POSE_FINISH);
}

void resetJointOffsets(void)
{
	for (int i = 0; i < NUM_AX12_SERVOS; i++) {
		state.new_joint_offset[i] = 0;
	}
}

void setJointOffsetById(u8 id, s16 offset)
{
	if (id == 0 || id > NUM_AX12_SERVOS) {
		printf("setJointOffsetById: invalid servo id, did you perhaps send the index?\n");
	}
	state.new_joint_offset[id-1] = offset;
}

uint16 * getCurrentGoalPose()
{
	if (state.current_sub_goal_index > -1)
		return state.sub_goal_poses[state.current_sub_goal_index];
	else
		return 0;
}

/* Calculate which pose we will be in in `time` ms. */
void poseAtTime(uint16 * ret_pose, uint16 time, uint16 total_goal_time)
{
	/* Option 1: ret_pose = current_pose + (goal_pose - current_pose) * time / time_left */
//	int time_left = state.sub_goal_times[state.current_sub_goal_index] - (millis() - state.poseStartedMillis);
//	for (int i = 0; i < NUM_AX12_SERVOS; i++) {
//		/*u16 travel = ((u32) state.goal_speed[i] * time) / 848; // TODO using goal speeds here, may want to use actual speeds.
//		ret_pose[i] = state.current_pose[i] + travel;*/
//		//ret_pose[i] = state.goal_pose_adjusted[i] + time * state.goal_speed[i] / 848;
//		s16 travel = (state.goal_pose_adjusted[i] - state.current_pose[i]) * (u32)time / time_left;
//		ret_pose[i] = state.current_pose[i] + travel;
//	}

	/* Option 2: ret_pose = current_pose + (end_pose-start_pose) * time/total_goal_time  */
	//const s32 total_goal_time = state.sub_goal_times[state.current_sub_goal_index];
	for (int i = 0; i < NUM_AX12_SERVOS; i++) {
		s16 travel = ((state.goal_pose_adjusted[i] - state.goal_start_pose[i]) * (s32)time) / total_goal_time;
		ret_pose[i] = state.current_pose[i] + travel;
	}
}

// return:
int popSubGoal()
{
	if (state.current_sub_goal_index >= 0) {
		state.current_sub_goal_index--;
		return 1;
	}
	return 0;
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
	++state.current_sub_goal_index;
	if (state.current_sub_goal_index < SUB_GOALS_STACK_SIZE) {
		return state.sub_goal_poses[state.current_sub_goal_index];
	} else {
		// TODO handle stack overflow somehow
		printf("Sub Goal pose stack overflow!\n");
		state.current_sub_goal_index--;
		return 0;
	}
}

void applyOffsets(uint16 time)
{
	if (state.offset_adjustments_steps > 0) {
		return; // TODO replace currently applied offsets instead of just returning.
	}

	bool anyDiff = 0;
	for (int i = 0; i < NUM_AX12_SERVOS; i++)
	{
		if (state.applied_joint_offset[i] != state.new_joint_offset[i])
			anyDiff = 1;
		state.applied_joint_offset[i] = state.new_joint_offset[i];
	}

	// TODO uncomment!
	//if (!anyDiff) {
		//printf("not applying new offsets as they are the same as old offsets.\n");
	//	return;
	//}

	u16 * new_sub_pose = pushSubGoal();
	if (new_sub_pose == 0) {
		printf("ERROR: Attempted to apply offsets but sub goal stack is full!\n");
		return;
	}

	state.offset_adjustments_steps++;
	//printf("Applying new offsets.\n");

	// TODO more effective read current pose?
	readCurrentPose();
	if (state.current_sub_goal_index > 0) { // NOTE its > 0 because we already pushed to the stack, thus incrementing stack pointer...
		//printf("applying offsets to current motion (stack index is %d)\n", state.current_sub_goal_index);
		poseAtTime(new_sub_pose, time, state.sub_goal_times[state.current_sub_goal_index-1]);
		//printf("the predicted pose at stack index %d is: \n", state.current_sub_goal_index);
		//for (int i = 0; i < NUM_AX12_SERVOS; i++)
		//{
		//	printf("  %i: %d \n", i, state.sub_goal_poses[state.current_sub_goal_index][i]);
		//}
		state.sub_goal_times[state.current_sub_goal_index-1] = state.sub_goal_times[state.current_sub_goal_index-1] - (millis() - state.poseStartedMillis) - time;
	} else {
		// We're not currenly in a movement,
		// take the last executed goal pose (which is at index 0 in the stack) and set that as the sub pose,
		// so we can execute it again with the offsets added.
		// actually we dont have to do anything here, as the pose at index 0 is == subPose;
		// We just have to update the time, which is done below.

		//printf("applying offsets to current pose.\n");
//		for (int i = 0; i < NUM_AX12_SERVOS; i++)
//		{
//			subPose[i] = state.sub_goal_poses[0][i];
//		}
	}
	state.sub_goal_times[state.current_sub_goal_index] = time;
	moveToGoalPose(0);
}

// check if a subgoal is reached. if so, pop and execute the next subgoal.
// Return: 0 if current subgoal not yet reached, 1 otherwise.
int pollNextSubGoal() {
	if (state.current_sub_goal_index >= 0) {
		if (millis() - state.poseStartedMillis >= state.sub_goal_times[state.current_sub_goal_index])
		{
			popSubGoal();
			//printf("Popped goal stack! index is now %d\n", state.current_sub_goal_index);

			 // this is to keep track of the number of offset adjustment steps left.
			if (state.offset_adjustments_steps > 0)
				state.offset_adjustments_steps--;

			if (state.current_sub_goal_index > -1)
			{
				//printf("moving to next sub goal\n");
				moveToGoalPose(0);
			}
			return 1;
		}
	}
	return 0;
}


