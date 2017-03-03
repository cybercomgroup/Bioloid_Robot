/*
 * pose.h - functions for assuming poses base on motion pages
 *	also provides the interpolation to smooth out movement
 *
 * Version 0.5		31/10/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
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


#ifndef POSE_H_
#define POSE_H_

#include "typedefs.h"
#include "global.h"

// define more readable constants for moveToGoalPose wait_flag
#define WAIT_FOR_POSE_FINISH		1
#define DONT_WAIT_FOR_POSE_FINISH	0
#define READ_ALL					1
#define READ_MOVING					0

typedef enum {
	POSE_MODE_DIRECT,
	POSE_MODE_SYNC
} POSE_MODE;

// global hardware definition variables
extern const uint8 AX12Servos[MAX_AX12_SERVOS];
extern const uint8 AX12_IDS[NUM_AX12_SERVOS];
// should keep the current pose in a global array
extern volatile int16 current_pose[NUM_AX12_SERVOS];
extern volatile uint8 motion_step_servos_moving[MAX_MOTION_STEPS][NUM_AX12_SERVOS];


// read in current servo positions to determine current pose
// takes between 260us and 456us per servo (mainly 260us or 300us)
// all up takes 5-6ms for READ_ALL mode
// Inputs:	(uint8)		read mode - all or only moving servos
//			(uint8)		current step
void readCurrentPose();

// Function to wait out any existing servo movement
void waitForPoseFinish();

// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// We ignore the Moving Speed entry which states that 0x3FF = 114rpm
// Instead we assume that Moving Speed 0x3FF = 59rpm
void calculatePoseServoSpeeds(uint16 time, uint16 goal_pose[NUM_AX12_SERVOS], uint16 goal_speed[NUM_AX12_SERVOS]);

// Moves from the current pose to the goal pose
// using calculated servo speeds and delay between steps
// to achieve the required step timing (actual play time)
// Inputs:  (uint16)  allocated step time in ms
//          (uint16)  array of goal positions for the actuators
//          (uint8)   flag = 0 don't wait for motion to finish
//					  flag = 1 wait for motion to finish and check alarms
// Returns	(int)	  -1  - communication error
//					   0  - all ok
//					   1  - alarm
int moveToGoalPose(uint16 time, const uint16 goal[], uint8 wait_flag);

// Assume default pose (Balance - MotionPage 224)
void moveToDefaultPose(void);

void resetJointOffsets(void);

s16 get_offset(int id);

void set_pose_mode(POSE_MODE mode);

void setJointOffsetById(u8 id, s16 offset);
void setJointOffsetSpeedById(u8 id, s16 offset, u16 speed);

uint16 * getCurrentGoalPose();

void apply_new_pose_and_offsets();

#endif /* POSE_H_ */
