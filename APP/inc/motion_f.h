/*
 * motion_f.h - function prototypes for motion.c
 * 
 * Version 0.5		31/10/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 *
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


#ifndef MOTION_F_H_
#define MOTION_F_H_

// motions from motion.h
#define MOTION_BOW 			1
#define MOTION_BRAVO 		2
#define MOTION_RAP_CHEST 	5
#define MOTION_SCRATCH_HEAD 7
#define MOTION_PUSH_UP		8
#define MOTION_HAND_STAND   11
#define MOTION_BLOCK_R		14
#define MOTION_BLOCK_L		16
#define MOTION_KICK_L		18
#define MOTION_KICK_R		19
#define MOTION_ATTACK_R		20
#define MOTION_ATTACK_L		21
#define MOTION_ATTACK_F		22
#define MOTION_DEFENCE		23
#define MOTION_SIT			25
#define MOTION_STAND		26
#define MOTION_GRAB			240


// set use_old_motions_code to one to use the old motion code and the motion definitions in this header.
#define use_old_motions_code 0

// use the new and improved motion unpacker.
// if use_old_motions_code you can still use the new unpacker code by calling unpackMotion2
#if !use_old_motions_code
#define unpackMotion(x) unpackMotion2(x)
#endif

// Initialize the motion pages by constructing a table of pointers to each page
// Motion pages are stored in Flash (PROGMEM) - see motion.h
void motionPageInit();

#if use_old_motions_code
// This function unpacks a motion stored in program memory (Flash) 
// in a struct stored in RAM to allow execution
// StartPage - number of the motion page to be unpacked
void unpackMotion(int StartPage);

#endif //use_old_motions_code

// This function unpacks a motion page as stored by RoboPlus Motion.
// StartPage - number of the motion page to be unpacked
void unpackMotion2(int StartPage);

// This function initiates the execution of a motion step in the given motion page
// Page - number of the motion page 
// Returns (long) start time of the step
unsigned long executeMotionStep(int Step);

// This function initializes the joint flexibility values for the motion page
// Returns (int)  0  - all ok
//				 -1  - communication error
int setMotionPageJointFlexibility();

// This function executes a single robot motion page defined in motion.h
// StartPage - number of the first motion page in the motion
// Returns StartPage of next motion in sequence (0 - no further motions)
int executeMotion(int StartPage);

// This function executes robot motions consisting of one or more motion 
// pages defined in motion.h
// It implements a finite state machine to know what it is doing and what to do next
// Code is meant to be reentrant so it can easily be converted to a task with a RTOS
// Returns:		motion_state
uint8 executeMotionSequence();

// This function executes the exit page motion for the  
// current motion page
 void executeMotionExitPage();

// Function to check for any remaining servo movement
// Returns:  (int)	number of servos still moving
int checkMotionStepFinished();

// Function to check motions state is idle
// Returns:  (int)	if still in a motion
int checkMotionFinished();

// set the new command flag and the next motion page to execute, when the robot goes idle.
void setNewMotionCommand(int motionPageId);

// for debugging
void printCurrentMotionPage();

#endif /* MOTION_F_H_ */
