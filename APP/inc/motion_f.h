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

// Initialize the motion pages by constructing a table of pointers to each page
// Motion pages are stored in Flash (PROGMEM) - see motion.h
void motionPageInit();

// This function unpacks a motion stored in program memory (Flash) 
// in a struct stored in RAM to allow execution
// StartPage - number of the motion page to be unpacked
void unpackMotion(int StartPage);

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

#endif /* MOTION_F_H_ */