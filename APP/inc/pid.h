/*
 * pid.h - proportional, integral, derivative controller with
 *    on-the-fly tuning changes and derivative kick avoidance
 *
 * Written by Peter Lanius
 * Version 0.6		18/01/2013
 * Based on the Arduino PID library (see below)
 */

/*
 * Arduino PID Library - Version 1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 *   http://www.arduino.cc/playground/Code/PIDLibrary
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 */

#ifndef PID_H_
#define PID_H_

#define PID_DIMENSION			2

//Constants used in some of the functions below
#define INT_SCALE_FACTOR 1000
#define AUTOMATIC		1
#define MANUAL			0
#define DIRECT			0
#define REVERSE			1
#define OUTPUT_LIMIT	153*INT_SCALE_FACTOR		// corresponds to 45deg servo offset
#define SAMPLE_INTERVAL	100		// in ms
#define DEFAULT_KP		1.0*INT_SCALE_FACTOR		// determine these through tuning
#define DEFAULT_KI		1.0*INT_SCALE_FACTOR		//
#define DEFAULT_KD		0.01*INT_SCALE_FACTOR	//

// Initialization of the PID controller
// Input  (int) dimension - number of PID channels (default is 2 - x and y axis)
// Initial tuning parameters are also set here
void pid_init();

// Initialize does all the things that need to happen to ensure a
// bumpless transfer from manual to automatic mode.
void pid_initialize();

// sets PID to either Manual (0) or Auto (1) mode
void pid_setMode(int mode);

// Performs the PID calculation. It should be
// called every time loop() cycles. ON/OFF and
// calculation frequency can be set using pid_setMode()
// pid_setSampleTime() respectively
int pid_compute();

//clamps the output to a specific range. 0-255 by default, but
//it's likely the user will want to change this depending on
//the application
void pid_setOutputLimits(int output_min, int output_max);

// While most users will set the tunings once in the
// constructor, this function gives the user the option
// of changing tunings during runtime for Adaptive control
void pid_setTunings(int Kp, int Ki, int Kd);

// Sets the Direction, or "Action" of the controller.
// DIRECT - means the output will increase when error is positive.
// REVERSE - means the opposite.  it's very unlikely that this will be needed
// once it is set in the constructor.
void pid_setControllerDirection(int direction);

//Display functions ****************************************************************
int pid_getKp();			// These functions query the pid for internal values.
int pid_getKi();			// they were created mainly for the pid front-end,
int pid_getKd();			// where it's important to know what is actually
int pid_getMode();			// inside the PID.
int pid_getDirection();		//

#endif /* PID_H_ */
