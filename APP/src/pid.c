/*
 * pid.c - proportional, integral, derivative controller with
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

#include "pid.h"
#import "time.h"
#include "typedefs.h"

// we assume that x and y axis use the same tuning parameters
// since gyro and actuators are the same
int kp;          // (P)roportional Tuning Parameter
int ki;          // (I)ntegral Tuning Parameter
int kd;          // (D)erivative Tuning Parameter

int dispKp;		// we'll hold on to the tuning parameters in user-entered
int dispKi;		// format for display purposes
int dispKd;		//

// Input, Output and Setpoint variables for the PID controller
extern volatile int pid_input[PID_DIMENSION];
extern volatile int pid_output[PID_DIMENSION];
extern volatile int pid_unscaled_output[PID_DIMENSION];
extern volatile int pid_setpoint[PID_DIMENSION];

// internal variables
int controller_direction = 0;		// direction - DIRECT or REVERSE
int sample_time = 16;				// fixed controller sample time in ms
unsigned long last_time = 0;		// last_time in millis the controller was run
int integral_term[PID_DIMENSION];	// integral terms
int last_input[PID_DIMENSION];	// last input values
int outMin, outMax;				// assumed to be the same in all dimensions
bool inAuto;						// automatic or manual mode

/*Initialization() *********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/

void pid_set_input(int channel, int value) {
	if (channel > PID_DIMENSION) return;
	pid_input[channel] = value;
}

int pid_get_output(int channel) {
	if (channel > PID_DIMENSION) while(1);
	return pid_output[channel];
}

int pid_get_output_unscaled(int channel) {
	if (channel > PID_DIMENSION) while(1);
	return pid_unscaled_output[channel];
}

void pid_init()
{
	// initialize the input, output and setpoint arrays
	for (uint8 i=0; i<PID_DIMENSION; i++) {
		pid_input[i] = 0;
		pid_output[i] = 0;
		pid_setpoint[i] = 0;
		integral_term[i] = 0;
	}

	// set basic properties
	pid_setOutputLimits(0-OUTPUT_LIMIT, OUTPUT_LIMIT);	// default output limit - see pid.h
    sample_time = SAMPLE_INTERVAL;			// default Controller sample time is 16ms
    pid_setControllerDirection(DIRECT);		// direct mode
	// pid_setControllerDirection(REVERSE);		// reverse mode

	// set default tuning values - see pid.h
    pid_setTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);

	// set sample time and initialize timer
	//pid_setSampleTime(SAMPLE_INTERVAL);
	last_time = millis() - sample_time;		// initialize time keeping variable
    inAuto = FALSE;							// don't start the controller until needed
}


/* Compute() **********************************************************************
 *   This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether
 *   a new pid Output needs to be computed
 **********************************************************************************/
int	pid_compute()
{
	int input, error, dInput, output;

	// only compute if in automatic mode
	if (!inAuto) {
		return 0;
	}

	// check if we are due for a calculation
	unsigned long now = millis();
	int timeChange = (now - last_time);

	if ( timeChange >= sample_time )
	{
		for (uint8 i=0; i<PID_DIMENSION; i++)
		{
			// Compute all the working error variables
			input = pid_input[i];
			error = pid_setpoint[i] - input;

			// debug
			printf("input: %d, setpoint: %d, error: %d", input, pid_setpoint[0], error);

			integral_term[i] += (ki * error)/INT_SCALE_FACTOR;

			// keep the integral term within limits
			if (integral_term[i] > outMax) {
				integral_term[i] = outMax;
			} else if (integral_term[i] < outMin) {
				integral_term[i] = outMin;
			}

			dInput = (input - last_input[i]);

			// Compute PID Output
			output = kp * error/INT_SCALE_FACTOR + integral_term[i] - kd * dInput/INT_SCALE_FACTOR;

			// TEST:
			//printf("\nCh: %i, PID input=%i, last=%i, output=%i, ", i, (int16)pid_input[i], (int16)last_input[i], (int16)output);
			//printf(" Err=%i, dI=%i, Int=%i", (int16)error, (int16)dInput, (int16)integral_term[i]);

			// apply output limits
			if (output > outMax) {
				output = outMax;
			} else if (output < outMin) {
				output = outMin;
			}
			pid_output[i] = output;
			pid_unscaled_output[i] = output/INT_SCALE_FACTOR;

			// Remember some variables for next time
			last_input[i] = input;
		}

		last_time = now;
		return 1;
	} else {
		return 0;
	}
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void pid_setTunings(int Kp, int Ki, int Kd)
{
	// this version of the controller only allows positive tuning parameters
	if ( Kp<0 || Ki<0 || Kd<0 ) return;

	// keep the user set values in case they get changed
	dispKp = Kp;
	dispKi = Ki;
	dispKd = Kd;

	// need to convert ms to s for calculations
	int SampleTimeInSec = (sample_time * INT_SCALE_FACTOR)/1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec / INT_SCALE_FACTOR;
	kd = Kd * INT_SCALE_FACTOR / SampleTimeInSec;

	if ( controller_direction == REVERSE )
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

/* SetOutputLimits(...)****************************************************
 *  This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,) the output will be a little different. Maybe they'll
 *  be doing a time window and will need 0-8000 or something. Or maybe they'll
 *  want to clamp it from 0-125.  who knows.
 **************************************************************************/
void pid_setOutputLimits(int output_min, int output_max)
{
	// sanity check
	if ( output_min >= output_max ) return;

	// set the output limits
	outMin = output_min;
	outMax = output_max;

	if ( inAuto )
	{
		// make sure outputs do not exceed maximum values
		for (uint8 i=0; i<PID_DIMENSION; i++)
		{
			if (pid_output[i] > outMax) {
				pid_output[i] = outMax;
			} else if (pid_output[i] < outMin) {
				pid_output[i] = outMin;
			}
		}

		// same applies to the integral terms
		for (uint8 i=0; i<PID_DIMENSION; i++)
		{
			if (integral_term[i] > outMax) {
				integral_term[i] = outMax;
			} else if (integral_term[i] < outMin) {
				integral_term[i] = outMin;
			}
		}
	}
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void pid_setMode(int mode)
{
	bool newAuto = (mode == AUTOMATIC);
	if (newAuto == !inAuto) {
		/*we just went from manual to auto*/
		pid_initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void pid_initialize()
{
	// set integral term and last input
	for (uint8 i=0; i<PID_DIMENSION; i++)
	{
		integral_term[i] = pid_output[i];
		last_input[i] = pid_input[i];
		// preserve output limits
		if (integral_term[i] > outMax) {
			integral_term[i] = outMax;
		} else if (integral_term[i] < outMin) {
			integral_term[i] = outMin;
		}
	}
	// initialize time keeping variable
	last_time = millis() - sample_time;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void pid_setControllerDirection(int direction)
{
   if (inAuto && direction != controller_direction)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controller_direction = direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID. They're here for display
 * purposes. This are the functions the PID Front-end uses for example
 ******************************************************************************/
int pid_getKp() { return  dispKp; }
int pid_getKi() { return  dispKi; }
int pid_getKd() { return  dispKd; }
int pid_getMode() { return  inAuto ? AUTOMATIC : MANUAL; }
int pid_getDirection() { return controller_direction; }
