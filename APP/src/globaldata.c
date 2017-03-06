// This file various global variables

#include "typedefs.h" // for bool
#include "global.h"

// Array showing which Dynamixel servos are enabled (ID from 0 to 25)
#ifdef HUMANOID_TYPEA
  const uint8 AX12Servos[MAX_AX12_SERVOS] = {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0};
  const uint8 AX12_IDS[NUM_AX12_SERVOS] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
#endif
#ifdef HUMANOID_TYPEB
  const uint8 AX12Servos[MAX_AX12_SERVOS] = {0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0};
  const uint8 AX12_IDS[NUM_AX12_SERVOS] = {1,2,3,4,5,6,7,8,11,12,13,14,15,16,17,18};
#endif
#ifdef HUMANOID_TYPEC
  const uint8 AX12Servos[MAX_AX12_SERVOS] = {0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0};
  const uint8 AX12_IDS[NUM_AX12_SERVOS] = {1,2,3,4,5,6,9,10,11,12,13,14,15,16,17,18};
#endif

// store the buzzer melodies in Flash
/*const char melody1[] PROGMEM = "!L16 cdefgab>cbagfedc";
const char melody2[] PROGMEM = "T240 L8 a gafaeada c+adaeafa";
const char melody3[] PROGMEM = "O6 T40 L16 d#<b<f#<d#<f#<bd#f#";
const char melody4[] PROGMEM = "! O6 L16 dcd<b-d<ad<g d<f+d<gd<ad<b-";
const char melody5[] PROGMEM = "! O3 T40 f.b.f.b.f.b.f.b.";
*/

// Define global variables for use in the ISRs

// Buzzer related global variables
volatile unsigned char buzzerFinished = 0;	// flag: 0 while playing
const char *buzzerSequence;

// ADC related global variables
// By default ports are assigned as follows:
//		GyroX = CM-510 Port3 = ADC3 = PORTF3
//		GyroY = CM-510 Port4 = ADC4 = PORTF4
//		DMS   = CM-510 Port5 = ADC5 = PORTF5
//		AccelY= CM-510 Port1 = ADC1 = PORTF0
//		AccelX= CM-510 Port2 = ADC2 = PORTF1
//		Ultra = CM-510 Port6 = ADC6 = PORTF5
#ifdef ACCEL_AND_ULTRASONIC
volatile uint8 adc_sensor_enable[ADC_CHANNELS] = {1, 1, 1, 1, 1, 1};
#endif
#ifdef GYRO_AND_DMS_ONLY
volatile uint8 adc_sensor_enable[ADC_CHANNELS] = {0, 0, 1, 1, 1, 0};
#endif
volatile int16 adc_sensor_val[ADC_CHANNELS] = {0, 0, 0, 0, 0, 0}; 	// array of sensor values
volatile uint16 adc_battery_val = 0;		// battery voltage in millivolts
volatile uint16 adc_gyrox_center = 0;		// gyro x center value
volatile uint16 adc_gyroy_center = 0;		// gyro y center value
volatile int16 adc_accelx = 0;				// accelerometer x value
volatile int16 adc_accely = 0;				// accelerometer y value
volatile uint16 adc_accelx_center = 0;		// accelerometer x center value
volatile uint16 adc_accely_center = 0;		// accelerometer y center value
volatile uint16 adc_ultrasonic_distance = 0;	// ultrasonic distance sensor value
volatile uint16 adc_dms_distance = 0;		// DMS sensor distance value

// Global variables related to the finite state machine that governs execution
volatile uint8 bioloid_command = 0;			// current command
volatile uint8 last_bioloid_command = 0;	// last command
volatile bool  new_command = FALSE;			// flag that we got a new command
volatile uint8 flag_receive_ready = 0;		// received complete command flag
volatile bool  major_alarm = FALSE;			// Major alarms that should stop execution

// keep the current pose and joint offsets as global variables
volatile int16 current_pose[NUM_AX12_SERVOS];
// arrays that indicate which servos move in each step/motion page
volatile uint8 motion_step_servos_moving[MAX_MOTION_STEPS][NUM_AX12_SERVOS];
volatile uint8 motion_servos_moving[NUM_AX12_SERVOS];

// and also the current and next motion pages
volatile uint8 current_motion_page = 0;
volatile uint8 next_motion_page = 0;		// next motion page if we got new command
volatile uint8 current_step = 0;			// number of the current motion page step

// Input, Output and Setpoint variables for the PID controller (x and y-axis)
volatile long pid_input[PID_DIMENSION] = { 0, 0 };
volatile long pid_output[PID_DIMENSION] = { 0, 0 };
volatile long pid_setpoint[PID_DIMENSION] = { 0, 0 };
volatile long pid_unscaled_output[PID_DIMENSION] = { 0, 0 };
