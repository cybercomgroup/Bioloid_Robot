#include "../inc/time.h"
#include "hw_setup.h"
#include "hw_functions.h"
#include "dynamixel.h"
#include "printf.h"
#include "pose.h"
#include "rc100.h"
#include "system_calls.h"
#include "mem_attrs.h"
#include "sensors.h"
#include "string.h"
#include "motion_f.h"

//TODO: Move this stuff to a suitable header file:
/* --- */
#define MAX_OBSTACLE_DISTANCE 200

typedef enum {
	CMD_STOP 			= 0,
	CMD_WALK_FORWARD 	= 1,
	CMD_WALK_BACKWARD 	= 2,
	CMD_WALK_AND_GRAB 	= 3,
	CMD_GRAB 			= 4,
	CMD_TURN_LEFT 		= 5,
	CMD_TURN_RIGHT 		= 6,
	CMD_WAVE 			= 7
} command;

typedef enum {
	START_DOWN = 1,
	LEFT_DOWN = 1 << 1,
	RIGHT_DOWN = 1 << 2,


} button_state;
/* --- */


/* Global variables */

// Button related variables
volatile bool button_up_pressed = FALSE;
volatile bool button_down_pressed = FALSE;
volatile bool button_left_pressed = FALSE;
volatile bool button_right_pressed = FALSE;
volatile bool start_button_pressed = FALSE;
volatile command current_command = CMD_STOP;


void _serial_putc(void*, char); // put a char in serial console

uint16 pose[NUM_AX12_SERVOS] = {235,788,279,744,462,561,358,666,507,516,341,682,240,783,647,376,507,516};
uint16 speeds[NUM_AX12_SERVOS];

//TODO: Most of these functions probably belong in another class.

/* Returns -1 if there is no new input, otherwise returns the data sent
 * from the controller.  */
void interpret_input(int input) {
	command cmd;
	if (input & RC100_BTN_U) {
		cmd = CMD_WALK_FORWARD;
		printf("Up!");
		//Do some stuff to make sure that the robot finishes its current motion!
	} else if (input & RC100_BTN_D) {
		cmd = CMD_WALK_BACKWARD;
		printf("Down!");
	} else if (input & RC100_BTN_L) {
		cmd = CMD_TURN_LEFT;
		printf("Left!");
	} else if (input & RC100_BTN_R) {
		cmd = CMD_TURN_RIGHT;
		printf("Right!");
	} else if (input & RC100_BTN_1) {

	} else if (input & RC100_BTN_2) {

	} else if (input & RC100_BTN_3) {

	} else if (input & RC100_BTN_4) {

	} else if (input & RC100_BTN_5) {

	} else if (input & RC100_BTN_6) {

	}
}

/*
 * Reads input from the remote. Returns 1 if successful, otherwise 0.
 */
int controller_read_input(void) {

	if (rc100_check()) {
		interpret_input(rc100_read_data);

		return 1;
	}
	return 0;
}

void update_walk(void) {
	/* Here we need to do the following:
	 *
	 * 1: Check if the motors are at their goal position.
	 * 2: If not, do nothing. Else, find the next motion page
	 *	  in our current walk sequence (forward or backward).
	 */
}

void evaluate_current_command(void) {
	switch(current_command) {
	case CMD_STOP:
		break;
	case CMD_WALK_FORWARD:
		update_walk();
		break;
	case CMD_WALK_BACKWARD:
		update_walk();
		break;
	case CMD_WALK_AND_GRAB:
		break;
	case CMD_GRAB:
		break;
	case CMD_TURN_LEFT:
		break;
	case CMD_TURN_RIGHT:
		break;
	case CMD_WAVE:
		break;
	}
}

void issue_command(command cmd) {
	/* Set motion page etc.
	 * We also most likely want to return to the
	 * default pose before starting a new one! */
}

/* Test functions. */
void testAbsFn();
void run_tests();
void dxl_test1();
void dxl_test2();
void testTimeFns();
void testIR();

int main(void)
{
	int res, exit = 0, ir_left, ir_right, i;

	/* Initialization */

	/* Setup minimal printf to send to serial console. */
	init_printf(0, _serial_putc);

	/* System Clocks Configuration */
	RCC_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* GPIO configuration */
	GPIO_Configuration();

	SysTick_Configuration();

	Timer_Configuration();


	/* Enable ZigBee receiver for rc100 controller */
	USART_Configuration(USART_ZIGBEE, 57600);

	printf("Init ADC... ");

	ADC_Configuration();

	printf("Loading motion pages...");
	motionPageInit();
	printf(" Done.\n");

	/* high level init fn, pings the DXLs to check their status.
	   Here with max 3 retires on failure. */
	res = dxl_init1( 1, 3 );

	if (res != 0){
		printf("DXL init failed, aborting.\n");
		return 0;
	}

	/* Initialize controller */
	rc100_init();

	printf("Press start!!.\r\n");

	start_button_pressed = 1;
	while (!start_button_pressed) {
		//TODO: Add some fancy blinking lights or other stuff

		/* Wait here until start command given.
		 * Presumingly, the value is changed in an interrupt handler. */
	}
	/* Enable interrupts */
	//sei(); Where is this function declared?

	printf("Starting main loop.\n");

	//testIR();
	//testTimeFns();
	dxl_test2();

	printf("\nProgram finished. Have a nice day!\n");
	return 0;
}

void run_tests() {

	testTimeFns();
	testAbsFn();
}

void dxl_test1() {
	int8 i;

	/* Example to set the speeds (of all servos) and position (of single servo). */
	// Set goal speed
	dxl_write_word( BROADCAST_ID, DXL_MOVING_SPEED_L, 26); // goal speed must be between 26 and 1023, other values default to max speed
	// Set goal position
	dxl_write_word( 1, DXL_GOAL_POSITION_L, 512 );
	mDelay(1000);

	//printf("Moving to default pose.\n");
	//for (i=0;i< NUM_AX12_SERVOS; i++ )
	//	pose[i] = current_pose[i];
	//pose[0] = 235;

	//moveToGoalPose(1000, pose, 0);
	//printf("done\n");
	//return 0;
//	printf("dbg goal speed: %d %d %d \n", dbgSpeeds[0], dbgSpeeds[1], dbgSpeeds[2]);

	for (i=0;i< NUM_AX12_SERVOS; i++ )
		speeds[i] = 100;

	//printf("goal speed2: %d\n", speeds[0]);
	//printf("speeds  %d %d %d \n", dbgSpeeds[0], dbgSpeeds[1], dbgSpeeds[2]);
	//dxl_set_goal_speed(NUM_AX12_SERVOS, AX12_IDS, pose, speeds);
	dxl_set_goal_speed(1, AX12_IDS, pose, speeds);
	//moveToDefaultPose();
	//printf("Moving to default pose done! commstatus %d\n", comSt);
	mDelay(1000);
}

void dxl_test2() {
	executeMotion(26); // stand up
	mDelay(2000);
	executeMotion(7);
	mDelay(2000);
	executeMotion(25); // sit down again
}

void testTimeFns() {
	printf("Testing time.h functions, delaying 1000 ms...");
	u32 t0 = millis();
	mDelay(1000);
	u32 t = millis();
	printf(" Done. Took %u ms.\n", t-t0);

	printf("Testing time.h functions, delaying 1 000 000 us...");
	t0 = micros();
	uDelay(1000000);
	t = micros();
	printf(" Done. Took %u us.\n", t-t0);

	t0 = micros();
	t = micros();
	printf("1st Call to micros take %u us (%u - %u).\n", t-t0, t, t0);

	t0 = micros();
	t = micros();
	printf("2nd Call to micros take %u us (%u - %u).\n", t-t0, t, t0);

	t0 = micros();
	t = micros();
	printf("3rd Call to micros take %u us (%u - %u).\n", t-t0, t, t0);
}

void testAbsFn() {
	printf("Testing abs on 1 and -1: %d, %d \n", abs(1), abs(-1));
}

/* This is the main program.... should be renamed as such. */
void testIR() {
	int ir_left, ir_right;
	while(1) {

		/* Interpret command from controller.
		 * This function automatically sets the next command if applicable. */
		controller_read_input();

		/* Read data from sensors */
		ir_left = read_ir_left();
		ir_right = read_ir_right();

		printf("ir left: %d, right: %d\n", ir_left, ir_right);

		delay_ms(500); // delay so that we dont go as fast as possible.

		/* Note that higher IR readings = closer! */
		if (ir_left > MAX_OBSTACLE_DISTANCE || ir_right > MAX_OBSTACLE_DISTANCE) {
			/* We might want to add separate handling depending on triggering foot */
			if (current_command == CMD_WALK_AND_GRAB) {
				/* Try to blindly pick up whatever is in front of you. */
				issue_command(CMD_GRAB);
			} else {
				/* Turn to avoid obstacle */
				issue_command(CMD_TURN_LEFT);
			}
		}

		/* Read gyro sensors? */

		evaluate_current_command();

	}
}

/* Put a character to the serial terminal.
 * Used in the custom printf function. */
void _serial_putc ( void* p, char c)
{
	if (c == '\n') // prepend a \r for each \n printed. Because our console likes that.
		TxDByte_PC('\r');
	TxDByte_PC(c);
}


