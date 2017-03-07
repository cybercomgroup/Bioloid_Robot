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
#include "motion_f.h"
#include "walk.h"
#include "balance.h"
#include "pid.h"

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

/* --- */

/* Global variables */

// Button related variables
volatile command current_command = CMD_WALK_AND_GRAB;

extern volatile bool new_command;             // current command
extern volatile uint8 bioloid_command;                // current command
extern volatile uint8 last_bioloid_command;   // last command

uint32 last_interpret_input_millis = -1; // dummy value to check at first iteration.


void _serial_putc(void*, char); // put a char in serial console

int lean_left_right(s16 amount);
#define lean_left( amount) lean_left_right( amount)
#define lean_right( amount) lean_left_right( -amount)

bool demo_walk_and_grab = 0;
bool do_balance = 1;

int16 last_pose[NUM_AX12_SERVOS];

//TODO: Most of these functions probably belong in another class.

// Set the new current motion if the robot is not currently executing a motion
// Return: 1 if a new motion was set, 0 if a motion already was active.
int startMotionIfIdle(int motionPageId) {
//	printf("startMotionIfIdle %d ?" , motionPageId);
	if (checkMotionFinished()) {
//		printf("idle!\n");
		setNewMotionCommand(motionPageId);
		return 1;
	}
//	printf("not idle!\n");
	return 0;
}

/*
 * Reads input from the remote. Returns 1 if successful, otherwise 0.
 */
int controller_read_input(void) {

	/*
	 * With the new controller system, this function can be replaced with the following:
	 * if (rc100_read_state(RC100_BTN_1) == STATE_PRESSED) {
	 * 		Do stuff!
	 * }
	 *
	 * HOWEVER!! It is crucial that rc100_update() is run before any button states are read, otherwise you will get old or useless values.
	 * It is also important that rc100_check() is not run outside of the rc100 file since this might 'steal' data from rc100_update(). */

	while(rc100_update()) {

		if (rc100_get_btn_change_state(RC100_BTN_U) == STATE_PRESSED) {
			// Start walking forward
			printf("Walking forward.\n");
			startMotionIfIdle(32);
			bioloid_command = COMMAND_WALK_FORWARD;
		}
		else if (rc100_get_btn_change_state(RC100_BTN_U) == STATE_RELEASED) {
			// Stop walking forward
			printf("Walking forward stop. btn state=%d\n", rc100_get_btn_state(RC100_BTN_U));
			bioloid_command = COMMAND_STOP;
			//new_command = 1;
		}

		else if (rc100_get_btn_change_state(RC100_BTN_D) == STATE_PRESSED) {
			// Start walking backward
			printf("Walking backward.\n");
			startMotionIfIdle(45);
			bioloid_command = COMMAND_WALK_BACKWARD;
		}
		else if (rc100_get_btn_change_state(RC100_BTN_D) == STATE_RELEASED) {
			// Stop walking backward
			printf("Walking backward stop.\n");
			bioloid_command = COMMAND_STOP;
			//new_command = 1;
		}

		if (rc100_get_btn_change_state(RC100_BTN_L) == STATE_PRESSED) {
			// toggle balance
			printf("Toggle balance: %d\n", !do_balance);
			do_balance = ! do_balance;
		}

		if (rc100_get_btn_change_state(RC100_BTN_R) == STATE_PRESSED) {
			printf("Right pressed.\n");
		}
		else if (rc100_get_btn_change_state(RC100_BTN_R) == STATE_RELEASED) {
			printf("Right released.\n");
		}


		if (rc100_get_btn_change_state(RC100_BTN_1) == STATE_PRESSED) {
			printf("Standing up.\n");
			startMotionIfIdle(MOTION_STAND);
		}
		else if (rc100_get_btn_change_state(RC100_BTN_2) == STATE_PRESSED) {
			printf("resetting gyro.\n");
			//startMotionIfIdle(MOTION_GRAB);
			gyro_calibrate();
			reset_pitch_roll();
		}
		else if (rc100_get_btn_change_state(RC100_BTN_3) == STATE_PRESSED) {
//			printf("Sitting down.\n");
//			startMotionIfIdle(MOTION_SIT);
			printf("Mode 1: walk and grab, no balance\n");
			do_balance = 0;
			demo_walk_and_grab = 1;
		}
		else if (rc100_get_btn_change_state(RC100_BTN_4) == STATE_PRESSED) {
//			printf("Rapping chest.\n");
//			startMotionIfIdle(MOTION_RAP_CHEST);
			printf("Mode 2: no grab, yes balance\n");
			do_balance = 1;
			demo_walk_and_grab = 0;
		}

		if (rc100_get_btn_change_state(RC100_BTN_5) == STATE_PRESSED) {
			printf("Leaning right.\n");
			//lean_left(10);
			lean_left_right(10);
		}
		else if (rc100_get_btn_change_state(RC100_BTN_6) == STATE_PRESSED) {
			printf("Leaning left.\n");
			lean_right(10);
		}
	//	if (rc100_get_btn_state(RC100_BTN_5)) {
	//		lean_right(10);
	//	} else if (rc100_get_btn_state(RC100_BTN_6)) {
	//		lean_left(10);
	//	} else {
	//		lean_left(0);
	//	}

	}
	return 0;
}

//void update_walk(void) {
//	/* Here we need to do the following:
//	 *
//	 * 1: Check if the motors are at their goal position.
//	 * 2: If not, do nothing. Else, find the next motion page
//	 *	  in our current walk sequence (forward or backward).
//	 */
//}
//
//void evaluate_current_command(void) {
//	switch(current_command) {
//	case CMD_STOP:
//		break;
//	case CMD_WALK_FORWARD:
//		update_walk();
//		break;
//	case CMD_WALK_BACKWARD:
//		update_walk();
//		break;
//	case CMD_WALK_AND_GRAB:
//		break;
//	case CMD_GRAB:
//		break;
//	case CMD_TURN_LEFT:
//		break;
//	case CMD_TURN_RIGHT:
//		break;
//	case CMD_WAVE:
//		break;
//	}
//}

//void issue_command(command cmd) {
//	/* Set motion page etc.
//	 * We also most likely want to return to the
//	 * default pose before starting a new one! */
//	switch(cmd) {
//	case CMD_STOP:
//		break;
//	case CMD_WALK_FORWARD:
//		update_walk();
//		break;
//	case CMD_WALK_BACKWARD:
//		update_walk();
//		break;
//	case CMD_WALK_AND_GRAB:
//		break;
//	case CMD_GRAB:
//		startMotionIfIdle(MOTION_GRAB);
//		break;
//	case CMD_TURN_LEFT:
//		break;
//	case CMD_TURN_RIGHT:
//		break;
//	case CMD_WAVE:
//		break;
//	}
//	current_command = cmd;
//}

void mainLoop();
void update_servo_positions();

/* Test functions. */
void testAbsFn();
void run_tests();
void dxl_test1();
void dxl_test2();
void testTimeFns();
void balance_left_right();
void test_load_motions();

int main(void)
{
	int res, exit = 0, i;

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

	gyro_init(); // power on the gyro as soon as possible, as it takes some time for it to stabilize drift, etc.

	pid_init();
	int Kp = 1000;
	int Ki = 0;
	int Kd = 0;
	pid_setTunings(Kp, Ki, Kd);
	pid_setMode(AUTOMATIC);

	/* Enable ZigBee receiver for rc100 controller */
	USART_Configuration(USART_ZIGBEE, 57600);

	printf("Init ADC...\n");
	ADC_Configuration();

	printf("Loading motion pages...\n");
	motionPageInit();

	/* high level init fn, pings the DXLs to check their status.
	   Here with max 3 retires on failure. */
	printf("Init dxl...\n");
	res = dxl_init1( 1, 3 );

	if (res != 0){
		printf("DXL init failed, aborting.\n");
		return 0;
	}

	// stand up on start to work around first jerky motion by rc100 (unknown why??)
	//startMotionIfIdle(MOTION_STAND);
	executeMotion(MOTION_STAND);

	/* Initialize controller */
	printf("Init rc100...\n");
	rc100_init();

	printf("Calibrating gyro...\n");
	//mDelay(80 * 1000); // sleep a long long time.
	gyro_calibrate();
	printf("Calbirating gyro done!\n");


	//printf("Press start!!.\r\n");

	//start_button_pressed = 1;
	//while (!start_button_pressed) {
		//TODO: Add some fancy blinking lights or other stuff

		/* Wait here until start command given.
		 * Presumingly, the value is changed in an interrupt handler. */
	//}

	//startMotionIfIdle(32);
	//bioloid_command = COMMAND_WALK_FORWARD;

	//dxl_write_word(3, DXL_GOAL_POSITION_L, 663);
	//dxl_write_word(3, DXL_MOVING_SPEED_L, 26);

	printf("Starting main loop.\n");
	mainLoop();
	//test_load_motions();

	printf("\nProgram finished. Have a nice day!\n");
	return 0;
}

void check_feet() {
	word ir_left, ir_right;
	/* Read data from sensors */
	ir_left = read_ir_left();
	ir_right = read_ir_right();
	//printf("ir left: %d, right: %d\n", ir_left, ir_right);
	//delay_ms(500); // delay so that we dont go as fast as possible.
	/* Note that higher IR readings = closer! */
	if (ir_left > MAX_OBSTACLE_DISTANCE || ir_right > MAX_OBSTACLE_DISTANCE) {
		/* We might want to add separate handling depending on triggering foot */
//		if (current_command == CMD_WALK_AND_GRAB) {
			/* Try to blindly pick up whatever is in front of you. */
//			issue_command(CMD_GRAB);
		if (walk_getWalkState() != 0 || checkMotionFinished()) {
			printf("Object detected! Lifting and grabbing!\n");
			setNewMotionCommand(MOTION_GRAB);
		}

//		} else {
			/* Turn to avoid obstacle */
//			issue_command(CMD_TURN_LEFT);
//		}
	}
}

void mainLoop() {
	int ir_left, ir_right;
	set_pose_mode(POSE_MODE_SYNC);
	int iteration = 0;
	int frame_time = 0;
	u32 start_frame_micros = micros();
	while(1) {
		iteration++;

		frame_time = micros() - start_frame_micros;
		start_frame_micros = start_frame_micros + frame_time;

		if (iteration % 1000 == 0) printf("last frame_time is: %d\n", frame_time);

		update_servo_positions();

		/* Interpret command from controller.
		 * This function automatically sets the next command if applicable. */
		controller_read_input();


		if (demo_walk_and_grab) {
			check_feet();
		}

		/* Read gyro sensors? */
		gyro_update();
		gyro_calibrate_incremental();
		if (iteration %500 == 0) {
			printf("Gyro values: x=%d, y=%d, center x=%d, center y=%d, pitch=%d, roll=%d\n",
					gyro_get_x(), gyro_get_y(), gyro_get_center_x(), gyro_get_center_y(), gyro_get_pitch(), gyro_get_roll());
//
//			printf("delta in servo 13: %d, balance value is: %d\n", current_pose[12] - last_pose[12], (uint16)gyro_get_x() - (uint16)gyro_get_center_x());
//			printf("delta in servo 14: %d, balance value is: %d\n", current_pose[13] - last_pose[13], (uint16)gyro_get_x() - (uint16)gyro_get_center_x());
//			printf("delta in servo 15: %d, balance value is: %d\n", current_pose[14] - last_pose[14], (uint16)gyro_get_x() - (uint16)gyro_get_center_x());
//			printf("delta in servo 16: %d, balance value is: %d\n", current_pose[15] - last_pose[15], (uint16)gyro_get_x() - (uint16)gyro_get_center_x());
		}

		if (do_balance) {
			if (walk_getWalkState() != 0 || checkMotionFinished())
				balance();
		}

//		evaluate_current_command();

		executeMotionSequence(); // update the current motion state (use startMotionIfIdle to start a new motion)

		apply_new_pose_and_offsets();
	}
}

// Update current servo positions.
// To save time:
// 		Check only servos that are expected to have moved.


void update_servo_positions() {
	for(int i=0; i<NUM_AX12_SERVOS; i++) {
		last_pose[i] = current_pose[i];
		current_pose[i] = dxl_read_word( i+1, DXL_PRESENT_POSITION_L );
	}
}

void balance_left_right() {
	// First get into stand up pose, then use the 9 and 10 motors to balance left and right.
	unpackMotion(1);
	setMotionPageJointFlexibility();
	u32 startTimeMs = executeMotionStep(1);
	if (startTimeMs != 0) { // == 0 if failed to start executing pose.
		printf("waitForPoseFinish... ");
		waitForPoseFinish();
		printf("pose finished. \n");

		u8 ids[] = {
				9, 10, // hips, left and right
				17, 18 // ankles, left and right
		};
		u8 joint_flex[] = {5,5,5,5};
		//dxl_set_joint_flexibility(1, ids, joint_flex, joint_flex);

		for (int i = 0; 1; i++) {
			int16 lean = (i % 2) ? 30 : -30;
			printf("Leaning %s (%d). \n", (i % 2) ? "left" : "right", lean);
			lean_left_right( lean);
			waitForPoseFinish();
		}
	}
}

// Read current pose and command the servos to go to the current pose + offset to hips and ankles
// non blocking.
// return: 0 on successful execution
int lean_left_right(s16 amount) {
	//u16 current_pose[NUM_AX12_SERVOS];
	// read current pose
	//for (int i = 0; i < NUM_AX12_SERVOS; i++) {
	//	current_pose[i] = dxl_read_word(AX12_IDS[i], DXL_PRESENT_POSITION_L);
	//}

	// apply offsets
	//resetJointOffsets();
//	setJointOffsetById(9, amount);
//	setJointOffsetById(10, amount);
//	setJointOffsetById(17, amount);
//	setJointOffsetById(18, amount);
//	return moveToGoalPose(time, getCurrentGoalPose(), 0);
	setJointOffsetById(9, amount);
	setJointOffsetById(10, amount);
	setJointOffsetById(17, amount);
	setJointOffsetById(18, amount);
	return 0;
}


void _test_load_motions(int mp) {
	u32 t0 = micros();
	printf(" Load motion page %d (old method)...\n", mp);
	unpackMotion(mp);
	printCurrentMotionPage();
	u32 t1 = micros() - t0;

	printf("Old method took %u us\n", t1);

	t0 = micros();
	printf(" Load motion page %d (new method)...\n", mp);
	unpackMotion2(mp);
	printCurrentMotionPage();
	t1 = micros() - t0;

	printf("New method took %u us\n", t1);
}

// use this to check that the new motion unpacker works the same as the old one.
// need to set use_old_motions_code in motion_f.h to 1 for comparision to be useful.
void test_load_motions() {
	printf("--- test_load_motions --- \n");

#if !use_old_motions_code
	printf("NOT USING OLD MOTIONS CODE!!!! please set test_load_motions to 1 for this function to work...\n");
#else
	_test_load_motions(32);
	_test_load_motions(33);

	_test_load_motions(34);
	_test_load_motions(35);

	_test_load_motions(36);
	_test_load_motions(37);

	_test_load_motions(38);
	_test_load_motions(39);
#endif
	printf("--- test_load_motions done --- \n");
}

void run_tests() {

	testTimeFns();
	testAbsFn();
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

/* Put a character to the serial terminal.
 * Used in the custom printf function. */
void _serial_putc ( void* p, char c)
{
	if (c == '\n') // prepend a \r for each \n printed. Because our console likes that.
		TxDByte_PC('\r');
	TxDByte_PC(c);
}


