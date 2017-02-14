/*
 * motion.c - functions for executing motion pages  
 *	requires a motion.h file created by the translate_motion Perl script 
 * 
 * Version 0.7		31/01/2013
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 */

/* Modified by Anton Olsson / Cybercom */

/*
 * You may freely modify and share this code, as long as you keep this
 * notice intact. Licensed under the Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, this work is provided
 * without any warranty. It might be defective, in which case you agree
 * to be responsible for all resulting costs an damages.
 */

#include "system_calls.h"
#include "printf.h"
#include "global.h"
#include "pose.h"
#include "motion.h"
#include "mem_attrs.h"

#include "time.h"
#include "motion_f.h"
#include "dynamixel.h"

// define the possible states for executeMotionSequence
#define MOTION_STOPPED		0
#define STEP_IN_MOTION		1
#define STEP_IN_PAUSE		2
#define STEP_FINISHED		3
#define PAUSE_FINISHED		4
#define	PAGE_FINISHED		5
#define MOTION_ALARM		6
#define ROBOT_SLIPPED		7

// and create the variables that guide these functions
uint8 motion_state = 7;					// motion state as per above definitions
unsigned long pause_start_time = 0;		// millis() at start of pause time
uint8 repeat_counter = 0;				// number of repeats of page already performed
uint8 exit_flag = 0;					// flag indicating we are on an exit page
uint8 last_joint_flex[NUM_AX12_SERVOS];		// last set of joint flex values
// timing variables
unsigned long step_start_time = 0, step_finish_time = 0, block_time = 0;

// Global variables related to the finite state machine that governs execution
extern volatile uint8 bioloid_command;			// current command
extern volatile uint8 last_bioloid_command;		// last command
extern volatile bool  new_command;				// flag that we got a new command

// global hardware definition variables
extern const uint8 AX12_IDS[NUM_AX12_SERVOS];
extern const uint8 AX12Servos[MAX_AX12_SERVOS];
// global variables that keep the current and next motion page
extern volatile uint8 current_motion_page;
extern volatile uint8 next_motion_page;			// next motion page if we got new command
extern uint8 current_step;						// number of the current motion page step

// should keep the current pose in a global array
extern volatile uint8 motion_step_servos_moving[MAX_MOTION_STEPS][NUM_AX12_SERVOS];
extern volatile uint8 motion_servos_moving[NUM_AX12_SERVOS];

// table of pointers to the motion pages
uint8 * motion_pointer[NUM_MOTION_PAGES+1];

// struct for an unpacked motion page
struct MotionPage
{
	uint8 JointFlex[NUM_AX12_SERVOS]; 
	uint8 NextPage; 
	uint8 ExitPage; 
	uint8 RepeatTime; 
	uint8 SpeedRate10; 
	uint8 InertialForce; 
	uint8 Steps; 
	uint16 StepValues[MAX_MOTION_STEPS][NUM_AX12_SERVOS];
	uint16 PauseTime[MAX_MOTION_STEPS]; 
	uint16 PlayTime[MAX_MOTION_STEPS]; 
} CurrentMotion;


// Initialize the motion pages by constructing a table of pointers to each page
// Motion pages are stored in Flash (PROGMEM) - see motion.h
void motionPageInit()
{
	// first we need to check file matches the configuration defined
	for (int i=0; i<MAX_AX12_SERVOS; i++)
	{
		if (AX12_ENABLED[i] != AX12Servos[i])
		{
			// configuration does not match
			printf("\nConfiguration of enabled AX-12 servos does not match motion.h. ABORT!\n");
			exit(-1);
		}
	}
	// We also need to check that the number of motion pages matches
	if (ACTIVE_MOTION_PAGES != NUM_MOTION_PAGES)
	{
			// configuration does not match
			printf("\nNumber of active motion pages does not match motion.h. ABORT!\n");
			exit(-1);
	}
	
	// set the initial motion state
	motion_state = MOTION_STOPPED;
	
	// Motion Page pointer assignment to PROGMEM 
	motion_pointer[0] = 0;
	motion_pointer[1] = (uint8*) &MotionPage1; 
	motion_pointer[2] = (uint8*) &MotionPage2; 
	motion_pointer[3] = (uint8*) &MotionPage3; 
	motion_pointer[4] = (uint8*) &MotionPage4; 
	motion_pointer[5] = (uint8*) &MotionPage5; 
	motion_pointer[6] = (uint8*) &MotionPage6; 
	motion_pointer[7] = (uint8*) &MotionPage7; 
	motion_pointer[8] = (uint8*) &MotionPage8; 
	motion_pointer[9] = (uint8*) &MotionPage9; 
	motion_pointer[10] = (uint8*) &MotionPage10; 
	motion_pointer[11] = (uint8*) &MotionPage11; 
	motion_pointer[12] = (uint8*) &MotionPage12; 
	motion_pointer[13] = (uint8*) &MotionPage13; 
//	motion_pointer[14] = (uint8*) &MotionPage14;
//	motion_pointer[15] = (uint8*) &MotionPage15;
//	motion_pointer[16] = (uint8*) &MotionPage16;
//	motion_pointer[17] = (uint8*) &MotionPage17;
//	motion_pointer[18] = (uint8*) &MotionPage18;
//	motion_pointer[19] = (uint8*) &MotionPage19;
//	motion_pointer[20] = (uint8*) &MotionPage20;
//	motion_pointer[21] = (uint8*) &MotionPage21;
//	motion_pointer[22] = (uint8*) &MotionPage22;
//	motion_pointer[23] = (uint8*) &MotionPage23;
//	motion_pointer[24] = (uint8*) &MotionPage24;
//	motion_pointer[25] = (uint8*) &MotionPage25;
//	motion_pointer[26] = (uint8*) &MotionPage26;
//	motion_pointer[27] = (uint8*) &MotionPage27;
//	motion_pointer[28] = (uint8*) &MotionPage28;
//	motion_pointer[29] = (uint8*) &MotionPage29;
//	motion_pointer[30] = (uint8*) &MotionPage30;
//	motion_pointer[31] = (uint8*) &MotionPage31;
//	motion_pointer[32] = (uint8*) &MotionPage32;
//	motion_pointer[33] = (uint8*) &MotionPage33;
//	motion_pointer[34] = (uint8*) &MotionPage34;
//	motion_pointer[35] = (uint8*) &MotionPage35;
//	motion_pointer[36] = (uint8*) &MotionPage36;
//	motion_pointer[37] = (uint8*) &MotionPage37;
//	motion_pointer[38] = (uint8*) &MotionPage38;
//	motion_pointer[39] = (uint8*) &MotionPage39;
//	motion_pointer[40] = (uint8*) &MotionPage40;
//	motion_pointer[41] = (uint8*) &MotionPage41;
//	motion_pointer[42] = (uint8*) &MotionPage42;
//	motion_pointer[43] = (uint8*) &MotionPage43;
//	motion_pointer[44] = (uint8*) &MotionPage44;
//	motion_pointer[45] = (uint8*) &MotionPage45;
//	motion_pointer[46] = (uint8*) &MotionPage46;
//	motion_pointer[47] = (uint8*) &MotionPage47;
//	motion_pointer[48] = (uint8*) &MotionPage48;
//	motion_pointer[49] = (uint8*) &MotionPage49;
//	motion_pointer[50] = (uint8*) &MotionPage50;
//	motion_pointer[51] = (uint8*) &MotionPage51;
//	motion_pointer[52] = (uint8*) &MotionPage52;
//	motion_pointer[53] = (uint8*) &MotionPage53;
//	motion_pointer[54] = (uint8*) &MotionPage54;
//	motion_pointer[55] = (uint8*) &MotionPage55;
//	motion_pointer[56] = (uint8*) &MotionPage56;
//	motion_pointer[57] = (uint8*) &MotionPage57;
//	motion_pointer[58] = (uint8*) &MotionPage58;
//	motion_pointer[59] = (uint8*) &MotionPage59;
//	motion_pointer[60] = (uint8*) &MotionPage60;
//	motion_pointer[61] = (uint8*) &MotionPage61;
//	motion_pointer[62] = (uint8*) &MotionPage62;
//	motion_pointer[63] = (uint8*) &MotionPage63;
//	motion_pointer[64] = (uint8*) &MotionPage64;
//	motion_pointer[65] = (uint8*) &MotionPage65;
//	motion_pointer[66] = (uint8*) &MotionPage66;
//	motion_pointer[67] = (uint8*) &MotionPage67;
//	motion_pointer[68] = (uint8*) &MotionPage68;
//	motion_pointer[69] = (uint8*) &MotionPage69;
//	motion_pointer[70] = (uint8*) &MotionPage70;
//	motion_pointer[71] = (uint8*) &MotionPage71;
//	motion_pointer[72] = (uint8*) &MotionPage72;
//	motion_pointer[73] = (uint8*) &MotionPage73;
//	motion_pointer[74] = (uint8*) &MotionPage74;
//	motion_pointer[75] = (uint8*) &MotionPage75;
//	motion_pointer[76] = (uint8*) &MotionPage76;
//	motion_pointer[77] = (uint8*) &MotionPage77;
//	motion_pointer[78] = (uint8*) &MotionPage78;
//	motion_pointer[79] = (uint8*) &MotionPage79;
//	motion_pointer[80] = (uint8*) &MotionPage80;
//	motion_pointer[81] = (uint8*) &MotionPage81;
//	motion_pointer[82] = (uint8*) &MotionPage82;
//	motion_pointer[83] = (uint8*) &MotionPage83;
//	motion_pointer[84] = (uint8*) &MotionPage84;
//	motion_pointer[85] = (uint8*) &MotionPage85;
//	motion_pointer[86] = (uint8*) &MotionPage86;
//	motion_pointer[87] = (uint8*) &MotionPage87;
//	motion_pointer[88] = (uint8*) &MotionPage88;
//	motion_pointer[89] = (uint8*) &MotionPage89;
//	motion_pointer[90] = (uint8*) &MotionPage90;
//	motion_pointer[91] = (uint8*) &MotionPage91;
//	motion_pointer[92] = (uint8*) &MotionPage92;
//	motion_pointer[93] = (uint8*) &MotionPage93;
//	motion_pointer[94] = (uint8*) &MotionPage94;
//	motion_pointer[95] = (uint8*) &MotionPage95;
//	motion_pointer[96] = (uint8*) &MotionPage96;
//	motion_pointer[97] = (uint8*) &MotionPage97;
//	motion_pointer[98] = (uint8*) &MotionPage98;
//	motion_pointer[99] = (uint8*) &MotionPage99;
//	motion_pointer[100] = (uint8*) &MotionPage100;
//	motion_pointer[101] = (uint8*) &MotionPage101;
//	motion_pointer[102] = (uint8*) &MotionPage102;
//	motion_pointer[103] = (uint8*) &MotionPage103;
//	motion_pointer[104] = (uint8*) &MotionPage104;
//	motion_pointer[105] = (uint8*) &MotionPage105;
//	motion_pointer[106] = (uint8*) &MotionPage106;
//	motion_pointer[107] = (uint8*) &MotionPage107;
//	motion_pointer[108] = (uint8*) &MotionPage108;
//	motion_pointer[109] = (uint8*) &MotionPage109;
//	motion_pointer[110] = (uint8*) &MotionPage110;
//	motion_pointer[111] = (uint8*) &MotionPage111;
//	motion_pointer[112] = (uint8*) &MotionPage112;
//	motion_pointer[113] = (uint8*) &MotionPage113;
//	motion_pointer[114] = (uint8*) &MotionPage114;
//	motion_pointer[115] = (uint8*) &MotionPage115;
//	motion_pointer[116] = (uint8*) &MotionPage116;
//	motion_pointer[117] = (uint8*) &MotionPage117;
//	motion_pointer[118] = (uint8*) &MotionPage118;
//	motion_pointer[119] = (uint8*) &MotionPage119;
//	motion_pointer[120] = (uint8*) &MotionPage120;
//	motion_pointer[121] = (uint8*) &MotionPage121;
//	motion_pointer[122] = (uint8*) &MotionPage122;
//	motion_pointer[123] = (uint8*) &MotionPage123;
//	motion_pointer[124] = (uint8*) &MotionPage124;
//	motion_pointer[125] = (uint8*) &MotionPage125;
//	motion_pointer[126] = (uint8*) &MotionPage126;
//	motion_pointer[127] = (uint8*) &MotionPage127;
//	motion_pointer[128] = (uint8*) &MotionPage128;
//	motion_pointer[129] = (uint8*) &MotionPage129;
//	motion_pointer[130] = (uint8*) &MotionPage130;
//	motion_pointer[131] = (uint8*) &MotionPage131;
//	motion_pointer[132] = (uint8*) &MotionPage132;
//	motion_pointer[133] = (uint8*) &MotionPage133;
//	motion_pointer[134] = (uint8*) &MotionPage134;
//	motion_pointer[135] = (uint8*) &MotionPage135;
//	motion_pointer[136] = (uint8*) &MotionPage136;
//	motion_pointer[137] = (uint8*) &MotionPage137;
//	motion_pointer[138] = (uint8*) &MotionPage138;
//	motion_pointer[139] = (uint8*) &MotionPage139;
//	motion_pointer[140] = (uint8*) &MotionPage140;
//	motion_pointer[141] = (uint8*) &MotionPage141;
//	motion_pointer[142] = (uint8*) &MotionPage142;
//	motion_pointer[143] = (uint8*) &MotionPage143;
//	motion_pointer[144] = (uint8*) &MotionPage144;
//	motion_pointer[145] = (uint8*) &MotionPage145;
//	motion_pointer[146] = (uint8*) &MotionPage146;
//	motion_pointer[147] = (uint8*) &MotionPage147;
//	motion_pointer[148] = (uint8*) &MotionPage148;
//	motion_pointer[149] = (uint8*) &MotionPage149;
//	motion_pointer[150] = (uint8*) &MotionPage150;
//	motion_pointer[151] = (uint8*) &MotionPage151;
//	motion_pointer[152] = (uint8*) &MotionPage152;
//	motion_pointer[153] = (uint8*) &MotionPage153;
//	motion_pointer[154] = (uint8*) &MotionPage154;
//	motion_pointer[155] = (uint8*) &MotionPage155;
//	motion_pointer[156] = (uint8*) &MotionPage156;
//	motion_pointer[157] = (uint8*) &MotionPage157;
//	motion_pointer[158] = (uint8*) &MotionPage158;
//	motion_pointer[159] = (uint8*) &MotionPage159;
//	motion_pointer[160] = (uint8*) &MotionPage160;
//	motion_pointer[161] = (uint8*) &MotionPage161;
//	motion_pointer[162] = (uint8*) &MotionPage162;
//	motion_pointer[163] = (uint8*) &MotionPage163;
//	motion_pointer[164] = (uint8*) &MotionPage164;
//	motion_pointer[165] = (uint8*) &MotionPage165;
//	motion_pointer[166] = (uint8*) &MotionPage166;
//	motion_pointer[167] = (uint8*) &MotionPage167;
//	motion_pointer[168] = (uint8*) &MotionPage168;
//	motion_pointer[169] = (uint8*) &MotionPage169;
//	motion_pointer[170] = (uint8*) &MotionPage170;
//	motion_pointer[171] = (uint8*) &MotionPage171;
//	motion_pointer[172] = (uint8*) &MotionPage172;
//	motion_pointer[173] = (uint8*) &MotionPage173;
//	motion_pointer[174] = (uint8*) &MotionPage174;
//	motion_pointer[175] = (uint8*) &MotionPage175;
//	motion_pointer[176] = (uint8*) &MotionPage176;
//	motion_pointer[177] = (uint8*) &MotionPage177;
//	motion_pointer[178] = (uint8*) &MotionPage178;
//	motion_pointer[179] = (uint8*) &MotionPage179;
//	motion_pointer[180] = (uint8*) &MotionPage180;
//	motion_pointer[181] = (uint8*) &MotionPage181;
//	motion_pointer[182] = (uint8*) &MotionPage182;
//	motion_pointer[183] = (uint8*) &MotionPage183;
//	motion_pointer[184] = (uint8*) &MotionPage184;
//	motion_pointer[185] = (uint8*) &MotionPage185;
//	motion_pointer[186] = (uint8*) &MotionPage186;
//	motion_pointer[187] = (uint8*) &MotionPage187;
//	motion_pointer[188] = (uint8*) &MotionPage188;
//	motion_pointer[189] = (uint8*) &MotionPage189;
//	motion_pointer[190] = (uint8*) &MotionPage190;
//	motion_pointer[191] = (uint8*) &MotionPage191;
//	motion_pointer[192] = (uint8*) &MotionPage192;
//	motion_pointer[193] = (uint8*) &MotionPage193;
//	motion_pointer[194] = (uint8*) &MotionPage194;
//	motion_pointer[195] = (uint8*) &MotionPage195;
//	motion_pointer[196] = (uint8*) &MotionPage196;
//	motion_pointer[197] = (uint8*) &MotionPage197;
//	motion_pointer[198] = (uint8*) &MotionPage198;
//	motion_pointer[199] = (uint8*) &MotionPage199;
//	motion_pointer[200] = (uint8*) &MotionPage200;
//	motion_pointer[201] = (uint8*) &MotionPage201;
//	motion_pointer[202] = (uint8*) &MotionPage202;
//	motion_pointer[203] = (uint8*) &MotionPage203;
//	motion_pointer[204] = (uint8*) &MotionPage204;
//	motion_pointer[205] = (uint8*) &MotionPage205;
//	motion_pointer[206] = (uint8*) &MotionPage206;
//	motion_pointer[207] = (uint8*) &MotionPage207;
//	motion_pointer[208] = (uint8*) &MotionPage208;
//	motion_pointer[209] = (uint8*) &MotionPage209;
//	motion_pointer[210] = (uint8*) &MotionPage210;
//	motion_pointer[211] = (uint8*) &MotionPage211;
//	motion_pointer[212] = (uint8*) &MotionPage212;
//	motion_pointer[213] = (uint8*) &MotionPage213;
//	motion_pointer[214] = (uint8*) &MotionPage214;
//	motion_pointer[215] = (uint8*) &MotionPage215;
//	motion_pointer[216] = (uint8*) &MotionPage216;
//	motion_pointer[217] = (uint8*) &MotionPage217;
//	motion_pointer[218] = (uint8*) &MotionPage218;
//	motion_pointer[219] = (uint8*) &MotionPage219;
//	motion_pointer[220] = (uint8*) &MotionPage220;
//	motion_pointer[221] = (uint8*) &MotionPage221;
//	motion_pointer[222] = (uint8*) &MotionPage222;
//	motion_pointer[223] = (uint8*) &MotionPage223;
//	motion_pointer[224] = (uint8*) &MotionPage224;
//	motion_pointer[225] = (uint8*) &MotionPage225;
//	motion_pointer[226] = (uint8*) &MotionPage226;
//	motion_pointer[227] = (uint8*) &MotionPage227;
}

// This function executes robot motions consisting of one or more motion 
// pages defined in motion.h
// It implements a finite state machine to know what it is doing and what to do next
// Code is meant to be reentrant so it can easily be converted to a task with a RTOS
// Returns:		motion_state
uint8 executeMotionSequence()
{
	uint8 moving_flag, temp1;
	int error_status, comm_status, left_right_step;
	
	// TEST: if ( motion_state != MOTION_STOPPED ) printf("\nMotion State = %i, Walk State = %i, Current Step = %i", motion_state, walk_getWalkState(), current_step);
	
	// check the states in order of likelihood of occurrence
	// the most likely state is that a motion step is still being executed or paused
	if ( motion_state == STEP_IN_MOTION )
	{
		// if walking we can't wait for motion to finish, go by step time instead
		if( walk_getWalkState() != 0 ) {
			if ( (millis()-step_start_time) >= CurrentMotion.PlayTime[current_step-1] ) {
				// step time is up, update state
				motion_state = STEP_FINISHED;
			} else {
				// play time isn't finished yet, return
				return motion_state;
			}
		} else {				
			// last state was step in motion - check if finished
			moving_flag = checkMotionStepFinished();
			// finished, update motion state
			if ( moving_flag == 0 ) {
				motion_state = STEP_FINISHED;
				step_finish_time = millis();
			} else {
				// step isn't finished yet, return
				return motion_state;
			}
		}		
	} else if( motion_state == STEP_IN_PAUSE ) {
		// check if we still need to wait for pause time to expire
		if ( (millis()-pause_start_time) >= CurrentMotion.PauseTime[current_step-1] )
		{
			// pause is finished, update state
			motion_state = PAUSE_FINISHED;
		} else {
			// pause isn't finished yet, return
			return motion_state;
		}		
	}
	
	// Next we check for any movement related alarms - at this point the only way the
	// motion state can be STEP_FINISHED is because it was changed above
	// Given this takes 11ms, that's too long for walking (may have to revisit)
	if ( motion_state == STEP_FINISHED && walk_getWalkState() == 0 )
	{
		// check that executing the last step didn't cause any alarms (takes 5ms)
		for (uint8 i=0; i<NUM_AX12_SERVOS; i++) {
			// ping the servo and unpack error code (if any)
			error_status = dxl_ping(AX12_IDS[i]);
			if(error_status != 0) {
				// there has been an error, disable torque
				comm_status = dxl_write_byte(BROADCAST_ID, DXL_TORQUE_ENABLE, 0);
				printf("\nexecuteMotionSequence Alarm ID%i - Error Code %i\n", AX12_IDS[i], error_status);
				motion_state = MOTION_ALARM;
				return motion_state;
			}
		}	
		// no alarm has occurred, read back current pose (takes 6ms)
		readCurrentPose(READ_ALL, 0);	
	}
	
	// We also need to check if we received a RESET command after alarm shutdown
	if ( motion_state == MOTION_ALARM && bioloid_command == COMMAND_RESET )
	{
		// Reset the Dynamixel actuators - reset torque limit and re-enable torque
		comm_status = dxl_write_word(BROADCAST_ID, DXL_TORQUE_LIMIT_L, 0x3FF);
		comm_status = dxl_write_byte(BROADCAST_ID, DXL_TORQUE_ENABLE, 1);
		motion_state = MOTION_STOPPED;
	}
	
	// Now we can figure out what to do next
	// Options are:	1. Switch to ExitPage
	//				2. Respond to a change in walk command (see WALK EXECUTE task in RoboPlus Task sample files)
	//				3. Start Pause Time after step
	//				4. Execute next step in current motion page
	//				5. Go back to first step in current motion page (repeat)
	//				6. Switch to NextPage
	//				7. Respond to a new non-walk command 
	//				8. Nothing to do - wait for new command
	
	// first we deal with state changes at the end of a motion page
	if ( current_step == CurrentMotion.Steps && (motion_state == STEP_FINISHED || motion_state == PAUSE_FINISHED) )
	{
		// check if we just finished an exit page
		if ( exit_flag == 1 )
		{
			// yes, reset flag and change motion state and then return to not complicate things
			exit_flag = 0;
			motion_state = MOTION_STOPPED;
			return motion_state;
		}
		
		// we have finished the current page - determine the next motion page
		if ( bioloid_command == COMMAND_STOP )
		{
		// Option 1 - switch to exit page
			if ( CurrentMotion.ExitPage == 0 ) {
				// no exit page, stop
				motion_state = MOTION_STOPPED;
				return motion_state;
			} else {
				// need to execute an Exit Page before stopping		
				current_motion_page = CurrentMotion.ExitPage;
				exit_flag = 1;		// flag that we need to stop after the exit page
			}
		} 
		// Option 2 - respond to change in walk command (seamless transitions only)
		else if ( new_command == TRUE ) 
		{
			if ( walk_shift() == 1 ) {
				// walkShift already updates the current motion page
				new_command = FALSE;
			} else {
				// to transition to new command we first need to execute the exit page
				if ( CurrentMotion.ExitPage == 0 ) {
					// no exit page
					current_motion_page = 0;
					motion_state = MOTION_STOPPED;
					return motion_state;
				} else {
					// need to execute an Exit Page before new command		
					current_motion_page = CurrentMotion.ExitPage;
					exit_flag = 1;		// flag that we need to stop after the exit page
				}				
			}	
		} 
		// Option 5 - repeat the current motion page
		else if ( CurrentMotion.RepeatTime > repeat_counter )
		{
			// Update step, repeat counter and motion status
			current_step = 1;
			repeat_counter++;
			motion_state = STEP_IN_MOTION;
			// can go straight to executing step 1 since we have executed this page before
			step_start_time = executeMotionStep(current_step);
			return motion_state;
		} 
		// Option 6 - switch to NextPage motion page
		else if ( CurrentMotion.NextPage > 0 && CurrentMotion.NextPage <= NUM_MOTION_PAGES )
		{
			current_motion_page = CurrentMotion.NextPage;
		}
		// Nothing else to do - stop motion
		else
		{
			motion_state = MOTION_STOPPED;
			return motion_state;
		}

		// in Options 1,2,6 above we had a change of motion page - start execution
		unpackMotion(current_motion_page);
		if ( setMotionPageJointFlexibility() == 0 ) {
			// joint flex values set ok, execute motion
			current_step = 1;
			repeat_counter = 1;
			motion_state = STEP_IN_MOTION;
			step_start_time = executeMotionStep(current_step);
		} else {
			// this shouldn't really happen, but we need to cater to the eventuality
			comm_status = dxl_write_byte(BROADCAST_ID, DXL_TORQUE_ENABLE, 0);
			motion_state = MOTION_ALARM;
		}
		
		// either way we are finished here - return
		return motion_state;
	}	
	
	// now we can deal with state changes during page execution
	if ( motion_state == STEP_FINISHED )
	{
		// Option 3 - start pause after step
		if ( CurrentMotion.PauseTime[current_step-1] > 0 && bioloid_command != COMMAND_STOP )
		{
			// set the timer for the pause
			pause_start_time = millis();
			motion_state = STEP_IN_PAUSE;
			return motion_state;
		} else {
			// no pause required, go straight to executing next step
			motion_state = PAUSE_FINISHED;
		}
	}	
	
	if ( motion_state == PAUSE_FINISHED )
	{
		// Option 4 - execute next step in this motion page
		if ( current_step < CurrentMotion.Steps )
		{
			// Update step and motion status
			current_step++;
			motion_state = STEP_IN_MOTION;
			step_start_time = executeMotionStep(current_step);
		}
		// should never end up here
		else 
		{
			// reset to default
			motion_state = MOTION_STOPPED;
			current_motion_page = 0;
		}
		return motion_state;
	}
	
	// Option 7 - Respond to new command - set associated motion page
	if ( motion_state == MOTION_STOPPED && new_command == TRUE )
	{
		// special case for walk commands we need to get walk ready if we weren't walking before
		if( (last_bioloid_command == COMMAND_STOP || last_bioloid_command > COMMAND_WALK_READY) &&
		    ( bioloid_command >= COMMAND_WALK_FORWARD && bioloid_command < COMMAND_WALK_READY ) ) {
				// this is the only time we wait for a motion to finish before returning to the command loop!
				walk_init();
		} 
		// special case of shifting between walk commands - non-seamless transitions
		else if ( walk_getWalkState() > 0 && (bioloid_command >= COMMAND_WALK_FORWARD && bioloid_command < COMMAND_WALK_READY) )
		{
				// calculate the page number relative to start of previous command
				left_right_step = current_motion_page - COMMAND_WALK_READY_MP;
				temp1 = left_right_step / 12U;
				left_right_step -= temp1 * 12;
				// check if we finished on left or right step
				if ( left_right_step == 10 ) {
					// right step is next
					left_right_step = 2;
				} else {
					// left step is next
					left_right_step = 0;
				}
				// can calculate next motion page as in WALK EXECUTE 
				next_motion_page = (bioloid_command-1)*12 + COMMAND_WALK_READY_MP + left_right_step + 1;
		}
		
		if ( bioloid_command != COMMAND_STOP )
		{
			// unpack the new motion page and start the motion
			unpackMotion(next_motion_page);
			current_motion_page = next_motion_page;
			next_motion_page = 0;
			// also need to set walk state if it's a walk command
			if ( bioloid_command >= COMMAND_WALK_FORWARD && bioloid_command < COMMAND_WALK_READY ) {
				walk_setWalkState(bioloid_command);
			} else {
				// not a walk command, reset walk state
				walk_setWalkState(0);
			}
			
			if ( setMotionPageJointFlexibility() == 0 ) {
				// joint flex values set ok, execute motion
				current_step = 1;
				repeat_counter = 1;
				motion_state = STEP_IN_MOTION;
				step_start_time = executeMotionStep(current_step);
				new_command = FALSE;
			} else {
				// something went wrong when setting compliance slope
				current_motion_page = 0;
				next_motion_page = 0;
				new_command = FALSE;
				motion_state = MOTION_STOPPED;
			}
		} else {
			// execute STOP command
			current_motion_page = 0;
			next_motion_page = 0;
			new_command = FALSE;
			motion_state = MOTION_STOPPED;
		}
	} 
	// Option 8 - Nothing to do - keep waiting for new command
	else {
		return motion_state;
	}

	return motion_state;
}

// This function unpacks a motion stored in program memory (Flash) 
// in a struct stored in RAM to allow execution
// StartPage - number of the motion page to be unpacked
void unpackMotion(int StartPage)
{
	uint8 i, s, num_packed_steps;
	uint32 packed_step_values;
	
	// first we retrieve the Compliance Slope values
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		CurrentMotion.JointFlex[i] = pgm_read_byte(motion_pointer[StartPage]+i);
	}
	// next we retrieve the play parameters (each are 1 byte)
	CurrentMotion.NextPage = pgm_read_byte(motion_pointer[StartPage]+NUM_AX12_SERVOS+0);
	CurrentMotion.ExitPage = pgm_read_byte(motion_pointer[StartPage]+NUM_AX12_SERVOS+1);
	CurrentMotion.RepeatTime = pgm_read_byte(motion_pointer[StartPage]+NUM_AX12_SERVOS+2);
	CurrentMotion.SpeedRate10 = pgm_read_byte(motion_pointer[StartPage]+NUM_AX12_SERVOS+3);
	CurrentMotion.InertialForce = pgm_read_byte(motion_pointer[StartPage]+NUM_AX12_SERVOS+4);
	CurrentMotion.Steps = pgm_read_byte(motion_pointer[StartPage]+NUM_AX12_SERVOS+5);
	
	// now we are ready to unpack the Step Values 
	// 3 values are packed into one 32bit integer - so use pgm_read_word twice
	num_packed_steps = NUM_AX12_SERVOS / 3;
	for (s=0; s<CurrentMotion.Steps; s++)
	{
		for (i=0; i<num_packed_steps; i++)
		{
			// higher 16bit
			packed_step_values = pgm_read_word(motion_pointer[StartPage]+NUM_AX12_SERVOS+6+(s*4*num_packed_steps)+4*i+2);
			packed_step_values = packed_step_values << 16;
			// lower 16bit
			packed_step_values += pgm_read_word(motion_pointer[StartPage]+NUM_AX12_SERVOS+6+(s*4*num_packed_steps)+4*i);
			// unpack and store
			CurrentMotion.StepValues[s][3*i+2] = packed_step_values & 0x3FF;
			packed_step_values = packed_step_values >> 11;
			CurrentMotion.StepValues[s][3*i+1] = packed_step_values & 0x3FF;
			packed_step_values = packed_step_values >> 11;
			CurrentMotion.StepValues[s][3*i] = packed_step_values & 0x3FF;
		}
		// Sanity Check - if the values are outside the overall Min/Max values we are probably accessing random memory
		for (i=0; i<NUM_AX12_SERVOS; i++)
		{
			if ( CurrentMotion.StepValues[s][i] > SERVO_MAX_VALUES[i] || CurrentMotion.StepValues[s][i] < SERVO_MIN_VALUES[i] )
			{
				// obviously have unpacked rubbish, stop right here
				printf("\nUnpack Motion Page %i, Step %i - rubbish data. STOP.", StartPage, s+1);
				printf("\nServo ID%i, Step Value = %i, Min = %i, Max = %i \n", AX12_IDS[i], CurrentMotion.StepValues[s][i], SERVO_MIN_VALUES[i],SERVO_MAX_VALUES[i] );
				exit(-1);
			}
		}
		
	}

	// and finally the play and pause times (in ms)
	// both need to be recalculated using the motion speed rate factor
	for (s=0; s<CurrentMotion.Steps; s++)
	{
		CurrentMotion.PauseTime[s] = pgm_read_word(motion_pointer[StartPage]+(NUM_AX12_SERVOS+6+CurrentMotion.Steps*4*num_packed_steps)+(s*2));
		if(CurrentMotion.PauseTime[s] != 0 && CurrentMotion.PauseTime[s] < 6500 ) {
			CurrentMotion.PauseTime[s] = (10*CurrentMotion.PauseTime[s]) / CurrentMotion.SpeedRate10; 
		} else {
			CurrentMotion.PauseTime[s] = 10 * (CurrentMotion.PauseTime[s]/CurrentMotion.SpeedRate10);
		}		
	}		
	for (s=0; s<CurrentMotion.Steps; s++)
	{
		CurrentMotion.PlayTime[s] = pgm_read_word(motion_pointer[StartPage]+(NUM_AX12_SERVOS+6+CurrentMotion.Steps*4*num_packed_steps+CurrentMotion.Steps*2)+(s*2));
		if(CurrentMotion.PlayTime[s] != 0 && CurrentMotion.PlayTime[s] < 6500 ) {
			CurrentMotion.PlayTime[s] = (10*CurrentMotion.PlayTime[s]) / CurrentMotion.SpeedRate10; 
		} else {
			CurrentMotion.PlayTime[s] = 10 * (CurrentMotion.PlayTime[s]/CurrentMotion.SpeedRate10);
		}		
	}		
	
	// Analyse which joints are moving during each step (to save time later)
	if ( CurrentMotion.Steps > 1 )
	{
		// start with step 1 based on current pose values
		for (i=0; i<NUM_AX12_SERVOS; i++)
		{
			if( abs(CurrentMotion.StepValues[0][i] - current_pose[i]) <= 3  )
			motion_step_servos_moving[0][i] = 1;
		}
		// now analyse the remaining motion steps
		for (s=1; s<CurrentMotion.Steps; s++)
		{
			// compare with previous step
			for (i=0; i<NUM_AX12_SERVOS; i++)
			{
				// compare each servo value within +/-3 steps (equivalent to +/- 0.9deg)
				if( abs(CurrentMotion.StepValues[s][i] - CurrentMotion.StepValues[s-1][i]) <= 3 )
					motion_step_servos_moving[s][i] = 0;
				else
					motion_step_servos_moving[s][i] = 1;
			}
		}
		
		// aggregate these to see which servos are moving at all
		for (i=0; i<NUM_AX12_SERVOS; i++)
		{
			// reset the value first
			motion_servos_moving[i] = 0;
			
			// add up the step values from above
			for (s=1; s<CurrentMotion.Steps; s++)
			{
				motion_servos_moving[i] += motion_step_servos_moving[s][i];
			}
			
			// normalise values to 1
			if ( motion_servos_moving[i] > 0 ) motion_servos_moving[i] = 1;
		}
	} else {
		// only 1 Motion Step in current page
		// can only be analysed base don current pose values
		for (i=0; i<NUM_AX12_SERVOS; i++)
		{
			if( abs(CurrentMotion.StepValues[0][i] - current_pose[i]) <= 3  )
			{
				// set both arrays
				motion_step_servos_moving[0][i] = 1;
				motion_servos_moving[i] = 1;
			}			
		}
	}
}

// This function initiates the execution of a motion step in the given motion page
// Step - number of the step to be initiated
// Returns (long) start time of the step
unsigned long executeMotionStep(int Step)
{
	uint16 goalPose[NUM_AX12_SERVOS];
	unsigned long step_start_time;

	// Make sure we never access random memory by accident and damage the robot
	if ( Step > 0 && Step <= CurrentMotion.Steps )
	{
		// TEST printf("\nStarting Motion Step %i", Step);
		
		// create the servo values array 
		for (int j=0; j<NUM_AX12_SERVOS; j++)
			{ goalPose[j] = CurrentMotion.StepValues[Step-1][j]; }
		// take the time
		step_start_time = millis();
		// execute the pose without waiting for completion
		moveToGoalPose(CurrentMotion.PlayTime[Step-1], goalPose, DONT_WAIT_FOR_POSE_FINISH);
		// return the start time to keep track of step timing
		return step_start_time;
	} else {
		// do nothing and return 0
		return 0;
	}
}

// This function initializes the joint flexibility values for the motion page
// Returns (int)  0  - all ok
//				 -1  - communication error
int setMotionPageJointFlexibility()
{
	uint8 complianceSlope;
	int commStatus;

	// now we can process the joint flexibility values
	for (uint8 i=0; i<NUM_AX12_SERVOS; i++) {
		// update is only required if different from last set of values
		if ( last_joint_flex[i] != CurrentMotion.JointFlex[i] )
		{
			// translation is bit shift operation (see AX-12 manual)
			complianceSlope = 1<<CurrentMotion.JointFlex[i]; 
			commStatus = dxl_write_byte(AX12_IDS[i], DXL_CCW_COMPLIANCE_SLOPE, complianceSlope);
			if(commStatus != COMM_RXSUCCESS) {
				// there has been an error, print and break
				printf("\nsetMotionPageJointFlexibility CCW ID%i - ", AX12_IDS[i]);
				dxl_printCommStatus(commStatus);
				return -1;
			}
			commStatus = dxl_write_byte(AX12_IDS[i], DXL_CW_COMPLIANCE_SLOPE, complianceSlope);
			if(commStatus != COMM_RXSUCCESS) {
				// there has been an error, print and break
				printf("\nsetMotionPageJointFlexibility CW ID%i - ", AX12_IDS[i]);
				dxl_printCommStatus(commStatus);
				return -1;
			}
		}
		// update values for next iteration
		last_joint_flex[i] = CurrentMotion.JointFlex[i];
	}
	return 0;
}

// Function to check for any remaining servo movement
// Returns:  (int)	number of servos still moving
int checkMotionStepFinished()
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

// This function executes a single robot motion page defined in motion.h
// It waits for the motion to finish to return control, so it's no good 
// for a command loop
// StartPage - number of the first motion page in the motion
// Returns (int) StartPage of next motion in sequence (0 - no further motions)
int executeMotion(int StartPage)
{
	uint8 complianceSlope;
	int commStatus;
	uint16 goalPose[NUM_AX12_SERVOS];

	// temporary array of timings to keep track of step timing
	unsigned long step_times[MAX_MOTION_STEPS];
	unsigned long pre_step_time, total_time;

	// set the currently executed motion page global variable
	current_motion_page = StartPage;
	
	// first step is to unpack the motion
	unpackMotion(StartPage);
	
	// now we can process the joint flexibility values
	for (uint8 i=0; i<NUM_AX12_SERVOS; i++) {
		// translation is bit shift operation (see AX-12 manual)
		complianceSlope = 1<<CurrentMotion.JointFlex[i]; 
		commStatus = dxl_write_byte(AX12_IDS[i], DXL_CCW_COMPLIANCE_SLOPE, complianceSlope);
		if(commStatus != COMM_RXSUCCESS) {
			// there has been an error, print and break
			printf("executeMotion Joint Flex %i - ", AX12_IDS[i]);
			dxl_printCommStatus(commStatus);
			return 0;
		}
		commStatus = dxl_write_byte(AX12_IDS[i], DXL_CW_COMPLIANCE_SLOPE, complianceSlope);
		if(commStatus != COMM_RXSUCCESS) {
			// there has been an error, print and break
			printf("executeMotion Joint Flex %i - ", AX12_IDS[i]);
			dxl_printCommStatus(commStatus);
			return 0;
		}
	}
	
	total_time = millis();
	
	// in case the motion repeats we need a loop
	for (int r=1; r<=CurrentMotion.RepeatTime; r++)
	{
		// loop over the steps and execute poses
		for (int s=0; s<CurrentMotion.Steps; s++)
		{
			// create the servo values array 
			for (int j=0; j<NUM_AX12_SERVOS; j++)
				{ goalPose[j] = CurrentMotion.StepValues[s][j]; }
			// take the time
			pre_step_time = millis();
			// execute each pose 
			moveToGoalPose(CurrentMotion.PlayTime[s], goalPose, WAIT_FOR_POSE_FINISH);
			// store the time
			step_times[s] = millis() - pre_step_time;
			
			// now pause if required
			if(CurrentMotion.PauseTime[s] > 0) 
				{ delay_ms(CurrentMotion.PauseTime[s]); }
		}
	}
	
	total_time = millis() - total_time; 
	
	// TEST: printf("\nMotion %i Timing :", StartPage);
	// TEST: for (int s=0; s<CurrentMotion.Steps; s++) { printf(" %lu,", step_times[s]); }
	// TEST: printf(" Total: %lu", total_time);
	
	// return the page of the next motion in sequence
	return (int) CurrentMotion.NextPage;	
}

// This function executes the exit page motion for the  
// current motion page
 void executeMotionExitPage()
{
	int	NextPage;
	
	// find the exit page
	NextPage = CurrentMotion.ExitPage;
	// execute the exit page if it exists
	if (NextPage != 0)
	{
		executeMotion(NextPage);
	}
}
