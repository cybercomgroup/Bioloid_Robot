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
uint8 motion_state = MOTION_STOPPED;	// motion state as per above definitions
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
void * motion_pointer[NUM_MOTION_PAGES+1];

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
//	if (ACTIVE_MOTION_PAGES != NUM_MOTION_PAGES)
//	{
//			// configuration does not match
//			printf("\nNumber of active motion pages does not match motion.h. ABORT!\n");
//			exit(-1);
//	}

	// set the initial motion state
	motion_state = MOTION_STOPPED;
	
	// Commented code below is deprecated and could be removed


#if use_old_motions_code

	// Motion Page pointer assignment to PROGMEM 
	motion_pointer[0] = 0;
	motion_pointer[1] = (void*) &MotionPage1;
	motion_pointer[2] = (void*) &MotionPage2;
	motion_pointer[3] = (void*) &MotionPage3;
	motion_pointer[4] = (void*) &MotionPage4;
	motion_pointer[5] = (void*) &MotionPage5;
	motion_pointer[6] = (void*) &MotionPage6;
	motion_pointer[7] = (void*) &MotionPage7;
	motion_pointer[8] = (void*) &MotionPage8;
	motion_pointer[9] = (void*) &MotionPage9;
	motion_pointer[10] = (void*) &MotionPage10;
	motion_pointer[11] = (void*) &MotionPage11;
	motion_pointer[12] = (void*) &MotionPage12;
	motion_pointer[13] = (void*) &MotionPage13;
	motion_pointer[14] = (void*) &MotionPage14;
	motion_pointer[15] = (void*) &MotionPage15;
	motion_pointer[16] = (void*) &MotionPage16;
	motion_pointer[17] = (void*) &MotionPage17;
	motion_pointer[18] = (void*) &MotionPage18;
	motion_pointer[19] = (void*) &MotionPage19;
	motion_pointer[20] = (void*) &MotionPage20;
	motion_pointer[21] = (void*) &MotionPage21;
	motion_pointer[22] = (void*) &MotionPage22;
	motion_pointer[23] = (void*) &MotionPage23;
	motion_pointer[24] = (void*) &MotionPage24;
	motion_pointer[25] = (void*) &MotionPage25;
	motion_pointer[26] = (void*) &MotionPage26;
	motion_pointer[27] = (void*) &MotionPage27;
	motion_pointer[28] = (void*) &MotionPage28;
	motion_pointer[29] = (void*) &MotionPage29;
	motion_pointer[30] = (void*) &MotionPage30;
	motion_pointer[31] = (void*) &MotionPage31;
	motion_pointer[32] = (void*) &MotionPage32;
	motion_pointer[33] = (void*) &MotionPage33;
	motion_pointer[34] = (void*) &MotionPage34;
	motion_pointer[35] = (void*) &MotionPage35;
	motion_pointer[36] = (void*) &MotionPage36;
	motion_pointer[37] = (void*) &MotionPage37;
	motion_pointer[38] = (void*) &MotionPage38;
	motion_pointer[39] = (void*) &MotionPage39;
	motion_pointer[40] = (void*) &MotionPage40;
	motion_pointer[41] = (void*) &MotionPage41;
	motion_pointer[42] = (void*) &MotionPage42;
//	motion_pointer[43] = (void*) &MotionPage43;
//	motion_pointer[44] = (void*) &MotionPage44;
//	motion_pointer[45] = (void*) &MotionPage45;
//	motion_pointer[46] = (void*) &MotionPage46;
//	motion_pointer[47] = (void*) &MotionPage47;
//	motion_pointer[48] = (void*) &MotionPage48;
//	motion_pointer[49] = (void*) &MotionPage49;
//	motion_pointer[50] = (void*) &MotionPage50;
//	motion_pointer[51] = (void*) &MotionPage51;
//	motion_pointer[52] = (void*) &MotionPage52;
//	motion_pointer[53] = (void*) &MotionPage53;
//	motion_pointer[54] = (void*) &MotionPage54;
//	motion_pointer[55] = (void*) &MotionPage55;
//	motion_pointer[56] = (void*) &MotionPage56;
//	motion_pointer[57] = (void*) &MotionPage57;
//	motion_pointer[58] = (void*) &MotionPage58;
//	motion_pointer[59] = (void*) &MotionPage59;
//	motion_pointer[60] = (void*) &MotionPage60;
//	motion_pointer[61] = (void*) &MotionPage61;
//	motion_pointer[62] = (void*) &MotionPage62;
//	motion_pointer[63] = (void*) &MotionPage63;
//	motion_pointer[64] = (void*) &MotionPage64;
//	motion_pointer[65] = (void*) &MotionPage65;
//	motion_pointer[66] = (void*) &MotionPage66;
//	motion_pointer[67] = (void*) &MotionPage67;
//	motion_pointer[68] = (void*) &MotionPage68;
//	motion_pointer[69] = (void*) &MotionPage69;
//	motion_pointer[70] = (void*) &MotionPage70;
//	motion_pointer[71] = (void*) &MotionPage71;
//	motion_pointer[72] = (void*) &MotionPage72;
//	motion_pointer[73] = (void*) &MotionPage73;
//	motion_pointer[74] = (void*) &MotionPage74;
//	motion_pointer[75] = (void*) &MotionPage75;
//	motion_pointer[76] = (void*) &MotionPage76;
//	motion_pointer[77] = (void*) &MotionPage77;
//	motion_pointer[78] = (void*) &MotionPage78;
//	motion_pointer[79] = (void*) &MotionPage79;
//	motion_pointer[80] = (void*) &MotionPage80;
//	motion_pointer[81] = (void*) &MotionPage81;
//	motion_pointer[82] = (void*) &MotionPage82;
//	motion_pointer[83] = (void*) &MotionPage83;
//	motion_pointer[84] = (void*) &MotionPage84;
//	motion_pointer[85] = (void*) &MotionPage85;
//	motion_pointer[86] = (void*) &MotionPage86;
//	motion_pointer[87] = (void*) &MotionPage87;
//	motion_pointer[88] = (void*) &MotionPage88;
//	motion_pointer[89] = (void*) &MotionPage89;
//	motion_pointer[90] = (void*) &MotionPage90;
//	motion_pointer[91] = (void*) &MotionPage91;
//	motion_pointer[92] = (void*) &MotionPage92;
//	motion_pointer[93] = (void*) &MotionPage93;
//	motion_pointer[94] = (void*) &MotionPage94;
//	motion_pointer[95] = (void*) &MotionPage95;
//	motion_pointer[96] = (void*) &MotionPage96;
//	motion_pointer[97] = (void*) &MotionPage97;
//	motion_pointer[98] = (void*) &MotionPage98;
//	motion_pointer[99] = (void*) &MotionPage99;
//	motion_pointer[100] = (void*) &MotionPage100;
//	motion_pointer[101] = (void*) &MotionPage101;
//	motion_pointer[102] = (void*) &MotionPage102;
//	motion_pointer[103] = (void*) &MotionPage103;
//	motion_pointer[104] = (void*) &MotionPage104;
//	motion_pointer[105] = (void*) &MotionPage105;
//	motion_pointer[106] = (void*) &MotionPage106;
//	motion_pointer[107] = (void*) &MotionPage107;
//	motion_pointer[108] = (void*) &MotionPage108;
//	motion_pointer[109] = (void*) &MotionPage109;
//	motion_pointer[110] = (void*) &MotionPage110;
//	motion_pointer[111] = (void*) &MotionPage111;
//	motion_pointer[112] = (void*) &MotionPage112;
//	motion_pointer[113] = (void*) &MotionPage113;
//	motion_pointer[114] = (void*) &MotionPage114;
//	motion_pointer[115] = (void*) &MotionPage115;
//	motion_pointer[116] = (void*) &MotionPage116;
//	motion_pointer[117] = (void*) &MotionPage117;
//	motion_pointer[118] = (void*) &MotionPage118;
//	motion_pointer[119] = (void*) &MotionPage119;
//	motion_pointer[120] = (void*) &MotionPage120;
//	motion_pointer[121] = (void*) &MotionPage121;
//	motion_pointer[122] = (void*) &MotionPage122;
//	motion_pointer[123] = (void*) &MotionPage123;
//	motion_pointer[124] = (void*) &MotionPage124;
//	motion_pointer[125] = (void*) &MotionPage125;
//	motion_pointer[126] = (void*) &MotionPage126;
//	motion_pointer[127] = (void*) &MotionPage127;
//	motion_pointer[128] = (void*) &MotionPage128;
//	motion_pointer[129] = (void*) &MotionPage129;
//	motion_pointer[130] = (void*) &MotionPage130;
//	motion_pointer[131] = (void*) &MotionPage131;
//	motion_pointer[132] = (void*) &MotionPage132;
//	motion_pointer[133] = (void*) &MotionPage133;
//	motion_pointer[134] = (void*) &MotionPage134;
//	motion_pointer[135] = (void*) &MotionPage135;
//	motion_pointer[136] = (void*) &MotionPage136;
//	motion_pointer[137] = (void*) &MotionPage137;
//	motion_pointer[138] = (void*) &MotionPage138;
//	motion_pointer[139] = (void*) &MotionPage139;
//	motion_pointer[140] = (void*) &MotionPage140;
//	motion_pointer[141] = (void*) &MotionPage141;
//	motion_pointer[142] = (void*) &MotionPage142;
//	motion_pointer[143] = (void*) &MotionPage143;
//	motion_pointer[144] = (void*) &MotionPage144;
//	motion_pointer[145] = (void*) &MotionPage145;
//	motion_pointer[146] = (void*) &MotionPage146;
//	motion_pointer[147] = (void*) &MotionPage147;
//	motion_pointer[148] = (void*) &MotionPage148;
//	motion_pointer[149] = (void*) &MotionPage149;
//	motion_pointer[150] = (void*) &MotionPage150;
//	motion_pointer[151] = (void*) &MotionPage151;
//	motion_pointer[152] = (void*) &MotionPage152;
//	motion_pointer[153] = (void*) &MotionPage153;
//	motion_pointer[154] = (void*) &MotionPage154;
//	motion_pointer[155] = (void*) &MotionPage155;
//	motion_pointer[156] = (void*) &MotionPage156;
//	motion_pointer[157] = (void*) &MotionPage157;
//	motion_pointer[158] = (void*) &MotionPage158;
//	motion_pointer[159] = (void*) &MotionPage159;
//	motion_pointer[160] = (void*) &MotionPage160;
//	motion_pointer[161] = (void*) &MotionPage161;
//	motion_pointer[162] = (void*) &MotionPage162;
//	motion_pointer[163] = (void*) &MotionPage163;
//	motion_pointer[164] = (void*) &MotionPage164;
//	motion_pointer[165] = (void*) &MotionPage165;
//	motion_pointer[166] = (void*) &MotionPage166;
//	motion_pointer[167] = (void*) &MotionPage167;
//	motion_pointer[168] = (void*) &MotionPage168;
//	motion_pointer[169] = (void*) &MotionPage169;
//	motion_pointer[170] = (void*) &MotionPage170;
//	motion_pointer[171] = (void*) &MotionPage171;
//	motion_pointer[172] = (void*) &MotionPage172;
//	motion_pointer[173] = (void*) &MotionPage173;
//	motion_pointer[174] = (void*) &MotionPage174;
//	motion_pointer[175] = (void*) &MotionPage175;
//	motion_pointer[176] = (void*) &MotionPage176;
//	motion_pointer[177] = (void*) &MotionPage177;
//	motion_pointer[178] = (void*) &MotionPage178;
//	motion_pointer[179] = (void*) &MotionPage179;
//	motion_pointer[180] = (void*) &MotionPage180;
//	motion_pointer[181] = (void*) &MotionPage181;
//	motion_pointer[182] = (void*) &MotionPage182;
//	motion_pointer[183] = (void*) &MotionPage183;
//	motion_pointer[184] = (void*) &MotionPage184;
//	motion_pointer[185] = (void*) &MotionPage185;
//	motion_pointer[186] = (void*) &MotionPage186;
//	motion_pointer[187] = (void*) &MotionPage187;
//	motion_pointer[188] = (void*) &MotionPage188;
//	motion_pointer[189] = (void*) &MotionPage189;
//	motion_pointer[190] = (void*) &MotionPage190;
//	motion_pointer[191] = (void*) &MotionPage191;
//	motion_pointer[192] = (void*) &MotionPage192;
//	motion_pointer[193] = (void*) &MotionPage193;
//	motion_pointer[194] = (void*) &MotionPage194;
//	motion_pointer[195] = (void*) &MotionPage195;
//	motion_pointer[196] = (void*) &MotionPage196;
//	motion_pointer[197] = (void*) &MotionPage197;
//	motion_pointer[198] = (void*) &MotionPage198;
//	motion_pointer[199] = (void*) &MotionPage199;
//	motion_pointer[200] = (void*) &MotionPage200;
//	motion_pointer[201] = (void*) &MotionPage201;
//	motion_pointer[202] = (void*) &MotionPage202;
//	motion_pointer[203] = (void*) &MotionPage203;
//	motion_pointer[204] = (void*) &MotionPage204;
//	motion_pointer[205] = (void*) &MotionPage205;
//	motion_pointer[206] = (void*) &MotionPage206;
//	motion_pointer[207] = (void*) &MotionPage207;
//	motion_pointer[208] = (void*) &MotionPage208;
//	motion_pointer[209] = (void*) &MotionPage209;
//	motion_pointer[210] = (void*) &MotionPage210;
//	motion_pointer[211] = (void*) &MotionPage211;
//	motion_pointer[212] = (void*) &MotionPage212;
//	motion_pointer[213] = (void*) &MotionPage213;
//	motion_pointer[214] = (void*) &MotionPage214;
//	motion_pointer[215] = (void*) &MotionPage215;
//	motion_pointer[216] = (void*) &MotionPage216;
//	motion_pointer[217] = (void*) &MotionPage217;
//	motion_pointer[218] = (void*) &MotionPage218;
//	motion_pointer[219] = (void*) &MotionPage219;
//	motion_pointer[220] = (void*) &MotionPage220;
//	motion_pointer[221] = (void*) &MotionPage221;
//	motion_pointer[222] = (void*) &MotionPage222;
//	motion_pointer[223] = (void*) &MotionPage223;
//	motion_pointer[224] = (void*) &MotionPage224;
//	motion_pointer[225] = (void*) &MotionPage225;
//	motion_pointer[226] = (void*) &MotionPage226;
//	motion_pointer[227] = (void*) &MotionPage227;
#endif // use_old_motions_code
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
		readCurrentPose();
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

// This function unpacks a motion page as stored by RoboPlus Motion
// StartPage - number of the motion page to be unpacked
// DONT DO PRINTF IN THIS FUNCTION! IT WILL TAKE TOO LONG AND MAY BREAK WALKING OR OTHER MOTIONS.
void unpackMotion2(int StartPage) {
	int i;
	const motion_page * motion = &motion_pointer_pages[StartPage-1];
	//printf("Unpacking motion %i: %s\n", StartPage, motion->name);

	CurrentMotion.RepeatTime = motion->repetitions;
	CurrentMotion.Steps = motion->n_steps;
	CurrentMotion.SpeedRate10 = 10 * 32 / motion->speed_rate; // not actually used just for comparision with old code
	//printf("speedrate in mem is: %d, calculated speedrate10 is: %d\n", motion->speed_rate, CurrentMotion.SpeedRate10);
	CurrentMotion.InertialForce = motion->ctrl_inertial_force;
	CurrentMotion.NextPage = motion->next_page;
	CurrentMotion.ExitPage = motion->exit_page;

	// servo compliance slope / joint softness
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		 CurrentMotion.JointFlex[i] = motion->joint_softness[i+1] & 0x0f; // actually both cw and ccw is sttored in 4 bits each, here we use the same value for both
	}

	// step servo positions and step pause/execution time
	for (int step = 0; step < CurrentMotion.Steps; step++)
	{
		for (i = 0; i < NUM_AX12_SERVOS; i++) {
			CurrentMotion.StepValues[step][i] = motion->steps[step].joint_pos[i+1];

			if ( CurrentMotion.StepValues[step][i] > SERVO_MAX_VALUES[i] || CurrentMotion.StepValues[step][i] < SERVO_MIN_VALUES[i] )
			{
				// obviously have unpacked rubbish, stop right here
				printf("\nUnpack Motion Page %i, Step %i - rubbish data. STOP.", StartPage, step+1);
				printf("\nServo ID%i, Step Value = %i, Min = %i, Max = %i \n", AX12_IDS[i], CurrentMotion.StepValues[step][i], SERVO_MIN_VALUES[i],SERVO_MAX_VALUES[i] );
				exit(-1);
			}
		}

		CurrentMotion.PauseTime[step] = motion->steps[step].pause * 8; //ms
		CurrentMotion.PlayTime[step] = motion->steps[step].time * 8; //ms

		// apply speed rate
		CurrentMotion.PauseTime[step] = (CurrentMotion.PauseTime[step] * motion->speed_rate) / 32; //ms
		CurrentMotion.PlayTime[step] = (CurrentMotion.PlayTime[step] * motion->speed_rate) / 32; //ms
	}

	// printCurrentMotionPage();
}

#if use_old_motions_code
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

#endif //use_old_motions_code

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

// This function initializes the joint flexibility values for the current motion page
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

int checkGoalPoseReached()
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

int checkMotionFinished() {
	return motion_state == MOTION_STOPPED;
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

void setNewMotionCommand(int motionPageId) {
	bioloid_command = COMMAND_MOTIONPAGE;
	next_motion_page = motionPageId;
	new_command = 1;
}

// for debugging
void printCurrentMotionPage() {
	printf("motion.c: CurrentMotion:\n");
	printf(" RepeatTime=%d \n Steps=%d \n SpeedRate10=%d \n InertialForce=%d \n NextPage=%d \n ExitPage=%d \n", CurrentMotion.RepeatTime, CurrentMotion.Steps, CurrentMotion.SpeedRate10, CurrentMotion.InertialForce, CurrentMotion.NextPage, CurrentMotion.ExitPage);
	for (int s=0; s<CurrentMotion.Steps; s++)
	{
		printf(" Steps:\n");
		printf("   step %i: play=%d pause=%d\n  ", s, CurrentMotion.PlayTime[s], CurrentMotion.PauseTime[s]);
		for (int i=0; i<NUM_AX12_SERVOS; i++)
		{
			printf("%d ", CurrentMotion.StepValues[s][i]);
		}
		printf("\n");
	}
	printf("Joint flex:\n ");
	for (int i=0; i<NUM_AX12_SERVOS; i++)
	{
		printf("%d ", CurrentMotion.JointFlex[i]);
	}
	printf("\n");
}

