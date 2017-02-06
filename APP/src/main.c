//#include "includes.h"
#include "dynamixel.h"
#include "printf.h"
#include "pose.h"

void dxl_test1();
void _serial_putc(void*, char);

uint16 pose[NUM_AX12_SERVOS] = {235,788,279,744,462,561,358,666,507,516,341,682,240,783,647,376,507,516};
uint16 speeds[NUM_AX12_SERVOS];

int main(void)
{
	int res, i;

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

	//res = dxl_initialize(0,1); // low level init fn, just tries to init without pinging DXLs.
	res = dxl_init1( 1, 3 ); // high level init fn, pings the DXLs to check their status. Here with max 3 retires on failure.

	if (res != 0){
		printf("DXL init failed, aborting.\r\n");
		return 0;
	}

	dxl_test1();

	printf("Program finished. Have a nice day!\r\n");

	return 0;
}

void dxl_test1() {
	int8 i;

	/* Example to set the speeds (of all servos) and position (of single servo). */
	// Set goal speed
	dxl_write_word( BROADCAST_ID, DXL_MOVING_SPEED_L, 26); // goal speed must be between 26 and 1023, other values default to max speed
	// Set goal position
	dxl_write_word( 1, DXL_GOAL_POSITION_L, 512 );
	mDelay(1000);

	//printf("Moving to default pose.\r\n");
	//for (i=0;i< NUM_AX12_SERVOS; i++ )
	//	pose[i] = current_pose[i];
	//pose[0] = 235;

	//moveToGoalPose(1000, pose, 0);
	//printf("done\r\n");
	//return 0;
//	printf("dbg goal speed: %d %d %d \r\n", dbgSpeeds[0], dbgSpeeds[1], dbgSpeeds[2]);

	for (i=0;i< NUM_AX12_SERVOS; i++ )
		speeds[i] = 100;

	//printf("goal speed2: %d\r\n", speeds[0]);
	//printf("speeds  %d %d %d \r\n", dbgSpeeds[0], dbgSpeeds[1], dbgSpeeds[2]);
	//dxl_set_goal_speed(NUM_AX12_SERVOS, AX12_IDS, pose, speeds);
	dxl_set_goal_speed(1, AX12_IDS, pose, speeds);
	//moveToDefaultPose();
	//printf("Moving to default pose done! commstatus %d\r\n", comSt);
	mDelay(1000);
}


/* Put a character to the serial terminal.
 * Used in the custom printf function. */
void _serial_putc ( void* p, char c)
{
	TxDByte_PC(c);
}


