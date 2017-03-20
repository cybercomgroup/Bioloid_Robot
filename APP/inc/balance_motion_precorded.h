/*
 * balance_motion_precorded.h
 *
 *  Created on: 15 mars 2017
 *      Author: install
 */

#ifndef APP_INC_BALANCE_MOTION_PRECORDED_H_
#define APP_INC_BALANCE_MOTION_PRECORDED_H_

#include "typedefs.h"

typedef struct
{
    u16 time;
    s16 x;
    s16 y;
} mtn_rot_vel_timings;


mtn_rot_vel_timings wlk_fwd[] =
{
	{0, -120, -57},
	{5, -129, -58},
	{10, -120, -48},
	{14, -108, -35},
	{19, -119, -44},
	{24, -129, -46},
	{30, -163, -76},
	{34, -199, -110},
	{39, -233, -143},
	{44, -259, -175},
	{49, -275, -199},
	{54, -279, -213},
	{60, -264, -206},
	{65, -256, -204},
	{70, -242, -194},
	{75, -220, -178},
	{79, -189, -151},
	{84, -163, -133},
	{90, -126, -102},
	{96, -100, -80},
	{101, -77, -61},
	{106, -62, -51},
	{111, -55, -49},
	{117, -56, -52},
	{122, -59, -53},
	{127, -58, -51},
	{132, -58, -45},
	{138, -55, -36},
	{144, -50, -24},
	{149, -45, -14},
	{154, -37, 0},
	{159, -34, 3},
	{164, -31, 6},
	{169, -25, 7},
	{176, -12, 12},
	{181, -10, 9},
	{186, -14, 3},
	{191, -18, -7},
	{196, -27, -13},
	{202, -33, -21},
	{207, -40, -25},
	{212, -52, -31},
	{218, -74, -47},
	{223, -86, -53},
	{229, -99, -71},
	{234, -120, -93},
	{239, -130, -105},
	{244, -133, -112},
	{249, -145, -124},
	{254, -143, -126},
	{261, -133, -121},
	{266, -138, -126},
	{271, -136, -122},
	{276, -128, -112},
	{281, -129, -111},
	{287, -127, -104},
	{292, -123, -96},
	{297, -127, -100},
	{303, -126, -96},
	{308, -131, -97},
	{314, -136, -96},
	{319, -135, -93},
	{324, -143, -93},
	{329, -154, -99},
	{334, -164, -104},
	{339, -173, -110},
	{345, -186, -117},
	{350, -195, -126},
	{355, -199, -135},
	{360, -205, -151},
	{365, -206, -164},
	{369, -206, -176},
	{375, -193, -178},
	{380, -181, -179},
	{385, -166, -174},
	{390, -141, -156},
	{395, -121, -136},
	{400, -96, -112},
	{406, -60, -62},
	{411, -37, -32},
	{416, -9, -14},
	{422, 13, 1},
	{426, 25, 16},
	{431, 38, 25},
	{437, 45, 33},
	{442, 43, 39},
	{447, 31, 35},
	{452, 28, 34},
	{457, 26, 30},
	{462, 21, 21},
	{468, 19, 16},
	{473, 10, 0},
	{478, 6, -12},
	{483, 3, -20},
	{488, 7, -23},
	{493, 9, -28},
	{500, 15, -29},
	{505, 20, -30},
	{509, 29, -24},
	{514, 43, -13},
	{519, 62, 1},
	{523, 81, 17},
	{529, 101, 33},
	{534, 119, 47},
	{539, 134, 59},
	{544, 145, 68},
	{549, 159, 81},
	{554, 180, 92},
	{560, 198, 108},
	{565, 215, 122},
	{570, 227, 138},
	{575, 231, 143},
	{580, 240, 151},
	{585, 250, 162},
	{591, 253, 168},
	{596, 257, 174},
	{601, 257, 177},
	{605, 254, 174},
	{610, 229, 158},
	{615, 214, 154},
	{622, 196, 142},
	{627, 183, 133},
	{631, 177, 125},
	{636, 175, 118},
	{640, 172, 113},
	{645, 176, 116},
	{651, 180, 115},
	{656, 175, 114},
	{661, 163, 102},
	{666, 154, 86},
	{671, 139, 71},
	{675, 129, 64},
	{681, 120, 57},
	{686, 123, 64},
	{690, 130, 76},
	{695, 136, 83},
	{700, 135, 87},
	{705, 136, 91},
	{711, 133, 89},
	{715, 134, 91},
	{720, 138, 94},
	{724, 135, 94},
	{729, 148, 108},
	{734, 152, 121},
	{739, 136, 116},
	{745, 114, 95},
	{749, 113, 97},
	{754, 118, 109},
	{758, 101, 99},
	{763, 84, 89},
	{769, 85, 92},
	{773, 77, 87},
	{778, 76, 92},
	{783, 90, 111},
	{787, 82, 105},
	{792, 83, 109},
	{797, 70, 105},
	{802, 58, 103},
	{807, 44, 85},
	{811, 31, 76},
	{816, 12, 61},
	{820, -7, 50},
	{-1, 0, 0},
};



#endif /* APP_INC_BALANCE_MOTION_PRECORDED_H_ */
