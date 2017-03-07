/*
 * lp_filter.h
 *
 *  Created on: 6 mars 2017
 *      Author: Anton Olsson / Cybercom
 */

#ifndef APP_INC_FILTERS_H_
#define APP_INC_FILTERS_H_

// filter the input array and return the last filtered value as output.
// cutoff: cutoff frequency in Hz
// points: input buffer size
// dt: time delta between sample points.
int lowPass(const int* input, int dt, int points, int cutoff);

// same as lowPass, but allows for using a circular buffer as input
int lowPassCircularBuf(const int* input, int dt, int start_item, int points, int cutoff);

// same as lowPassCircularBuf, but save the intermediate filtered output values in output array. start_item applies to both input and output arrays.
int lowPassCircularBuf2(const int* input, int * output, const int dt, const int start_item, const int points, const int cutoff);

#endif /* APP_INC_FILTERS_H_ */
