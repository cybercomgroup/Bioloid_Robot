/*
 * lp_filter.c
 *
 *  Created on: 6 mars 2017
 *      Author: Anton Olsson / Cybercom
 */


int lowPass(const int* input, int dt, int points, int cutoff)
{
    long RC = (1000/cutoff) / 6;
    int output = input[0];
    for(int i = 1; i < points; ++i)
    {
    	output = output + ((input[i] - output) * dt / (RC+dt));
    }
    return output;
}


int lowPassCircularBuf(const int* input, int dt, int start_item, int points, int cutoff)
{
    long RC = (1000/cutoff) / 6;
    int output = input[start_item];
    for(int i = 1; i < points; ++i)
    {
    	output = output + ((input[(start_item + i) % points] - output) * dt / (RC+dt));
    }
    return output;
}


int lowPassCircularBuf2(const int* input, int * output, const int dt, const int start_item, const int points, const int cutoff)
{
    long RC = (1000/cutoff) / 6;
    int last_output = output[start_item] = input[start_item];
    for(int i = 1; i < points; ++i)
    {
    	last_output = output[(start_item + i) % points] = last_output + ((input[(start_item + i) % points] - last_output) * dt / (RC+dt));
    }
    return last_output;
}

