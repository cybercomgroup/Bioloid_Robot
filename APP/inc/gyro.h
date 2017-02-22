/*
 * gyro.h
 *
 *  Created on: 21 feb. 2017
 *      Author: install
 */

#include "sensors.h"

#ifndef APP_INC_GYRO_H_
#define APP_INC_GYRO_H_


#define PIN_GYRO_PITCH_MOTP		   GPIO_Pin_6
#define PIN_GYRO_PITCH_MOTM		   GPIO_Pin_7
#define PIN_GYRO_ROLL_MOTP	       GPIO_Pin_8
#define PIN_GYRO_ROLL_MOTM		   GPIO_Pin_9

#define DEG_TO_RAD				   0.0174532925
#define RAD_TO_DEG				   57.2957795

#define R_ANGLE			           .3
#define Q_ANGLE			   	  	   0.002
#define Q_GYRO				   	   0.1

#define PI						   3.14159265358979323

typedef struct
{
  /* These variables represent our state matrix x */
  double x_angle,
         x_bias;

  /* Our error covariance matrix */
  double p_00,
         p_01,
         p_10,
         p_11;

  /*
   * Q is a 2x2 matrix of the covariance. Because we
   * assume the gyro and accelerometer noise to be independent
   * of each other, the covariances on the / diagonal are 0.
   *
   * Covariance Q, the process noise, from the assumption
   *    x = F x + B u + w
   * with w having a normal distribution with covariance Q.
   * (covariance = E[ (X - E[X])*(X - E[X])' ]
   * We assume is linear with dt
   */
  double q_angle, q_gyro;

  /*
   * Covariance R, our observation noise (from the accelerometer)
   * Also assumed to be linear with dt
   */
  double r_angle;
} gyro;

//Gyro
void gyro_setup();
void gyro_process();
void gyro_read();

#endif /* APP_INC_GYRO_H_ */
