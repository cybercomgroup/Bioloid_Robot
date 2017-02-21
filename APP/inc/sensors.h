#include "time.h"
#include "hw_functions.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_map.h"
#include "stm32f10x_adc.h"


#define PORT_IR_LEFT 			   GPIOA
#define PORT_IR_RIGHT 			   GPIOB

#define PORT_GYRO				   GPIOC
#define PORT_DMS			       GPIOA

//ADC multiplexer
#define PORT_ADX_MUX GPIOC

#define PIN_IR_LEFT_MOTP           GPIO_Pin_0
#define PIN_IR_LEFT_MOTM           GPIO_Pin_1
#define PIN_IR_RIGHT_MOTP		   GPIO_Pin_8
#define PIN_IR_RIGHT_MOTM		   GPIO_Pin_9
#define PIN_GYRO_PITCH_MOTP		   GPIO_Pin_6
#define PIN_GYRO_PITCH_MOTM		   GPIO_Pin_7
#define PIN_GYRO_ROLL_MOTP	       GPIO_Pin_8
#define PIN_GYRO_ROLL_MOTM		   GPIO_Pin_9
#define PIN_DMS_MOTP		       GPIO_Pin_8
#define PIN_DMS_MOTM		       GPIO_Pin_11

#define DEG_TO_RAD				   0.0174532925
#define RAD_TO_DEG				   57.2957795

#define R_ANGLE			           .3
#define Q_ANGLE			   	  	   0.002
#define Q_GYRO				   	   0.1

#define PI						   3.14159265358979323

//Defined for performance reasons
#define SECONDS_PER_MILLIS 		   0.001

//Used with the ADC multiplexer to select signal source for conversion
#define PIN_ADC_SELECT0            GPIO_Pin_1
#define PIN_ADC_SELECT1            GPIO_Pin_2

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


//IR

word read_ir_right(void);
word read_ir_left(void);

//Gyro

void gyro_setup();
void gyro_process();
void gyro_read();
