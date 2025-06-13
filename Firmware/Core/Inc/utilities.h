/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

#include <stdint.h>
#include "stm32l476xx.h"

typedef struct
{
	double ax;
	double ay;
	double az;
	double gx;
	double gy;
	double gz;
} Utils_IMU_Data_t;

void DelayUs(TIM_TypeDef *TIMx, uint8_t timerRateMHz, uint16_t useconds);
Utils_IMU_Data_t CalibrateIMU(double *IMU_Data, double *biases, double n, double *cosines, double *sines);
double CalculateIMU_GlobalAngle(double accel_1, double accel_2, double accel_3, double gyro_3, double dt, double alpha);
void QuaternionsToYPR(float r, float i, float j, float k, float *yaw, float *pitch, float *roll);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_UTILITIES_H_ */
