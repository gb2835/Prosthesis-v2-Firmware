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

typedef struct
{
	float r;
	float i;
	float j;
	float k;
} Utils_Quaternion_t;

typedef struct
{
	float angle;
	float x;
	float y;
	float z;
} Utils_Rotation_t;

void Utils_DelayUs(TIM_TypeDef *TIMx, uint8_t timerRateMHz, uint16_t useconds);
Utils_IMU_Data_t Utils_CalibrateIMU(double *IMU_Data, double *biases, double n, double *cosines, double *sines);
double Utils_CalculateIMU_GlobalAngle(double accel_1, double accel_2, double accel_3, double gyro_3, double dt, double alpha);
void Utils_QuaternionToYPR(float r, float i, float j, float k, float *yaw, float *pitch, float *roll);
Utils_Quaternion_t Utils_RotateQuaternion(Utils_Rotation_t *Rotation, Utils_Quaternion_t *Quaternion);
void Utils_Normalize(float *vector, uint8_t length);
float Utils_LinearInterpolate(float x, float x1, float y1, float x2, float y2);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_UTILITIES_H_ */
