/*******************************************************************************
 *
 * TITLE: Utility Functions
 *
 * NOTES
 * 1. None.
 *
 ******************************************************************************/

#include <math.h>
#include <stdint.h>
#include "stm32l476xx.h"
#include "utilities.h"

/**
 * Due to overhead faster delays will be less accurate. From observations on scope with TIM6 = 10 MHz:
 *
 * useconds		us delay
 * --------		--------
 * 	1			~2
 *  2			~2.9
 *  5			~5.9
 *  10			~10.9
 *  50			~50.8
 *  100			~100.6
 *  500			~500.0
 *  1000		~998.0
 */
void Utils_DelayUs(TIM_TypeDef *TIMx, uint8_t timerRateMHz, uint16_t useconds)
{
	TIMx->CNT = 0;
	uint16_t duration = useconds * timerRateMHz;
	while(TIMx->CNT < duration);
}

Utils_IMU_Data_t Utils_CalibrateIMU(double *IMU_Data, double *biases, double n, double *cosines, double *sines)
{
	Utils_IMU_Data_t Utils_IMU_Data;

	double ax = IMU_Data[0];
	double ay = IMU_Data[1];
	double az = IMU_Data[2];
	double gx = IMU_Data[3];
	double gy = IMU_Data[4];
	double gz = IMU_Data[5];

	double axBias = biases[0];
	double ayBias = biases[1];
	double azBias = biases[2];
	double gxBias = biases[3];
	double gyBias = biases[4];
	double gzBias = biases[5];

	double c1 = cosines[0];
	double c2 = cosines[1];
	double c3 = cosines[2];
	double s1 = sines[0];
	double s2 = sines[1];
	double s3 = sines[2];

	Utils_IMU_Data.ax = n * (ax*(c1*c3 - c2*s1*s3) + ay*(-c3*s1    - c1*c2*s3) + az*( s2*s3)) - axBias;
	Utils_IMU_Data.ay = n * (ax*(c1*s3 + c2*c3*s1) + ay*( c1*c2*c3 - s1*s3   ) + az*(-c3*s2)) - ayBias;
	Utils_IMU_Data.az = n * (ax*(s1*s2           ) + ay*( c1*s2              ) + az*( c2   )) - azBias;
	Utils_IMU_Data.gx = n * (gx*(c1*c3 - c2*s1*s3) + gy*(-c3*s1    - c1*c2*s3) + gz*( s2*s3)) - gxBias;
	Utils_IMU_Data.gy = n * (gx*(c1*s3 + c2*c3*s1) + gy*( c1*c2*c3 - s1*s3   ) + gz*(-c3*s2)) - gyBias;
	Utils_IMU_Data.gz = n * (gx*(s1*s2           ) + gy*( c1*s2              ) + gz*( c2   )) - gzBias;

	return Utils_IMU_Data;
}

/**
 * The angle is calculated on axis 3 using a complementary filter.
 * Axis 1 points horizontally, axis 2 points vertically.
 * Units for gyroscope are degrees/second.
 * Units for accelerometer do not matter.
 * An optimal alpha was previously found to be 0.002 from trial and error experiment of MSE for Invensense MPU9255.
 */
double Utils_CalculateIMU_GlobalAngle(double accel_1, double accel_2, double accel_3, double gyro_3, double dt, double alpha)
{
	double accelAngle = (atan(accel_1 / sqrt(pow(accel_2, 2) + pow(accel_3, 2)))) * 180.0 / M_PI;
	static double globalAngle = 0.0;
	static double dGyroAngle = 0.0;

	dGyroAngle = dt/2 * (gyro_3 + dGyroAngle);										// Change in angle from gyro (trapezoidal used)
	globalAngle = accelAngle*alpha + (1 - alpha) * (dGyroAngle + globalAngle);

	return globalAngle;
}

void Utils_QuaternionToYPR(float r, float i, float j, float k, float *yaw, float *pitch, float *roll)
{
	float siny_cosp = 2 * (r * k + i * j);
	float cosy_cosp = 1 - 2 * (j * j + k * k);
	*yaw = atan2(siny_cosp, cosy_cosp);

	float sinp = sqrt(1 + 2 * (r * j - i * k));
	float cosp = sqrt(1 - 2 * (r * j - i * k));
    *pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    float sinr_cosp = 2 * (r * i + j * k);
    float cosr_cosp = 1 - 2 * (i * i + j * j);
    *roll = atan2(sinr_cosp, cosr_cosp);
}

Utils_Quaternion_t Utils_RotateQuaternion(Utils_Rotation_t *Rotation, Utils_Quaternion_t *Quaternion)
{
    float w = cos(Rotation->angle / 2.0f);

    float factor = sin(Rotation->angle / 2.0f);
    float x = Rotation->x * factor;
    float y = Rotation->y * factor;
    float z = Rotation->z * factor;

    float rotation[4] = {w, x, y, z};
    Utils_Normalize(rotation, 4);

    float r = Quaternion->r * rotation[0] - Quaternion->i * rotation[1] - Quaternion->j * rotation[2] - Quaternion->k * rotation[3];
    float i = Quaternion->r * rotation[1] + Quaternion->i * rotation[0] + Quaternion->j * rotation[3] - Quaternion->k * rotation[2];
    float j = Quaternion->r * rotation[2] - Quaternion->i * rotation[3] + Quaternion->j * rotation[0] + Quaternion->k * rotation[1];
    float k = Quaternion->r * rotation[3] + Quaternion->i * rotation[2] - Quaternion->j * rotation[1] + Quaternion->k * rotation[0];

    float quaternion[4] = {r, i, j, k};
    Utils_Normalize(quaternion, 4);

    Utils_Quaternion_t Result;
    Result.r = quaternion[0];
    Result.i = quaternion[1];
    Result.j = quaternion[2];
    Result.k = quaternion[3];

    return Result;
}

void Utils_Normalize(float *vector, uint8_t length)
{
	float squaredMagnitude = 0.0f;
	for(uint8_t i = 0; i < length; i++)
		squaredMagnitude += vector[i]*vector[i];

	if(squaredMagnitude == 0.0f)
		return;

	float inverseMagnitude = 1.0f / sqrt(squaredMagnitude);

	for(uint8_t i = 0; i < length; i++)
		vector[i] *= inverseMagnitude;
}

float Utils_LinearInterpolate(float x, float x1, float y1, float x2, float y2)
{
	// Avoid division by zero if x1 and x2 are the same
    if (x1 == x2)
        return y1;

    return y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
}

/*******************************************************************************
* END
*******************************************************************************/
