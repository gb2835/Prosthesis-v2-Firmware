/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_MPU925X_SPI_H_
#define INC_MPU925X_SPI_H_

#include "stm32l4xx_ll_spi.h"

#define MPU9250_DEVICE_ID	0x71
#define MPU9255_DEVICE_ID	0x73

#define MPU925X_REG_SMPLRT_DIV		0x19
#define MPU925X_REG_CONFIG			0x1A
#define MPU925X_REG_GYRO_CONFIG		0x1B
#define MPU925X_REG_ACCEL_CONFIG	0x1C
#define MPU925X_REG_ACCEL_CONFIG_2	0x1D
#define MPU925X_REG_ACCEL_XOUT_H	0x3B
#define MPU925X_REG_ACCEL_XOUT_L	0x3C
#define MPU925X_REG_ACCEL_YOUT_H	0x3D
#define MPU925X_REG_ACCEL_YOUT_L	0x3E
#define MPU925X_REG_ACCEL_ZOUT_H	0x3F
#define MPU925X_REG_ACCEL_ZOUT_L	0x40
#define MPU925X_REG_TEMP_OUT_H		0x41
#define MPU925X_REG_TEMP_OUT_L		0x42
#define MPU925X_REG_GYRO_XOUT_H		0x43
#define MPU925X_REG_GYRO_XOUT_L		0x44
#define MPU925X_REG_GYRO_YOUT_H		0x45
#define MPU925X_REG_GYRO_YOUT_L		0x46
#define MPU925X_REG_GYRO_ZOUT_H		0x47
#define MPU925X_REG_GYRO_ZOUT_L		0x48
#define MPU925X_REG_WHO_AM_I		0X75

#define MPU925X_ACCEL_SENSITIVITY_2G		16384
#define MPU925X_ACCEL_SENSITIVITY_4G		8192
#define MPU925X_ACCEL_SENSITIVITY_8G		4096
#define MPU925X_ACCEL_SENSITIVITY_16G		2048
#define MPU925X_GYRO_SENSITIVITY_250DPS		131
#define MPU925X_GYRO_SENSITIVITY_500DPS		65.5
#define MPU925X_GYRO_SENSITIVITY_1000DPS	32.8
#define MPU925X_GYRO_SENSITIVITY_2000DPS	16.4
#define MPU925X_TEMP_ROOMTEMP				21
#define MPU925X_TEMP_SENSITIVITY			333.87

#define MPU925X_NUMBER_OF_DEVICES	1

typedef enum
{
	MPU925x_AccelDLPF_BandWidth_5hz,
	MPU925x_AccelDLPF_BandWidth_10hz,
	MPU925x_AccelDLPF_BandWidth_20hz,
	MPU925x_AccelDLPF_BandWidth_41hz,
	MPU925x_AccelDLPF_BandWidth_92hz,
	MPU925x_AccelDLPF_BandWidth_184hz,
	MPU925x_AccelDLPF_BandWidth_460hz,
} MPU925x_AccelDLPF_BandWidth_e;

typedef enum
{
	MPU925x_AccelSensitivity_2g,
	MPU925x_AccelSensitivity_4g,
	MPU925x_AccelSensitivity_8g,
	MPU925x_AccelSensitivity_16g
} MPU925x_AccelSensitivity_e;

typedef enum
{
	MPU925x_NoError,
	MPU925x_WhoAmI_Error
} MPU925x_Error_e;

typedef enum
{
	MPU925x_GyroDLPF_BandWidth_5hz,
	MPU925x_GyroDLPF_BandWidth_10hz,
	MPU925x_GyroDLPF_BandWidth_20hz,
	MPU925x_GyroDLPF_BandWidth_41hz,
	MPU925x_GyroDLPF_BandWidth_92hz,
	MPU925x_GyroDLPF_BandWidth_184hz,
	MPU925x_GyroDLPF_BandWidth_250hz,
} MPU925x_GyroDLPF_BandWidth_e;

typedef enum
{
	MPU925x_GyroSensitivity_250dps,
	MPU925x_GyroSensitivity_500dps,
	MPU925x_GyroSensitivity_1000dps,
	MPU925x_GyroSensitivity_2000dps
} MPU925x_GyroSensitivity_e;

typedef union
{
	double array[6];
	struct
	{
		double ax;
		double ay;
		double az;
		double gx;
		double gy;
		double gz;
	} Struct;
} MPU925x_IMU_Data_t;

typedef struct
{
	SPI_TypeDef *SPI_Handle;
	GPIO_TypeDef *CS_GPIOx;
	uint16_t csPin;
} MPU925x_Init_t;

MPU925x_Error_e MPU925x_Init(uint8_t deviceIndex, MPU925x_Init_t *Device_Init);
void MPU925x_SetAccelSensitivity(uint8_t deviceIndex, MPU925x_AccelSensitivity_e sensitivity);
void MPU925x_SetGyroSensitivity(uint8_t deviceIndex, MPU925x_GyroSensitivity_e sensitivity);
void MPU925x_SetAccelDlpfBandwidth(uint8_t deviceIndex, MPU925x_AccelDLPF_BandWidth_e bandwidth);
void MPU925x_SetGyroDlpfBandwidth(uint8_t deviceIndex, MPU925x_GyroDLPF_BandWidth_e bandwidth);
void MPU925x_SetSampleRateDiv(uint8_t deviceIndex, uint8_t divider);
MPU925x_IMU_Data_t MPU925x_ReadIMU(uint8_t deviceIndex);
void MPU925x_ReadRegData(uint8_t deviceIndex, uint8_t startAddress, uint8_t *data, uint8_t nBytes);
void MPU925x_WriteRegData(uint8_t deviceIndex, uint8_t startAddress, uint8_t *data, uint8_t nBytes);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_MPU925X_SPI_H_ */
