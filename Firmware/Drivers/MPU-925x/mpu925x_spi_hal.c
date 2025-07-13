/*******************************************************************************
*
* TITLE: Driver for InvenSense MPU-9250 and MPU-9255 IMU using SPI with HAL
*
* NOTES
* 1. This driver is based on
* 		- MPU-9255 Register Map and Descriptions
* 			- Document Number: RM-000008
* 			- Revision: 1.0
* 		- MPU-9255 Product Specification
* 			- Document Number: DS-000007
* 			- Revision: 1.0
* 1. Unless otherwise specified, units are
* 		- Accelerometer = g's
* 		- Gyroscope     = degrees/second
* 2. #define NUMBER_OF_DEVICES must be updated to (at least) the number of devices used.
* 3. Additional bandwidths are available though not present here (additional programming required).
*
*******************************************************************************/

#include "mpu925x_spi_hal.h"

#include <string.h>


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

typedef struct
{
	SPI_HandleTypeDef *SPI_Handle;
	GPIO_TypeDef *CS_GPIOx;
	uint16_t csPin;
	uint32_t isInit;
} Device_t;

static Device_t Device[MPU925X_NUMBER_OF_DEVICES];
static double accelSensitivity = MPU925X_ACCEL_SENSITIVITY_2G;		// ±2 g is default
static double gyroSensitivity = MPU925X_GYRO_SENSITIVITY_250DPS;	// ±250 degrees/second is default

static void ReadRegData(uint8_t deviceIndex, uint8_t startAddress, uint8_t *data, uint8_t nBytes);
static void ReadData_IT(uint8_t deviceIndex, uint8_t *data, uint8_t nBytes);
static void WriteRegData(uint8_t deviceIndex, uint8_t startAdress, uint8_t *data, uint8_t nBytes);
static void WriteData_IT(uint8_t deviceIndex, uint8_t *data, uint8_t nBytes);
static inline void ClearChipSelect(uint8_t deviceIndex);
static inline void SetChipSelect(uint8_t deviceIndex);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

MPU925x_Error_e MPU925x_Init(uint8_t deviceIndex, MPU925x_Init_t *Device_Init)
{
	if(deviceIndex + 1 > MPU925X_NUMBER_OF_DEVICES)
		while(1);

	memcpy(&Device[deviceIndex], Device_Init, sizeof(MPU925x_Init_t));

	ClearChipSelect(deviceIndex);

	uint8_t whoAmI;
	ReadRegData(deviceIndex, MPU925X_REG_WHO_AM_I, &whoAmI, 1);
	if((whoAmI != MPU9250_DEVICE_ID) && (whoAmI != MPU9255_DEVICE_ID))
		return MPU925x_WhoAmI_Error;

	Device[deviceIndex].isInit = 1;

	return MPU925x_NoError;
}

void MPU925x_SetAccelSensitivity(uint8_t deviceIndex, MPU925x_AccelSensitivity_e sensitivity)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	uint8_t data;
	switch(sensitivity)
	{
	case MPU925x_AccelSensitivity_2g:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = data & ~0x18;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_2G;
		break;

	case MPU925x_AccelSensitivity_4g:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x08;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_4G;
		break;

	case MPU925x_AccelSensitivity_8g:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x10;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_8G;
		break;

	case MPU925x_AccelSensitivity_16g:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x18;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_16G;
		break;
	}
}

void MPU925x_SetGyroSensitivity(uint8_t deviceIndex, MPU925x_GyroSensitivity_e sensitivity)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	uint8_t data;
	switch(sensitivity)
	{
	case MPU925x_GyroSensitivity_250dps:
		ReadRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = data & ~0x18;
		WriteRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_250DPS;
		break;

	case MPU925x_GyroSensitivity_500dps:
		ReadRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x08;
		WriteRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_500DPS;
		break;

	case MPU925x_GyroSensitivity_1000dps:
		ReadRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x10;
		WriteRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_1000DPS;
		break;

	case MPU925x_GyroSensitivity_2000dps:
		ReadRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x18;
		WriteRegData(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_2000DPS;
		break;
	}
}

void MPU925x_SetAccelDlpfBandwidth(uint8_t deviceIndex, MPU925x_AccelDLPF_BandWidth_e BandWidth)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	uint8_t data;
	switch (BandWidth)
	{
	case MPU925x_AccelDLPF_BandWidth_5hz:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x06;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		break;

	case MPU925x_AccelDLPF_BandWidth_10hz:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x05;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		break;

	case MPU925x_AccelDLPF_BandWidth_20hz:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x04;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		break;

	case MPU925x_AccelDLPF_BandWidth_41hz:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x03;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		break;

	case MPU925x_AccelDLPF_BandWidth_92hz:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x02;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		break;

	case MPU925x_AccelDLPF_BandWidth_184hz:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x01;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		break;

	case MPU925x_AccelDLPF_BandWidth_460hz:
		ReadRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = data & ~0x0F;
		WriteRegData(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		break;
	}
}

void MPU925x_SetGyroDlpfBandwidth(uint8_t deviceIndex, MPU925x_GyroDLPF_BandWidth_e BandWidth)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	uint8_t data;
	switch (BandWidth)
	{
	case MPU925x_GyroDLPF_BandWidth_5hz:
		ReadRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x06;
		WriteRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		break;

	case MPU925x_GyroDLPF_BandWidth_10hz:
		ReadRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x05;
		WriteRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		break;

	case MPU925x_GyroDLPF_BandWidth_20hz:
		ReadRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x04;
		WriteRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		break;

	case MPU925x_GyroDLPF_BandWidth_41hz:
		ReadRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x03;
		WriteRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		break;

	case MPU925x_GyroDLPF_BandWidth_92hz:
		ReadRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x02;
		WriteRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		break;

	case MPU925x_GyroDLPF_BandWidth_184hz:
		ReadRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x01;
		WriteRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		break;

	case MPU925x_GyroDLPF_BandWidth_250hz:
		ReadRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = data & ~0x07;
		WriteRegData(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		break;
	}
}

void MPU925x_GetGyroOffsets(uint8_t deviceIndex, double *offsets)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	double sumGx = 0.0;
	double sumGy = 0.0;
	double sumGz = 0.0;
	for(uint16_t i = 0; i < 1000; i++)
	{
		MPU925x_IMU_Data_t IMU_Data = MPU925x_ReadIMU(deviceIndex);
		sumGx += IMU_Data.Struct.gx;
		sumGy += IMU_Data.Struct.gy;
		sumGz += IMU_Data.Struct.gz;
	}

	offsets[0] = sumGx / 1000.0;
	offsets[1] = sumGy / 1000.0;
	offsets[2] = sumGz / 1000.0;
}

// Only applies when sample frequency = 1 kHz
// New sample rate = 1 kHz / (1 + divider)
void MPU925x_SetSampleRateDiv(uint8_t deviceIndex, uint8_t divider)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	WriteRegData(deviceIndex, MPU925X_REG_SMPLRT_DIV, &divider, 1);
}

MPU925x_IMU_Data_t MPU925x_ReadIMU(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	uint8_t data[14];
	ReadRegData(deviceIndex, MPU925X_REG_ACCEL_XOUT_H, data, 14);

	return MPU925x_ConvertIMU_Data(data);
}

void MPU925x_StartReadIMU_IT(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	uint8_t startAddress = MPU925X_REG_ACCEL_XOUT_H | 0x80;
	WriteData_IT(deviceIndex, &startAddress, 1);
}

void MPU925x_ReadIMU_IT(uint8_t deviceIndex, uint8_t *data)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	ReadData_IT(deviceIndex, data, 14);
}

void MPU925x_ReadRegData(uint8_t deviceIndex, uint8_t startAddress, uint8_t *data, uint8_t nBytes)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	ReadRegData(deviceIndex, startAddress, data, nBytes);
}

void MPU925x_ReadData_IT(uint8_t deviceIndex, uint8_t *data, uint8_t nBytes)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	ReadData_IT(deviceIndex, data, nBytes);
}

void MPU925x_WriteRegData(uint8_t deviceIndex, uint8_t startAdress, uint8_t *data, uint8_t nBytes)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	WriteRegData(deviceIndex, startAdress, data, nBytes);
}

void MPU925x_WriteData_IT(uint8_t deviceIndex, uint8_t *data, uint8_t nBytes)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	WriteData_IT(deviceIndex, data, nBytes);
}

MPU925x_IMU_Data_t MPU925x_ConvertIMU_Data(uint8_t *data)
{
	int16_t ax = ((int16_t) data[0] << 8) | data[1];
	int16_t ay = ((int16_t) data[2] << 8) | data[3];
	int16_t az = ((int16_t) data[4] << 8) | data[5];
	int16_t gx = ((int16_t) data[8] << 8) | data[9];
	int16_t gy = ((int16_t) data[10] << 8) | data[11];
	int16_t gz = ((int16_t) data[12] << 8) | data[13];

	MPU925x_IMU_Data_t IMU_Data;
	IMU_Data.Struct.ax = ax / accelSensitivity;
	IMU_Data.Struct.ay = ay / accelSensitivity;
	IMU_Data.Struct.az = az / accelSensitivity;
	IMU_Data.Struct.gx = gx / gyroSensitivity;
	IMU_Data.Struct.gy = gy / gyroSensitivity;
	IMU_Data.Struct.gz = gz / gyroSensitivity;

	return IMU_Data;
}

void MPU925x_ClearChipSelect(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	ClearChipSelect(deviceIndex);
}

void MPU925x_SetChipSelect(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	SetChipSelect(deviceIndex);
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void ReadRegData(uint8_t deviceIndex, uint8_t startAddress, uint8_t *data, uint8_t nBytes)
{
	SetChipSelect(deviceIndex);

	startAddress = startAddress | 0x80;
	HAL_SPI_Transmit(Device[deviceIndex].SPI_Handle, &startAddress, 1, 10);
	HAL_SPI_Receive(Device[deviceIndex].SPI_Handle, data, nBytes, 10);

	ClearChipSelect(deviceIndex);
}

static void ReadData_IT(uint8_t deviceIndex, uint8_t *data, uint8_t nBytes)
{
	HAL_SPI_Receive_IT(Device[deviceIndex].SPI_Handle, data, nBytes);
}

static void WriteRegData(uint8_t deviceIndex, uint8_t startAddress, uint8_t *data, uint8_t nBytes)
{
	SetChipSelect(deviceIndex);

	HAL_SPI_Transmit(Device[deviceIndex].SPI_Handle, &startAddress, 1, 10);
	HAL_SPI_Transmit(Device[deviceIndex].SPI_Handle, data, nBytes, 10);

	ClearChipSelect(deviceIndex);
}

static void WriteData_IT(uint8_t deviceIndex, uint8_t *data, uint8_t nBytes)
{
	HAL_SPI_Transmit_IT(Device[deviceIndex].SPI_Handle, data, nBytes);
}

static inline void ClearChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_SetOutputPin(Device[deviceIndex].CS_GPIOx, Device[deviceIndex].csPin);
}

static inline void SetChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_ResetOutputPin(Device[deviceIndex].CS_GPIOx, Device[deviceIndex].csPin);
}


/*******************************************************************************
* END
*******************************************************************************/
