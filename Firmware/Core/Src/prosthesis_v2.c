/*******************************************************************************
*
* TITLE: Application for Prosthesis v2
*
* NOTES (check this??)
* 1. Unless otherwise specified, units are
* 		- Accelerometer	= m/s^2
* 		- Angle			= degrees
* 		- Current		= Amperes
* 		- Gyroscope		= degrees/second
* 		- Load Cell		= ADC
* 		- Torque		= N*m
* 		- Speed			= degrees/second
*
*******************************************************************************/

#include "bno08x_spi_hal.h"
#include "prosthesis_v2.h"
#include <stdint.h>
#include <string.h>
#include "stm32l4xx_ll_adc.h"
#include "utilities.h"


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

uint8_t isProsthesisControlRequired = 0;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define RAD_TO_DEG	180 / 3.1416f

typedef union
{
	float array[9];
	struct
	{
		float ax;
		float ay;
		float az;
		float gx;
		float gy;
		float gz;
		float yaw;
		float pitch;
		float roll;
	} Struct;
} IMU_Data_t;

typedef struct
{
	struct
	{
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2
	} Raw;
	struct
	{
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2
	} Filtered;
	float outOfStanceThreshold;
	float intoStanceThreshold;
} LoadCell_t;

static Prosthesis_Init_t Device;

static uint8_t isFirst = 0;
static uint8_t isSecond = 0;

static IMU_Data_t CM_IMU_Data;
static LoadCell_t CM_LoadCell;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void InitProsthesisControl(Prosthesis_Init_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(Device));
}

void RunProsthesisControl(void)
{
	GetInputs();
	ProcessInputs();

	// Check for first and second executions, needed for derivatives, filters, etc.
	if(isFirst)
	{
		isFirst = 0;
		isSecond = 1;
	}
	else if(isSecond)
		isSecond = 0;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void GetInputs(void)
{
	if (BNO08x_resetOccurred)
	{
		BNO08x_resetOccurred = 0;
		if(BNO08x_StartReports())
		{
			// User may add error handling for this if desired
		}
	}

	BNO08x_ReadSensors();

	CM_LoadCell.Raw.bot[0] = ReadLoadCell(ADC1);
	CM_LoadCell.Raw.top[0] = ReadLoadCell(ADC2);
}

static uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while (!LL_ADC_IsActiveFlag_EOC(ADCx));
	uint16_t data = LL_ADC_REG_ReadConversionData12(ADCx);
	return data;
}

static void ProcessInputs(void)
{
	// Get accel and gyro data
	for(uint8_t i = 0; i < 6; i++)
		CM_IMU_Data.array[i] = BNO08x_IMU_Data[i];

	float yaw, pitch, roll;
	QuaternionsToYPR(BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9], &yaw, &pitch, &roll);
	CM_IMU_Data.Struct.yaw = yaw * RAD_TO_DEG;
	CM_IMU_Data.Struct.pitch = pitch * RAD_TO_DEG;
	CM_IMU_Data.Struct.roll = roll * RAD_TO_DEG;

	// Filtering of load cells
	if(isFirst)
	{
		CM_LoadCell.Raw.bot[2] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[2] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[0] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Filtered.top[0] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[2] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[2] = CM_LoadCell.Filtered.top[0];
	}
	else if(isSecond)
	{
		CM_LoadCell.Raw.bot[1] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[1] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[0] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Filtered.top[0] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[1] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[1] = CM_LoadCell.Filtered.top[0];
	}
	else
	{
		// 2nd order low-pass Butterworth (fc = 20 Hz)
		CM_LoadCell.Filtered.bot[0] =   1.6556 * CM_LoadCell.Filtered.bot[1] - 0.7068 * CM_LoadCell.Filtered.bot[2]
									  + 0.0128 * CM_LoadCell.Raw.bot[0] + 0.0256 * CM_LoadCell.Raw.bot[1] + 0.0128 * CM_LoadCell.Raw.bot[2];
		CM_LoadCell.Filtered.top[0] =   1.6556 * CM_LoadCell.Filtered.top[1] - 0.7068 * CM_LoadCell.Filtered.top[2]
									  + 0.0128 * CM_LoadCell.Raw.top[0] + 0.0256 * CM_LoadCell.Raw.top[1] + 0.0128 * CM_LoadCell.Raw.top[2];

		CM_LoadCell.Raw.bot[2] = CM_LoadCell.Raw.bot[1];
		CM_LoadCell.Raw.bot[1] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[2] = CM_LoadCell.Raw.top[1];
		CM_LoadCell.Raw.top[1] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[2] = CM_LoadCell.Filtered.bot[1];
		CM_LoadCell.Filtered.bot[1] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[2] = CM_LoadCell.Filtered.top[1];
		CM_LoadCell.Filtered.top[1] = CM_LoadCell.Filtered.top[0];
	}
}


/*******************************************************************************
* END
*******************************************************************************/
