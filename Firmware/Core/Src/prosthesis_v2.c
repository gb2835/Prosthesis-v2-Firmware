/*******************************************************************************
*
* TITLE: Application for Prosthesis v2
*
* NOTES
* 1. IMPORTANT: Motor position must be re-zeroed whenever the motor is reassembled into the device.
*    A test program is provided to zero the motors. ??
* 2. Unless otherwise specified, units are
* 		- Accelerometer	= m/s^2
* 		- Angle			= degrees
* 		- Gyroscope		= degrees/second
* 		- Load Cell		= ADC
* 		- Torque		= Nm
* 		- Speed			= degrees/second
*
*******************************************************************************/

#include "akxx-x.h"
#include "bno08x_spi_hal.h"
#include "error_handler.h"
#include "main.h"
#include "prosthesis_v2.h"
#include "utilities.h"

#include <stdint.h>
#include <string.h>
#include <stm32l4xx_ll_adc.h>


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

uint8_t isProsthesisControlRequired = 0;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define DEG_TO_RAD	(3.1416f / 180.0f)
#define RAD_TO_DEG	(180.0f / 3.1416f)

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
	AKxx_x_ReadData_t MotorReadData;
	AKxx_x_WriteData_t ProsCtrl;
	AKxx_x_WriteData_t EarlyStanceCtrl;
	AKxx_x_WriteData_t MidStanceCtrl;
	AKxx_x_WriteData_t LateStanceCtrl;
	AKxx_x_WriteData_t SwingFlexCtrl;
	AKxx_x_WriteData_t SwingExtCtrl;
	AKxx_x_WriteData_t SwingDescCtrl;
	uint8_t motorCanId;
	uint8_t motorDataReceived;
} Joint_t;

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

static AKxx_x_ReadData_t RxData_Float[AKXX_X_NUMBER_OF_DEVICES];
static AKxx_x_WriteData_t TxData_Float;
static Prosthesis_Init_t Device;
static TestProgram_e testProgram;

static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;

static IMU_Data_t CM_Ankle_IMU_Data;
static Joint_t CM_Ankle;
static LoadCell_t CM_LoadCell;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void RunStateMachine(void);
static void ServiceMotor(DeviceIndex_e deviceIndex);
static void RunTestProgram(void);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void InitProsthesisControl(Prosthesis_Init_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(Device));

	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		memset(&CM_Ankle, 0, sizeof(CM_Ankle));

		float startPosition = 0.0f;
		float startKd = 0.0f;
		float startKp = 0.0f;

		CM_Ankle.motorCanId = 1;

		CM_Ankle.EarlyStanceCtrl.position = startPosition;
		CM_Ankle.EarlyStanceCtrl.kd = startKd;
		CM_Ankle.EarlyStanceCtrl.kp = startKp;

		CM_Ankle.MidStanceCtrl.position = startPosition;
		CM_Ankle.MidStanceCtrl.kd = startKd;
		CM_Ankle.MidStanceCtrl.kp = startKp;

		CM_Ankle.LateStanceCtrl.position = startPosition;
		CM_Ankle.LateStanceCtrl.kd = startKd;
		CM_Ankle.LateStanceCtrl.kp = startKp;

		CM_Ankle.SwingFlexCtrl.position = startPosition;
		CM_Ankle.SwingFlexCtrl.kd = startKd;
		CM_Ankle.SwingFlexCtrl.kp = startKp;

		CM_Ankle.SwingExtCtrl.position = startPosition;
		CM_Ankle.SwingExtCtrl.kd = startKd;
		CM_Ankle.SwingExtCtrl.kp = startKp;

		CM_Ankle.SwingDescCtrl.position = startPosition;
		CM_Ankle.SwingDescCtrl.kd = startKd;
		CM_Ankle.SwingDescCtrl.kp = startKp;
	}

	CM_LoadCell.intoStanceThreshold = 1300; //??
	CM_LoadCell.outOfStanceThreshold = 1300 + 50; //??


	if(testProgram != ZeroMotorPosition)
	{
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
			if(AKxx_x_EnterMotorCtrlMode(AnkleIndex))
				ErrorHandler_AKxx_x(AnkleIndex);
	}

	// setup led pins??
}

void RequireTestProgram(TestProgram_e option)
{
	testProgram = option;
	if(testProgram != None)
		isTestProgramRequired = 1;
}

void RunProsthesisControl(void)
{
	GetInputs();
	ProcessInputs();

	if(isTestProgramRequired)
		RunTestProgram();

	RunStateMachine();

	if((Device.Joint == Ankle) || (Device.Joint == Combined))
		if(CM_Ankle.motorDataReceived)
			ServiceMotor(AnkleIndex);

	// Check for first and second executions, needed for load cell filter
	if(isFirst)
	{
		isFirst = 0;
		isSecond = 1;
	}
	else if(isSecond)
		isSecond = 0;
}

void ShutdownMotors(void)
{
	while(1)
	{
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
			AKxx_x_ExitMotorCtrlMode(AnkleIndex);
	}
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void GetInputs(void)
{
	CM_LoadCell.Raw.bot[0] = ReadLoadCell(ADC1);
	CM_LoadCell.Raw.top[0] = ReadLoadCell(ADC2);

	if(BNO08x_resetOccurred)
	{
		BNO08x_resetOccurred = 0;
		if(BNO08x_StartReports())
			ErrorHandler_BNO08x();
	}

	BNO08x_ReadSensors();
}

static uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while (!LL_ADC_IsActiveFlag_EOC(ADCx));
	return LL_ADC_REG_ReadConversionData12(ADCx);
}

static void ProcessInputs(void)
{
	// Filter of load cells
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
		// 2nd order low-pass Butterworth (fc = 20 Hz, fs = 500 Hz)
		CM_LoadCell.Filtered.bot[0] =   1.6475 * CM_LoadCell.Filtered.bot[1] - 0.7009 * CM_LoadCell.Filtered.bot[2]
									  + 0.0134 * CM_LoadCell.Raw.bot[0] + 0.0267 * CM_LoadCell.Raw.bot[1] + 0.0134 * CM_LoadCell.Raw.bot[2];
		CM_LoadCell.Filtered.top[0] =   1.6475 * CM_LoadCell.Filtered.top[1] - 0.7009 * CM_LoadCell.Filtered.top[2]
									  + 0.0134 * CM_LoadCell.Raw.top[0] + 0.0267 * CM_LoadCell.Raw.top[1] + 0.0134 * CM_LoadCell.Raw.top[2];

		CM_LoadCell.Raw.bot[2] = CM_LoadCell.Raw.bot[1];
		CM_LoadCell.Raw.bot[1] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[2] = CM_LoadCell.Raw.top[1];
		CM_LoadCell.Raw.top[1] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[2] = CM_LoadCell.Filtered.bot[1];
		CM_LoadCell.Filtered.bot[1] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[2] = CM_LoadCell.Filtered.top[1];
		CM_LoadCell.Filtered.top[1] = CM_LoadCell.Filtered.top[0];
	}

	// Get accelerometer and gyroscope data
	for(uint8_t i = 0; i < 6; i++)
		CM_Ankle_IMU_Data.array[i] = BNO08x_IMU_Data[i];

	float yaw, pitch, roll;
	QuaternionsToYPR(BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9], &yaw, &pitch, &roll);
	CM_Ankle_IMU_Data.Struct.yaw = yaw * RAD_TO_DEG;
	CM_Ankle_IMU_Data.Struct.pitch = pitch * RAD_TO_DEG;
	CM_Ankle_IMU_Data.Struct.roll = roll * RAD_TO_DEG;
}

static void RunStateMachine(void)
{

}

static void ServiceMotor(DeviceIndex_e deviceIndex)
{
	CM_Ankle.MotorReadData.position = RxData_Float[deviceIndex].position * RAD_TO_DEG;
	CM_Ankle.MotorReadData.speed = RxData_Float[deviceIndex].speed * RAD_TO_DEG;
	CM_Ankle.MotorReadData.torque = RxData_Float[deviceIndex].torque;

	if((testProgram == None) || (testProgram == ImpedanceControl))
	{
		memcpy(&TxData_Float, &CM_Ankle.ProsCtrl, sizeof(AKxx_x_WriteData_t));

		TxData_Float.position = CM_Ankle.ProsCtrl.position * DEG_TO_RAD;
		if(AKxx_x_WriteMotor(deviceIndex, &TxData_Float))
			ErrorHandler_AKxx_x(deviceIndex);
	}
	else
		if(AKxx_x_EnterMotorCtrlMode(deviceIndex))
			ErrorHandler_AKxx_x(deviceIndex);
}

static void RunTestProgram(void)
{
	switch(testProgram)
	{
	case None:
		break;

	case ReadOnly:
		break;

	case ZeroMotorPosition:
		if(isFirst)
		{
			if(HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
				ErrorHandler_Pv2(CAN_Error);

			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				if(AKxx_x_ZeroMotorPosition(AnkleIndex))
					ErrorHandler_AKxx_x(AnkleIndex);
				if(AKxx_x_PollMotorReadWithTimeout(&RxData_Float[AnkleIndex]))
					ErrorHandler_AKxx_x(AnkleIndex);
			}

			if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
				ErrorHandler_Pv2(CAN_Error);

			if((Device.Joint == Ankle) || (Device.Joint == Combined))
				if(AKxx_x_EnterMotorCtrlMode(AnkleIndex))
					ErrorHandler_AKxx_x(AnkleIndex);
		}

		break;

	case ImpedanceControl:
		break;
	}
}


/*******************************************************************************
* CALLBACKS
*******************************************************************************/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	AKxx_x_ReadData_t temp;
	if(AKxx_x_ReadMotor(CAN_RX_FIFO0, &temp))
		ErrorHandler_Pv2(MotorError);

	if(temp.canId == 1)
	{
		CM_Ankle.motorDataReceived = 1;
		memcpy(&RxData_Float[0], &temp, sizeof(AKxx_x_ReadData_t));
	}
	else
		ErrorHandler_Pv2(MotorError);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	AKxx_x_ReadData_t temp;
	if(AKxx_x_ReadMotor(CAN_RX_FIFO1, &temp))
		ErrorHandler_Pv2(MotorError);

	if(temp.canId == 1)
	{
		CM_Ankle.motorDataReceived = 1;
		memcpy(&RxData_Float[0], &temp, sizeof(AKxx_x_ReadData_t));
	}
	else
		ErrorHandler_Pv2(MotorError);
}


/*******************************************************************************
* END
*******************************************************************************/
