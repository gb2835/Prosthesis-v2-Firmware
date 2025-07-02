/*******************************************************************************
*
* TITLE: Application for Prosthesis v2
*
* NOTES
* 1. Unless otherwise specified, units are
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
#include "main.h"
#include "mpu925x_spi.h"
#include "prosthesis_v2.h"
#include "utilities.h"

#include <stdint.h>
#include <stm32l4xx_ll_adc.h>
#include <string.h>


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

uint8_t isProsthesisControlRequired = 0;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define ANKLE_GEAR_RATIO	1.0f / 1.0f // 90.0f / 15.0f??
#define KNEE_GEAR_RATIO		1.0f / 1.0f // 70.0f / 16.0f??
#define DEG_TO_RAD			3.1416f / 180.0f
#define RAD_TO_DEG			180.0f / 3.1416f

typedef enum
{
	Blue,
	Green,
	Red
} LED_Color_e;

typedef enum
{
	EarlyStance,
	MidStance,
	LateStance,
	SwingFlexion,
	SwingExtension,
	SwingDescension
} StateMachine_e;

typedef union
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
} AnkleIMU_Data_t;

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
	AnkleIMU_Data_t IMU_Data;
	uint8_t motorDataReceived;
} AnkleJoint_t;

typedef union
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
} KneeIMU_Data_t;

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
	KneeIMU_Data_t IMU_Data;
	uint8_t motorDataReceived;
} KneeJoint_t;

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

static AKxx_x_ReadData_t MotorRxData[AKXX_X_NUMBER_OF_DEVICES];
static AKxx_x_WriteData_t MotorTxData;
static Prosthesis_Init_t Device;

static TestProgram_e testProgram = None;
static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;

static AnkleJoint_t CM_AnkleJoint;
static float CM_footSpeedThreshold;
static int8_t CM_state_angles, CM_state_torques;
static int16_t CM_state_speeds;
static KneeJoint_t CM_KneeJoint;
static LoadCell_t CM_LoadCell;
static uint16_t CM_state_loadCells;

static Error_e CM_ledCode = NoError;
static float CM_footSpeed = 0.0f;
static float CM_hipAngle = 0.0f;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void RunStateMachine(void);
static void ServiceMotor(DeviceIndex_e deviceIndex);
static void RunTestProgram(void);
static void ActivateLED(LED_Color_e color);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void InitProsthesisControl(Prosthesis_Init_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(Device));

	memset(&CM_AnkleJoint, 0, sizeof(CM_AnkleJoint));
	memset(&CM_KneeJoint, 0, sizeof(CM_KneeJoint));

	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		float startPosition = 0.0f;
		float startKd = 0.0f;
		float startKp = 0.0f;

		CM_AnkleJoint.EarlyStanceCtrl.position = startPosition;
		CM_AnkleJoint.EarlyStanceCtrl.kd = startKd;
		CM_AnkleJoint.EarlyStanceCtrl.kp = startKp;

		CM_AnkleJoint.MidStanceCtrl.position = startPosition;
		CM_AnkleJoint.MidStanceCtrl.kd = startKd;
		CM_AnkleJoint.MidStanceCtrl.kp = startKp;

		CM_AnkleJoint.LateStanceCtrl.position = startPosition;
		CM_AnkleJoint.LateStanceCtrl.kd = startKd;
		CM_AnkleJoint.LateStanceCtrl.kp = startKp;

		CM_AnkleJoint.SwingFlexCtrl.position = startPosition;
		CM_AnkleJoint.SwingFlexCtrl.kd = startKd;
		CM_AnkleJoint.SwingFlexCtrl.kp = startKp;

		CM_AnkleJoint.SwingExtCtrl.position = startPosition;
		CM_AnkleJoint.SwingExtCtrl.kd = startKd;
		CM_AnkleJoint.SwingExtCtrl.kp = startKp;

		CM_AnkleJoint.SwingDescCtrl.position = startPosition;
		CM_AnkleJoint.SwingDescCtrl.kd = startKd;
		CM_AnkleJoint.SwingDescCtrl.kp = startKp;
	}
	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		float startPosition = 0.0f;
		float startKd = 0.0f;
		float startKp = 0.0f;

		CM_KneeJoint.EarlyStanceCtrl.position = startPosition;
		CM_KneeJoint.EarlyStanceCtrl.kd = startKd;
		CM_KneeJoint.EarlyStanceCtrl.kp = startKp;

		CM_KneeJoint.MidStanceCtrl.position = startPosition;
		CM_KneeJoint.MidStanceCtrl.kd = startKd;
		CM_KneeJoint.MidStanceCtrl.kp = startKp;

		CM_KneeJoint.LateStanceCtrl.position = startPosition;
		CM_KneeJoint.LateStanceCtrl.kd = startKd;
		CM_KneeJoint.LateStanceCtrl.kp = startKp;

		CM_KneeJoint.SwingFlexCtrl.position = startPosition;
		CM_KneeJoint.SwingFlexCtrl.kd = startKd;
		CM_KneeJoint.SwingFlexCtrl.kp = startKp;

		CM_KneeJoint.SwingExtCtrl.position = startPosition;
		CM_KneeJoint.SwingExtCtrl.kd = startKd;
		CM_KneeJoint.SwingExtCtrl.kp = startKp;

		CM_KneeJoint.SwingDescCtrl.position = startPosition;
		CM_KneeJoint.SwingDescCtrl.kd = startKd;
		CM_KneeJoint.SwingDescCtrl.kp = startKp;
	}

	CM_LoadCell.intoStanceThreshold = 1300; //??
	CM_LoadCell.outOfStanceThreshold = 1300 + 50; //??

	CM_footSpeedThreshold = 0.0f;

	if(testProgram != ZeroMotorPosition)
	{
		uint32_t txMailbox;
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
			if(AKxx_x_EnterMotorCtrlMode(AnkleIndex, &txMailbox))
				ErrorHandler(AnkleMotorError);

		if((Device.Joint == Knee) || (Device.Joint == Combined))
			if(AKxx_x_EnterMotorCtrlMode(KneeIndex, &txMailbox))
				ErrorHandler(KneeMotorError);
	}

	ActivateLED(Blue);
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
	{
		static uint8_t missedAnkleMotorCalls = 0;
		if(CM_AnkleJoint.motorDataReceived)
		{
			missedAnkleMotorCalls = 0;
			CM_AnkleJoint.motorDataReceived = 0;
			ServiceMotor(AnkleIndex);
		}
		else
			missedAnkleMotorCalls++;

		if(missedAnkleMotorCalls >= 5)
			ErrorHandler(AnkleMotorError);
	}
	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		static uint8_t missedKneeMotorCalls = 0;
		if(CM_KneeJoint.motorDataReceived)
		{
			missedKneeMotorCalls = 0;
			CM_KneeJoint.motorDataReceived = 0;
			ServiceMotor(KneeIndex);
		}
		else
			missedKneeMotorCalls++;

		if(missedKneeMotorCalls >= 5)
			ErrorHandler(KneeMotorError);
	}

	// Check for first and second executions, needed for load cell filter
	if(isFirst)
	{
		isFirst = 0;
		isSecond = 1;
	}
	else if(isSecond)
		isSecond = 0;
}

void ErrorHandler(Error_e error)
{
	ActivateLED(Red);

	HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

	CM_ledCode = error;

	while(1)
	{
		uint32_t txMailbox;
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
			AKxx_x_ExitMotorCtrlMode(AnkleIndex, &txMailbox);
		if((Device.Joint == Knee) || (Device.Joint == Combined))
			AKxx_x_ExitMotorCtrlMode(KneeIndex, &txMailbox);
	}
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void GetInputs(void)
{
	CM_LoadCell.Raw.bot[0] = ReadLoadCell(ADC1);
	CM_LoadCell.Raw.top[0] = ReadLoadCell(ADC2);

	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		if(BNO08x_resetOccurred) //move to knee??
		{
			BNO08x_resetOccurred = 0;
			if(BNO08x_StartReports())
				ErrorHandler(AnkleIMU_Error);
		}

		BNO08x_ReadSensors();
	}
	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		if(BNO08x_resetOccurred)
		{
			BNO08x_resetOccurred = 0;
			if(BNO08x_StartReports())
				ErrorHandler(KneeIMU_Error);
		}

		BNO08x_ReadSensors();
	}
}

static uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while (!LL_ADC_IsActiveFlag_EOC(ADCx));
	return LL_ADC_REG_ReadConversionData12(ADCx);
}

static void ProcessInputs(void)
{
	// Filter load cells
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

	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		if(Device.Side == Left)
		{
			CM_AnkleJoint.IMU_Data.ax = -BNO08x_IMU_Data[0];
			CM_AnkleJoint.IMU_Data.ay = BNO08x_IMU_Data[1];
			CM_AnkleJoint.IMU_Data.az = -BNO08x_IMU_Data[2];
			CM_AnkleJoint.IMU_Data.gx = -BNO08x_IMU_Data[3] * RAD_TO_DEG;
			CM_AnkleJoint.IMU_Data.gy = BNO08x_IMU_Data[4] * RAD_TO_DEG;
			CM_AnkleJoint.IMU_Data.gz = -BNO08x_IMU_Data[5] * RAD_TO_DEG;
		}
		else
			memcpy(&CM_AnkleJoint.IMU_Data, &BNO08x_IMU_Data, sizeof(AnkleIMU_Data_t));

		float yaw, pitch, roll;
		QuaternionsToYPR(BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9], &yaw, &pitch, &roll);
		CM_AnkleJoint.IMU_Data.yaw = yaw * RAD_TO_DEG;
		CM_AnkleJoint.IMU_Data.pitch = pitch * RAD_TO_DEG;
		CM_AnkleJoint.IMU_Data.roll = roll * RAD_TO_DEG;

		CM_footSpeed = CM_AnkleJoint.MotorReadData.speed + CM_AnkleJoint.IMU_Data.gz;
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		if(Device.Side == Left)
		{
			CM_KneeJoint.IMU_Data.ax = -BNO08x_IMU_Data[0];
			CM_KneeJoint.IMU_Data.ay = BNO08x_IMU_Data[1];
			CM_KneeJoint.IMU_Data.az = -BNO08x_IMU_Data[2];
			CM_KneeJoint.IMU_Data.gx = -BNO08x_IMU_Data[3] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gy = BNO08x_IMU_Data[4] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gz = -BNO08x_IMU_Data[5] * RAD_TO_DEG;
		}
		else
			memcpy(&CM_KneeJoint.IMU_Data, &BNO08x_IMU_Data, sizeof(KneeIMU_Data_t));

		float yaw, pitch, roll;
		QuaternionsToYPR(BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9], &yaw, &pitch, &roll);
		CM_KneeJoint.IMU_Data.yaw = yaw * RAD_TO_DEG;
		CM_KneeJoint.IMU_Data.pitch = pitch * RAD_TO_DEG;
		CM_KneeJoint.IMU_Data.roll = roll * RAD_TO_DEG;

		CM_hipAngle = CM_KneeJoint.MotorReadData.speed - CM_KneeJoint.IMU_Data.pitch;
	}
}

static void RunStateMachine(void)
{
	static StateMachine_e state = EarlyStance;
	switch(state)
	{
	case EarlyStance:
		CM_state_angles = -10;
		CM_state_loadCells = 1100; //??
		CM_state_torques = -30;
		CM_state_speeds = -200;

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.EarlyStanceCtrl.position;
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.EarlyStanceCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.EarlyStanceCtrl.kp;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.EarlyStanceCtrl.position;
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.EarlyStanceCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.EarlyStanceCtrl.kp;
			}
		}

		if(CM_footSpeed > CM_footSpeedThreshold)
			state = MidStance;

		break;

	case MidStance:
		CM_state_angles = 5;
		CM_state_loadCells = 1200;
		CM_state_torques = -20;
		CM_state_speeds = -120;

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.MidStanceCtrl.position;
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.MidStanceCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.MidStanceCtrl.kp;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.MidStanceCtrl.position;
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.MidStanceCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.MidStanceCtrl.kp;
			}
		}

		if(CM_AnkleJoint.MotorReadData.speed < 0)
			state = LateStance;

		break;

	case LateStance:
		CM_state_angles = 20;
		CM_state_loadCells = 1300;
		CM_state_torques = -10;
		CM_state_speeds = -40;

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.LateStanceCtrl.position;
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.LateStanceCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.LateStanceCtrl.kp;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.LateStanceCtrl.position;
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.LateStanceCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.LateStanceCtrl.kp;
			}
		}

		if(CM_LoadCell.Filtered.bot[0] > CM_LoadCell.outOfStanceThreshold)
			state = SwingExtension; //SwingFlexion;??

		break;

	case SwingFlexion:
		CM_state_angles = 35;
		CM_state_loadCells = 1400;
		CM_state_torques = 0;
		CM_state_speeds = 40;

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.SwingFlexCtrl.position;
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.SwingFlexCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.SwingFlexCtrl.kp;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.SwingFlexCtrl.position;
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.SwingFlexCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.SwingFlexCtrl.kp;
			}
		}

		if(CM_KneeJoint.MotorReadData.speed < 0)
			state = SwingExtension;

		break;

	case SwingExtension:
		CM_state_angles = 50;
		CM_state_loadCells = 1500;
		CM_state_torques = 10;
		CM_state_speeds = 120;

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.SwingExtCtrl.position;
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.SwingExtCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.SwingExtCtrl.kp;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.SwingExtCtrl.position;
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.SwingExtCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.SwingExtCtrl.kp;
			}
		}

		if(CM_footSpeed < 0)
			state = SwingDescension;

		break;

	case SwingDescension:
		CM_state_angles = 65;
		CM_state_loadCells = 1600;
		CM_state_torques = 20;
		CM_state_speeds = 200;

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.SwingDescCtrl.position;
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.SwingDescCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.SwingDescCtrl.kp;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.SwingDescCtrl.position;
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.SwingDescCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.SwingDescCtrl.kp;
			}
		}

		if(CM_LoadCell.Filtered.bot[0] < CM_LoadCell.intoStanceThreshold)
			state = EarlyStance;

		break;
	}
}

static void ServiceMotor(DeviceIndex_e deviceIndex)
{
	static uint8_t firstCall = 1;
	if(firstCall)
	{
		firstCall = 0;
		ActivateLED(Green);
	}

	if(deviceIndex == AnkleIndex)
	{
		if(CM_AnkleJoint.MotorReadData.error)
			ErrorHandler(AnkleMotorError);

		CM_AnkleJoint.MotorReadData.position = -MotorRxData[deviceIndex].position / ANKLE_GEAR_RATIO * RAD_TO_DEG;
		CM_AnkleJoint.MotorReadData.speed = -MotorRxData[deviceIndex].speed / ANKLE_GEAR_RATIO * RAD_TO_DEG;
		CM_AnkleJoint.MotorReadData.torque = -MotorRxData[deviceIndex].torque * ANKLE_GEAR_RATIO ;

		uint32_t txMailbox;
		if((testProgram == None) || (testProgram == ImpedanceControl))
		{
			MotorTxData.position = -CM_AnkleJoint.ProsCtrl.position * ANKLE_GEAR_RATIO * DEG_TO_RAD;
			MotorTxData.kd = CM_AnkleJoint.ProsCtrl.kd;
			MotorTxData.kp = CM_AnkleJoint.ProsCtrl.kp;

			if(AKxx_x_WriteMotor(deviceIndex, &MotorTxData, &txMailbox))
				ErrorHandler(AnkleMotorError);
		}
		else
			if(AKxx_x_EnterMotorCtrlMode(deviceIndex, &txMailbox))
				ErrorHandler(AnkleMotorError);
	}
	if(deviceIndex == KneeIndex)
	{
		if(CM_KneeJoint.MotorReadData.error)
			ErrorHandler(KneeMotorError);

		CM_KneeJoint.MotorReadData.position = MotorRxData[deviceIndex].position / KNEE_GEAR_RATIO * RAD_TO_DEG;
		CM_KneeJoint.MotorReadData.speed = MotorRxData[deviceIndex].speed / KNEE_GEAR_RATIO * RAD_TO_DEG;
		CM_KneeJoint.MotorReadData.torque = MotorRxData[deviceIndex].torque * KNEE_GEAR_RATIO ;

		uint32_t txMailbox;
		if((testProgram == None) || (testProgram == ImpedanceControl))
		{
			MotorTxData.position = CM_KneeJoint.ProsCtrl.position * KNEE_GEAR_RATIO * DEG_TO_RAD;
			MotorTxData.kd = CM_KneeJoint.ProsCtrl.kd;
			MotorTxData.kp = CM_KneeJoint.ProsCtrl.kp;

			if(AKxx_x_WriteMotor(deviceIndex, &MotorTxData, &txMailbox))
				ErrorHandler(KneeMotorError);
		}
		else
			if(AKxx_x_EnterMotorCtrlMode(deviceIndex, &txMailbox))
				ErrorHandler(KneeMotorError);
	}
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
				ErrorHandler(CAN_Error);

			uint32_t txMailbox;
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				if(AKxx_x_ZeroMotorPosition(AnkleIndex, &txMailbox))
					ErrorHandler(AnkleMotorError);
				if(AKxx_x_PollMotorReadWith10msTimeout(&MotorRxData[AnkleIndex]))
					ErrorHandler(AnkleMotorError);
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				if(AKxx_x_ZeroMotorPosition(KneeIndex, &txMailbox))
					ErrorHandler(KneeMotorError);
				if(AKxx_x_PollMotorReadWith10msTimeout(&MotorRxData[KneeIndex]))
					ErrorHandler(KneeMotorError);
			}

			if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
				ErrorHandler(CAN_Error);

			if((Device.Joint == Ankle) || (Device.Joint == Combined))
				if(AKxx_x_EnterMotorCtrlMode(AnkleIndex, &txMailbox))
					ErrorHandler(AnkleMotorError);

			if((Device.Joint == Knee) || (Device.Joint == Combined))
				if(AKxx_x_EnterMotorCtrlMode(KneeIndex, &txMailbox))
					ErrorHandler(KneeMotorError);
		}

		break;

	case ImpedanceControl:
		break;
	}
}

static void ActivateLED(LED_Color_e color)
{
	if(color == Blue)
	{
		LL_GPIO_ResetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		LL_GPIO_SetOutputPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	}
	else if(color == Green)
	{
		LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		LL_GPIO_ResetOutputPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	}
	else
	{
		LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		LL_GPIO_SetOutputPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	}
}


/*******************************************************************************
* CALLBACKS
*******************************************************************************/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	AKxx_x_ReadData_t temp;
	if(AKxx_x_ReadMotor(CAN_RX_FIFO0, &temp))
		ErrorHandler(MotorReadError);

	CM_AnkleJoint.motorDataReceived = 1;
	memcpy(&MotorRxData[AnkleIndex], &temp, sizeof(AKxx_x_ReadData_t));
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	AKxx_x_ReadData_t temp;
	if(AKxx_x_ReadMotor(CAN_RX_FIFO1, &temp))
		ErrorHandler(MotorReadError);

	CM_KneeJoint.motorDataReceived = 1;
	memcpy(&MotorRxData[KneeIndex], &temp, sizeof(AKxx_x_ReadData_t));
}


/*******************************************************************************
* END
*******************************************************************************/
