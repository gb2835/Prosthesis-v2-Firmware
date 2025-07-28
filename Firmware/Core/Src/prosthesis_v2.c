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
#include "mpu925x_spi_hal.h"
#include "prosthesis_v2.h"
#include "utilities.h"

#include <stdint.h>
#include <stm32l4xx_ll_adc.h>
#include <string.h>


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define ANKLE_GEAR_RATIO								(90.0f / 15.0f)
#define KNEE_GEAR_RATIO									(70.0f / 16.0f)
#define ANKLE_POSITION_OFFSET_FROM_PLANARFLEXION_BUMPER	31.0f
#define KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER		10.0f
#define DEG_TO_RAD										(3.1416f / 180.0f)
#define RAD_TO_DEG										(180.0f / 3.1416f)

typedef enum
{
	EarlyStance,
	MidStance,
	LateStance,
	SwingFlexion,
	SwingExtension,
	SwingDescension
} StateMachine_e;

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
	float position;
	float speed;
	float torque;
	MPU925x_IMU_Data_t IMU_Data;
	uint8_t motorDataReceived;
} AnkleJoint_t;

typedef struct
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
	float position;
	float speed;
	float torque;
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
	float intoStanceThreshold;
	float outOfStanceThreshold;
} LoadCell_t;

static AKxx_x_ReadData_t MotorRxData[AKXX_X_NUMBER_OF_DEVICES];
static AKxx_x_WriteData_t MotorTxData;
static MPU925x_IMU_Data_t IMU_Data;
static Prosthesis_Init_t Device;

static TestProgram_e testProgram = None;
static uint8_t imuReadStarted = 0;
static uint8_t imuDataReceived = 0;
static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;

static const int8_t state_angles[3][6] = {{-20, -14, -8, -2,  4, 10},	// Ankle only
									 	  {-20,  -4, 12, 28, 44, 60},	// Combined
										  {  0,  12, 24, 36, 48, 60}};	// Knee only

static const int8_t state_torques[3][6] = {{-100, -70, -40, -10, 20, 50},	// Ankle only
										   {-100, -70, -40, -10, 20, 50},	// Combined
										   { -50, -30, -10,  10, 30, 50}};	// Knee only

static const int16_t state_speeds[6] = {-600, -360, -120, 120, 360, 600};

static const uint16_t state_loadCells[6] = {1100, 1200, 1300, 1400, 1500, 1600};

static AnkleJoint_t CM_AnkleJoint;
static int8_t CM_state_angles, CM_state_torques;
static int16_t CM_state_speeds;
static uint16_t CM_state_loadCells;
static KneeJoint_t CM_KneeJoint;
static LoadCell_t CM_LoadCell;

static Error_e CM_ledCode = NoError;
static float CM_footSpeed = 0.0f;
static float CM_footSpeedThreshold = 0.0f;
static float CM_hipAngle = 0.0f;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void RunStateMachine(void);
static void CheckMotorCalls(void);
static void ServiceMotor(DeviceIndex_e deviceIndex);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void InitProsthesisControl(Prosthesis_Init_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(Device));

	memset(&CM_AnkleJoint, 0, sizeof(CM_AnkleJoint));
	memset(&CM_KneeJoint, 0, sizeof(CM_KneeJoint));

	CM_LoadCell.intoStanceThreshold = 1300;
	CM_LoadCell.outOfStanceThreshold = 1270;

	uint32_t txMailbox;
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		MPU925x_SetChipSelect(0);
		MPU925x_StartReadIMU_IT(0);

		if(AKxx_x_EnterMotorCtrlMode(AnkleIndex, &txMailbox))
			ErrorHandler(AnkleMotorError);
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
		if(AKxx_x_EnterMotorCtrlMode(KneeIndex, &txMailbox))
			ErrorHandler(KneeMotorError);
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

	RunStateMachine();
	CheckMotorCalls();

	// Check for first and second executions, needed for load cell filter
	if(isFirst)
	{
		isFirst = 0;
		isSecond = 1;
	}
	else if(isSecond)
		isSecond = 0;
}

void ActivateLED(LED_Color_e color)
{
	if(color == NoColor)
	{
		LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		LL_GPIO_SetOutputPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	}
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
	else if(color == Red)
	{
		LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		LL_GPIO_SetOutputPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	}
	else if(color == White)
	{
		LL_GPIO_ResetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		LL_GPIO_ResetOutputPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	}
}

void ErrorHandler(Error_e error)
{
	ActivateLED(Red);

	HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

	CM_ledCode = error;

	uint32_t txMailbox;
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
		AKxx_x_ExitMotorCtrlMode(AnkleIndex, &txMailbox);
	if((Device.Joint == Knee) || (Device.Joint == Combined))
		AKxx_x_ExitMotorCtrlMode(KneeIndex, &txMailbox);

	while(1);
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
		static uint8_t tempImuData[14];
		if(imuReadStarted)
		{
			imuReadStarted = 0;
			MPU925x_ReadIMU_IT(0, tempImuData);
		}

		static uint8_t missedAnkleImuCalls = 0;
		if(imuDataReceived)
		{
			missedAnkleImuCalls = 0;
			imuDataReceived = 0;
			MPU925x_ClearChipSelect(0);

			MPU925x_SetChipSelect(0);
			MPU925x_StartReadIMU_IT(0);

			IMU_Data = MPU925x_ConvertIMU_Data(tempImuData);

			// Gyro offsets previously found
			IMU_Data.Struct.gx -= 3.1266768292682952;
			IMU_Data.Struct.gy -= 0.59624999999999995;
			IMU_Data.Struct.gz -= -1.578993902439024;
		}
		else
			missedAnkleImuCalls++;

		if(missedAnkleImuCalls >= 5)
			ErrorHandler(AnkleIMU_Error);
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
			CM_AnkleJoint.IMU_Data.Struct.ax = -IMU_Data.Struct.ax;
			CM_AnkleJoint.IMU_Data.Struct.ay = IMU_Data.Struct.ay;
			CM_AnkleJoint.IMU_Data.Struct.az = -IMU_Data.Struct.az;
			CM_AnkleJoint.IMU_Data.Struct.gx = -IMU_Data.Struct.gx;
			CM_AnkleJoint.IMU_Data.Struct.gy = IMU_Data.Struct.gy;
			CM_AnkleJoint.IMU_Data.Struct.gz = -IMU_Data.Struct.gz;
		}
		else if(Device.Side == Right)
			memcpy(&CM_AnkleJoint.IMU_Data, &IMU_Data, sizeof(MPU925x_IMU_Data_t));

		CM_footSpeed = CM_AnkleJoint.speed + CM_AnkleJoint.IMU_Data.Struct.gz;
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
		else if(Device.Side == Right)
			memcpy(&CM_KneeJoint.IMU_Data, &BNO08x_IMU_Data, sizeof(KneeIMU_Data_t));

		float yaw, pitch, roll;
		QuaternionsToYPR(BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9], &yaw, &pitch, &roll);
		CM_KneeJoint.IMU_Data.yaw = yaw * RAD_TO_DEG;
		CM_KneeJoint.IMU_Data.pitch = pitch * RAD_TO_DEG;
		CM_KneeJoint.IMU_Data.roll = roll * RAD_TO_DEG;

		CM_hipAngle = CM_KneeJoint.speed - CM_KneeJoint.IMU_Data.pitch;
	}
}

static void RunStateMachine(void)
{
	static StateMachine_e state = EarlyStance;
	switch(state)
	{
	case EarlyStance:
		CM_state_loadCells = state_loadCells[EarlyStance];
		CM_state_speeds = state_speeds[EarlyStance];

		if(Device.Joint == Ankle)
		{
			CM_state_angles = state_angles[Ankle][EarlyStance];
			CM_state_torques = state_torques[Ankle][EarlyStance];
		}
		if(Device.Joint == Knee)
		{
			CM_state_angles = state_angles[Knee][EarlyStance];
			CM_state_torques = state_torques[Knee][EarlyStance];
		}
		if(Device.Joint == Combined)
		{
			CM_state_angles = state_angles[Combined][EarlyStance];
			CM_state_torques = state_torques[Combined][EarlyStance];
		}

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.EarlyStanceCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.EarlyStanceCtrl.kp;
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.EarlyStanceCtrl.position;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.EarlyStanceCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.EarlyStanceCtrl.kp;
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.EarlyStanceCtrl.position;
			}
		}

//		if(CM_footSpeed > CM_footSpeedThreshold) ??
//			state = MidStance;

		if(CM_AnkleJoint.speed < 0.0f)
			state = LateStance;

		break;

	case MidStance:
		CM_state_loadCells = state_loadCells[MidStance];
		CM_state_speeds = state_speeds[MidStance];

		if(Device.Joint == Ankle)
		{
			CM_state_angles = state_angles[Ankle][MidStance];
			CM_state_torques = state_torques[Ankle][MidStance];
		}
		if(Device.Joint == Knee)
		{
			CM_state_angles = state_angles[Knee][MidStance];
			CM_state_torques = state_torques[Knee][MidStance];
		}
		if(Device.Joint == Combined)
		{
			CM_state_angles = state_angles[Combined][MidStance];
			CM_state_torques = state_torques[Combined][MidStance];
		}

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.MidStanceCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.MidStanceCtrl.kp;
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.MidStanceCtrl.position;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.MidStanceCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.MidStanceCtrl.kp;
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.MidStanceCtrl.position;
			}
		}

		if(CM_AnkleJoint.speed < 0.0f)
			state = LateStance;

		break;

	case LateStance:
		CM_state_loadCells = state_loadCells[LateStance];
		CM_state_speeds = state_speeds[LateStance];

		if(Device.Joint == Ankle)
		{
			CM_state_angles = state_angles[Ankle][LateStance];
			CM_state_torques = state_torques[Ankle][LateStance];
		}
		if(Device.Joint == Knee)
		{
			CM_state_angles = state_angles[Knee][LateStance];
			CM_state_torques = state_torques[Knee][LateStance];
		}
		if(Device.Joint == Combined)
		{
			CM_state_angles = state_angles[Combined][LateStance];
			CM_state_torques = state_torques[Combined][LateStance];
		}

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.LateStanceCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.LateStanceCtrl.kp;
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.LateStanceCtrl.position;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.LateStanceCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.LateStanceCtrl.kp;
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.LateStanceCtrl.position;
			}
		}

		if(CM_LoadCell.Filtered.top[0] < CM_LoadCell.outOfStanceThreshold)
			state = SwingFlexion;

		break;

	case SwingFlexion:
		CM_state_loadCells = state_loadCells[SwingFlexion];
		CM_state_speeds = state_speeds[SwingFlexion];

		if(Device.Joint == Ankle)
		{
			CM_state_angles = state_angles[Ankle][SwingFlexion];
			CM_state_torques = state_torques[Ankle][SwingFlexion];
		}
		if(Device.Joint == Knee)
		{
			CM_state_angles = state_angles[Knee][SwingFlexion];
			CM_state_torques = state_torques[Knee][SwingFlexion];
		}
		if(Device.Joint == Combined)
		{
			CM_state_angles = state_angles[Combined][SwingFlexion];
			CM_state_torques = state_torques[Combined][SwingFlexion];
		}

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.SwingFlexCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.SwingFlexCtrl.kp;
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.SwingFlexCtrl.position;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.SwingFlexCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.SwingFlexCtrl.kp;
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.SwingFlexCtrl.position;
			}
		}

//		if(CM_KneeJoint.speed < 0.0f) ??
//			state = SwingExtension;

		if(CM_LoadCell.Filtered.top[0] > CM_LoadCell.intoStanceThreshold)
			state = EarlyStance;

		break;

	case SwingExtension:
		CM_state_loadCells = state_loadCells[SwingExtension];
		CM_state_speeds = state_speeds[SwingExtension];

		if(Device.Joint == Ankle)
		{
			CM_state_angles = state_angles[Ankle][SwingExtension];
			CM_state_torques = state_torques[Ankle][SwingExtension];
		}
		if(Device.Joint == Knee)
		{
			CM_state_angles = state_angles[Knee][SwingExtension];
			CM_state_torques = state_torques[Knee][SwingExtension];
		}
		if(Device.Joint == Combined)
		{
			CM_state_angles = state_angles[Combined][SwingExtension];
			CM_state_torques = state_torques[Combined][SwingExtension];
		}

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.SwingExtCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.SwingExtCtrl.kp;
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.SwingExtCtrl.position;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.SwingExtCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.SwingExtCtrl.kp;
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.SwingExtCtrl.position;
			}
		}

		if(CM_footSpeed < 0.0f)
			state = SwingDescension;

		break;

	case SwingDescension:
		CM_state_loadCells = state_loadCells[SwingDescension];
		CM_state_speeds = state_speeds[SwingDescension];

		if(Device.Joint == Ankle)
		{
			CM_state_angles = state_angles[Ankle][SwingDescension];
			CM_state_torques = state_torques[Ankle][SwingDescension];
		}
		if(Device.Joint == Knee)
		{
			CM_state_angles = state_angles[Knee][SwingDescension];
			CM_state_torques = state_torques[Knee][SwingDescension];
		}
		if(Device.Joint == Combined)
		{
			CM_state_angles = state_angles[Combined][SwingDescension];
			CM_state_torques = state_torques[Combined][SwingDescension];
		}

		if(testProgram != ImpedanceControl)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.SwingDescCtrl.kd;
				CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.SwingDescCtrl.kp;
				CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.SwingDescCtrl.position;
			}
			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.SwingDescCtrl.kd;
				CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.SwingDescCtrl.kp;
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.SwingDescCtrl.position;
			}
		}

		if(CM_LoadCell.Filtered.top[0] > CM_LoadCell.intoStanceThreshold)
			state = EarlyStance;

		break;
	}
}

static void CheckMotorCalls(void)
{
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
		{
			uint32_t txMailbox;
			AKxx_x_EnterMotorCtrlMode(AnkleIndex, &txMailbox);
				if(missedAnkleMotorCalls >= 10)
					ErrorHandler(AnkleMotorError);
		}
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
		if(MotorRxData[AnkleIndex].error)
			ErrorHandler(AnkleMotorError);

		CM_AnkleJoint.position = -MotorRxData[deviceIndex].position / ANKLE_GEAR_RATIO * RAD_TO_DEG - ANKLE_POSITION_OFFSET_FROM_PLANARFLEXION_BUMPER;
		CM_AnkleJoint.speed = -MotorRxData[deviceIndex].speed / ANKLE_GEAR_RATIO * RAD_TO_DEG;
		CM_AnkleJoint.torque = -MotorRxData[deviceIndex].torque * ANKLE_GEAR_RATIO ;

		uint32_t txMailbox;
		if((testProgram == None) || (testProgram == ImpedanceControl))
		{
			MotorTxData.kd = CM_AnkleJoint.ProsCtrl.kd;
			MotorTxData.kp = CM_AnkleJoint.ProsCtrl.kp;
			MotorTxData.position = (-CM_AnkleJoint.ProsCtrl.position - ANKLE_POSITION_OFFSET_FROM_PLANARFLEXION_BUMPER) * ANKLE_GEAR_RATIO * DEG_TO_RAD;

			if(AKxx_x_WriteMotor(deviceIndex, &MotorTxData, &txMailbox))
				ErrorHandler(AnkleMotorError);
		}
		else
			if(AKxx_x_EnterMotorCtrlMode(deviceIndex, &txMailbox))
				ErrorHandler(AnkleMotorError);
	}
	else if(deviceIndex == KneeIndex)
	{
		if(MotorRxData[KneeIndex].error)
			ErrorHandler(KneeMotorError);

		CM_KneeJoint.position = MotorRxData[deviceIndex].position / KNEE_GEAR_RATIO * RAD_TO_DEG - KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER;
		CM_KneeJoint.speed = MotorRxData[deviceIndex].speed / KNEE_GEAR_RATIO * RAD_TO_DEG;
		CM_KneeJoint.torque = MotorRxData[deviceIndex].torque * KNEE_GEAR_RATIO ;

		uint32_t txMailbox;
		if((testProgram == None) || (testProgram == ImpedanceControl))
		{
			MotorTxData.kd = CM_KneeJoint.ProsCtrl.kd;
			MotorTxData.kp = CM_KneeJoint.ProsCtrl.kp;
			MotorTxData.position = (CM_KneeJoint.ProsCtrl.position - KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER) * KNEE_GEAR_RATIO * DEG_TO_RAD;

			if(AKxx_x_WriteMotor(deviceIndex, &MotorTxData, &txMailbox))
				ErrorHandler(KneeMotorError);
		}
		else
			if(AKxx_x_EnterMotorCtrlMode(deviceIndex, &txMailbox))
				ErrorHandler(KneeMotorError);
	}
}


/*******************************************************************************
* CALLBACKS
*******************************************************************************/

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	imuReadStarted = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	imuDataReceived = 1;
}

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
