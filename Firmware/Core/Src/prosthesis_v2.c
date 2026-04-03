/*******************************************************************************
*
* TITLE: Application for Prosthesis v2
*
* NOTES
* 1. Unless otherwise specified, units are
* 		- Accelerometer	= m/s^2
* 		- Angle			= °
* 		- Gyroscope		= °/s
* 		- Load Cell		= ADC
* 		- Torque		= N·m
* 		- Speed			= °/s
* 2. Some variables are scaled to 9 to fill the plots better in CubeMonitor.
*
*******************************************************************************/

#include "akxx-x.h"
#include "bno08x_spi_hal.h"
#include "main.h"
#include "mpu925x_spi_hal.h"
#include "prosthesis_v2.h"
#include "utilities.h"
#include "winter_bio_data.h"

#include <math.h>
#include <stdint.h>
#include <stm32l4xx_ll_adc.h>
#include <string.h>


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

TestProgram_e testProgram = ReadOnly;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define ANKLE_GEAR_RATIO								(90.0f / 15.0f)
#define ANKLE_POSITION_OFFSET_FROM_PLANARFLEXION_BUMPER	31.0f
#define DEG_TO_RAD										(M_PI / 180.0f)
#define DT												(1 / 500.0)
#define KNEE_GEAR_RATIO									(70.0f / 16.0f)
#define KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER		10.0f
#define RAD_TO_DEG										(180.0f / M_PI)

typedef enum
{
	EarlyStance,
	MidStance,
	LateStance,
	SwingFlexion,
	SwingExtension,
	CPC
} StateMachine_e;

typedef enum
{
	StateVals,
	CtrlParams
} StateMachineMethod_e;

typedef struct
{
	AKxx_x_ReadData_t MotorReadData;
	AKxx_x_WriteData_t ProsCtrl;
	AKxx_x_WriteData_t BypassStateMachineCtrl;
	AKxx_x_WriteData_t EarlyStanceCtrl;
	AKxx_x_WriteData_t MidStanceCtrl;
	AKxx_x_WriteData_t LateStanceCtrl;
	AKxx_x_WriteData_t SwingFlexCtrl;
	AKxx_x_WriteData_t SwingExtCtrl;
	AKxx_x_WriteData_t CPC_Ctrl;
	AKxx_x_WriteData_t PassEmulCtrl;
	float position;
	float speed;
	float torque;
	MPU925x_IMU_Data_t IMU_Data;
	uint8_t motorDataReceived;
} AnkleJoint_t;

typedef struct
{
	float CPV_1;
	float CPV_2;
	float CPV_3;
	float P_1;
	float P_2;
	float P_3;
	float P_4;
} CPC_Params_t;

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
	AKxx_x_WriteData_t BypassStateMachineCtrl;
	AKxx_x_WriteData_t EarlyStanceCtrl;
	AKxx_x_WriteData_t MidStanceCtrl;
	AKxx_x_WriteData_t LateStanceCtrl;
	AKxx_x_WriteData_t SwingFlexCtrl;
	AKxx_x_WriteData_t SwingExtCtrl;
	AKxx_x_WriteData_t CPC_Ctrl;
	AKxx_x_WriteData_t PassEmulExtCtrl;
	AKxx_x_WriteData_t PassEmulFlexCtrl;
	AKxx_x_WriteData_t PassEmulStanceCtrl;
	CPC_Params_t CPC_Params;
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
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2 where k is the current time step
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2 where k is the current time step
	} Raw;

	struct
	{
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2 where k is the current time step
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2 where k is the current time step
	} Filtered;
} LoadCell_t;

static AKxx_x_WriteData_t MotorTxData;
static float kneeAngleAtHeelStrike;
static float state_angle[3][6];			// 3 joint options, 6 states
static float state_torque[3][6];		// 3 joint options, 6 states
static float state_speed[3][6];			// 3 joint options, 6 states
static float state_loadCell[6];			// 6 states
static MPU925x_IMU_Data_t IMU_Data;
static Prosthesis_Init_t Device;

static uint8_t heelStrike = 0;
static uint8_t ankleImuTxCplt = 0;
static uint8_t ankleImuRxCplt = 0;
static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;
static uint8_t toeOff = 0;

static AnkleJoint_t CM_AnkleJoint;
static double CM_thighAngle[2];														// [0] = k-0, [1] = k-1 where k is the current time step
static float CM_cpvx9;
static int8_t CM_state_quadrant;
static float CM_trajectory;
static float CM_xPhaseAngle, CM_yPhaseAngle;
static float CM_state_angle, CM_state_torque, CM_state_speed, CM_state_loadCell;
static KneeJoint_t CM_KneeJoint;
static LoadCell_t CM_LoadCell;

static double CM_thighAngle_unbiased[2] = {0.0, 0.0};	// [0] = k-0, [1] = k-1 where k is the current time step
static double CM_thighIntegral_unbiased = 0.0;
static Error_e CM_ledCode = NoError;
static float CM_cpv = 0.0f;
static float CM_footSpeed = 0.0f;
static float CM_threshold_ankleSpeed = -5.0f;
static float CM_threshold_footSpeed = -5.0f;
static float CM_threshold_intoStanceLC = 1270.0f;
static float CM_threshold_intoSwingLC = 1270.0f;
static uint8_t CM__startCPC = 0;
static uint8_t CM__startProgram = 0;
static uint8_t CM_healthyStride = 0;

static void InitStateVals(void);
static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static StateMachine_e RunStateMachine(StateMachineMethod_e method);
static void GetCPV(void);
static void GetSegmentConstants(float *cpv, float *a1, float *a2, float *a3);
static void GetThirdOrderSegmentConstants(float cpv_1, float cpv_2, float p_1, float p_2, float v_1, float v_2, float *a);
static void GetSecondOrderSegmentConstants(float cpv_1, float cpv_2, float p_1, float p_2, float v_1, float *a);
static void GetTrajectory(float *cpv, float *a1, float *a2, float *a3);
static void SetStateVals(Joint_e joint, StateMachine_e state);
static void SetCtrlParams(Joint_e joint, StateMachine_e state, AKxx_x_WriteData_t *AnkleMotorWriteData, AKxx_x_WriteData_t *KneeMotorWriteData);
static void RunPassiveEmulation(void);
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

	InitStateVals();
	CM_state_angle = state_angle[Device.Joint][EarlyStance];
	CM_state_speed = state_speed[Device.Joint][EarlyStance];
	CM_state_torque = state_torque[Device.Joint][EarlyStance];
	CM_state_loadCell = state_loadCell[EarlyStance];

	HAL_NVIC_EnableIRQ(SPI1_IRQn);

	uint32_t txMailbox;
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		CM_AnkleJoint.EarlyStanceCtrl.kd = 0.05f;
		CM_AnkleJoint.EarlyStanceCtrl.kp = 2.0f;
		CM_AnkleJoint.EarlyStanceCtrl.position = -5.0f;

		CM_AnkleJoint.MidStanceCtrl.kd = 0.05f;
		CM_AnkleJoint.MidStanceCtrl.kp = 2.0f;
		CM_AnkleJoint.MidStanceCtrl.position = -5.0f;

		CM_AnkleJoint.LateStanceCtrl.kd = 0.05f;
		CM_AnkleJoint.LateStanceCtrl.kp = 2.0f;
		CM_AnkleJoint.LateStanceCtrl.position = -5.0f;

		CM_AnkleJoint.SwingFlexCtrl.kd = 0.05f;
		CM_AnkleJoint.SwingFlexCtrl.kp = 2.0f;
		CM_AnkleJoint.SwingFlexCtrl.position = -5.0f;

		CM_AnkleJoint.SwingExtCtrl.kd = 0.05f;
		CM_AnkleJoint.SwingExtCtrl.kp = 2.0f;
		CM_AnkleJoint.SwingExtCtrl.position = -5.0f;

		MPU925x_SetChipSelect(0);
		MPU925x_StartReadIMU_IT(0);

		if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
			ErrorHandler(CAN_Error);
		if(AKxx_x_EnterMotorCtrlMode(AnkleIndex, &txMailbox))
			ErrorHandler(AnkleMotorError);
	}
	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		CM_KneeJoint.EarlyStanceCtrl.kd = 0.05f;
		CM_KneeJoint.EarlyStanceCtrl.kp = 2.0f;
		CM_KneeJoint.EarlyStanceCtrl.position = 0.0f;

		CM_KneeJoint.MidStanceCtrl.kd = 0.05f;
		CM_KneeJoint.MidStanceCtrl.kp = 2.0f;
		CM_KneeJoint.MidStanceCtrl.position = 0.0f;

		CM_KneeJoint.LateStanceCtrl.kd = 0.05f;
		CM_KneeJoint.LateStanceCtrl.kp = 2.0f;
		CM_KneeJoint.LateStanceCtrl.position = 0.0f;

		CM_KneeJoint.SwingFlexCtrl.kd = 0.05f;
		CM_KneeJoint.SwingFlexCtrl.kp = 2.0f;
		CM_KneeJoint.SwingFlexCtrl.position = 0.0f;

		CM_KneeJoint.SwingExtCtrl.kd = 0.05f;
		CM_KneeJoint.SwingExtCtrl.kp = 2.0f;
		CM_KneeJoint.SwingExtCtrl.position = 0.0f;

		switch(Device.CPC_Spec)
		{
		case Specific:
			// ??
			break;
		case Winter:
			CM_KneeJoint.CPC_Params.CPV_1 = 0.717203740538403f;
			CM_KneeJoint.CPC_Params.CPV_2 = 0.786907375601145f;
			CM_KneeJoint.CPC_Params.CPV_3 = 0.980305166790268f;
			CM_KneeJoint.CPC_Params.P_1 = 57.540000000000000f;
			CM_KneeJoint.CPC_Params.P_2 = 64.860000000000000f;
			CM_KneeJoint.CPC_Params.P_3 = 0.540000000000000f;
			CM_KneeJoint.CPC_Params.P_4 = 3.970000000000000f;
			break;
		}

		if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
			ErrorHandler(CAN_Error);
		if(AKxx_x_EnterMotorCtrlMode(KneeIndex, &txMailbox))
			ErrorHandler(KneeMotorError);
	}
}

void RequireTestProgram(TestProgram_e option)
{
	testProgram = option;
	if(testProgram != NoTestProgram)
		isTestProgramRequired = 1;
}

void RunProsthesisControl(void)
{
	GetInputs();
	ProcessInputs();

	StateMachine_e state;
	state = RunStateMachine(StateVals);

	if(Device.Joint != Ankle)
	{
		GetCPV();

		static float cpv[3] = {0.0f, 0.0f, 0.0f};
		static float a1[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		static float a2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		static float a3[3] = {0.0f, 0.0f, 0.0f};
		if(toeOff)
		{
			toeOff = 0;
			GetSegmentConstants(cpv, a1, a2, a3);
		}

		if((state == SwingFlexion) || (state == SwingExtension) || (state == CPC))
			GetTrajectory(cpv, a1, a2, a3);
		else
			CM_trajectory = 0.0f;
	}

	if(testProgram == NoTestProgram)
		RunStateMachine(CtrlParams);
	else if(testProgram == BypassStateMachine)
		SetCtrlParams(Device.Joint, 0, &CM_AnkleJoint.BypassStateMachineCtrl, &CM_KneeJoint.BypassStateMachineCtrl);
	else if(testProgram == PassiveEmulation)
		RunPassiveEmulation();

	CheckMotorCalls();

	// Check for first and second executions, needed for load cell filter and miscellaneous initializations
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
	else if(color == Blue)
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

static void InitStateVals(void)
{
	float state_angle_max[3] = { 10.0f,  60.0f, 60.0f};		// {Ankle, Combined, Knee}
	float state_angle_min[3] = {-20.0f, -20.0f,  0.0f};		// {Ankle, Combined, Knee}

	float state_torque_max[3] = {  50.0f,   50.0f,  50.0f};	// {Ankle, Combined, Knee}
	float state_torque_min[3] = {-100.0f, -100.0f, -50.0f};	// {Ankle, Combined, Knee}

	float state_speed_max[3] = { 600.0f,  600.0f,  600.0f};	// {Ankle, Combined, Knee}
	float state_speed_min[3] = {-600.0f, -600.0f, -600.0f};	// {Ankle, Combined, Knee}

	float state_loadCell_max = 1600.0f;
	float state_loadCell_min = 1100.0f;

	uint8_t nStates = 6;
	for(uint8_t j = 0; j < nStates-1; j++)
	{
		for(uint8_t i = 0; i < 3; i++)
		{
			state_angle[i][j] = (state_angle_max[i] - state_angle_min[i]) / (float)(nStates - 2) * j + state_angle_min[i];
			state_torque[i][j] = (state_torque_max[i] - state_torque_min[i]) / (float)(nStates - 2) * j + state_torque_min[i];
			state_speed[i][j] = (state_speed_max[i] - state_speed_min[i]) / (float)(nStates - 2) * j + state_speed_min[i];
		}

		state_loadCell[j] = (state_loadCell_max - state_loadCell_min) / (float)(nStates - 2) * j + state_loadCell_min;
	}

	for(uint8_t i = 0; i < 3; i++)
	{
		state_angle[i][nStates-1] = state_angle_max[i];
		state_torque[i][nStates-1] = state_torque_max[i];
		state_speed[i][nStates-1] = state_speed_max[i];
	}

	state_loadCell[nStates-1] = state_loadCell_max;
}

static void GetInputs(void)
{
	CM_LoadCell.Raw.bot[0] = ReadLoadCell(ADC1);
	CM_LoadCell.Raw.top[0] = ReadLoadCell(ADC2);

	static uint8_t ankleImuInUse = 0;
	static uint8_t kneeImuInUse = 0;
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		if(!kneeImuInUse)
			if(!ankleImuInUse)
			{
				ankleImuInUse = 1;
				MPU925x_SetChipSelect(0);
				MPU925x_StartReadIMU_IT(0);
			}

		static uint8_t tempImuData[14];
		if(ankleImuTxCplt)
		{
			ankleImuTxCplt = 0;
			MPU925x_ReadIMU_IT(0, tempImuData);
		}

		static uint8_t missedAnkleImuCalls = 0;
		if(ankleImuRxCplt)
		{
			ankleImuRxCplt = 0;
			missedAnkleImuCalls = 0;

			MPU925x_ClearChipSelect(0);
			ankleImuInUse = 0;

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
		if(!ankleImuInUse)
		{
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
			kneeImuInUse = 1;

			static uint8_t missedKneeImuCalls = 0;
			if(BNO08x_resetOccurred)
			{
				BNO08x_resetOccurred = 0;
				if(BNO08x_StartReports())
					missedKneeImuCalls++;
				else
					missedKneeImuCalls = 0;

				if(missedKneeImuCalls >= 5)
					ErrorHandler(KneeIMU_Error);
			}

			BNO08x_ReadSensors();

			if(BNO08x_readEventOccurred)
			{
				HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
				HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
				HAL_NVIC_DisableIRQ(SPI1_IRQn);
				HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
				HAL_NVIC_EnableIRQ(SPI1_IRQn);

				BNO08x_readEventOccurred = 0;
				kneeImuInUse = 0;
			}
		}
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
			CM_KneeJoint.IMU_Data.ax = BNO08x_IMU_Data[1];
			CM_KneeJoint.IMU_Data.ay = BNO08x_IMU_Data[0];
			CM_KneeJoint.IMU_Data.az = -BNO08x_IMU_Data[2];
			CM_KneeJoint.IMU_Data.gx = BNO08x_IMU_Data[4] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gy = BNO08x_IMU_Data[3] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gz = -BNO08x_IMU_Data[5] * RAD_TO_DEG;

			Utils_Quaternion_t Quaternion = {BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9]};

			// Rotating quaternion helps stabilize values due to IMU being vertically mounted
			Utils_Rotation_t RotateY_90 = {90.0f * M_PI/180.0f, 1.0f, 0.0f, 0.0f};
			Quaternion = Utils_RotateQuaternion(&RotateY_90, &Quaternion);

			float yaw, pitch, roll;
			Utils_QuaternionToYPR(Quaternion.r, Quaternion.i, Quaternion.j, Quaternion.k, &yaw, &pitch, &roll);
			CM_KneeJoint.IMU_Data.yaw = yaw * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.pitch = roll * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.roll = pitch * RAD_TO_DEG;
		}
		else if(Device.Side == Right)
		{
			CM_KneeJoint.IMU_Data.ax = -BNO08x_IMU_Data[1];
			CM_KneeJoint.IMU_Data.ay = BNO08x_IMU_Data[0];
			CM_KneeJoint.IMU_Data.az = BNO08x_IMU_Data[2];
			CM_KneeJoint.IMU_Data.gx = -BNO08x_IMU_Data[4] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gy = BNO08x_IMU_Data[3] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gz = BNO08x_IMU_Data[5] * RAD_TO_DEG;

			Utils_Quaternion_t Quaternion = {BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9]};

			// Rotating quaternion helps stabilize values due to IMU being vertically mounted
			Utils_Rotation_t RotateY_90 = {90.0f * M_PI/180.0f, 0.0f, 1.0f, 0.0f};
			Quaternion = Utils_RotateQuaternion(&RotateY_90, &Quaternion);

			float yaw, pitch, roll;
			Utils_QuaternionToYPR(Quaternion.r, Quaternion.i, Quaternion.j, Quaternion.k, &yaw, &pitch, &roll);
			CM_KneeJoint.IMU_Data.yaw = yaw * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.pitch = -roll * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.roll = -pitch * RAD_TO_DEG;
		}

		CM_thighAngle[0] = -(CM_KneeJoint.position + CM_KneeJoint.IMU_Data.pitch);
	}
}

static StateMachine_e RunStateMachine(StateMachineMethod_e method)
{
	static StateMachine_e state = EarlyStance;
	switch(state)
	{
	case EarlyStance:
		if(method == StateVals)
		{
			SetStateVals(Device.Joint, state);

			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				if(CM_footSpeed > CM_threshold_footSpeed)
					state = MidStance;
			}
			else if(Device.Joint == Knee)
				if(CM_LoadCell.Filtered.bot[0] < CM_threshold_intoSwingLC)
				{
					toeOff = 1;
					state = SwingFlexion;
				}
		}
		else if(method == CtrlParams)
			SetCtrlParams(Device.Joint, state, &CM_AnkleJoint.EarlyStanceCtrl, &CM_KneeJoint.EarlyStanceCtrl);

		break;

	case MidStance:
		if(method == StateVals)
		{
			SetStateVals(Device.Joint, state);

			if(CM_AnkleJoint.speed < CM_threshold_ankleSpeed)
				state = LateStance;
		}
		else if(method == CtrlParams)
			SetCtrlParams(Device.Joint, state, &CM_AnkleJoint.MidStanceCtrl, &CM_KneeJoint.MidStanceCtrl);

		break;

	case LateStance:
		if(method == StateVals)
		{
			SetStateVals(Device.Joint, state);

			if(CM_AnkleJoint.speed > 0.0f) // can we use load cell??
			{
				toeOff = 1;

				if(CM__startCPC)
					state = CPC;
				else
					state = SwingFlexion;
			}
		}
		else if(method == CtrlParams)
			SetCtrlParams(Device.Joint, state, &CM_AnkleJoint.LateStanceCtrl, &CM_KneeJoint.LateStanceCtrl);

		break;

	case SwingFlexion:
		if(method == StateVals)
		{
			SetStateVals(Device.Joint, state);

			if(Device.Joint == Ankle)
			{
				if(CM_LoadCell.Filtered.bot[0] > CM_threshold_intoStanceLC)
				{
					heelStrike = 1;
					state = EarlyStance;
				}

			}
			else if((Device.Joint == Knee) || (Device.Joint == Combined))
				if(CM_KneeJoint.speed < 0.0f)
					state = SwingExtension;
		}
		else if(method == CtrlParams)
			SetCtrlParams(Device.Joint, state, &CM_AnkleJoint.SwingFlexCtrl, &CM_KneeJoint.SwingFlexCtrl);

		break;

	case SwingExtension:
		if(method == StateVals)
		{
			SetStateVals(Device.Joint, state);

			if(CM_LoadCell.Filtered.bot[0] > CM_threshold_intoStanceLC)
			{
				heelStrike = 1;
				state = EarlyStance;
			}
		}
		else if(method == CtrlParams)
			SetCtrlParams(Device.Joint, state, &CM_AnkleJoint.SwingExtCtrl, &CM_KneeJoint.SwingExtCtrl);

		break;

	case CPC:
		if(method == StateVals)
		{
			SetStateVals(Device.Joint, state);

			if(CM_LoadCell.Filtered.bot[0] > CM_threshold_intoStanceLC)
			{
				heelStrike = 1;
				state = EarlyStance;
			}
		}
		else if(method == CtrlParams)
			SetCtrlParams(Device.Joint, state, &CM_AnkleJoint.CPC_Ctrl, &CM_KneeJoint.CPC_Ctrl);

		break;
	}

	return state;
}

static void SetStateVals(Joint_e joint, StateMachine_e state)
{
	CM_state_loadCell = state_loadCell[state];

	if(joint == Ankle)
	{
		CM_state_angle = state_angle[Ankle][state];
		CM_state_torque = state_torque[Ankle][state];
		CM_state_speed = state_torque[Ankle][state];
	}
	else if(joint == Combined)
	{
		CM_state_angle = state_angle[Combined][state];
		CM_state_torque = state_torque[Combined][state];
		CM_state_speed = state_torque[Combined][state];
	}
	else if(joint == Knee)
	{
		CM_state_angle = state_angle[Knee][state];
		CM_state_torque = state_torque[Knee][state];
		CM_state_speed = state_torque[Knee][state];
	}
}

static void GetCPV(void)
{
	static double thighAngle_bias = 0.0;
	static double thighIntegral = 0.0;
	static float maxThighIntegral_unbiased = 0.0f;
	static float minThighIntegral_unbiased = 0.0f;
	static float strideTime = 0.0f;
	static float z = 1.0f;
	static uint8_t firstHeelStrike = 1;
	static uint8_t quadrant[2] = {0, 0};

	static float maxThighAngle_unbiased;
	static float minThighAngle_unbiased;
	if(isFirst)
	{
		maxThighAngle_unbiased = CM_thighAngle[0] - thighAngle_bias;
		minThighAngle_unbiased = CM_thighAngle[0] - thighAngle_bias;
	}

	if(heelStrike)
	{
		heelStrike = 0;
		kneeAngleAtHeelStrike = CM_KneeJoint.position;

		if(!firstHeelStrike)
		{
			if(CM_healthyStride)
			{
				if(maxThighIntegral_unbiased != minThighIntegral_unbiased)
					z = fabs(maxThighAngle_unbiased - minThighAngle_unbiased) / fabs(maxThighIntegral_unbiased - minThighIntegral_unbiased);
			}

			thighAngle_bias = thighIntegral / strideTime;
		}

		thighIntegral = 0.0f;
		CM_thighIntegral_unbiased = 0.0f;
		CM_healthyStride = 9;
		CM_cpv = 0.0f;
		quadrant[0] = 0;
		quadrant[1] = 0;

		maxThighAngle_unbiased = CM_thighAngle[0] - thighAngle_bias;
		minThighAngle_unbiased = CM_thighAngle[0] - thighAngle_bias;
		maxThighIntegral_unbiased = 0.0f;
		minThighIntegral_unbiased = 0.0f;

		strideTime = 0.0;

		firstHeelStrike = 0;
	}

	CM_thighAngle_unbiased[0] = CM_thighAngle[0] - thighAngle_bias;
	if(CM_thighAngle_unbiased[0] > maxThighAngle_unbiased)
		maxThighAngle_unbiased = CM_thighAngle_unbiased[0];
	else if(CM_thighAngle_unbiased[0] < minThighAngle_unbiased)
		minThighAngle_unbiased = CM_thighAngle_unbiased[0];

	if(!isFirst)
	{
		thighIntegral += (CM_thighAngle[0] + CM_thighAngle[1]) * DT/2.0;								// trapezoidal integration used
		CM_thighIntegral_unbiased += (CM_thighAngle_unbiased[0] + CM_thighAngle_unbiased[1]) * DT/2.0;	// trapezoidal integration used
		if(CM_thighIntegral_unbiased > maxThighIntegral_unbiased)
			maxThighIntegral_unbiased = CM_thighIntegral_unbiased;
		else if(CM_thighIntegral_unbiased < minThighIntegral_unbiased)
			minThighIntegral_unbiased = CM_thighIntegral_unbiased;
	}

	CM_xPhaseAngle = -CM_thighAngle_unbiased[0];
	CM_yPhaseAngle = -z * CM_thighIntegral_unbiased;

	if((CM_xPhaseAngle < 0.0f) && (CM_yPhaseAngle <= 0.0f))
	{
		CM_state_quadrant = 0;
		quadrant[0] = 1;
	}
	else if((CM_xPhaseAngle >= 0.0f) && (CM_yPhaseAngle < 0.0f))
	{
		CM_state_quadrant = 3;
		quadrant[0] = 2;
	}
	else if((CM_xPhaseAngle > 0.0f) && (CM_yPhaseAngle >= 0.0f))
	{
		CM_state_quadrant = 6;
		quadrant[0] = 3;
	}
	else if((CM_xPhaseAngle <= 0.0f) && (CM_yPhaseAngle > 0.0f))
	{
		CM_state_quadrant = 9;
		quadrant[0] = 4;
	}
	else
	{
		CM_healthyStride = 0;
		CM_state_quadrant = -1;
		quadrant[0] = 0;
		quadrant[1] = 0;
	}

	if((quadrant[0] == 1) && (quadrant[1] == 4))
		quadrant[0] = 4;

	if(CM_healthyStride)
	{
		if(quadrant[0] >= quadrant[1])
			CM_healthyStride = 9;
		else
			CM_healthyStride = 0;
	}

	if(CM_cpv < ((atan2(CM_yPhaseAngle, CM_xPhaseAngle) + M_PI) / (2.0f*M_PI)))
		CM_cpv = (atan2(CM_yPhaseAngle, CM_xPhaseAngle) + M_PI) / (2.0f*M_PI);

	CM_cpvx9 = CM_cpv * 9.0f;

	CM_thighAngle[1] = CM_thighAngle[0];
	CM_thighAngle_unbiased[1] = CM_thighAngle_unbiased[0];
	quadrant[1] = quadrant[0];

	strideTime += DT;
}

static void GetSegmentConstants(float *cpv, float *a1, float *a2, float *a3)
{
	float p[4];
	float v[3];

	float CPV_1 = CM_KneeJoint.CPC_Params.CPV_1;
	float CPV_2 = CM_KneeJoint.CPC_Params.CPV_2;
	float CPV_3 = CM_KneeJoint.CPC_Params.CPV_3;
	float P_1 = CM_KneeJoint.CPC_Params.P_1;
	float P_2 = CM_KneeJoint.CPC_Params.P_2;
	float P_3 = CM_KneeJoint.CPC_Params.P_3;
	float P_4 = CM_KneeJoint.CPC_Params.P_4;

	cpv[0] = CM_cpv;
	p[0] = CM_KneeJoint.position;
	v[0] = CM_KneeJoint.speed;

	cpv[1] = cpv[0] + (CPV_2 - CPV_1)/(1.0f - CPV_1) * (1.0f - cpv[0]);
	p[1] = p[0] * P_2/P_1;
	v[1] = 0.0f;

	p[3] = kneeAngleAtHeelStrike;

	float dP_34 = P_4 - P_3;
	cpv[2] = cpv[0] + (CPV_3 - CPV_1)/(1.0f - CPV_1) * (1.0f - cpv[0]);
	p[2] = p[3] - dP_34;
	v[2] = 0;

	GetThirdOrderSegmentConstants(cpv[0], cpv[1], p[0], p[1], v[0], v[1], a1);
	GetThirdOrderSegmentConstants(cpv[1], cpv[2], p[1], p[2], v[1], v[2], a2);
	GetSecondOrderSegmentConstants(cpv[2], 1, p[2], p[3], v[2], a3);
}

static void GetThirdOrderSegmentConstants(float cpv_1, float cpv_2, float p_1, float p_2, float v_1, float v_2, float *a)
{
	float dcpv = cpv_2 - cpv_1;
	a[0] = p_1;
	a[1] = v_1;
	a[2] = (3.0f*p_2 - 3.0f*p_1 - 2.0f*v_1*dcpv - v_2*dcpv) / (dcpv*dcpv);
	a[3] = (2.0f*p_1 + (v_1 + v_2)*dcpv - 2.0f*p_2) / (dcpv*dcpv*dcpv);
}

static void GetSecondOrderSegmentConstants(float cpv_1, float cpv_2, float p_1, float p_2, float v_1, float *a)
{
	float dcpv = cpv_2 - cpv_1;
	a[0] = p_1;
	a[1] = v_1;
	a[2] = (p_2 - a[0] - a[1]*dcpv) / (dcpv*dcpv);
}

static void GetTrajectory(float *cpv, float *a1, float *a2, float *a3)
{
	if((CM_cpv >= cpv[0]) && (CM_cpv < cpv[1]))
		CM_trajectory = a1[0] + a1[1]*(CM_cpv-cpv[0]) + a1[2]*(CM_cpv-cpv[0])*(CM_cpv-cpv[0]) + a1[3]*(CM_cpv-cpv[0])*(CM_cpv-cpv[0])*(CM_cpv-cpv[0]);
	else if((CM_cpv >= cpv[1]) && (CM_cpv < cpv[2]))
		CM_trajectory = a2[0] + a2[1]*(CM_cpv-cpv[1]) + a2[2]*(CM_cpv-cpv[1])*(CM_cpv-cpv[1]) + a2[3]*(CM_cpv-cpv[1])*(CM_cpv-cpv[1])*(CM_cpv-cpv[1]);
	else if((CM_cpv >= cpv[2]) && (CM_cpv <= 1.0f))
		CM_trajectory = a3[0] + a3[1]*(CM_cpv-cpv[2]) + a3[2]*(CM_cpv-cpv[2])*(CM_cpv-cpv[2]);
}

static void SetCtrlParams(Joint_e joint, StateMachine_e state, AKxx_x_WriteData_t *AnkleMotorWriteData, AKxx_x_WriteData_t *KneeMotorWriteData)
{
	if((joint == Ankle) || (joint == Combined))
	{
		CM_AnkleJoint.ProsCtrl.kd = AnkleMotorWriteData->kd;
		CM_AnkleJoint.ProsCtrl.kp = AnkleMotorWriteData->kp;
		CM_AnkleJoint.ProsCtrl.position = AnkleMotorWriteData->position;
	}
	if((joint == Knee) || (joint == Combined))
	{
		CM_KneeJoint.ProsCtrl.kd = KneeMotorWriteData->kd;
		CM_KneeJoint.ProsCtrl.kp = KneeMotorWriteData->kp;

		if(state == CPC)
			CM_KneeJoint.ProsCtrl.position = CM_trajectory;
		else
			CM_KneeJoint.ProsCtrl.position = KneeMotorWriteData->position;
	}
}

// do i need this still??
static void RunPassiveEmulation(void)
{
	// Avoid unstable kp calcs
	if(CM_KneeJoint.PassEmulExtCtrl.position < 1.0f)
		CM_KneeJoint.PassEmulExtCtrl.position = 1.0f;
	if(CM_KneeJoint.PassEmulFlexCtrl.position < 1.0f)
		CM_KneeJoint.PassEmulFlexCtrl.position = 1.0f;

	if(CM_KneeJoint.position < CM_KneeJoint.PassEmulStanceCtrl.position)
	{
		CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.PassEmulStanceCtrl.kd;
		CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.PassEmulStanceCtrl.kp;
		CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.PassEmulStanceCtrl.position;
	}
	else
	{
		if(CM_KneeJoint.speed >= 0)
		{
			CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.PassEmulFlexCtrl.kd;
			CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.PassEmulFlexCtrl.torque / CM_KneeJoint.PassEmulFlexCtrl.position;

			if(CM_KneeJoint.position > CM_KneeJoint.PassEmulFlexCtrl.position + CM_KneeJoint.PassEmulStanceCtrl.position)
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.position - CM_KneeJoint.PassEmulFlexCtrl.position;
			else
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.PassEmulStanceCtrl.position;
		}
		else
		{
			CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.PassEmulExtCtrl.kd;
			CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.PassEmulExtCtrl.torque / CM_KneeJoint.PassEmulExtCtrl.position;

			if(CM_KneeJoint.position > CM_KneeJoint.PassEmulExtCtrl.position + CM_KneeJoint.PassEmulStanceCtrl.position)
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.position - CM_KneeJoint.PassEmulExtCtrl.position;
			else
				CM_KneeJoint.ProsCtrl.position = CM_KneeJoint.PassEmulStanceCtrl.position;
		}
	}

	if(Device.Joint == Combined)
	{
		CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.PassEmulCtrl.kd;
		CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.PassEmulCtrl.kp;
		CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.PassEmulCtrl.position;
	}
}

static void CheckMotorCalls(void)
{
	uint32_t txMailbox;
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
		{
			AKxx_x_EnterMotorCtrlMode(KneeIndex, &txMailbox);
				if(missedKneeMotorCalls >= 10)
					ErrorHandler(KneeMotorError);
		}
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

	uint32_t txMailbox;
	if(deviceIndex == AnkleIndex)
	{
		if(CM_AnkleJoint.MotorReadData.error)
			ErrorHandler(AnkleMotorError);

		CM_AnkleJoint.position = -CM_AnkleJoint.MotorReadData.position / ANKLE_GEAR_RATIO * RAD_TO_DEG - ANKLE_POSITION_OFFSET_FROM_PLANARFLEXION_BUMPER;
		CM_AnkleJoint.speed = -CM_AnkleJoint.MotorReadData.speed / ANKLE_GEAR_RATIO * RAD_TO_DEG;
		CM_AnkleJoint.torque = -CM_AnkleJoint.MotorReadData.torque * ANKLE_GEAR_RATIO;

		if((testProgram == ReadOnly) || ((testProgram == NoTestProgram) && !CM__startProgram) || ((testProgram == PassiveEmulation) && !CM__startProgram))
		{
			MotorTxData.kd = 0.0f;
			MotorTxData.kp = 0.0f;
		}
		else
		{
			MotorTxData.kd = CM_AnkleJoint.ProsCtrl.kd / (ANKLE_GEAR_RATIO * ANKLE_GEAR_RATIO * DEG_TO_RAD);
			MotorTxData.kp = CM_AnkleJoint.ProsCtrl.kp / (ANKLE_GEAR_RATIO * ANKLE_GEAR_RATIO * DEG_TO_RAD);
			MotorTxData.position = (-CM_AnkleJoint.ProsCtrl.position - ANKLE_POSITION_OFFSET_FROM_PLANARFLEXION_BUMPER) * ANKLE_GEAR_RATIO * DEG_TO_RAD;
		}

		if(AKxx_x_WriteMotor(deviceIndex, &MotorTxData, &txMailbox))
			ErrorHandler(AnkleMotorError);
	}
	else if(deviceIndex == KneeIndex)
	{
		if(CM_KneeJoint.MotorReadData.error)
			ErrorHandler(KneeMotorError);

		CM_KneeJoint.position = -CM_KneeJoint.MotorReadData.position / KNEE_GEAR_RATIO * RAD_TO_DEG - KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER;
		CM_KneeJoint.speed = -CM_KneeJoint.MotorReadData.speed / KNEE_GEAR_RATIO * RAD_TO_DEG;
		CM_KneeJoint.torque = -CM_KneeJoint.MotorReadData.torque * KNEE_GEAR_RATIO / 0.6f; //divide 0.6??

		if((testProgram == ReadOnly) || ((testProgram == NoTestProgram) && !CM__startProgram) || ((testProgram == PassiveEmulation) && !CM__startProgram))
		{
			MotorTxData.kd = 0.0f;
			MotorTxData.kp = 0.0f;
		}
		else
		{
			MotorTxData.kd = CM_KneeJoint.ProsCtrl.kd / (KNEE_GEAR_RATIO * KNEE_GEAR_RATIO * DEG_TO_RAD);
			MotorTxData.kp = CM_KneeJoint.ProsCtrl.kp / (KNEE_GEAR_RATIO * KNEE_GEAR_RATIO * DEG_TO_RAD);
			MotorTxData.position = (-CM_KneeJoint.ProsCtrl.position - KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER) * KNEE_GEAR_RATIO * DEG_TO_RAD;
		}

		if(AKxx_x_WriteMotor(deviceIndex, &MotorTxData, &txMailbox))
			ErrorHandler(KneeMotorError);
	}
}


/*******************************************************************************
* CALLBACKS
*******************************************************************************/

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	ankleImuTxCplt = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	ankleImuRxCplt = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CM_AnkleJoint.motorDataReceived = 1;

	if(AKxx_x_ReadMotor(CAN_RX_FIFO0, &CM_AnkleJoint.MotorReadData))
		ErrorHandler(MotorReadError);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CM_KneeJoint.motorDataReceived = 1;

	if(AKxx_x_ReadMotor(CAN_RX_FIFO1, &CM_KneeJoint.MotorReadData))
		ErrorHandler(MotorReadError);
}


/*******************************************************************************
* END
*******************************************************************************/
