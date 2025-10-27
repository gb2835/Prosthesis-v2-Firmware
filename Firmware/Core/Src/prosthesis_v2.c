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
#include "winter_bio_data.h"

#include <math.h>
#include <stdint.h>
#include <stm32l4xx_ll_adc.h>
#include <string.h>


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

TestProgram_e testProgram = None;


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
	SwingDescension,
	CPC
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
	AKxx_x_WriteData_t CPC_Ctrl;
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
	AKxx_x_WriteData_t EarlyStanceCtrl;
	AKxx_x_WriteData_t MidStanceCtrl;
	AKxx_x_WriteData_t LateStanceCtrl;
	AKxx_x_WriteData_t SwingFlexCtrl;
	AKxx_x_WriteData_t SwingExtCtrl;
	AKxx_x_WriteData_t SwingDescCtrl;
	AKxx_x_WriteData_t CPC_Ctrl;
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
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2
	} Raw;

	struct
	{
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2
	} Filtered;

	float intoStanceThreshold;
	float intoSwingThreshold;
} LoadCell_t;

static AKxx_x_WriteData_t MotorTxData;
static float kneeAngleAtHeelStrike = 3.97f;
static MPU925x_IMU_Data_t IMU_Data;
static Prosthesis_Init_t Device;

static uint8_t heelStrike = 0;
static uint8_t imuReadStarted = 0;
static uint8_t imuDataReceived = 0;
static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;
static double stridePeriod = 2.0;			// Used for CPC simulations
static uint8_t toeOff = 0;

static AnkleJoint_t CM_AnkleJoint;
static double CM_thighAngle[2];						// [0] = k-0, [1] = k-1
static float CM_cpvx9;
static float CM_state_quadrant;
static float CM_trajectory;
static float CM_xPhaseAngle, CM_yPhaseAngle;
static int8_t CM_state_angles, CM_state_torques;
static int16_t CM_state_speeds;
static uint16_t CM_state_loadCells;
static KneeJoint_t CM_KneeJoint;
static LoadCell_t CM_LoadCell;

static double CM_thighAngle_unbiased[2] = {0.0, 0.0};	// [0] = k-0, [1] = k-1
static double CM_thighIntegral_unbiased = 0.0;
static Error_e CM_ledCode = NoError;
static float CM_ankleSpeedThreshold = -5.0f;
static float CM_cpv = 0.0f;
static float CM_footSpeed = 0.0f;
static float CM_footSpeedThreshold = -5.0f;
static uint8_t CM__StartCPC = 0;
static uint8_t CM_healthyStride = 0;

static const int8_t state_angles[3][6] = {{-20, -14, -8, -2,  4, 10},	// Ankle only
									 	  {-20,  -4, 12, 28, 44, 60},	// Combined
										  {  0,  12, 24, 36, 48, 60}};	// Knee only

static const int8_t state_torques[3][6] = {{-100, -70, -40, -10, 20, 50},	// Ankle only
										   {-100, -70, -40, -10, 20, 50},	// Combined
										   { -50, -30, -10,  10, 30, 50}};	// Knee only

static const int16_t state_speeds[6] = {-600, -360, -120, 120, 360, 600};

static const uint16_t state_loadCells[6] = {1100, 1200, 1300, 1400, 1500, 1600};

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void GetSimulatedThighAngle(void);
static void GetCPV(void);
static void GetSegmentConstants(float *cpv, float *a1, float *a2, float *a3);
static void GetThirdOrderSegmentConstants(float cpv_1, float cpv_2, float p_1, float p_2, float v_1, float v_2, float *a);
static void GetSecondOrderSegmentConstants(float cpv_1, float cpv_2, float p_1, float p_2, float v_1, float *a);
static void GetTrajectory(float *cpv, float *a1, float *a2, float *a3);
static void RunCPC_Simulation(void);
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

	CM_LoadCell.intoStanceThreshold = 1325.0f;

	uint32_t txMailbox;
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		float startKd = 0.4f;
		float startKp = 5.0f;
		float startPos = 0.0f;

		CM_AnkleJoint.EarlyStanceCtrl.kd = startKd;
		CM_AnkleJoint.EarlyStanceCtrl.kp = startKp;
		CM_AnkleJoint.EarlyStanceCtrl.position = startPos;

		CM_AnkleJoint.MidStanceCtrl.kd = startKd;
		CM_AnkleJoint.MidStanceCtrl.kp = startKp;
		CM_AnkleJoint.MidStanceCtrl.position = startPos;

		CM_AnkleJoint.LateStanceCtrl.kd = startKd;
		CM_AnkleJoint.LateStanceCtrl.kp = startKp;
		CM_AnkleJoint.LateStanceCtrl.position = startPos;

		CM_AnkleJoint.SwingFlexCtrl.kd = startKd;
		CM_AnkleJoint.SwingFlexCtrl.kp = startKp;
		CM_AnkleJoint.SwingFlexCtrl.position = startPos;

		CM_AnkleJoint.SwingExtCtrl.kd = startKd;
		CM_AnkleJoint.SwingExtCtrl.kp = startKp;
		CM_AnkleJoint.SwingExtCtrl.position = startPos;

		CM_AnkleJoint.SwingDescCtrl.kd = startKd;
		CM_AnkleJoint.SwingDescCtrl.kp = startKp;
		CM_AnkleJoint.SwingDescCtrl.position = startPos;

		CM_AnkleJoint.CPC_Ctrl.kd = startKd;
		CM_AnkleJoint.CPC_Ctrl.kp = startKp;
		CM_AnkleJoint.CPC_Ctrl.position = startPos;

		MPU925x_SetChipSelect(0);
		MPU925x_StartReadIMU_IT(0);

		if(AKxx_x_EnterMotorCtrlMode(AnkleIndex, &txMailbox))
			ErrorHandler(AnkleMotorError);
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		float startKd = 0.0f;
		float startKp = 0.0f;
		float startPos = 0.0f;

		CM_KneeJoint.EarlyStanceCtrl.kd = startKd;
		CM_KneeJoint.EarlyStanceCtrl.kp = startKp;
		CM_KneeJoint.EarlyStanceCtrl.position = startPos;

		CM_KneeJoint.MidStanceCtrl.kd = startKd;
		CM_KneeJoint.MidStanceCtrl.kp = startKp;
		CM_KneeJoint.MidStanceCtrl.position = startPos;

		CM_KneeJoint.LateStanceCtrl.kd = startKd;
		CM_KneeJoint.LateStanceCtrl.kp = startKp;
		CM_KneeJoint.LateStanceCtrl.position = startPos;

		CM_KneeJoint.SwingFlexCtrl.kd = startKd;
		CM_KneeJoint.SwingFlexCtrl.kp = startKp;
		CM_KneeJoint.SwingFlexCtrl.position = startPos;

		CM_KneeJoint.SwingExtCtrl.kd = startKd;
		CM_KneeJoint.SwingExtCtrl.kp = startKp;
		CM_KneeJoint.SwingExtCtrl.position = startPos;

		CM_KneeJoint.SwingDescCtrl.kd = startKd;
		CM_KneeJoint.SwingDescCtrl.kp = startKp;
		CM_KneeJoint.SwingDescCtrl.position = startPos;

		CM_KneeJoint.CPC_Ctrl.kd = startKd;
		CM_KneeJoint.CPC_Ctrl.kp = startKp;

		if(testProgram == CPC_Simulation_Ideal)
		{
			CM_KneeJoint.CPC_Params.CPV_1 = 0.660000000000000f;
			CM_KneeJoint.CPC_Params.CPV_2 = 0.720000000000000f;
			CM_KneeJoint.CPC_Params.CPV_3 = 0.980000000000000f;
			CM_KneeJoint.CPC_Params.P_1 = 57.540000000000000f;
			CM_KneeJoint.CPC_Params.P_2 = 64.860000000000000f;
			CM_KneeJoint.CPC_Params.P_3 = 0.540000000000000f;
			CM_KneeJoint.CPC_Params.P_4 = 3.970000000000000f;
		}
		else if((testProgram == CPC_Simulation_Winter || (testProgram == CPC_Simulation_WinterUnsteady)))
		{
			CM_KneeJoint.CPC_Params.CPV_1 = 0.717203740538403f;
			CM_KneeJoint.CPC_Params.CPV_2 = 0.786907375601145f;
			CM_KneeJoint.CPC_Params.CPV_3 = 0.980305166790268f;
			CM_KneeJoint.CPC_Params.P_1 = 57.540000000000000f;
			CM_KneeJoint.CPC_Params.P_2 = 64.860000000000000f;
			CM_KneeJoint.CPC_Params.P_3 = 0.540000000000000f;
			CM_KneeJoint.CPC_Params.P_4 = 3.970000000000000f;
		}
		else
			switch(Device.CPC_Spec)
			{
			case Kaden:
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

		if(AKxx_x_EnterMotorCtrlMode(KneeIndex, &txMailbox))
			ErrorHandler(KneeMotorError);
	}
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

	GetTrajectory(cpv, a1, a2, a3);

	if((testProgram == CPC_Simulation_Ideal) || (testProgram == CPC_Simulation_Winter) || (testProgram == CPC_Simulation_WinterUnsteady))
		RunCPC_Simulation();
	else if(testProgram == None)
		RunStateMachine();

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

			Utils_Rotation_t RotateX_90 = {-90.0f * M_PI/180.0f, 1.0f, 0.0f, 0.0f};
			Utils_Quaternion_t Quaternion = {BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9]};
			Quaternion = Utils_RotateQuaternion(&RotateX_90, &Quaternion);

			Utils_Rotation_t RotateY_90 = {90.0f * M_PI/180.0f, 0.0f, 1.0f, 0.0f};
			Quaternion = Utils_RotateQuaternion(&RotateY_90, &Quaternion);

			float yaw, pitch, roll;
			Utils_QuaternionToYPR(Quaternion.r, Quaternion.i, Quaternion.j, Quaternion.k, &yaw, &pitch, &roll);
			CM_KneeJoint.IMU_Data.yaw = yaw * RAD_TO_DEG + 90.0f;
			CM_KneeJoint.IMU_Data.pitch = pitch * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.roll = -roll * RAD_TO_DEG;
		}
		else if(Device.Side == Right)
		{
			CM_KneeJoint.IMU_Data.ax = -BNO08x_IMU_Data[1];
			CM_KneeJoint.IMU_Data.ay = BNO08x_IMU_Data[0];
			CM_KneeJoint.IMU_Data.az = BNO08x_IMU_Data[2];
			CM_KneeJoint.IMU_Data.gx = -BNO08x_IMU_Data[4] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gy = BNO08x_IMU_Data[3] * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.gz = BNO08x_IMU_Data[5] * RAD_TO_DEG;

			Utils_Rotation_t RotateX_90 = {90.0f * M_PI/180.0f, 1.0f, 0.0f, 0.0f};
			Utils_Quaternion_t Quaternion = {BNO08x_IMU_Data[6], BNO08x_IMU_Data[7], BNO08x_IMU_Data[8], BNO08x_IMU_Data[9]};
			Quaternion = Utils_RotateQuaternion(&RotateX_90, &Quaternion);

			Utils_Rotation_t RotateY_90 = {90.0f * M_PI/180.0f, 0.0f, 1.0f, 0.0f};
			Quaternion = Utils_RotateQuaternion(&RotateY_90, &Quaternion);

			float yaw, pitch, roll;
			Utils_QuaternionToYPR(Quaternion.r, Quaternion.i, Quaternion.j, Quaternion.k, &yaw, &pitch, &roll);
			CM_KneeJoint.IMU_Data.yaw = yaw * RAD_TO_DEG - 90.f;
			CM_KneeJoint.IMU_Data.pitch = pitch * RAD_TO_DEG;
			CM_KneeJoint.IMU_Data.roll = -roll * RAD_TO_DEG;
		}

		CM_footSpeed = CM_AnkleJoint.speed + CM_AnkleJoint.IMU_Data.Struct.gz;

		if((testProgram != CPC_Simulation_Ideal) && (testProgram != CPC_Simulation_Winter) && (testProgram != CPC_Simulation_WinterUnsteady))
			CM_thighAngle[0] = CM_KneeJoint.position + CM_KneeJoint.IMU_Data.pitch;
		else
			GetSimulatedThighAngle();
	}
}

static void GetSimulatedThighAngle(void)
{
	static double time = 0.0;

	if(testProgram == CPC_Simulation_Ideal)
	{
		double w = 2 * M_PI / stridePeriod;

		CM_thighAngle[0] = 20.0*cos(w*time);

		if(((-20.0*w*sin(w*time)) <= 0.0) && ((-20.0*w*sin(w*(time-DT))) > 0.0))
			heelStrike = 1;

		time += DT;
	}
	else
	{
		static double unsteadyTime = 0.0;

		double unsteady;
		if(testProgram == CPC_Simulation_WinterUnsteady)
		{
			double w = (2 * M_PI / stridePeriod) * (5.0 / M_PI);	// This ratio works well to generate unsteady gait cycles for a given stride period
			unsteady = 5.0*cos(w*unsteadyTime);
		}
		else
			unsteady = 0.0;

		static double winterHipAngle[51][2];
		if(isFirst)
		{
			for(uint8_t row = 0; row < 51; row++)
			{
				winterHipAngle[row][0] = winterBioData[row][Winter_Stride] * (stridePeriod/100.0);
				winterHipAngle[row][1] = winterBioData[row][Winter_HipAngle];
			}
		}

		static uint8_t start = 0;
		static uint8_t row;
		for(row = start; row < (51-1); row++)
			if((time > winterHipAngle[row][0]) && (time < winterHipAngle[row+1][0]))
				break;

		CM_thighAngle[0] = Utils_LinearInterpolate(time, winterHipAngle[row][0], winterHipAngle[row][1], winterHipAngle[row+1][0], winterHipAngle[row+1][1]) + unsteady;

		time += DT;
		if(time >= stridePeriod)
		{
			time = 0.0;
			start = 0;
		}

		unsteadyTime += DT;
	}
}

static void GetCPV(void)
{
	static double thighAngle_bias = 0.0;
	static double thighIntegral = 0.0;
	static float maxThighIntegral_unbiased = 0.0f;
	static float minThighIntegral_unbiased = 0.0f;
	static float startTime = 0.0f;
	static float time = 0.0f;
	static float z = 1.0f;
	static uint8_t firstCall = 1;
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
		quadrant[0] = 0;
		quadrant[1] = 0;

		float dtime = time - startTime;

		if(firstCall)
			firstCall = 0;
		else
		{
			thighAngle_bias = thighIntegral / dtime;

			if(CM_healthyStride)
			{
				CM_healthyStride = 0;
				z = fabs(maxThighAngle_unbiased - minThighAngle_unbiased) / fabs(maxThighIntegral_unbiased - minThighIntegral_unbiased);
			}
		}

		thighIntegral = 0.0f;
		CM_thighIntegral_unbiased = 0.0f;
		CM_healthyStride = 9;
		CM_cpv = 0.0f;

		maxThighAngle_unbiased = CM_thighAngle[0] - thighAngle_bias;
		minThighAngle_unbiased = CM_thighAngle[0] - thighAngle_bias;
		maxThighIntegral_unbiased = 0.0f;
		minThighIntegral_unbiased = 0.0f;

		startTime = time;
	}

	CM_thighAngle_unbiased[0] = CM_thighAngle[0] - thighAngle_bias;
	if(CM_thighAngle_unbiased[0] > maxThighAngle_unbiased)
		maxThighAngle_unbiased = CM_thighAngle_unbiased[0];
	if(CM_thighAngle_unbiased[0] < minThighAngle_unbiased)
		minThighAngle_unbiased = CM_thighAngle_unbiased[0];

	if(!isFirst)
	{
		thighIntegral += (CM_thighAngle[0] + CM_thighAngle[1]) * DT/2.0;								// trapezoidal integration used
		CM_thighIntegral_unbiased += (CM_thighAngle_unbiased[0] + CM_thighAngle_unbiased[1]) * DT/2.0;	// trapezoidal integration used
		if(CM_thighIntegral_unbiased > maxThighIntegral_unbiased)
			maxThighIntegral_unbiased = CM_thighIntegral_unbiased;
		if(CM_thighIntegral_unbiased < minThighIntegral_unbiased)
			minThighIntegral_unbiased = CM_thighIntegral_unbiased;
	}

	CM_xPhaseAngle = -CM_thighAngle_unbiased[0];
	CM_yPhaseAngle = -z * CM_thighIntegral_unbiased;

	if((CM_xPhaseAngle < 0.0f) && (CM_yPhaseAngle <= 0.0f))
	{
		CM_state_quadrant = 0.0f;
		quadrant[0] = 1;
	}
	if((CM_xPhaseAngle >= 0.0f) && (CM_yPhaseAngle < 0.0f))
	{
		CM_state_quadrant = 3.0f;
		quadrant[0] = 2;
	}
	if((CM_xPhaseAngle > 0.0f) && (CM_yPhaseAngle >= 0.0f))
	{
		CM_state_quadrant = 6.0f;
		quadrant[0] = 3;
	}
	if((CM_xPhaseAngle <= 0.0f) && (CM_yPhaseAngle > 0.0f))
	{
		CM_state_quadrant = 9.0f;
		quadrant[0] = 4;
	}

	if(CM_healthyStride)
	{
		if((quadrant[0] == 1) && (quadrant[1] == 4))
			quadrant[0] = 4;
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

	time += DT;
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

	cpv[1] = p[0] + (CPV_2 - CPV_1)/(1.0f - CPV_1) * (1.0f - cpv[0]);
	p[1] = p[0] * P_2/P_1;
	v[1] = 0.0f;

	p[3] = kneeAngleAtHeelStrike;

	float dP_34 = P_4 - P_3;
	cpv[2] = p[0] + (CPV_3 - CPV_1)/(1.0f - CPV_1) * (1.0f - cpv[0]);
	p[2] = p[3] - dP_34;
	v[2] = 0;

	GetThirdOrderSegmentConstants(cpv[0], cpv[1], p[0], p[1], v[0], v[1], a1);
	GetThirdOrderSegmentConstants(cpv[1], cpv[2], p[1], p[2], v[1], v[2], a2);
	GetSecondOrderSegmentConstants(cpv[1], cpv[2], p[1], p[2], v[1], a3);
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
		CM_trajectory = a1[0] + a1[1]*CM_cpv + a1[2]*CM_cpv*CM_cpv + a1[3]*CM_cpv*CM_cpv*CM_cpv;
	if((CM_cpv >= cpv[1]) && (CM_cpv < cpv[2]))
		CM_trajectory = a2[0] + a2[1]*CM_cpv + a2[2]*CM_cpv*CM_cpv + a2[3]*CM_cpv*CM_cpv*CM_cpv;
	if((CM_cpv >= cpv[2]) && (CM_cpv < 1))
		CM_trajectory = a3[0] + a3[1]*CM_cpv + a3[2]*CM_cpv*CM_cpv;
	else
		CM_trajectory = 0.0f;
}

static void RunCPC_Simulation(void)
{
	static float time = 0.0f;

	static float winterKneeAngle[51][2];
	if(isFirst)
	{
		for(uint8_t row = 0; row < 51; row++)
		{
			winterKneeAngle[row][0] = winterBioData[row][Winter_Stride] * (stridePeriod/100.0f);
			winterKneeAngle[row][1] = winterBioData[row][Winter_KneeAngle];
		}
	}

	static uint8_t row;
	static uint8_t start = 0;
	for(row = start; row < (51-1); row++)
		if((time > winterKneeAngle[row][0]) && (time < winterKneeAngle[row+1][0]))
			break;

	CM_KneeJoint.ProsCtrl.kd = 0.0f;
	CM_KneeJoint.ProsCtrl.kp = 0.0f;

	float initialPosition;
	if(isFirst)
		initialPosition = CM_KneeJoint.position;

	if(time < (stridePeriod/100.0f * 0.66f))	// Stance phase
		CM_KneeJoint.ProsCtrl.position = Utils_LinearInterpolate(time, winterKneeAngle[row][0], winterKneeAngle[row][1], winterKneeAngle[row+1][0], winterKneeAngle[row+1][1]) + (initialPosition - winterKneeAngle[0][1]);
	else
		CM_KneeJoint.ProsCtrl.position = CM_trajectory;

	time += DT;
	if(time >= stridePeriod)
	{
		time = 0.0;
		start = 0;
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

		if((Device.Joint == Ankle) || (Device.Joint == Combined))
		{
			if(CM_footSpeed > CM_footSpeedThreshold)
				state = MidStance;
		}
		else if(Device.Joint == Knee)
			if(CM_LoadCell.Filtered.bot[0] < CM_LoadCell.intoSwingThreshold)
			{
				state = SwingFlexion;
				toeOff = 1;
			}

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

		if(CM_AnkleJoint.speed < CM_ankleSpeedThreshold) // check with angle plot (not speed plot)??
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

		if(CM_AnkleJoint.speed > 0.0f) // can we use load cell??
		{
			if(CM__StartCPC)
				state = CPC;
			else
				state = SwingFlexion;

			toeOff = 1;
		}

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

		if(Device.Joint == Ankle)
		{
			if(CM_LoadCell.Filtered.bot[0] > CM_LoadCell.intoStanceThreshold)
			{
				state = EarlyStance;
				heelStrike = 1;
			}

		}
		else if((Device.Joint == Knee) || (Device.Joint == Combined))
			if(CM_KneeJoint.speed < 0.0f)
				state = SwingExtension;

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

		if(Device.Joint == Combined)
		{
			if(CM_footSpeed < 0.0f)
				state = SwingDescension;
		}
		else if(Device.Joint == Knee)
			if(CM_LoadCell.Filtered.bot[0] > CM_LoadCell.intoStanceThreshold)
			{
				state = EarlyStance;
				heelStrike = 1;
			}


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

		if(CM_LoadCell.Filtered.bot[0] > CM_LoadCell.intoStanceThreshold)
		{
			state = EarlyStance;
			heelStrike = 1;
		}

		break;

	case CPC:
		CM_state_loadCells = state_loadCells[CPC];
		CM_state_speeds = state_speeds[CPC];

		if(Device.Joint == Ankle)
		{
			CM_state_angles = state_angles[Ankle][CPC];
			CM_state_torques = state_torques[Ankle][CPC];
		}
		if(Device.Joint == Knee)
		{
			CM_state_angles = state_angles[Knee][CPC];
			CM_state_torques = state_torques[Knee][CPC];
		}
		if(Device.Joint == Combined)
		{
			CM_state_angles = state_angles[Combined][CPC];
			CM_state_torques = state_torques[Combined][CPC];
		}

		if((Device.Joint == Ankle) || (Device.Joint == Combined))
		{
			CM_AnkleJoint.ProsCtrl.kd = CM_AnkleJoint.CPC_Ctrl.kd;
			CM_AnkleJoint.ProsCtrl.kp = CM_AnkleJoint.CPC_Ctrl.kp;
			CM_AnkleJoint.ProsCtrl.position = CM_AnkleJoint.CPC_Ctrl.position;
		}
		if((Device.Joint == Knee) || (Device.Joint == Combined))
		{
			CM_KneeJoint.ProsCtrl.kd = CM_KneeJoint.CPC_Ctrl.kd;
			CM_KneeJoint.ProsCtrl.kp = CM_KneeJoint.CPC_Ctrl.kp;
			CM_KneeJoint.ProsCtrl.position = CM_trajectory;
		}

		if(CM_LoadCell.Filtered.bot[0] > CM_LoadCell.intoStanceThreshold)
		{
			state = EarlyStance;
			heelStrike = 1;
		}

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
		{
			uint32_t txMailbox;
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

	if(deviceIndex == AnkleIndex)
	{
		if(CM_AnkleJoint.MotorReadData.error)
			ErrorHandler(AnkleMotorError);

		CM_AnkleJoint.position = -CM_AnkleJoint.MotorReadData.position / ANKLE_GEAR_RATIO * RAD_TO_DEG - ANKLE_POSITION_OFFSET_FROM_PLANARFLEXION_BUMPER;
		CM_AnkleJoint.speed = -CM_AnkleJoint.MotorReadData.speed / ANKLE_GEAR_RATIO * RAD_TO_DEG;
		CM_AnkleJoint.torque = -CM_AnkleJoint.MotorReadData.torque * ANKLE_GEAR_RATIO ;

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
		if(CM_KneeJoint.MotorReadData.error)
			ErrorHandler(KneeMotorError);

		CM_KneeJoint.position = -CM_KneeJoint.MotorReadData.position / KNEE_GEAR_RATIO * RAD_TO_DEG - KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER;
		CM_KneeJoint.speed = -CM_KneeJoint.MotorReadData.speed / KNEE_GEAR_RATIO * RAD_TO_DEG;
		CM_KneeJoint.torque = -CM_KneeJoint.MotorReadData.torque * KNEE_GEAR_RATIO ;

		uint32_t txMailbox;
		if((testProgram == None) || (testProgram == ImpedanceControl))
		{
			MotorTxData.kd = CM_KneeJoint.ProsCtrl.kd;
			MotorTxData.kp = CM_KneeJoint.ProsCtrl.kp;
			MotorTxData.position = (-CM_KneeJoint.ProsCtrl.position - KNEE_POSITION_OFFSET_FROM_EXTENSION_BUMPER) * KNEE_GEAR_RATIO * DEG_TO_RAD;

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
	memcpy(&CM_AnkleJoint.MotorReadData, &temp, sizeof(AKxx_x_ReadData_t));
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	AKxx_x_ReadData_t temp;
	if(AKxx_x_ReadMotor(CAN_RX_FIFO1, &temp))
		ErrorHandler(MotorReadError);

	CM_KneeJoint.motorDataReceived = 1;
	memcpy(&CM_KneeJoint.MotorReadData, &temp, sizeof(AKxx_x_ReadData_t));
}


/*******************************************************************************
* END
*******************************************************************************/
