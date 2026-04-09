/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_PROSTHESIS_V2_H_
#define INC_PROSTHESIS_V2_H_

#include <stdint.h>

typedef enum
{
	AnkleMotorCAN_ID = 1,
	KneeMotorCAN_ID
} CAN_ID_e;

typedef enum
{
	Specific,
	Winter
} CPC_Spec_e;

typedef enum
{
	AnkleIndex,
	KneeIndex
} DeviceIndex_e;

typedef enum
{
	NoError,
	AnkleIMU_Error,
	AnkleMotorError,
	CAN_Error,
	KneeIMU_Error,
	KneeMotorError,
	MotorReadError,
	PassiveEmulationError
} Error_e;

typedef enum
{
	Ankle,
	Combined,
	Knee
} Joint_e;

typedef enum
{
	NoColor,
	Blue,
	Green,
	Red,
	White
} LED_Color_e;

typedef enum
{
	Left,
	Right
} Side_e;

typedef enum
{
	StateMachineCtrlWithPE,		// PE = Passive Emulation
	StateMachineCtrlWithoutPE,	// PE = Passive Emulation
	PassiveEmulation,
	ConstantImpedance,
	ReadOnly
} OperationMode_e;

typedef struct
{
	CPC_Spec_e CPC_Spec;
	Joint_e Joint;
	Side_e Side;
} Prosthesis_Init_t;

extern OperationMode_e operationMode;

void InitProsthesisControl(Prosthesis_Init_t *Device_Init);
void SetOperationMode(OperationMode_e mode);
void RunProsthesisControl(void);
void ActivateLED(LED_Color_e color);;
void ErrorHandler(Error_e error);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_V2_H_ */
