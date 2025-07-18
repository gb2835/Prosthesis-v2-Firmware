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
	MotorReadError
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
	Red
} LED_Color_e;

typedef enum
{
	Left,
	Right
} Side_e;

typedef enum
{
	None,
	ReadOnly,
	ImpedanceControl
} TestProgram_e;

typedef struct
{
	Joint_e Joint;
	Side_e Side;
} Prosthesis_Init_t;

void InitProsthesisControl(Prosthesis_Init_t *Device_Init);
void RequireTestProgram(TestProgram_e option);
void RunProsthesisControl(void);
void ActivateLED(LED_Color_e color);;
void ErrorHandler(Error_e error);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_V2_H_ */
