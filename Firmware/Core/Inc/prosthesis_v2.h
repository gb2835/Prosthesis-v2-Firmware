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
	Ankle,
	Combined,
	Knee
} Joint_e;

typedef enum
{
	Left,
	Right
} Side_e;

typedef enum
{
	None,
	ReadOnly,
	ZeroMotorPosition,
	ImpedanceControl
} TestProgram_e;

typedef struct
{
	Side_e Side;
	Joint_e Joint;
} Prosthesis_Init_t;

extern uint8_t isProsthesisControlRequired;

void InitProsthesisControl(Prosthesis_Init_t *Device_Init);
void RequireTestProgram(TestProgram_e option);
void RunProsthesisControl(void);
void ShutdownMotors(void);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_V2_H_ */
