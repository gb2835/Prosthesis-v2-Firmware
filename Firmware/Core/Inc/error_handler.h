/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_ERROR_HANDLER_H_
#define INC_ERROR_HANDLER_H_

#include "akxx-x.h"
#include "bno08x_spi_hal.h"
#include "prosthesis_v2.h"

typedef enum
{
	NoError,
	AnkleIMU_Error,
	AnkleMotorError,
	CAN_Error,
	KneeIMU_Error,
	KneeMotorError,
	MotorReadError
} LED_Code_e;

void ErrorHandler_AKxx_x(DeviceIndex_e deviceIndex);
void ErrorHandler_BNO08x(void);
void ErrorHandler_Pv2(LED_Code_e code);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_ERROR_HANDLER_H_ */
