/*******************************************************************************
*
* TITLE: Error Handler for Prosthesis v2
*
* NOTES
* 1. The order of the LED_code_e matters if using the provided CubeMonitor file.
*
*******************************************************************************/

#include "error_handler.h"

static LED_Code_e CM_ledCode = NoError;

void ErrorHandler_AKxx_x(DeviceIndex_e deviceIndex)
{
	if(deviceIndex == AnkleIndex)
		CM_ledCode = AnkleMotorError;
	else
		CM_ledCode = KneeMotorError;

	ShutdownMotors();
}

void ErrorHandler_BNO08x()
{
	CM_ledCode = KneeIMU_Error;
	ShutdownMotors();
}

void ErrorHandler_Pv2(LED_Code_e code)
{
	CM_ledCode = code;
	ShutdownMotors();
}


/*******************************************************************************
* END
*******************************************************************************/
