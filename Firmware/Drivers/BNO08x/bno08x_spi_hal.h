/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_BNO08X_SPI_HAL_H_
#define INC_BNO08X_SPI_HAL_H_

#include <stdint.h>

typedef enum
{
	BNO08x_NoError,
	BNO08x_InitError,
	BNO08x_StartReportError
} BNO08x_Error_e;

extern float BNO08x_IMU_Data[10];
extern uint8_t BNO08x_resetOccurred;

BNO08x_Error_e BNO08x_Init(void);
BNO08x_Error_e BNO08x_StartReports(void);
void BNO08x_ReadSensors(void);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_BNO08X_SPI_HAL_H_ */
