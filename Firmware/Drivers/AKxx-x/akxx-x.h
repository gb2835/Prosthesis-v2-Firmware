/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_AKXX_X_H_
#define INC_AKXX_X_H_

#include <stdint.h>
#include <stm32l4xx.h>

#define AKXX_X_NUMBER_OF_DEVICES	2

typedef enum
{
	AKxx_x_NoError,
	AKxx_x_InitError,
	AKxx_x_EnterMotorCtrlModeError,
	AKxx_x_ExitMotorCtrlModeError,
	AKxx_x_ZeroMotorPositionError,
	AKxx_x_PollMotorReadWithTimeoutError,
	AKxx_x_PollTxMessagePendingWithTimeoutError,
	AKxx_x_ReadDataError,
	AKxx_x_WriteDataError
} AKxx_x_Error_e;

typedef enum
{
	AKxx_x_NoFault,
	AKxx_x_OverTemperatureFault,
	AKxx_x_OverCurrentFault,
	AKxx_x_OverVoltageFault,
	AKxx_x_UnderVoltageFault,
	AKxx_x_EncoderFault,
	AKxx_x_PhaseCurrentUnbalanceFault
} AKxx_x_ErrorCode_e;

typedef enum
{
	AK70_10,
	AK80_9
} AKxx_x_Motor_e;

typedef struct
{
	uint8_t canId;
	float position;
	float speed;
	float torque;
	uint8_t error;
} AKxx_x_ReadData_t;

typedef struct
{
	float kd;
	float kp;
	float position;
	float speed;
	float torque;
} AKxx_x_WriteData_t;

typedef struct
{
	uint16_t canId;
	AKxx_x_Motor_e Motor;
} AKxx_x_Init_t;

extern CAN_HandleTypeDef hcan1;

AKxx_x_Error_e AKxx_x_Init(uint8_t deviceIndex, AKxx_x_Init_t *Device_Init);
AKxx_x_Error_e AKxx_x_ReadMotor(uint32_t rxFifo, AKxx_x_ReadData_t *RxData_Float);
AKxx_x_Error_e AKxx_x_WriteMotor(uint8_t deviceIndex, AKxx_x_WriteData_t *txData, uint32_t *txMailbox);
AKxx_x_Error_e AKxx_x_EnterMotorCtrlMode(uint8_t deviceIndex, uint32_t *txMailbox);
AKxx_x_Error_e AKxx_x_ExitMotorCtrlMode(uint8_t deviceIndex, uint32_t *txMailbox);
AKxx_x_Error_e AKxx_x_ZeroMotorPosition(uint8_t deviceIndex, uint32_t *txMailbox);
AKxx_x_Error_e AKxx_x_PollTxMessagePendingWith10msTimeout(uint32_t txMailbox);
AKxx_x_Error_e AKxx_x_PollMotorReadWith10msTimeout(AKxx_x_ReadData_t *RxData_Float);
AKxx_x_ErrorCode_e AKxx_x_DecodeError(uint8_t error);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_AKXX_X_H_ */
