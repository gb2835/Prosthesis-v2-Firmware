/*******************************************************************************
*
* TITLE: Driver for CubeMars AK Series Motors
*
* NOTES
* 1. This driver is based on:
*		- CubeMars AK Series Actuator Driver Manual
*			- Version: 1.0.9
* 2. Unless otherwise specified, units are
* 		- Position	= radians
* 		- Speed		= rad/s
* 		- Torque	= Nm
* 3. #define AKK_X_NUMBER_OF_DEVICES must be updated to (at least) the number of devices used.
* 4. Polling is used for initialization.
*    If interrupts are desired then HAL_CAN_ActivateNotification() must be called after AKxx_x_Init() in user application.
* 5. AKxx-x motors require 1 Mbit/s CAN rate.
*
*******************************************************************************/

#include "akxx-x.h"

#include <math.h>
#include <string.h>


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

typedef struct
{
	AKxx_x_Init_t InitVals;
	float speedMax;
	float speedMin;
	float torqueMax;
	float torqueMin;
	uint8_t isInit;
} Device_t;

static Device_t Device[AKXX_X_NUMBER_OF_DEVICES];

static AKxx_x_Error_e ReadData(uint32_t rxFifo, AKxx_x_ReadData_t *RxData_Float);
static AKxx_x_Error_e WriteData(uint8_t deviceIndex, AKxx_x_WriteData_t *TxData_Float);
static AKxx_x_Error_e EnterMotorCtrlMode(uint8_t deviceIndex);
static void PackData(uint8_t deviceIndex, AKxx_x_WriteData_t *TxData_Float, uint8_t *txData_uint);
static void UnpackData(uint8_t *rxData_uint, AKxx_x_ReadData_t *RxData_Float);
static float UintToFloat(uint16_t x_uint, float xMin_float, float xMax_float, uint8_t nBits);
static uint16_t FloatToUint(float x_float, float xMin_float, float xMax_float, uint8_t nBits);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

AKxx_x_Error_e AKxx_x_Init(uint8_t deviceIndex, AKxx_x_Init_t *Device_Init)
{
	if(deviceIndex >= AKXX_X_NUMBER_OF_DEVICES)
		while(1);

	memcpy(&Device[deviceIndex], Device_Init, sizeof(AKxx_x_Init_t));

	if(HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING))	// Polling is used for initialization
		return AKxx_x_InitError;

	if(EnterMotorCtrlMode(deviceIndex))
		return AKxx_x_InitError;

	AKxx_x_ReadData_t RxData_Float;
	if(AKxx_x_PollMotorReadWithTimeout(&RxData_Float))
		return AKxx_x_InitError;

	switch(Device[deviceIndex].InitVals.Motor)
	{
	case AK70_10:
		Device[deviceIndex].speedMax = 50.0f;
		Device[deviceIndex].speedMin = -50.0f;
		Device[deviceIndex].torqueMax = 25.0f;
		Device[deviceIndex].torqueMin = -25.0f;
		break;
	case AK80_9:
		Device[deviceIndex].speedMax = 50.0f;
		Device[deviceIndex].speedMin = -50.0f;
		Device[deviceIndex].torqueMax = 18.0f;
		Device[deviceIndex].torqueMin = -18.0f;
		break;
	}

	Device[deviceIndex].isInit = 1;

	return AKxx_x_NoError;
}

AKxx_x_Error_e AKxx_x_ReadMotor(uint32_t rxFifo, AKxx_x_ReadData_t *RxData_Float)
{
	return ReadData(rxFifo, RxData_Float);
}

AKxx_x_Error_e AKxx_x_WriteMotor(uint8_t deviceIndex, AKxx_x_WriteData_t *TxData_Float)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return WriteData(deviceIndex, TxData_Float);
}

AKxx_x_Error_e AKxx_x_EnterMotorCtrlMode(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return EnterMotorCtrlMode(deviceIndex);
}

AKxx_x_Error_e AKxx_x_ExitMotorCtrlMode(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = 8;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = Device[deviceIndex].InitVals.canId;
	TxHeader.TransmitGlobalTime = DISABLE;

	uint8_t txData_uint[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD};
	uint32_t txMailbox;
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData_uint, &txMailbox) != HAL_OK)
		return AKxx_x_ExitMotorCtrlModeError;

	if(AKxx_x_PollTxMessagePendingWithTimeout(txMailbox)) //??
		return AKxx_x_ExitMotorCtrlModeError;

	return AKxx_x_NoError;
}

AKxx_x_Error_e AKxx_x_ZeroMotorPosition(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = 8;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = Device[deviceIndex].InitVals.canId;
	TxHeader.TransmitGlobalTime = DISABLE;

	uint8_t txData_uint[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFE};
	uint32_t txMailbox;
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData_uint, &txMailbox) != HAL_OK)
		return AKxx_x_ZeroMotorPositionError;

	if(AKxx_x_PollTxMessagePendingWithTimeout(txMailbox)) //??
		return AKxx_x_ZeroMotorPositionError;

	return AKxx_x_NoError;
}

AKxx_x_Error_e AKxx_x_PollMotorReadWithTimeout(AKxx_x_ReadData_t *RxData_Float)
{
	uint8_t timeoutOccurred = 1;
	uint32_t tickstart = HAL_GetTick();
	while ((HAL_GetTick() - tickstart) < 10U)
	{
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
		{
			timeoutOccurred = 0;
			if(ReadData(CAN_RX_FIFO0, RxData_Float))
				return AKxx_x_PollMotorReadWithTimeoutError;
			break;
		}
		else if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1))
		{
			timeoutOccurred = 0;
			if(ReadData(CAN_RX_FIFO1, RxData_Float))
				return AKxx_x_PollMotorReadWithTimeoutError;
			break;
		}
	}

	if(timeoutOccurred)
		return AKxx_x_PollMotorReadWithTimeoutError;

	return AKxx_x_NoError;
}

AKxx_x_Error_e AKxx_x_PollTxMessagePendingWithTimeout(uint32_t txMailbox)
{
	uint8_t timeoutOccurred = 1;
	uint32_t tickstart = HAL_GetTick();
	while ((HAL_GetTick() - tickstart) < 10U)
	{
		if(!HAL_CAN_IsTxMessagePending(&hcan1,txMailbox))
		{
			timeoutOccurred = 0;
			break;
		}
	}

	if(timeoutOccurred)
		return AKxx_x_PollTxMessagePendingWithTimeoutError;

	return AKxx_x_NoError;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static AKxx_x_Error_e ReadData(uint32_t rxFifo, AKxx_x_ReadData_t *RxData_Float)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rxData_uint[8];
	if(HAL_CAN_GetRxMessage(&hcan1, rxFifo, &RxHeader, rxData_uint) != HAL_OK)
		return AKxx_x_ReadDataError;

	UnpackData(rxData_uint, RxData_Float);

	return AKxx_x_NoError;
}

static AKxx_x_Error_e WriteData(uint8_t deviceIndex, AKxx_x_WriteData_t *TxData_Float)
{
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = 8;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = Device[deviceIndex].InitVals.canId;
	TxHeader.TransmitGlobalTime = DISABLE;

	uint8_t txData_uint[8];
	PackData(deviceIndex, TxData_Float, txData_uint);

	uint32_t txMailbox;
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData_uint, &txMailbox) != HAL_OK)
		return AKxx_x_WriteDataError;

	if(AKxx_x_PollTxMessagePendingWithTimeout(txMailbox)) //??
		return AKxx_x_WriteDataError;

	return AKxx_x_NoError;
}

static AKxx_x_Error_e EnterMotorCtrlMode(uint8_t deviceIndex)
{
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = 8;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = Device[deviceIndex].InitVals.canId;
	TxHeader.TransmitGlobalTime = DISABLE;

	uint8_t txData_uint[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC};
	uint32_t txMailbox;
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData_uint, &txMailbox) != HAL_OK)
		return AKxx_x_EnterMotorCtrlModeError;

	if(AKxx_x_PollTxMessagePendingWithTimeout(txMailbox)) //??
		return AKxx_x_EnterMotorCtrlModeError;

	return AKxx_x_NoError;
}

static void PackData(uint8_t deviceIndex, AKxx_x_WriteData_t *TxData_Float, uint8_t *txData_uint)
{
	uint16_t position = FloatToUint(TxData_Float->position, -12.5, 12.5, 16);
	uint16_t speed = FloatToUint(TxData_Float->speed, Device[deviceIndex].speedMin, Device[deviceIndex].speedMax, 12);
	uint16_t torque = FloatToUint(TxData_Float->torque, Device[deviceIndex].torqueMin, Device[deviceIndex].torqueMax, 12);
	uint16_t kd = FloatToUint(TxData_Float->kd, 0, 5, 12);
	uint16_t kp = FloatToUint(TxData_Float->kp, 0, 500, 12);

	txData_uint[0] = position >> 8;
	txData_uint[1] = position & 0xFF;
	txData_uint[2] = speed >> 4;
	txData_uint[3] = ((speed & 0x0F) << 4) | (kp >> 8);
	txData_uint[4] = kp & 0xFF;
	txData_uint[5] = kd >> 4;
	txData_uint[6] = ((kd & 0x0F) << 4) | (torque >> 8);
	txData_uint[7] = torque & 0xFF;
}

static void UnpackData(uint8_t *rxData_uint, AKxx_x_ReadData_t *RxData_Float)
{
	uint8_t canId = rxData_uint[0];
	uint16_t position = (rxData_uint[1] << 8) | rxData_uint[2];
	uint16_t speed = (rxData_uint[3] << 4) | (rxData_uint[4] >> 4);
	uint16_t torque = ((rxData_uint[4] & 0x0F) << 8) | rxData_uint[5];

	uint8_t i;
	for(i = 0; i < AKXX_X_NUMBER_OF_DEVICES; i++)
		if(Device[i].InitVals.canId == canId)
		{
			RxData_Float->canId = canId;
			break;
		}

	RxData_Float->position = UintToFloat(position, -12.5, 12.5, 16);
	RxData_Float->speed = UintToFloat(speed, Device[i].speedMin, Device[i].speedMax, 12);
	RxData_Float->torque = UintToFloat(torque, Device[i].torqueMin, Device[i].torqueMax, 12);
}

static float UintToFloat(uint16_t x_uint, float xMin_float, float xMax_float, uint8_t nBits)
{
	float offset = xMin_float;
	float span = xMax_float - xMin_float;
	float x_float;
	if(nBits == 12)
		x_float = (((float)x_uint) * span / 4095.0f) + offset;
	else if(nBits == 16)
		x_float = (((float)x_uint) * span / 65535.0f) + offset;

	return x_float;
}

static uint16_t FloatToUint(float x_float, float xMin_float, float xMax_float, uint8_t nBits)
{
	if(x_float < xMin_float)
		x_float = xMin_float;
	if(x_float > xMax_float)
		x_float = xMax_float;

	float offset = xMin_float;
	float span = xMax_float - xMin_float;
	uint16_t x_uint;
	if(nBits == 12)
		x_uint = (uint16_t)((x_float - offset) * 4095.0f / span);
	else if(nBits == 16)
		x_uint = (uint16_t)((x_float - offset) * 65535.0f / span);

	return x_uint;
}


/*******************************************************************************
* END
*******************************************************************************/
