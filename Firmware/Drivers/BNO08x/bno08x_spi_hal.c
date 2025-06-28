/*******************************************************************************
*
* TITLE: Driver for CEVA BNO085 and BNO086 IMU using SPI with HAL
*
* NOTES
* 1. This driver is based on:
*		https://github.com/ceva-dsp/sh2-demo-nucleo/blob/main/app/demo_app.c
* 2. User may add their desired reports to StartReports() and ReadEvent().
* 3. This driver is setup to work on SPI1.
*
*******************************************************************************/

#include "bno08x_spi_hal.h"
#include "spi_hal.h"
#include "euler.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "sh2_util.h"

#include <string.h>


/*******************************************************************************
* PUBLIC DEFINTIONS
*******************************************************************************/

float BNO08x_IMU_Data[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t BNO08x_resetOccurred = 0;


/*******************************************************************************
* PRIVATE DEFINTIONS
*******************************************************************************/

static uint8_t isInit = 0;

static BNO08x_Error_e StartReports(void);
static void EventHandler(void * cookie, sh2_AsyncEvent_t *pEvent);
static void ReadEvent(void * cookie, sh2_SensorEvent_t * event, int16_t *data);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

BNO08x_Error_e BNO08x_Init(void)
{
	sh2_Hal_t *pSh2Hal = 0;
	pSh2Hal = sh2_hal_init();
	int status = sh2_open(pSh2Hal, EventHandler, NULL);
	if(status != SH2_OK)
		return BNO08x_InitError;

	sh2_setSensorCallback(ReadEvent, NULL); // incompatible??

  	if(StartReports())
  		return BNO08x_InitError;

  	isInit = 1;

	return BNO08x_NoError;
}

BNO08x_Error_e BNO08x_StartReports(void)
{
	if(!isInit)
		while(1);

	return StartReports();
}


void BNO08x_ReadSensors(void)
{
	if(!isInit)
		while(1);

	sh2_service();
}

/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static BNO08x_Error_e StartReports(void)
{
    static const struct
	{
        int sensorId;
        sh2_SensorConfig_t config;
    }

    sensorConfig[] =
    {
		{SH2_ACCELEROMETER, {.reportInterval_us = 2000}},			// m/s^2, max interval = 500 Hz = 2000 us
		{SH2_GYROSCOPE_CALIBRATED, {.reportInterval_us = 2500}},	// rad/s, max interval = 400 Hz = 2500 us
        {SH2_GAME_ROTATION_VECTOR, {.reportInterval_us = 2500}},	// quaternions, max interval = 400 Hz = 2500 us
    };

    for (int n = 0; n < ARRAY_LEN(sensorConfig); n++)
    {
        int sensorId = sensorConfig[n].sensorId;

        int status = sh2_setSensorConfig(sensorId, &sensorConfig[n].config);
        if (status != 0)
        	return BNO08x_StartReportError;
    }

    return BNO08x_NoError;
}

static void EventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    if (pEvent->eventId == SH2_RESET)
        BNO08x_resetOccurred = 1;
}

static void ReadEvent(void * cookie, sh2_SensorEvent_t * event, int16_t *data)
{
    int rc;
    sh2_SensorValue_t value;

    rc = sh2_decodeSensorEvent(&value, event);
    if (rc != SH2_OK)
        return;

    switch(value.sensorId)
    {
        case SH2_ACCELEROMETER:
        	BNO08x_IMU_Data[0] = value.un.accelerometer.x;
        	BNO08x_IMU_Data[1] = value.un.accelerometer.y;
        	BNO08x_IMU_Data[2] = value.un.accelerometer.z;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
        	BNO08x_IMU_Data[3] = value.un.gyroscope.x;
        	BNO08x_IMU_Data[4] = value.un.gyroscope.y;
        	BNO08x_IMU_Data[5] = value.un.gyroscope.z;
            break;

        case SH2_GAME_ROTATION_VECTOR:
        	BNO08x_IMU_Data[6] = value.un.gameRotationVector.real;
        	BNO08x_IMU_Data[7] = value.un.gameRotationVector.i;
        	BNO08x_IMU_Data[8] = value.un.gameRotationVector.j;
        	BNO08x_IMU_Data[9] = value.un.gameRotationVector.k;
            break;
    }
}


/*******************************************************************************
* END
*******************************************************************************/
