/*******************************************************************************
*
* TITLE: SPI Driver for CEVA BNO085 and BNO086 IMU with HAL
*
* NOTES
* 1. This driver is directly from:
*		https://github.com/ceva-dsp/sh2-demo-nucleo/blob/main/app/demo_app.c
* 2. Only pin names were changed in the appropriate locations.
*
*******************************************************************************/


#include "spi_hal.h"

#include "sh2_hal.h"
#include "sh2_err.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "spi.h"
#include "tim.h"


// Keep reset asserted this long.
// (Some targets have a long RC decay on reset.)
#define RESET_DELAY_US (10000)

// Wait up to this long to see first interrupt from SH
#define START_DELAY_US (2000000)

// Wait this long before assuming bootloader is ready
#define DFU_BOOT_DELAY_US (50000)

#define DFU_CS_TIMING_US (20)
#define DFU_BYTE_TIMING_US (28)
#define DFU_CS_DEASSERT_DELAY_RX_US (0)
#define DFU_CS_DEASSERT_DELAY_TX_US (5000)

// How many bytes to read when reading the length field
#define READ_LEN (4)

// ------------------------------------------------------------------------
// Private types

typedef enum SpiState_e
{
    SPI_INIT,
    SPI_DUMMY,
    SPI_DFU,
    SPI_IDLE,
    SPI_RD_HDR,
    SPI_RD_BODY,
    SPI_WRITE
} SpiState_t;

// ------------------------------------------------------------------------
// Private data

// Dummy transmit data for SPI reads
static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER_IN] = {0};

// SPI Bus access state machine state
static SpiState_t spiState = SPI_INIT;

// Timestamp
static volatile uint32_t rxTimestamp_us;

// true from time SH is put in reset until first INTN indication
static volatile bool inReset;

// set true when INTN is observed, until RX operation starts
static volatile bool rxReady;

// Receive support
static uint8_t rxBuf[SH2_HAL_MAX_TRANSFER_IN];
static volatile uint32_t rxBufLen;
static volatile bool rxDataReady;

// Transmit support
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_OUT];
static uint32_t txBufLen;

// Instances of the SPI HAL for SH2 and DFU
static sh2_Hal_t sh2Hal;
static sh2_Hal_t dfuHal;

static bool isOpen = false;

// ------------------------------------------------------------------------
// Private methods

static void bootn(bool state)
{
    HAL_GPIO_WritePin(ANKLE_IMU_BT_GPIO_Port, ANKLE_IMU_BT_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rstn(bool state)
{
    HAL_GPIO_WritePin(ANKLE_IMU_RST_GPIO_Port, ANKLE_IMU_RST_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ps0_waken(bool state)
{
    HAL_GPIO_WritePin(ANKLE_IMU_P0_GPIO_Port, ANKLE_IMU_P0_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ps1(bool state)
{
    HAL_GPIO_WritePin(ANKLE_IMU_P1_GPIO_Port, ANKLE_IMU_P1_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void csn(bool state)
{
    HAL_GPIO_WritePin(ANKLE_IMU_CS_GPIO_Port, ANKLE_IMU_CS_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t timeNowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

static void hal_init_timer(void)
{

    HAL_TIM_Base_Start(&htim2);
}



static void spiDummyOp(void)
{
    // We need to establish SCLK in proper initial state.
    // Do one SPI operation with reset asserted and no CS asserted to get clock sorted.
    uint8_t dummyTx[1];
    uint8_t dummyRx[1];

    memset(dummyTx, 0xAA, sizeof(dummyTx));

    HAL_SPI_TransmitReceive(&hspi1, dummyTx, dummyRx, sizeof(dummyTx), 2);
}



static void hal_init_hw(bool dfu)
{
    hal_init_timer();

}

static void enableInts(void)
{
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

static void disableInts()
{
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

// Attempt to start a SPI operation.
// This can be done from interrupt context or with interrupts disabled.
// If SPI periph is not in use and there is data to send or receive,
// this will start a SPI operation.
static void spiActivate(void)
{
    if ((spiState == SPI_IDLE) && (rxBufLen == 0))
    {
        if (rxReady)
        {
            // reset flag that was set with INTN
            rxReady = false;

            // assert CSN
            csn(false);

            if (txBufLen > 0)
            {
                spiState = SPI_WRITE;

                // Start operation to write (and, incidentally, read)
                HAL_SPI_TransmitReceive_IT(&hspi1, txBuf, rxBuf, txBufLen);

                // Deassert Wake
                ps0_waken(true);
            }
            else
            {
                spiState = SPI_RD_HDR;

                // Start SPI operation to read header (writing zeros)
                HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)txZeros, rxBuf, READ_LEN);
            }
        }
    }
}

// Handle the end of a SPI operation.
// This can be done from interrupt context or with interrupts disabled.
// Depending on spiState, it may start a follow-up operation or transition
// to idle.  In the latter case, it will call spiActivate
static void spiCompleted(void)
{
    // Get length of payload available
    uint16_t rxLen = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;

    // Truncate that to max len we can read
    if (rxLen > sizeof(rxBuf))
    {
        rxLen = sizeof(rxBuf);
    }

    if (spiState == SPI_DUMMY)
    {
        // SPI Dummy operation completed, transition now to idle
        spiState = SPI_IDLE;
    }
    else if (spiState == SPI_RD_HDR)
    {
        // We read a header

        if (rxLen > READ_LEN) {
            // There is more to read

            // Transition to RD_BODY state
            spiState = SPI_RD_BODY;

            // Start a read operation for the remaining length.  (We already read the first READ_LEN bytes.)
            HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)txZeros, rxBuf+READ_LEN, rxLen-READ_LEN);
        }
        else
        {
            // No SHTP payload was received, this operation is done
            csn(true);            // deassert CSN
            rxBufLen = 0;         // no rx data available
            spiState = SPI_IDLE;  // back to idle state
            spiActivate();        // activate next operation, if any.
        }
    }
    else if (spiState == SPI_RD_BODY)
    {
        // We completed the read or write of a payload
        // deassert CSN.
        csn(true);

        // Check len of data read and set rxBufLen
        rxBufLen = rxLen;

        // transition back to idle state
        spiState = SPI_IDLE;

        // Activate the next operation, if any.
        spiActivate();
    }
    else if (spiState == SPI_WRITE)
    {
        // We completed the read or write of a payload
        // deassert CSN.
        csn(true);

        // Since operation was a write, transaction was for txBufLen bytes.  So received
        // data len is, at a maximum, txBufLen.
        rxBufLen = (txBufLen < rxLen) ? txBufLen : rxLen;

        // Tx buffer is empty now.
        txBufLen = 0;

        // transition back to idle state
        spiState = SPI_IDLE;

        // Activate the next operation, if any.
        spiActivate();
    }
}


// Interrupt handlers and SPI operation callbacks

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    rxTimestamp_us = timeNowUs();

    inReset = false;
    rxReady = true;

    // Start read, if possible
    spiActivate();
}

// Handle INTN Interrupt through STM32 HAL
// (It, in turn, calls HAL_GPIO_EXTI_Callback, above)
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi1)
{
    if (isOpen)
    {
        spiCompleted();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi1)
{
    // Shouldn't happen
    while (1);
}

//// Handle SPI1 Global Interrupt
//void SPI1_IRQHandler(void)
//{
//    HAL_SPI_IRQHandler(&hspi1);
//}

void delayUs(uint32_t delay)
{
    volatile uint32_t now = timeNowUs();
    uint32_t start = now;
    while ((now - start) < delay) {
        now = timeNowUs();
    }
}

void resetDelayUs(uint32_t delay)
{
    volatile uint32_t now = timeNowUs();
    uint32_t start = now;
    while (((now - start) < delay) && (inReset))
    {
        now = timeNowUs();
    }
}

// ------------------------------------------------------------------------
// SH2 SPI Hal Methods

static int sh2_spi_hal_open(sh2_Hal_t *self)
{
    int retval = SH2_OK;

    if (isOpen)
    {
        // Can't open if another instance is already open
        return SH2_ERR;
    }

    isOpen = true;

    // Init hardware (false -> non-DFU config)
    hal_init_hw(false);

    // Hold in reset
    rstn(false);

    // deassert CSN
    csn(true);

    // Clear rx, tx buffers
    rxBufLen = 0;
    txBufLen = 0;
    rxDataReady = false;
    rxReady = false;

    inReset = true;  // will change back to false when INTN serviced

    // Do dummy SPI operation
    // (First SPI op after reconfig has bad initial state of signals
    // so this is a throwaway operation.  Afterward, all is well.)
    spiState = SPI_DUMMY;
    spiDummyOp();
    spiState = SPI_IDLE;

    // Delay for RESET_DELAY_US to ensure reset takes effect
    delayUs(RESET_DELAY_US);

    // To boot in SHTP-SPI mode, must have PS1=1, PS0=1.
    // PS1 is set via jumper.
    // PS0 will be 1 PS1 jumper is 1 AND PS0_WAKEN sig is 1.
    // So we set PS0_WAKEN signal to 1
    ps0_waken(true);
    ps1(true);

    // Deassert reset, boot in non-DFU mode
    bootn(true);
    rstn(true);

    // enable interrupts
    enableInts();

    // Wait for INTN to be asserted
    resetDelayUs(START_DELAY_US);

    return retval;
}

static void sh2_spi_hal_close(sh2_Hal_t *self)
{
    // Disable interrupts
    disableInts();

    // Set state machine to INIT state
    spiState = SPI_INIT;

    // Hold sensor hub in reset
    rstn(false);

    // deassert CSN
    csn(true);

    // Deinit SPI peripheral
    HAL_SPI_DeInit(&hspi1);

    // Deinit timer
    __HAL_TIM_DISABLE(&htim2);

    // No longer open
    isOpen = false;
}

static int sh2_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    int retval = 0;

    // If there is received data available...
    if (rxBufLen > 0)
    {
        // And if the data will fit in this buffer...
        if (len >= rxBufLen)
        {
            // Copy data to the client buffer
            memcpy(pBuffer, rxBuf, rxBufLen);
            retval = rxBufLen;

            // Set timestamp of that data
            *t = rxTimestamp_us;

            // Clear rxBuf so we can receive again
            rxBufLen = 0;
        }
        else
        {
            // Discard what was read and return error because buffer was too small.
            retval = SH2_ERR_BAD_PARAM;
            rxBufLen = 0;
        }

        // Now that rxBuf is empty, activate SPI processing to send any
        // potential write that was blocked.
        disableInts();
        spiActivate();
        enableInts();
    }

    return retval;
}

static int sh2_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    int retval = SH2_OK;

    // Validate parameters
    if ((self == 0) || (len > sizeof(txBuf)) ||
        ((len > 0) && (pBuffer == 0)))
    {
        return SH2_ERR_BAD_PARAM;
    }

    // If tx buffer is not empty, return 0
    if (txBufLen != 0)
    {
        return 0;
    }

    // Copy data to tx buffer
    memcpy(txBuf, pBuffer, len);
    txBufLen = len;
    retval = len;

    // disable SH2 interrupts for a moment
    disableInts();

    // Assert Wake
    ps0_waken(false);

    // re-enable SH2 interrupts.
    enableInts();

    return retval;
}

static uint32_t sh2_spi_hal_getTimeUs(sh2_Hal_t *self)
{
    return timeNowUs();
}

// ------------------------------------------------------------------------
// DFU SPI Hal Methods

static int dfu_spi_hal_open(sh2_Hal_t *self)
{
    int retval = SH2_OK;

    if (isOpen)
    {
        // Can't open if another instance is already open
        return SH2_ERR;
    }

    isOpen = true;

    // Init hardware (true -> DFU config)
    hal_init_hw(true);

    // Hold in reset, for DFU
    rstn(false);

    // deassert CSN
    csn(true);

    // Clear rx, tx buffers
    rxBufLen = 0;
    txBufLen = 0;
    rxDataReady = false;
    rxReady = false;

    inReset = true;  // will change back to false when INTN serviced

    // Do dummy SPI operation
    // (First SPI op after reconfig has bad initial state of signals
    // so this is a throwaway operation.  Afterward, all is well.)
    spiState = SPI_DUMMY;
    spiDummyOp();
    spiState = SPI_DFU;

    // Delay for RESET_DELAY_US to ensure reset takes effect
    delayUs(RESET_DELAY_US);

    // To boot in SHTP-SPI mode, must have PS1=1, PS0=1.
    // PS1 is set via jumper.
    // PS0 will be 1 PS1 jumper is 1 AND PS0_WAKEN sig is 1.
    // So we set PS0_WAKEN signal to 1
    ps0_waken(true);
    ps1(true);

    // Deassert reset, boot in DFU mode
    bootn(false);
    rstn(true);

    // enable interrupts
    enableInts();

    // Wait for bootloader to be ready
    delayUs(DFU_BOOT_DELAY_US);

    return retval;
}

static void dfu_spi_hal_close(sh2_Hal_t *self)
{
    // Disable interrupts
    disableInts();

    // Set state machine to INIT state
    spiState = SPI_INIT;

    // Hold sensor hub in reset
    rstn(false);

    // deassert CSN
    csn(true);

    // Deinit SPI peripheral
    HAL_SPI_DeInit(&hspi1);

    // Deinit timer
    __HAL_TIM_DISABLE(&htim2);

    // No longer open
    isOpen = false;
}

static int dfu_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    int retval = 0;
    int rc = 0;

    // DFU HAL always returns 0 in timestamp param
    *t = 0;

    // assert CSN, delay to meet bootloader timing requirements
    csn(false);
    delayUs(DFU_CS_TIMING_US);

    // perform transfer, one byte at a time
    for (int n = 0; n < len; n++)
    {
        rc = HAL_SPI_Receive(&hspi1, pBuffer+n, 1, 2);
        delayUs(DFU_BYTE_TIMING_US);
        if (rc != 0)
        {
            break;
        }
    }

    // Set return status
    if (rc == 0)
    {
        retval = len;
    }
    else
    {
        retval = SH2_ERR_IO;
    }

    // deassert CSN, delay to meet bootloader timing requirements
    csn(true);
    delayUs(DFU_CS_DEASSERT_DELAY_RX_US);

    return retval;
}

static int dfu_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    int retval = SH2_OK;
    int rc;

    rc = 0;

    // assert CSN, delay
    csn(false);
    delayUs(DFU_CS_TIMING_US);

    // perform bytewise writes
    for (int n = 0; n < len; n++)
    {
        rc = HAL_SPI_Transmit(&hspi1, pBuffer+n, 1, 2);
        delayUs(DFU_BYTE_TIMING_US);
        if (rc != 0)
        {
            break;
        }
    }

    // set up return status
    if (rc != 0)
    {
        retval = SH2_ERR_IO;
    }
    else
    {
        retval = len;
    }

    // deassert CSN, delay
    csn(true);
    delayUs(DFU_CS_DEASSERT_DELAY_TX_US);

    return retval;
}

static uint32_t dfu_spi_hal_getTimeUs(sh2_Hal_t *self)
{
    return timeNowUs();
}

// ------------------------------------------------------------------------
// Public methods

sh2_Hal_t *sh2_hal_init(void)
{
    // Set up the HAL reference object for the client
    sh2Hal.open = sh2_spi_hal_open;
    sh2Hal.close = sh2_spi_hal_close;
    sh2Hal.read = sh2_spi_hal_read;
    sh2Hal.write = sh2_spi_hal_write;
    sh2Hal.getTimeUs = sh2_spi_hal_getTimeUs;

    return &sh2Hal;
}

sh2_Hal_t *dfu_hal_init(void)
{
    // Set up the HAL reference object for the client
    dfuHal.open = dfu_spi_hal_open;
    dfuHal.close = dfu_spi_hal_close;
    dfuHal.read = dfu_spi_hal_read;
    dfuHal.write = dfu_spi_hal_write;
    dfuHal.getTimeUs = dfu_spi_hal_getTimeUs;

    return &dfuHal;
}



