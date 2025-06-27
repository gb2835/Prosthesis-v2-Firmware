/* USER CODE BEGIN Header */

/*******************************************************************************
*
* TITLE: Prosthesis v2 Firmware
*
* NOTES
* 1. IMPORTANT: Motor position must be re-zeroed whenever the motor is reassembled into the device.
*    A test program is provided to zero the motors. ??
* 2. The below lines can be used to measure on oscilloscope (#include main.h may need to be added to certain files):
*		- LL_GPIO_SetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_ResetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_TogglePin(OSCOPE_GPIO_Port, OSCOPE_Pin);
* 3. Search -> File on "* USER ADDED " will show code added to MX auto-generated files.
*
*******************************************************************************/


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*******************************************************************************
* USER ADDED PRELIMINARIES
*******************************************************************************/

#include "akxx-x.h"
#include "bno08x_spi_hal.h"
#include "error_handler.h"
#include "prosthesis_v2.h"

#define LPTIM2_PERIOD	0x3F	// Timer frequency = timer clock frequency / (prescaler * (period + 1))


/******************************************************************************/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_LPTIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


/*******************************************************************************
* USER ADDED DEFINITIONS
*******************************************************************************/

  	Prosthesis_Init_t Prosthesis_Init;
	Prosthesis_Init.Joint = Combined;
	Prosthesis_Init.Side = Left;

  	AKxx_x_Init_t Motor_Init[AKXX_X_NUMBER_OF_DEVICES];
  	Motor_Init[AnkleIndex].canId = AnkleMotorCAN_ID;
  	Motor_Init[AnkleIndex].Motor = AK80_9;

  	// how to use both fifos??
	CAN_FilterTypeDef CAN1_FilterInit;
	CAN1_FilterInit.FilterActivation = ENABLE;
	CAN1_FilterInit.FilterBank = 0; //??
	CAN1_FilterInit.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN1_FilterInit.FilterIdHigh = 0x0000;
	CAN1_FilterInit.FilterIdLow = 0x0000;
	CAN1_FilterInit.FilterMaskIdHigh = 0x0000;
	CAN1_FilterInit.FilterMaskIdLow = 0x0000;
	CAN1_FilterInit.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN1_FilterInit.FilterScale = CAN_FILTERSCALE_32BIT;//??


/*******************************************************************************
* USER ADDED INITIALIZATIONS
*******************************************************************************/

	LL_LPTIM_Enable(LPTIM2);
	LL_LPTIM_EnableIT_ARRM(LPTIM2);
	LL_LPTIM_SetAutoReload(LPTIM2, LPTIM2_PERIOD);
	LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);

	if(HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterInit))
		ErrorHandler_Pv2(CAN_Error);
	if(HAL_CAN_Start(&hcan1))
		ErrorHandler_Pv2(CAN_Error);

  	if(BNO08x_Init())
  		ErrorHandler_BNO08x();

	if(AKxx_x_Init(AnkleIndex, &Motor_Init[AnkleIndex]))
		ErrorHandler_AKxx_x(AnkleIndex);

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
		ErrorHandler_Pv2(CAN_Error);

	InitProsthesisControl(&Prosthesis_Init);


/*******************************************************************************
* USER ADDED TEST PROGRAMS
*******************************************************************************/

	RequireTestProgram(ImpedanceControl);


/*******************************************************************************
* USER ADDED MAIN LOOP
*******************************************************************************/

  while(1)
  {
	  if(isProsthesisControlRequired)
	  {
		  RunProsthesisControl();
		  isProsthesisControlRequired = 0;
	  }


/******************************************************************************/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 10, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(80000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
