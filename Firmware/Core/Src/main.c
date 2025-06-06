/* USER CODE BEGIN Header */

/*******************************************************************************
*
* TITLE: Prosthesis v2 Firmware
*
* NOTES (check this??)
* 1. The below lines can be used to measure PB2 on oscilloscope (#include main.h may need to be added to certain files):
*		- LL_GPIO_SetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_ResetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_TogglePin(OSCOPE_GPIO_Port, OSCOPE_Pin);
* 2. Search -> File on "* USER ADDED " will show code added to MX auto-generated files.
*
*******************************************************************************/


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lptim.h"
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
* USER ADDED MAIN.C
*******************************************************************************/

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPTIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


/*******************************************************************************
* USER ADDED DEFINITIONS
*******************************************************************************/


/*******************************************************************************
* USER ADDED INITIALIZATIONS
*******************************************************************************/

//	LL_SYSTICK_EnableIT(); do i actually need this?

	LL_LPTIM_Enable(LPTIM2);
	LL_LPTIM_EnableIT_ARRM(LPTIM2);
	LL_LPTIM_SetAutoReload(LPTIM2, LPTIM2_PERIOD);
	LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);


/*******************************************************************************
* USER ADDED TEST PROGRAMS
*******************************************************************************/


/*******************************************************************************
* USER ADDED MAIN LOOP
*******************************************************************************/

  while (1)
  {
	  if(isProsthesisControlRequired)
	  {
		  LL_GPIO_TogglePin(OSCOPE_GPIO_Port, OSCOPE_Pin);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(4000000);

  LL_SetSystemCoreClock(4000000);
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
