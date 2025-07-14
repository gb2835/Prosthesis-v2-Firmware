/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_lptim.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSCOPE_Pin GPIO_PIN_13
#define OSCOPE_GPIO_Port GPIOC
#define ANKLE_IMU_CS_Pin GPIO_PIN_4
#define ANKLE_IMU_CS_GPIO_Port GPIOA
#define IMU_SCL_Pin GPIO_PIN_5
#define IMU_SCL_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_0
#define LED_BLUE_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define KNEE_IMU_BT_Pin GPIO_PIN_12
#define KNEE_IMU_BT_GPIO_Port GPIOB
#define KNEE_IMU_P0_Pin GPIO_PIN_13
#define KNEE_IMU_P0_GPIO_Port GPIOB
#define KNEE_IMU_P1_Pin GPIO_PIN_14
#define KNEE_IMU_P1_GPIO_Port GPIOB
#define KNEE_IMU_RST_Pin GPIO_PIN_15
#define KNEE_IMU_RST_GPIO_Port GPIOB
#define KNEE_IMU_CS_Pin GPIO_PIN_4
#define KNEE_IMU_CS_GPIO_Port GPIOB
#define KNEE_IMU_INT_Pin GPIO_PIN_5
#define KNEE_IMU_INT_GPIO_Port GPIOB
#define KNEE_IMU_INT_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */


/*******************************************************************************
* USER ADDED MAIN.H
*******************************************************************************/

extern uint8_t isProsthesisControlRequired;


/******************************************************************************/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
