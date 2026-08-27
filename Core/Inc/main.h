/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define NOR_T1OC_Pin GPIO_PIN_0
#define NOR_T1OC_GPIO_Port GPIOA
#define NOR_T1OC_EXTI_IRQn EXTI0_IRQn
#define CK_READ_Pin GPIO_PIN_2
#define CK_READ_GPIO_Port GPIOA
#define OUT_HG_Pin GPIO_PIN_4
#define OUT_HG_GPIO_Port GPIOA
#define OUT_LG_Pin GPIO_PIN_5
#define OUT_LG_GPIO_Port GPIOA
#define CLK_SM_I2C_Pin GPIO_PIN_9
#define CLK_SM_I2C_GPIO_Port GPIOA
#define RSTN_I2C_Pin GPIO_PIN_0
#define RSTN_I2C_GPIO_Port GPIOD
#define RSTN_READ_Pin GPIO_PIN_1
#define RSTN_READ_GPIO_Port GPIOD
#define HOLDEXT_Pin GPIO_PIN_2
#define HOLDEXT_GPIO_Port GPIOD
#define ERRORN_SC_Pin GPIO_PIN_3
#define ERRORN_SC_GPIO_Port GPIOD
#define ERRORN_SC_EXTI_IRQn EXTI3_IRQn
#define RESET_N_Pin GPIO_PIN_4
#define RESET_N_GPIO_Port GPIOD
#define RSTN_SC_Pin GPIO_PIN_7
#define RSTN_SC_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
