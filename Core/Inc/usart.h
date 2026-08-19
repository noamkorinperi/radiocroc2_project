/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/* USART3 is wired to the ST-Link's Virtual COM Port on this board
 * (PD8 = TX, PD9 = RX), so the debugger cable carries the data stream
 * as well. That is the whole point: one USB cable, not two.
 *
 * 921600 is the fastest rate the ST-LINK/V2-1 bridge on a NUCLEO-F722ZE
 * will hold. At PCLK1 = 54 MHz the divisor lands on 59, giving 915254
 * baud - a 0.7% error, comfortably inside the ~2% a UART tolerates.
 * Drop to 460800 (0.16% error) if the bridge turns out to drop bytes
 * under sustained load; V2-1 is not built for continuous streaming.
 */
#define RR2_LINK_BAUD   921600u
/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

