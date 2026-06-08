/**
  ******************************************************************************
  * @file    interfaces_conf.h
  * @author  MCD Application Team
  * @brief   Contains Interfaces configuration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once
#include "main.h"

#define MEMORIES_SUPPORTED                7U

/* ------------------------- Definitions for USART -------------------------- */
//#define USE_DEBUG_HANDLE

#ifdef USE_DEBUG_HANDLE
#define USARTx                            (USART2)
#define USARTx_CLK_ENABLE()               __HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOD_CLK_ENABLE()

#define USARTx_TX_PIN                     DEBUG_TX_Pin
#define USARTx_TX_GPIO_PORT               DEBUG_TX_GPIO_Port
#define USARTx_RX_PIN                     DEBUG_RX_Pin
#define USARTx_RX_GPIO_PORT               DEBUG_RX_GPIO_Port
#define USARTx_ALTERNATE                  GPIO_AF7_USART2

#else

#define USARTx                            (USART1)
#define USARTx_CLK_ENABLE()               __HAL_RCC_USART1_CLK_ENABLE()
#define USARTx_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOE_CLK_ENABLE()

#define USARTx_TX_PIN                     DFU_TX_Pin
#define USARTx_TX_GPIO_PORT               DFU_TX_GPIO_Port
#define USARTx_RX_PIN                     DFU_RX_Pin
#define USARTx_RX_GPIO_PORT               DFU_RX_GPIO_Port
#define USARTx_ALTERNATE                  GPIO_AF7_USART1

#endif

// USARTx_ALTERNATE is GPIO_AF7_USART1 if using comms handle
