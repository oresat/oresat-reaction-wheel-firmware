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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

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
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_6
#define DEBUG_RX_GPIO_Port GPIOD
#define DEBUG_TX_Pin GPIO_PIN_5
#define DEBUG_TX_GPIO_Port GPIOD
#define DEBUG0_Pin GPIO_PIN_4
#define DEBUG0_GPIO_Port GPIOD
#define SPI_MOSI_Pin GPIO_PIN_12
#define SPI_MOSI_GPIO_Port GPIOC
#define DFU_RX_Pin GPIO_PIN_1
#define DFU_RX_GPIO_Port GPIOE
#define CAN_ADDR0_Pin GPIO_PIN_7
#define CAN_ADDR0_GPIO_Port GPIOB
#define CAN_ADDR1_Pin GPIO_PIN_5
#define CAN_ADDR1_GPIO_Port GPIOB
#define DEBUG2_Pin GPIO_PIN_7
#define DEBUG2_GPIO_Port GPIOD
#define DEBUG1_Pin GPIO_PIN_2
#define DEBUG1_GPIO_Port GPIOD
#define SPI_nCS_Pin GPIO_PIN_15
#define SPI_nCS_GPIO_Port GPIOA
#define DFU_TX_Pin GPIO_PIN_0
#define DFU_TX_GPIO_Port GPIOE
#define DEBUG3_Pin GPIO_PIN_3
#define DEBUG3_GPIO_Port GPIOD
#define SPI_MISO_Pin GPIO_PIN_11
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_SCK_Pin GPIO_PIN_10
#define SPI_SCK_GPIO_Port GPIOC
#define PHB_L_Pin GPIO_PIN_11
#define PHB_L_GPIO_Port GPIOA
#define PHB_H_Pin GPIO_PIN_10
#define PHB_H_GPIO_Port GPIOA
#define PHA_L_Pin GPIO_PIN_9
#define PHA_L_GPIO_Port GPIOA
#define PHC_H_Pin GPIO_PIN_8
#define PHC_H_GPIO_Port GPIOC
#define PHC_L_Pin GPIO_PIN_9
#define PHC_L_GPIO_Port GPIOC
#define PHA_H_Pin GPIO_PIN_8
#define PHA_H_GPIO_Port GPIOA
#define nRST_Pin GPIO_PIN_10
#define nRST_GPIO_Port GPIOG
#define PHC_I_Pin GPIO_PIN_1
#define PHC_I_GPIO_Port GPIOA
#define PHA_I_Pin GPIO_PIN_0
#define PHA_I_GPIO_Port GPIOA
#define THERM0_Pin GPIO_PIN_12
#define THERM0_GPIO_Port GPIOE
#define PHA_V_Pin GPIO_PIN_2
#define PHA_V_GPIO_Port GPIOA
#define DAC_1_Pin GPIO_PIN_4
#define DAC_1_GPIO_Port GPIOA
#define THERM1_Pin GPIO_PIN_8
#define THERM1_GPIO_Port GPIOE
#define PHB_V_Pin GPIO_PIN_9
#define PHB_V_GPIO_Port GPIOE
#define DAC_2_Pin GPIO_PIN_5
#define DAC_2_GPIO_Port GPIOA
#define THERM2_Pin GPIO_PIN_11
#define THERM2_GPIO_Port GPIOE
#define VBUS_V_Pin GPIO_PIN_14
#define VBUS_V_GPIO_Port GPIOE
#define PHC_V_Pin GPIO_PIN_7
#define PHC_V_GPIO_Port GPIOA
#define PHB_I_Pin GPIO_PIN_1
#define PHB_I_GPIO_Port GPIOB
#define THERM3_Pin GPIO_PIN_10
#define THERM3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
