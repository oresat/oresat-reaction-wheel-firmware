/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
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
/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

/* Override the __weak HAL_IncTick from stm32g4xx_hal.c so the SysTick ISR
 * also refreshes the IWDG every 1ms. Restores the v7 auto-kick that was lost
 * when CubeMX regenerated the HAL driver.
 *
 * Kick via the peripheral address directly, NOT through hiwdg: the bootloader
 * jumps here with SysTick still running and SystemInit points VTOR at this
 * image's vector table immediately, so this ISR can fire during Reset_Handler's
 * copy/fill loops -- before .bss is zeroed -- where hiwdg.Instance is leftover
 * RAM garbage and the deref is a bus fault (a NULL check does not cover that
 * window). Reload-while-stopped is a no-op, so no guard is needed at all. */
void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
  IWDG->KR = IWDG_KEY_RELOAD;
}

/* USER CODE END 1 */

