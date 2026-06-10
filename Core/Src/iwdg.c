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

/* Foreground-liveness gate for the IWDG kick below. An unconditional SysTick feed proves only
 * that interrupts run — but every host-facing service (SDO server, NMT incl. reset-node, the
 * 0x4007 reboot register, TPDO payload updates) lives in the FOREGROUND loop, so a wedged
 * foreground with live ISRs was permanently uncommandable while the watchdog stayed happily
 * fed (flight-review P1-8). app_main's while(1) strokes this every pass; the gate arms on the
 * first stroke, so the entire boot path stays unconditionally fed. 2 s dwarfs every legitimate
 * foreground stall (cal-store save ≈35 ms, full console drain ≈22 ms); once exceeded the feed
 * stops and the IWDG fires ≤512 ms later — worst-case ~2.5 s to a clean reboot. */
#define IWDG_FOREGROUND_WINDOW_MS 2000u
static volatile uint32_t iwdg_fg_last_alive = 0;
static volatile uint32_t iwdg_fg_gate_armed = 0;

void iwdg_foreground_alive(void)
{
  iwdg_fg_last_alive = uwTick;
  iwdg_fg_gate_armed = 1;
}

/* Override the __weak HAL_IncTick from stm32g4xx_hal.c so the SysTick ISR
 * also refreshes the IWDG every 1ms. Restores the v7 auto-kick that was lost
 * when CubeMX regenerated the HAL driver.
 *
 * Kick via the peripheral address directly, NOT through hiwdg: the bootloader
 * jumps here with SysTick still running and SystemInit points VTOR at this
 * image's vector table immediately, so this ISR can fire during Reset_Handler's
 * copy/fill loops -- before .bss is zeroed -- where hiwdg.Instance is leftover
 * RAM garbage and the deref is a bus fault (a NULL check does not cover that
 * window). Reload-while-stopped is a no-op, so no guard is needed at all.
 * (.bss zeroing also disarms the liveness gate for the boot phase, which is
 * exactly the desired pre-main behavior.) */
void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
  if (iwdg_fg_gate_armed == 0 || (uwTick - iwdg_fg_last_alive) < IWDG_FOREGROUND_WINDOW_MS)
    IWDG->KR = IWDG_KEY_RELOAD;
}

/* USER CODE END 1 */

