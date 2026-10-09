/**
  ******************************************************************************
  * @file    common_interface.c
  * @author  MCD Application Team
  * @brief   Contains common functions used by different interfaces
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

/* Includes ------------------------------------------------------------------*/
#include "platform.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Assigns the given value to the Main Stack Pointer (MSP).
  * @param  TopOfMainStack  Main Stack Pointer value to set.
  * @retval None.
  */
void Common_SetMsp(uint32_t TopOfMainStack)
{
  __set_MSP(TopOfMainStack);
}

/**
  * @brief  Set the MSP and branch, with no compiler-generated code in between. Does not return.
  * @param  TopOfMainStack  Main Stack Pointer value to set.
  * @param  Address         Branch target (Thumb bit set).
  * @retval None.
  */
/* Naked on purpose: in C the compiler may reload the jump target from the stack after the MSP
 * write (GCC 16 -Ofast does) -> branch to garbage, HardFault reset loop. */
__attribute__((naked, noreturn)) void Common_JumpWithMsp(uint32_t TopOfMainStack, uint32_t Address)
{
  __asm volatile("msr msp, r0\n"
                 "bx  r1\n");
}

/**
  * @brief  Enable IRQ Interrupts.
  * @retval None.
  */
void Common_EnableIrq(void)
{
  __enable_irq();
}

/**
  * @brief  Disable IRQ Interrupts.
  * @retval None.
  */
void Common_DisableIrq(void)
{
  __disable_irq();
}


