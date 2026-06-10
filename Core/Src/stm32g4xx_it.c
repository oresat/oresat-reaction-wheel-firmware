/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "util.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

void controller_run_state_machine();
void canopen_1khz_isr();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "crash_record.h"

extern void hrtim_outputs_disable(void); /* pwm.c — lock-free ODISR write, all six outputs */
extern volatile uint32_t controller_errors; /* control_ll.c */

/* Common fault tail: bridge off FIRST (HRTIM autonomously free-runs the last compare values
 * into a spinning rotor while the core is wedged), breadcrumb second, reset third. The old
 * while(1) bodies just burned up to ~512 ms of uncontrolled drive waiting for the IWDG to do
 * the same reset with zero record (flight-review P1-9). No printf in here: the console may be
 * the thing that faulted, and ISR prints only land in a RAM ring the reset wipes anyway —
 * read back post-reboot via the boot banner / OD 0x4006 instead. */
void fault_record_and_reset(uint32_t kind, uint32_t *stack_frame) {
    hrtim_outputs_disable();
    CRASH_RECORD->kind = kind;
    CRASH_RECORD->cfsr = SCB->CFSR;
    CRASH_RECORD->hfsr = SCB->HFSR;
    CRASH_RECORD->bfar = SCB->BFAR;
    CRASH_RECORD->mmfar = SCB->MMFAR;
    CRASH_RECORD->stacked_pc = (stack_frame != NULL) ? stack_frame[6] : 0u;
    CRASH_RECORD->stacked_lr = (stack_frame != NULL) ? stack_frame[5] : 0u;
    CRASH_RECORD->controller_errors = controller_errors;
    CRASH_RECORD->aux = 0u;
    CRASH_RECORD->magic = CRASH_RECORD_MAGIC;
    NVIC_SystemReset();
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;
extern DMA_HandleTypeDef hdma_adc5;
extern DAC_HandleTypeDef hdac2;
extern FDCAN_HandleTypeDef hfdcan1;
extern HRTIM_HandleTypeDef hhrtim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim17;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
    /* The only NMI sources on this configuration are flash ECC double-bit errors (ECCD/ECCD2
     * — CSS and SRAM parity are not enabled). A double-bit error in the cal page (torn save,
     * radiation) raises this on the READ; the old while(1) body starved SysTick, the IWDG
     * reset, and the same boot-time read re-faulted forever: a permanent boot loop before CAN
     * ever came up (flight-review P0-3). Instead: latch the raw ECCR for cal_store /
     * diagnostics, write-1-clear the flags, and RETURN — the load completes with garbage, the
     * blob CRC rejects it, and boot continues uncalibrated (re-cal over CAN: 9→15→21→18). */
    uint32_t eccr = FLASH->ECCR;
    if (eccr & (FLASH_ECCR_ECCD | FLASH_ECCR_ECCD2)) {
        noinit_cell_set(FLASH_ECC_LATCH_CELL, eccr);
        FLASH->ECCR = eccr; /* C/D flags are write-1-to-clear; ADDR/BK are read-only */
        return;
    }
    /* Unknown NMI source: record + reset beats spinning (the IWDG would reset us anyway,
     * ~512 ms later, with no breadcrumb). */
    fault_record_and_reset(CRASH_KIND_NMI_UNKNOWN, NULL);
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
    /* NAKED: no compiler prologue may run before the MSP read, or the stacked-frame pointer
     * is offset and the recorded PC/LR lie (verified prologue-free at -Ofast, but -Og emits
     * a frame — naked makes it build-independent). All exception frames here stack on MSP
     * (bare metal, nothing uses PSP); frame[5]/[6] = LR/PC per the AAPCS exception frame.
     * MemManage/Bus/UsageFault are not enabled in SHCSR, so every fault escalates to here.
     * fault_record_and_reset never returns (NVIC_SystemReset). */
    __asm volatile(
        "mrs r1, msp              \n"
        "movs r0, %0              \n"
        "b fault_record_and_reset \n"
        :: "i" ((int) CRASH_KIND_HARDFAULT));
  /* USER CODE END HardFault_IRQn 0 */
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc5);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc4);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

    controller_run_state_machine();

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  canopen_1khz_isr();

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
  */
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */

  /* USER CODE END TIM7_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  HAL_DAC_IRQHandler(&hdac2);
  /* USER CODE BEGIN TIM7_DAC_IRQn 1 */

  /* USER CODE END TIM7_DAC_IRQn 1 */
}

/**
  * @brief This function handles HRTIM timer A global interrupt.
  */
// __ccmram (via util.h -> typedefs.h): the fast-loop entry point should run from CCM like
// the rest of the per-tick chain — it was the last hot-path code still fetching from flash
// (~1% of CPU in the 2026-06-09 PC-sampling profile). Empty in Debug builds, same as every
// other hot function. NOTE: hand-edited outside USER CODE markers — this project never
// regenerates from CubeMX (owner's decree, 2026-06-09).
__ccmram void HRTIM1_TIMA_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 0 */

    HRTIM_HandleTypeDef *hhrtim = &hhrtim1;
    u32 TimerIdx = HRTIM_TIMERINDEX_TIMER_A;

    uint32_t tisrflags = READ_REG(hhrtim->Instance->sTimerxRegs[TimerIdx].TIMxISR);
    uint32_t tdierits = READ_REG(hhrtim->Instance->sTimerxRegs[TimerIdx].TIMxDIER);

    if ((uint32_t) (tisrflags & HRTIM_TIM_FLAG_REP) != (uint32_t) RESET) {
        if ((uint32_t) (tdierits & HRTIM_TIM_IT_REP) != (uint32_t) RESET) {
            __HAL_HRTIM_TIMER_CLEAR_IT(hhrtim, TimerIdx, HRTIM_TIM_IT_REP);

#if (USE_HAL_HRTIM_REGISTER_CALLBACKS == 1)
            hhrtim->RepetitionEventCallback(hhrtim, TimerIdx);
#else
            HAL_HRTIM_RepetitionEventCallback(hhrtim, TimerIdx);
#endif
            // REP is the only TIMA interrupt enabled (hrtim.c: pTimerCfg.InterruptRequests
            // = HRTIM_TIM_IT_REP), so skip HAL_HRTIM_IRQHandler's full flag walk — it was
            // ~5% of total CPU finding nothing. If more TIMA ITs are ever enabled this
            // stays safe: a still-pending enabled flag keeps the IRQ line asserted and
            // the NVIC re-enters immediately after exit.
            return;
        }
    }

  /* USER CODE END HRTIM1_TIMA_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A);
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 1 */

  /* USER CODE END HRTIM1_TIMA_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
