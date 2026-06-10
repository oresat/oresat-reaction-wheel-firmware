//
// Created by solomon on 11/17/22.
//

#pragma once

#include "typedefs.h"

#include "adc.h"
#include "cordic.h"
#include "fdcan.h"
#include "fmac.h"
#include "gpio.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"


// ADCs
#define PHASE_A_ADC_HANDLE hadc1
#define PHASE_B_ADC_HANDLE hadc3
#define PHASE_C_ADC_HANDLE hadc2
#define VBUS_ADC_HANDLE hadc4
#define AUX_ADC_HANDLE hadc5

// CORDIC
#define CORDIC_HANDLE hcordic

// FDCAN
#define CAN_HANDLE hfdcan1

// FMAC
#define FMAC_HANDLE hfmac

// GPIO

typedef struct gpio_pin {
    GPIO_TypeDef* port;
    const u16 pin;
} gpio_pin_t;

static const gpio_pin_t spi_ncs = {SPI_nCS_GPIO_Port, SPI_nCS_Pin};
//static const gpio_pin_t encoder_power_ctrl = {SW_3V3_GPIO_Port, SW_3V3_Pin};
static const gpio_pin_t boot0_pin = {GPIOB, GPIO_PIN_8};

// HRTIM
#define HRTIM_HANDLE hhrtim1

// SPI
#define ENCODER_SPI_HANDLE hspi3

// TIM (TIM1/TIM15 entries removed — neither peripheral exists in the .ioc; timekeeping is the
// Cortex SysTick, the encoder is SPI, not timer-quadrature)
#define AUX_ADC_TRIG_HANDLE htim7
#define CAN_TIM_HANDLE htim4
#define CONTROLLER_FSM_TIM_HANDLE htim17

// USART
//#define DEBUG_UART_HANDLE huart1
//#define COMMS_UART_HANDLE huart1
#define DEBUG_UART_HANDLE USART2
#define COMMS_UART_HANDLE USART1
