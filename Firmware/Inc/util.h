//
// Created by solomon on 11/17/22.
//

#pragma once

#include "dac.h"
#include "mathutil.h"
#include "typedefs.h"
#include "periphmap.h"
#include "control_ll.h"
#include "printf.h"

void user_assert_fail(uint8_t *file, uint32_t line);

#define assert(expr) ((expr) ? (void)0U : user_assert_fail((uint8_t *)__FILE__, __LINE__))

u32 micros();

void delay_us(u32 us);

static inline void STRINGIFY_CONTROLLER_STATE(char *buf, u32 max_size, CONTROLLER_STATE_t state) {
    snprintf(buf, max_size, "%s",
             (state == CTRLR_STATE_IDLE)
                 ? "CTRLR_STATE_IDLE"
                 : (state == CTRLR_STATE_ERR)
                       ? "CTRLR_STATE_ERR"
                       : (state == CTRLR_STATE_SHUNT_OFFSET_CAL)
                             ? "CTRLR_STATE_SHUNT_OFFSET_CAL"
                             : (state == CTRLR_STATE_MOTOR_RESISTANCE_CAL)
                                   ? "CTRLR_STATE_MOTOR_RESISTANCE_CAL"
                                   : (state == CTRLR_STATE_MOTOR_INDUCTANCE_CAL)
                                         ? "CTRLR_STATE_MOTOR_INDUCTANCE_CAL"
                                         : (state == CTRLR_STATE_MOTOR_ENCODER_DIR_CAL)
                                               ? "CTRLR_STATE_MOTOR_INDUCTANCE_CAL"
                                               : (state == CTRLR_STATE_MOTOR_ENCODER_PHASE_CAL)
                                                     ? "CTRLR_STATE_MOTOR_ENCODER_PHASE_CAL"
                                                     : ""
    );
}

// CTRLR_ERR_NONE = 0,
// CTRLR_ERR_INVERTER_CALIBRATION_INVALID=1<<0,
// CTRLR_ERR_PHASE_CURRENTS_INVALID=1<<1,
// CTRLR_ERR_PHASE_CURRENTS_MEASUREMENT_MISSING=1<<2,
// CTRLR_ERR_PWM_TIMING_INVALID=1<<3,
// CTRLR_ERR_PWM_TIMING_UPDATE_MISSING=1<<4,
// CTRLR_ERR_VBUS_OVERVOLTAGE=1<<5,
// CTRLR_ERR_VBUS_UNDERVOLTAGE=1<<6,
// CTRLR_ERR_IBUS_OVERCURRENT=1<<7,
// CTRLR_ERR_MOTOR_OVERCURRENT=1<<8,
// CTRLR_ERR_MOTOR_PHASE_LEAKAGE=1<<9,
// CTRLR_ERR_MOTOR_RESISTANCE_OUT_OF_RANGE=1<<10,
// CTRLR_ERR_MOTOR_INDUCTANCE_OUT_OF_RANGE=1<<11,
// CTRLR_ERR_ENCODER_READING_MISSING=1<<12,
// CTRLR_ERR_ENCODER_ESTIMATE_MISSING=1<<13,
// CTRLR_ERR_ENCODER_READING_INVALID=1<<14,
// CTRLR_ERR_ENCODER_FAILURE=1<<15,
// CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING=1<<16,
// CTRLR_ERR_PWM_TIMING_USAGE_MISSING=1<<17,
// CTRLR_ERR_PHASE_CURRENT_LEAKAGE=1<<18,


typedef struct error_name {
    char string[64];
    u32 err_val;
} error_name_t;

#define tmp(enum_name) ((error_name_t) {#enum_name, enum_name})
static error_name_t errors[32] = {
    tmp(CTRLR_ERR_INVERTER_CALIBRATION_INVALID),
    tmp(CTRLR_ERR_PHASE_CURRENTS_INVALID),
    tmp(CTRLR_ERR_PHASE_CURRENTS_MEASUREMENT_MISSING),
    tmp(CTRLR_ERR_PWM_TIMING_INVALID),
    tmp(CTRLR_ERR_PWM_TIMING_UPDATE_MISSING),
    tmp(CTRLR_ERR_VBUS_OVERVOLTAGE),
    tmp(CTRLR_ERR_VBUS_UNDERVOLTAGE),
    tmp(CTRLR_ERR_IBUS_OVERCURRENT),
    tmp(CTRLR_ERR_MOTOR_OVERCURRENT),
    tmp(CTRLR_ERR_MOTOR_PHASE_LEAKAGE),
    tmp(CTRLR_ERR_MOTOR_RESISTANCE_OUT_OF_RANGE),
    tmp(CTRLR_ERR_MOTOR_INDUCTANCE_OUT_OF_RANGE),
    tmp(CTRLR_ERR_ENCODER_READING_MISSING),
    tmp(CTRLR_ERR_ENCODER_ESTIMATE_MISSING),
    tmp(CTRLR_ERR_ENCODER_READING_INVALID),
    tmp(CTRLR_ERR_ENCODER_FAILURE),
    tmp(CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING),
    tmp(CTRLR_ERR_PWM_TIMING_USAGE_MISSING),
    tmp(CTRLR_ERR_PHASE_CURRENT_LEAKAGE),
    tmp(CTRLR_ERR_ENCODER_READING_USAGE_MISSING),
    tmp(CTRLR_ERR_MOTOR_UNBALANCED_PHASES),
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
};
#undef tmp

static inline void STRINGIFY_CONTROLLER_ERRORS(char *buf, u32 max_size, u32 err) {
    bool found_err = false;

    for (u8 i = 0; i < 32; i++) {
        if (err & errors[i].err_val) {
            found_err = true;
            u32 wrote = snprintf(buf, max_size, "%s ", errors[i].string);
            if (wrote > max_size) return;
            max_size -= wrote;
            buf += wrote;
        }
    }

    if (!found_err)
        snprintf(buf, max_size, "%s", "CTRLR_ERR_NONE");
}


static inline void gpio_debug0(bool level) {
    //HAL_GPIO_WritePin(DEBUG0_GPIO_Port, DEBUG0_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (level)
        DEBUG0_GPIO_Port->BSRR = (u32) DEBUG0_Pin;
    else
        DEBUG0_GPIO_Port->BRR = (u32) DEBUG0_Pin;
}

static inline void gpio_debug1(bool level) {
    //HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (level)
        DEBUG1_GPIO_Port->BSRR = (u32) DEBUG1_Pin;
    else
        DEBUG1_GPIO_Port->BRR = (u32) DEBUG1_Pin;
}

static inline void gpio_debug2(bool level) {
    //HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (level)
        DEBUG2_GPIO_Port->BSRR = (u32) DEBUG2_Pin;
    else
        DEBUG2_GPIO_Port->BRR = (u32) DEBUG2_Pin;
}

static inline void gpio_debug3(bool level) {
    //HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (level)
        DEBUG3_GPIO_Port->BSRR = (u32) DEBUG3_Pin;
    else
        DEBUG3_GPIO_Port->BRR = (u32) DEBUG3_Pin;
}

static __fn_inline void dac_debug0(u16 cnt) {
    *(volatile u32 *) ((u32) hdac1.Instance + DAC_DHR12R1_ALIGNMENT(DAC_ALIGN_12B_R)) = cnt;
}
static __fn_inline void dac_debug1(u16 cnt) {
    *(volatile u32 *) ((u32) hdac1.Instance + DAC_DHR12R2_ALIGNMENT(DAC_ALIGN_12B_R)) = cnt;
}
static __fn_inline void dac_debug2(u16 cnt) {
    *(volatile u32 *) ((u32) hdac2.Instance + DAC_DHR12R1_ALIGNMENT(DAC_ALIGN_12B_R)) = cnt;
}

static __fn_inline void dac_debug0_V(f32 V) {
    u16 v_u16 = (u16) (4095.f * (V / 3.3f));
    if (v_u16 > 4095) v_u16 = 4095;
    dac_debug0(v_u16);
}

static __fn_inline void dac_debug1_V(f32 V) {
    u16 v_u16 = (u16) (4095.f * (V / 3.3f));
    if (v_u16 > 4095) v_u16 = 4095;
    dac_debug1(v_u16);
}

static __fn_inline void dac_debug2_V(f32 V) {
    u16 v_u16 = (u16) (4095.f * (V / 3.3f));
    if (v_u16 > 4095) v_u16 = 4095;
    dac_debug2(v_u16);
}


#define blank_console() printf("\r\n\u033c\033[2J\033[300A")

// We can't use the onboard stm32 bootloader since it FUCKS OUR SHIT UP
// instead, we're going to check the pin state manually on boot, and jump to a compatible bootloader impl
// so that it's as if we're using the UART bootloader


//void uart_transmit(USART_TypeDef* uart, u8* buf, u8 len);

void uart_transmit(USART_TypeDef *uart, u8 *buf, u8 len);

static __fn_inline void uart_transmit_one(USART_TypeDef *uart, char ch) {
    while (LL_USART_IsActiveFlag_TXE_TXFNF(uart) == 0);
    LL_USART_TransmitData8(uart, ch);
    while (LL_USART_IsActiveFlag_TC(uart) == 0);
}

static __fn_inline void uart_transmit_buf(USART_TypeDef *uart, char *buf, u32 len) {
    for (u32 i = 0; i < len; i++)
        uart_transmit_one(uart, buf[i]);
}
