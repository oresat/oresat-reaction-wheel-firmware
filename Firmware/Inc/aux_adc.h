//
// Created by solomon on 11/27/22.
//

#include "typedefs.h"
#include "stm32g4xx_hal.h"
#include "adc.h"

// Samples at 1kHz

// Field order = ADC5 regular-sequence rank order (DMA writes straight into this struct).
typedef struct {
    volatile u16 therm_aux_sns;     // rank 1: CH6,  PE8  — aux/board thermistor
    volatile u16 therm_phase_a_sns; // rank 2: CH14, PE10 — phase A thermistor
    volatile u16 therm_phase_b_sns; // rank 3: CH15, PE11 — phase B thermistor
    volatile u16 therm_phase_c_sns; // rank 4: CH16, PE12 — phase C thermistor
    volatile u16 thermint_sns;      // rank 5: MCU die temperature sensor
    volatile u16 vvcc_sns;          // rank 6: VREFINT
} aux_adc_raw_t;

/*
typedef struct {
    volatile u32 vvcc_mV;
    volatile i32 therm1_cDegC;
    volatile i32 therm2_cDegC;
    volatile i32 thermint_cDegC;
} aux_adc_result_t;
 */

extern volatile aux_adc_raw_t aux_adc_last_reading_raw;
//volatile extern aux_adc_result_t aux_adc_last_reading_conv;

void aux_adc_start();

void aux_adc_stop();

u32 aux_adc_conv_vdd_voltage_mV(u16 vdd_raw);

u32 aux_adc_conv_vref_mV_unclamped(u16 vdd_raw); // no nominal fallback; 0 = garbage

i32 aux_adc_conv_therm_cDegC(u16 therm_raw);

i32 aux_adc_conv_thermint_cDegC(u16 therm_raw);

/*
u32 aux_adc_vdd_voltage_mV();

i32 aux_adc_therm1_cDegC();

i32 aux_adc_therm2_cDegC();

i32 aux_adc_tempint_cDegC();

 */