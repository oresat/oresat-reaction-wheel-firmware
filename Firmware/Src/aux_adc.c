//
// Created by solomon on 11/27/22.
//


#include "aux_adc.h"
#include "printf.h"
#include "periphmap.h"
#include "util.h"
#include "THERM_LUT.h"


/*
 * All this shit manages ADC5, which is responsible for:
 * - Vrefint reading
 * - Thermistor 1 reading
 * - Thermistor 2 reading
 * - Internal tempsensor reading
 *
 * TIM7 is setup to produce TRGO pulses at 1KHz, which triggers ADC5 DMA
 * DMA is written into adc_dma_buf, then aux_adc_conv_complete_cb is called,
 * where the values are converted from raw ADC counts (oversampled, so 0-65535), to useful units
 * TODO: in the future, the ISR callback should be a RTOS task, as it's not time critical
 */

// Last raw ADC count from DMA
volatile aux_adc_raw_t aux_adc_last_reading_raw = {0, 0, 0, 0, 0, 0};

static volatile bool convComplete = false;


u16 VREFINT_CAL = 0;
const u16 VREFINT_CAL_VOLTAGE_mV = VREFINT_CAL_VREF;

// TODO move all this math from a 1kHz ISR to a seperate process() function or RTOS task
void aux_adc_conv_complete_cb(ADC_HandleTypeDef *hadc) {
    convComplete = true;

    //u32 vvcc_mV = (((((u32) VREFINT_CAL) << 20) / vvcc_sns) * VREFINT_CAL_VOLTAGE_mV) >> 16;
    //i32 therm1_cDegC = NTC_THERM_ADC_TO_cDegC(therm1_sns >> 4);
    //i32 therm2_cDegC = NTC_THERM_ADC_TO_cDegC(therm2_sns >> 4);
    //i32 thermint_cDegC = 100 * __LL_ADC_CALC_TEMPERATURE(3300, thermint_sns >> 4, ADC_RESOLUTION_12B);

    //aux_adc_last_reading_raw = (aux_adc_raw_t) {therm1_sns, therm2_sns, thermint_sns, vvcc_sns};
    //aux_adc_last_reading_conv = (aux_adc_result_t) {vvcc_mV, therm1_cDegC, therm2_cDegC, thermint_cDegC};
}

void aux_adc_start() {
    HAL_ADCEx_Calibration_Start(&AUX_ADC_HANDLE, ADC_SINGLE_ENDED);
    HAL_ADC_RegisterCallback(&hadc5, HAL_ADC_CONVERSION_COMPLETE_CB_ID, &aux_adc_conv_complete_cb);
    HAL_TIM_Base_Start_IT(&AUX_ADC_TRIG_HANDLE);
    HAL_ADC_Start_DMA(&hadc5, (u32 *) &aux_adc_last_reading_raw, 6);

    VREFINT_CAL = *VREFINT_CAL_ADDR;
}

void aux_adc_stop() {
    HAL_ADC_UnRegisterCallback(&hadc5, HAL_ADC_CONVERSION_COMPLETE_CB_ID);
    HAL_TIM_Base_Stop_IT(&AUX_ADC_TRIG_HANDLE);
    HAL_ADC_Stop_DMA(&hadc5);
}


u32 aux_adc_conv_vdd_voltage_mV(u16 vdd_raw) {
    return (((((u32) VREFINT_CAL) << 20) / vdd_raw) * VREFINT_CAL_VOLTAGE_mV) >> 16;
}

i32 aux_adc_conv_therm_cDegC(u16 therm_raw) {
    return NTC_THERM_ADC_TO_cDegC(therm_raw >> 4);
}

i32 aux_adc_conv_thermint_cDegC(u16 therm_raw) {
    return 100 * __LL_ADC_CALC_TEMPERATURE(3300, therm_raw >> 4, ADC_RESOLUTION_12B);
}

/*
// 16x oversample factor, so it's 0-65535
u32 aux_adc_vdd_voltage_mV() {
    u32 vrefint_sample_times16 = (u32) aux_adc_last_reading_raw.vvcc_sns;
    u32 vrefint_cal_times16 = ((u32) VREFINT_CAL) << 4;
    u32 vrefint_cal_voltage_mV = VREFINT_CAL_VOLTAGE_mV;

    // Need to express vrefint_cal_times16/vrefint_sample_times16 as a u32 ratio
    // SHL vrefint_cal_times16 by 16B, u32/u16 divide => 0-65535 ratio

    u32 vrefint_cal_over_meas_q16 = (vrefint_cal_times16 << 16) / vrefint_sample_times16;
    u32 supply_voltage_mV = (((u32) vrefint_cal_voltage_mV) * vrefint_cal_over_meas_q16) >> 16;
    return supply_voltage_mV;
}

i32 aux_adc_therm1_cDegC() {
    return NTC_THERM_ADC_TO_cDegC(aux_adc_last_reading_raw.therm1_sns >> 4);
}

i32 aux_adc_therm2_cDegC() {
    return NTC_THERM_ADC_TO_cDegC(aux_adc_last_reading_raw.therm2_sns >> 4);
}

i32 aux_adc_tempint_cDegC() {
    return 100 * __LL_ADC_CALC_TEMPERATURE(3300, aux_adc_last_reading_raw.thermint_sns >> 4, ADC_RESOLUTION_12B);
}
 */