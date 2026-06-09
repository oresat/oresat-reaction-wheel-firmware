//
// Created by solomon on 11/26/22.
//


#include <math.h>
#include "hrtim.h"
#include "inverter.h"

#include "pwm.h"
#include "adc.h"
#include "printf.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_hrtim.h"
#include "util.h"
#include "aux_adc.h"
#include "mathutil.h"
#include "config.h"

/*
 * This is the main file for the inverter + current sense. It controls:
 * - ADC setup and timing
 * - HRTIM setup
 * - HRTIM enable/disable and duty cycle setting
 * - ADC current and voltage sense processing
 * - ADC offset management
 * - Encoder triggering & data acq
 *
 *
 * ADC 1-4 are used, each in DMA mode, with the respective HRTIM trigger starting a DMA conv.
 * We're switching at ~325kHz (3us), with all ADCs using fast channels
 * ADC1-3 is direct connection to INA240 output, which is nominally ~3.3kohm, but we're running that bitch at 200ns sample, which is going to bite us in the ass but oh well
 * Theoretically, we should be aligning our sample point so that it's exactly Tsample/2 before the halfway of the PWM period
 * But who cares. That's a later problem.
 *
 * Actually - we have the opportunity to do that sample thingy now
 * ADC1 is trig'd by timer B compare 3
 * ADC3 is trig'd by timer F compare 3
 * ADC2 is trig'd by timer E compare 3
 * ADC4 is trig'd by timer B compare 2
 *
 * ADC4 is connected to a 5.1k // 3.24k voltage divider for vsense, effectively 2k. But we'll still run that bitch at 200ns sample.
 *
 */


/*  ************ CONSTANTS **************** */


const u32 TIMER_CLK_HZ = 144000000 * (u32) HRTIM_DLL_MUL;
const u32 TIMER_PERIOD = PWM_CNTR;
const u32 TIMER_PERIOD_HZ = (TIMER_CLK_HZ / TIMER_PERIOD) / 2;
const u32 ADC_POSTSCALER = 12 - 1;
const u32 ADC_SAMPLE_HZ = TIMER_PERIOD_HZ / (ADC_POSTSCALER + 1);


/*  ************* ADC Stuff ****************** */

volatile struct {
    volatile u32 A_raw;
    volatile u32 B_raw;
    volatile u32 C_raw;
    volatile u32 Vbus_raw;
} ADC_DMA_buf = {0, 0, 0, 0};

struct {
    volatile f32 A_isns;
    volatile f32 B_isns;
    volatile f32 C_isns;
    volatile f32 Vbus_vsns;
} ADC_processed = {0.f, 0.f, 0.f, 0.f};

struct {
    volatile i16 A_offset;
    volatile i16 B_offset;
    volatile i16 C_offset;
} INA240_offset_correction = {0, 0, 0};

volatile bool inverter_is_calibrated = false;


volatile bool OFFSET_CORRECTION_FLAG = false;

void ADC_FAKE_DMA_CB(ADC_HandleTypeDef *e) {
    //gpio_debug1(true);
    if (e == &hadc1)
        ADC_DMA_buf.A_raw = HAL_ADCEx_InjectedGetValue(e, ADC_INJECTED_RANK_1);
    else if (e == &hadc2)
        ADC_DMA_buf.B_raw = HAL_ADCEx_InjectedGetValue(e, ADC_INJECTED_RANK_1);
    else if (e == &hadc3)
        ADC_DMA_buf.C_raw = HAL_ADCEx_InjectedGetValue(e, ADC_INJECTED_RANK_1);
    else if (e == &hadc4)
        ADC_DMA_buf.Vbus_raw = HAL_ADCEx_InjectedGetValue(e, ADC_INJECTED_RANK_1);
    //gpio_debug1(false);
}

void TOGGLE_CB(ADC_HandleTypeDef *e) {
    //gpio_debug1(true);
    //gpio_debug1(false);
}

void OFFSET_CORRECTION_HRTIM_CB(HRTIM_HandleTypeDef *e, u32 tim) {
    //gpio_debug0(true);
    ADC_DMA_buf.A_raw = HAL_ADCEx_InjectedGetValue(&PHASE_A_ADC_HANDLE, ADC_INJECTED_RANK_1);
    ADC_DMA_buf.B_raw = HAL_ADCEx_InjectedGetValue(&PHASE_B_ADC_HANDLE, ADC_INJECTED_RANK_1);
    ADC_DMA_buf.C_raw = HAL_ADCEx_InjectedGetValue(&PHASE_C_ADC_HANDLE, ADC_INJECTED_RANK_1);
    ADC_DMA_buf.Vbus_raw = HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_1);
    OFFSET_CORRECTION_FLAG = true;
    //gpio_debug0(false);
}

static volatile u8 ADC_NUM_CPLT = 0; // When this equals 0b1111 we know we're done sampling

void ADC_DMA_CPLT_CB() {

}

void inverter_amp_offset_correction() {
    u32 n_samples = 20000;
    //u32 n_samples = 5;

    i32 A_accum = 0;
    i32 B_accum = 0;
    i32 C_accum = 0;

    // Make sure we're not gonna fuck anything up with this next move
    if (hadc1.ConvCpltCallback != &HAL_ADC_ConvCpltCallback) {
        //assert(false);
        //return;
    }

    HAL_HRTIM_TIMxRegisterCallback(&hhrtim1, HAL_HRTIM_REPETITIONEVENTCALLBACK_CB_ID,
                                   &OFFSET_CORRECTION_HRTIM_CB);
    inverter_start();
    hrtim_pwm_set_dcycle_f32(0.5f, 0.5f, 0.5f);

    u32 t_start = HAL_GetTick();
    for (u32 i = n_samples; i > 0; i--) {
        //if (i%1000 == 0) printf("%d ", i);
        OFFSET_CORRECTION_FLAG = false;
        while (!OFFSET_CORRECTION_FLAG);
        //printf("%f ", ADC_DMA_buf.A_raw);
        // Wait for a new ADC cycle
        A_accum += ((i16) ADC_DMA_buf.A_raw) - 2048;
        B_accum += ((i16) ADC_DMA_buf.B_raw) - 2048;
        C_accum += ((i16) ADC_DMA_buf.C_raw) - 2048;
    }
    u32 t_end = HAL_GetTick();
    printf("Calibration time for %lu samples: %lu ms\r\n", (unsigned long) n_samples, (unsigned long) (t_end - t_start));
    //printf("\r\n");

    f32 A_accum_avg = ((f32) A_accum) / ((f32) n_samples);
    f32 B_accum_avg = ((f32) B_accum) / ((f32) n_samples);
    f32 C_accum_avg = ((f32) C_accum) / ((f32) n_samples);

    i16 A_accum_avg_i16 = (i16) round_f32(A_accum_avg);
    i16 B_accum_avg_i16 = (i16) round_f32(B_accum_avg);
    i16 C_accum_avg_i16 = (i16) round_f32(C_accum_avg);

    // Call the sane range -100 to 100
    if ((A_accum_avg_i16 < -100) || (A_accum_avg_i16 > 100) ||
        (B_accum_avg_i16 < -100) || (B_accum_avg_i16 > 100) ||
        (C_accum_avg_i16 < -100) || (C_accum_avg_i16 > 100)) {
        assert(false);
        //return;
    }


    INA240_offset_correction.A_offset = A_accum_avg_i16;
    INA240_offset_correction.B_offset = B_accum_avg_i16;
    INA240_offset_correction.C_offset = C_accum_avg_i16;

    printf("\tAvg offset: %.1fmV %.1fmV %.1fmV\r\n", A_accum_avg * 3300.f / 4095.f, B_accum_avg * 3300.f / 4095.f,
           C_accum_avg * 3300.f / 4095.f);

    inverter_is_calibrated = true;

    inverter_stop();

    HAL_HRTIM_TIMxUnRegisterCallback(&hhrtim1, HAL_HRTIM_REPETITIONEVENTCALLBACK_CB_ID);
}

void inverter_adc_process_dma() {
    u32 A_raw = ADC_DMA_buf.A_raw - INA240_offset_correction.A_offset;
    u32 B_raw = ADC_DMA_buf.B_raw - INA240_offset_correction.B_offset;
    u32 C_raw = ADC_DMA_buf.C_raw - INA240_offset_correction.C_offset;
    u32 Vbus_raw = ADC_DMA_buf.Vbus_raw;

    f32 vvcc = ((f32) aux_adc_conv_vdd_voltage_mV(aux_adc_last_reading_raw.vvcc_sns)) / 1000.f;
    f32 Vbus = VVCC_CORRECT(Vbus_raw, vvcc) * 1 / VSNS_VOLTAGE_DIVIDER;

    f32 A_i = VVCC_CORRECT(((i16) A_raw) - 2048, vvcc) * SHUNT_AMP_CONDUCTANCE;
    f32 B_i = VVCC_CORRECT(((i16) B_raw) - 2048, vvcc) * SHUNT_AMP_CONDUCTANCE;
    f32 C_i = VVCC_CORRECT(((i16) C_raw) - 2048, vvcc) * SHUNT_AMP_CONDUCTANCE;

    ADC_processed.Vbus_vsns = Vbus;
    ADC_processed.A_isns = A_i;
    ADC_processed.B_isns = B_i;
    ADC_processed.C_isns = C_i;
}

__ccmram void inverter_set_processed(f32 A, f32 B, f32 C, f32 Vbus) {
    ADC_processed.Vbus_vsns = Vbus;
    ADC_processed.A_isns = A;
    ADC_processed.B_isns = B;
    ADC_processed.C_isns = C;
}

void inverter_get_phase_currents(f32 *A, f32 *B, f32 *C) {
    *A = ADC_processed.A_isns;
    *B = ADC_processed.B_isns;
    *C = ADC_processed.C_isns;
}

__ccmram void inverter_get_adc_raw(u32 *A, u32 *B, u32 *C, u32 *vbus) {

    *A = (&PHASE_A_ADC_HANDLE)->Instance->JDR1;
    *B = (&PHASE_B_ADC_HANDLE)->Instance->JDR1;
    *C = (&PHASE_C_ADC_HANDLE)->Instance->JDR1;
    *vbus = (&VBUS_ADC_HANDLE)->Instance->JDR1;
}

// Raw phase-voltage readings (PHx_V dividers) from injected rank 2 of each current-sense ADC:
// PHA_V=ADC1_IN3, PHB_V=ADC3_IN2, PHC_V=ADC2_IN4 (same ADCs as the shunts; see periphmap.h).
// Used by back-EMF coast-down cal. Rank 2 is enabled by MANUAL EDITs in Core/Src/adc.c
// (ScanConvMode=ENABLE + 2-conversion injected sequence; a CubeMX regen reverts them and this
// reads stale zeros again). Values update every injected trigger (96 kHz) but are only
// *meaningful* while the bridge is hi-Z (hrtim_outputs_disable()) — during normal PWM the phase
// node is switching and this samples chop. See docs/backemf-coastdown-implementation.md §3.
__ccmram void inverter_get_phase_voltages_raw(u32 *A, u32 *B, u32 *C) {
    *A = (&PHASE_A_ADC_HANDLE)->Instance->JDR2;
    *B = (&PHASE_B_ADC_HANDLE)->Instance->JDR2;
    *C = (&PHASE_C_ADC_HANDLE)->Instance->JDR2;
}

__ccmram void inverter_get_offset_correction(i16 *A, i16 *B, i16 *C) {
    *A = INA240_offset_correction.A_offset;
    *B = INA240_offset_correction.B_offset;
    *C = INA240_offset_correction.C_offset;
}

// Safety measure to ensure we're always getting new data
void inverter_blank_dma_buf() {
    ADC_DMA_buf = (typeof(ADC_DMA_buf)) {0, 0, 0, 0};
}

void inverter_get_vbus_voltage(f32 *Vbus) {
    *Vbus = ADC_processed.Vbus_vsns;
}

void inverter_adc_init() {
    HAL_ADCEx_Calibration_Start(&PHASE_A_ADC_HANDLE, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&PHASE_B_ADC_HANDLE, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&PHASE_C_ADC_HANDLE, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&VBUS_ADC_HANDLE, ADC_SINGLE_ENDED);

    HAL_ADC_RegisterCallback(&PHASE_A_ADC_HANDLE, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, &ADC_FAKE_DMA_CB);
    HAL_ADC_RegisterCallback(&PHASE_B_ADC_HANDLE, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, &ADC_FAKE_DMA_CB);
    HAL_ADC_RegisterCallback(&PHASE_C_ADC_HANDLE, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, &ADC_FAKE_DMA_CB);
    HAL_ADC_RegisterCallback(&VBUS_ADC_HANDLE, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, &ADC_FAKE_DMA_CB);

    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart(&hadc3);
    HAL_ADCEx_InjectedStart(&hadc4);

}

void inverter_adc_deinit() {
    HAL_ADC_UnRegisterCallback(&PHASE_A_ADC_HANDLE, HAL_ADC_CONVERSION_COMPLETE_CB_ID);
    HAL_ADC_UnRegisterCallback(&PHASE_B_ADC_HANDLE, HAL_ADC_CONVERSION_COMPLETE_CB_ID);
    HAL_ADC_UnRegisterCallback(&PHASE_C_ADC_HANDLE, HAL_ADC_CONVERSION_COMPLETE_CB_ID);
    HAL_ADC_UnRegisterCallback(&VBUS_ADC_HANDLE, HAL_ADC_CONVERSION_COMPLETE_CB_ID);
}


/* ****************  Inverter / PWM ************* */

void inverter_setup() {
    hrtim_pwm_setup();
    inverter_adc_init();
}

void inverter_start() {

    hrtim_pwm_start();
    hrtim_pwm_set_dcycle_f32(0.5f, 0.5f, 0.5f);

}

void inverter_stop() {

    hrtim_pwm_stop();

}


__ccmram bool inverter_get_calibrated() {
    return inverter_is_calibrated;
}