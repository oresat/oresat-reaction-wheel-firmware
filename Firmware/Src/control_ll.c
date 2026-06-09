#include "control_ll.h"
#include "inverter.h"
#include "hrtim.h"
#include "adc.h"
#include "mathutil.h"
#include "pwm.h"
#include "aux_adc.h"
#include "config.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_cortex.h"
#include "util.h"
#include "encoder.h"
#include "fsm.h"
#include "ma702.h"
#include "swo_util.h"

// Controller errors and state
volatile u32 controller_errors = CTRLR_ERR_NONE;
volatile bool controller_armed = false;
volatile CONTROLLER_STATE_t current_state = CTRLR_STATE_IDLE;
volatile CONTROLLER_STATE_t requested_state = CTRLR_STATE_IDLE;

// Controller timing/diag
volatile u32 n_loop_count = 0;

// Currents
volatile f32 last_phase_currents[3] = {0.f, 0.f, 0.f};
volatile f32 last_vbus_voltage = 0.f;
volatile bool is_currents_updated = false;
volatile bool is_currents_valid = false;

// PWM
volatile svm_vec_t new_pwm_timings = {0.f, 0.f, 0.f};
volatile bool is_pwm_timings_updated = false;
volatile bool is_pwm_timings_valid = false;

// Shunt selection (best-2-of-3 high-modulation phase-current reconstruction). Runtime flag
// seeded from config so it can be A/B-toggled live over SWD without a reflash.
volatile bool shunt_selection_enabled = FOC_USE_SHUNT_SELECTION;
volatile i8 last_shunt_drop_phase = -1; // -1 = none/disabled, 0/1/2 = A/B/C reconstructed (capture/debug)

// Fast-loop ISR timing (see control_ll.h). Only meaningful while armed (the repetition
// callback is unregistered otherwise).
volatile u32 isr_cycles_last = 0;
volatile u32 isr_cycles_max = 0;

// Live-tunable ADC trigger position (Timer A CMP4, HRTIM ticks). Seeded from config; the fast
// loop applies it on change in pwm_update_callback (CMP4 is preloaded, transfers at the REP
// update -> glitchless). SWD-pokeable for the CMP4 sweep (tools/cmp4_sweep.sh); clamped to
// >= 2 fHRTIM clocks from the 0/PWM_CNTR direction changes (RM0440 p918 cross-unit rule is 1).
volatile u32 adc_trig_cmp4_ticks = ADC_TRIG_CMP4_TICKS;

// ADC sample-point sweep accumulator (bench/SWD only — see docs/adc-sampling-tuning.md).
// Host zeroes the block, then writes req_n = N; the fast loop then accumulates N ticks of the
// PRE-CM-removal phase currents (CM removal / shunt selection would smear the per-phase edge
// artifact being measured across all three) + Vbus, then stops. Host polls count == req_n and
// reads the block. Idle cost: one load + branch per tick.
volatile struct {
    u32 req_n;    // host: 0 = idle, N = accumulate N ticks (write last, after zeroing the rest)
    u32 count;    // fw: ticks accumulated so far
    f32 sum[4];   // A, B, C [A], Vbus [V]
    f32 sumsq[3]; // A^2, B^2, C^2
} adcsweep = {0};

// ibus
static volatile f32 ibus = 0.0f;

// Encoder
volatile bool is_encoder_updated = false;
volatile bool is_encoder_valid = false;
volatile u16 last_encoder_reading = 0;

// Control law def
volatile control_law_t control_law = {
    .control_law_init = NULL,
    .control_law_exec = NULL,
    .current_consumer = NULL,
    .encoder_consumer = NULL,
    .is_complete = NULL,
    .pwm_producer = NULL,
    .control_law_finish = NULL
};

volatile bool control_law_needs_init = true;
volatile bool control_law_is_done = false;

// Function decl
void controller_set_current_valid(bool is_valid);

void controller_set_pwm_timings_valid(bool is_valid);

void controller_set_encoder_valid(bool is_valid);

void controller_add_errors(u32 err);

// Per-tick record of which phase will be CM-corrupted at the crest ADC sample: the one with
// the smallest SVM magnitude → largest compare (= PWM_CNTR*(1-mag), pwm.c) → switching edge
// nearest the crest. Written when the duty is latched; read SHUNT_SEL_DELAY_TICKS ticks later
// in adc_update_callback, when that duty's resulting current comes back from the ADC.
#define SHUNT_SEL_HIST_LEN 4u  // power of two; must exceed SHUNT_SEL_DELAY_TICKS
static volatile u8 shunt_untrusted_hist[SHUNT_SEL_HIST_LEN] = {0};
static volatile u32 shunt_untrusted_head = 0;

static __fn_inline u8 svm_smallest_mag_phase(svm_vec_t m) {
    if (m.A <= m.B) return (m.A <= m.C) ? 0u : 2u;
    return (m.B <= m.C) ? 1u : 2u;
}

__ccmram void pwm_update_callback() {
    debug_pwm_callback(true);
    // About 4us

    if (control_law.pwm_producer != NULL && controller_armed) {
        svm_vec_t pwm_timing_producer_output;
        bool pwm_ok = control_law.pwm_producer(&pwm_timing_producer_output);
        if (pwm_ok) {
            controller_set_new_pwm_timings(&pwm_timing_producer_output);
            new_pwm_timings = pwm_timing_producer_output;
            controller_set_pwm_timings_valid(true);
        }
    }

    svm_vec_t pwm_timings_buf = new_pwm_timings;

    // Check if we have new PWM timings
    if (!is_pwm_timings_updated)
        controller_add_errors(CTRLR_ERR_PWM_TIMING_UPDATE_MISSING);

    // Check timing validity
    is_pwm_timings_valid = is_pwm_timings_valid & svm_output_valid(pwm_timings_buf);

    // Check to make sure these timings don't suck
    if (!svm_output_valid(pwm_timings_buf))
        controller_add_errors(CTRLR_ERR_MODULATION);
    if (!is_pwm_timings_valid)
        controller_add_errors(CTRLR_ERR_PWM_TIMING_INVALID);

    // Disarm if we have any errors
    if (controller_errors != CTRLR_ERR_NONE)
        controller_disarm();

    // Update HRTIM PWM and invalidate existing timings
    if (controller_armed) {
        hrtim_pwm_set_dcycle_f32(pwm_timings_buf.A, pwm_timings_buf.B, pwm_timings_buf.C);
        is_pwm_timings_updated = false;
        // Record the to-be-corrupted phase for these just-latched duties (consumed N ticks later).
        shunt_untrusted_head++;
        shunt_untrusted_hist[shunt_untrusted_head & (SHUNT_SEL_HIST_LEN - 1u)] =
            svm_smallest_mag_phase(pwm_timings_buf);

        // Apply the ADC trigger position on change (clamped; preloaded -> lands at REP update).
        static u32 cmp4_applied = 0u; // 0 = never applied (real values clamp to >= 64)
        u32 cmp4 = adc_trig_cmp4_ticks;
        cmp4 = (cmp4 < 2u * HRTIM_DLL_MUL) ? 2u * HRTIM_DLL_MUL
             : (cmp4 > PWM_CNTR - 2u * HRTIM_DLL_MUL) ? PWM_CNTR - 2u * HRTIM_DLL_MUL
             : cmp4;
        if (cmp4 != cmp4_applied) {
            __HAL_HRTIM_SETCOMPARE(&HRTIM_HANDLE, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, cmp4);
            cmp4_applied = cmp4;
        }
    }
    debug_pwm_callback(false);
}

__ccmram void adc_update_callback() {
    debug_adc_callback(true);
    // About 5us
    // Gotta make sure we've calibrated at least once otherwise Bad Things will happen
    if (!inverter_get_calibrated())
        controller_add_errors(CTRLR_ERR_INVERTER_CALIBRATION_INVALID);

    // Something has to specifically consume the phase currents each cycle, or maybe our higher-level controller isn't working
    if (is_currents_updated)
        controller_add_errors(CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING);

    // Y'know what? Fine. *invalidates your phase currents*
    if (!is_currents_valid)
        controller_add_errors(CTRLR_ERR_PHASE_CURRENTS_INVALID);


    // Define here for below
    i16 A_offset_cal, B_offset_cal, C_offset_cal;
    u32 A_raw, B_raw, C_raw, Vbus_raw;

    // Get all data from inverter and get offset correction
    inverter_get_adc_raw(&A_raw, &B_raw, &C_raw, &Vbus_raw);
    //dac_debug0(A_raw);

    inverter_get_offset_correction(&A_offset_cal, &B_offset_cal, &C_offset_cal);
    //inverter_blank_dma_buf();

    if (A_raw == 0 || B_raw == 0 || C_raw == 0 || Vbus_raw == 0) {
        is_currents_valid = false;
        controller_add_errors(CTRLR_ERR_PHASE_CURRENTS_MEASUREMENT_MISSING);
    }


    // TODO should be offset_cal_completed variable or something
    if (A_offset_cal == 0 && B_offset_cal == 0 && C_offset_cal == 0) {
        is_currents_valid = false;
        controller_add_errors(CTRLR_ERR_INVERTER_CALIBRATION_INVALID);
    }

    // Offset calibration
    i32 A_offset_corrected = (i32)A_raw - A_offset_cal;
    i32 B_offset_corrected = (i32)B_raw - B_offset_cal;
    i32 C_offset_corrected = (i32)C_raw - C_offset_cal;

    // Get VCC voltage
    f32 vvcc = ((f32)aux_adc_conv_vdd_voltage_mV(aux_adc_last_reading_raw.vvcc_sns)) / 1000.f;

    // Gain and correction
    f32 Vbus = VVCC_CORRECT(Vbus_raw, vvcc) * 1 / VSNS_VOLTAGE_DIVIDER;
    f32 A_i = VVCC_CORRECT(((i16) A_offset_corrected) - 2048, vvcc) * SHUNT_AMP_CONDUCTANCE;
    f32 B_i = VVCC_CORRECT(((i16) B_offset_corrected) - 2048, vvcc) * SHUNT_AMP_CONDUCTANCE;
    f32 C_i = VVCC_CORRECT(((i16) C_offset_corrected) - 2048, vvcc) * SHUNT_AMP_CONDUCTANCE;

    inverter_set_processed(A_i, B_i, C_i, Vbus);

    // ADC sweep accumulator tap (bench): PRE-CM-removal currents — see decl at top of file.
    {
        u32 sweep_req = adcsweep.req_n;
        if (sweep_req != 0u && adcsweep.count < sweep_req) {
            adcsweep.sum[0] += A_i;  adcsweep.sumsq[0] += A_i * A_i;
            adcsweep.sum[1] += B_i;  adcsweep.sumsq[1] += B_i * B_i;
            adcsweep.sum[2] += C_i;  adcsweep.sumsq[2] += C_i * C_i;
            adcsweep.sum[3] += Vbus;
            adcsweep.count++;
        }
    }

    // Force Ia+Ib+Ic = 0. Default: symmetric common-mode removal (spreads the imbalance over
    // all three). With shunt selection on: drop the phase whose switching edge crowded the crest
    // ADC sample (decided SHUNT_SEL_DELAY_TICKS ticks ago, when its duty was latched) and rebuild
    // it from the trusted two via KCL — keeps the two clean measurements untouched.
    if (shunt_selection_enabled) {
        u32 idx = (shunt_untrusted_head - (u32) SHUNT_SEL_DELAY_TICKS) & (SHUNT_SEL_HIST_LEN - 1u);
        u8 drop = shunt_untrusted_hist[idx];
        if (drop == 0u)      A_i = -(B_i + C_i);
        else if (drop == 1u) B_i = -(A_i + C_i);
        else                 C_i = -(A_i + B_i);
        last_shunt_drop_phase = (i8) drop;
    } else {
        f32 zero = A_i + B_i + C_i;
        A_i -= zero / 3.f;
        B_i -= zero / 3.f;
        C_i -= zero / 3.f;
        last_shunt_drop_phase = -1;
    }

    if (Vbus > DC_BUS_OVERVOLTAGE_THRESHOLD_V)
        controller_add_errors(CTRLR_ERR_VBUS_OVERVOLTAGE);
    if (Vbus < DC_BUS_UNDERVOLTAGE_THRESHOLD_V)
        controller_add_errors(CTRLR_ERR_VBUS_UNDERVOLTAGE);

    // We shouldn't go further if things are already fucked up
    if (controller_errors != CTRLR_ERR_NONE) {
        controller_disarm();
        goto fail;
        //return;
    }

    // Check for current limit violation
    // Now we can assume A_i + B_i + C_i == 0 (nominally)
    // Inorm^2 = Id^2 + Id^2 = Ialpha^2 + Ibeta^2 = 2/3 * (Ia^2 + Ib^2 + Ic^2)
    f32 Inorm_squared = 2.f / 3.f * ((A_i * A_i) + (B_i * B_i) + (C_i * C_i));
    f32 Imax_squared = MOTOR_CURRENT_TRIP_LIMIT * MOTOR_CURRENT_TRIP_LIMIT;
    if (Inorm_squared > Imax_squared)
        controller_add_errors(CTRLR_ERR_IBUS_OVERCURRENT);

    if (controller_errors != CTRLR_ERR_NONE) {
        controller_disarm();
        goto fail;
        //return;
    }

    last_phase_currents[0] = A_i;
    last_phase_currents[1] = B_i;
    last_phase_currents[2] = C_i;
    last_vbus_voltage = Vbus;

    is_currents_valid = true;
    is_currents_updated = true;

    if (control_law.current_consumer != NULL && controller_armed) {
        bool ret_val = control_law.current_consumer(A_i, B_i, C_i, Vbus);
        if (ret_val)
            is_currents_updated = false;
    }
    fail:
    debug_adc_callback(false);
}

__ccmram void encoder_update_callback() {
    debug_encoder_callback(true);
    if (!is_encoder_valid)
        controller_add_errors(CTRLR_ERR_ENCODER_READING_INVALID);

    if (!encoder_read_success_dma) {
        controller_add_errors(CTRLR_ERR_ENCODER_READING_INVALID);
        is_encoder_valid = false;
    }

    if (control_law.encoder_consumer != NULL && controller_armed) {
        bool ret_val = control_law.encoder_consumer(current_angle_dma_raw);
        if (ret_val)
            is_encoder_updated = false;
    }

    // This should be set false above
    if (is_encoder_updated)
        controller_add_errors(CTRLR_ERR_ENCODER_READING_USAGE_MISSING);

    if (controller_errors != CTRLR_ERR_NONE)
        controller_disarm();

    if (controller_armed) {
        last_encoder_reading = current_angle_dma_raw;
        is_encoder_updated = true;
        is_encoder_valid = true;
    }
    debug_encoder_callback(false);
}

__ccmram void hrtim_repetition_callback(HRTIM_HandleTypeDef* hhrtim, u32 timidx) {
    debug_hrtim_isr(true);
    u32 isr_t0 = DWT->CYCCNT;

    n_loop_count++;

    bool ran_init = false;
    if (control_law_needs_init && control_law.control_law_init != NULL) {
        control_law_needs_init = false;
        control_law.control_law_init();
        ran_init = true;
    }


    // Kick off SPI read
    __MA702_FAST_ANGLE_READ_START();

    // Run ADC update callback
    adc_update_callback();
    __maybe_unused f32 A_i = last_phase_currents[0];
    __maybe_unused f32 B_i = last_phase_currents[1];
    __maybe_unused f32 C_i = last_phase_currents[2];

    // Dump phase currents to DAC
    // Rescale from 0-13.5A (clamped) to 0.3-3V (margins for DAC buffer swing)
    //f32 AV = clamp_f32(-1.35f, A_i / 10.f, 1.35f) + 1.65f;
    //f32 BV = clamp_f32(-1.35f, B_i / 10.f, 1.35f) + 1.65f;
    //f32 CV = clamp_f32(-1.35f, C_i / 10.f, 1.35f) + 1.65f;

    //dac_debug0_V(AV);
    //dac_debug1_V(BV);
    //dac_debug2_V(CV);
    //ITM->PORT[0].u32 = *(u32*) &A_i;
    //ITM->PORT[1].u32 = *(u32*) &B_i;
    //ITM->PORT[2].u32 = *(u32*) &C_i;

    // Finish SPI read
    current_angle_dma_raw = __MA702_FAST_ANGLE_READ_FINISH();
    encoder_read_success_dma = true; // TODO this is a joke

    encoder_update_callback();

    if (control_law.control_law_exec != NULL)
        control_law.control_law_exec();

    pwm_update_callback();

    if (control_law.is_complete != NULL && control_law.is_complete()) {
        controller_disarm();
        control_law_is_done = true;
    }

    //gpio_debug0(false);
    u32 isr_dt = DWT->CYCCNT - isr_t0;
    isr_cycles_last = isr_dt;
    // The arm tick is excluded from the high-water mark: control_law_init runs in-ISR and
    // init_foc's printfs make that one tick ~141k cycles (~1 ms, ≈31 dropped control ticks
    // per arm — measured RelWithDebInfo 2026-06-09). Real cost, but it would permanently
    // mask the steady-state max this counter exists to expose.
    if (isr_dt > isr_cycles_max && !ran_init) isr_cycles_max = isr_dt;
    debug_hrtim_isr(false);
}

void controller_init() {
}

void controller_arm() {
    if (controller_armed) {
        controller_disarm();
    }

    assert(HAL_HRTIM_TIMxRegisterCallback(&hhrtim1, HAL_HRTIM_REPETITIONEVENTCALLBACK_CB_ID,
        &hrtim_repetition_callback) == HAL_OK);


    controller_set_current_valid(true);
    controller_set_encoder_valid(true);
    controller_set_pwm_timings_valid(true);
    controller_set_new_pwm_timings(&((svm_vec_t){0.5f, 0.5f, 0.5f}));

    inverter_start();
    controller_armed = true;
}

void controller_disarm() {
    inverter_stop();

    assert(HAL_HRTIM_TIMxUnRegisterCallback(&hhrtim1, HAL_HRTIM_REPETITIONEVENTCALLBACK_CB_ID) == HAL_OK);

    controller_armed = false;
}

// Calls at ~1kHz by TIM17
void controller_run_state_machine() {
    fsm_exec();
}

/* ============ GETTER/SETTER CRUFT ============ */

u32 controller_get_errors() {
    return controller_errors;
}

__ccmram void controller_add_errors(const u32 err) {
    controller_errors |= err;
}

void controller_clear_errors() {
    controller_errors = CTRLR_ERR_NONE;
}

bool controller_is_armed() {
    return controller_armed;
}

u32 controller_get_loop_count() {
    return n_loop_count;
}

void controller_set_control_law(const control_law_t* _control_law) {
    control_law = *_control_law;

    control_law_needs_init = true;
    control_law_is_done = false;
}

void controller_clear_control_law() {
    control_law.encoder_consumer = NULL;
    control_law.current_consumer = NULL;
    control_law.pwm_producer = NULL;
    control_law.control_law_exec = NULL;
    control_law.is_complete = NULL;
    control_law.control_law_init = NULL;
    control_law.control_law_finish = NULL;

    control_law_needs_init = false;
    control_law_is_done = false;
}

volatile control_law_t* controller_get_control_law() {
    return &control_law;
}

bool controller_is_control_law_done() {
    return control_law_is_done;
}


__ccmram void controller_set_pwm_timings_valid(bool is_valid) {
    is_pwm_timings_valid = is_valid;
}

__ccmram void controller_set_new_pwm_timings(svm_vec_t* timings) {
    new_pwm_timings = *timings;
    is_pwm_timings_updated = true;
}


void controller_set_current_valid(bool is_valid) {
    is_currents_valid = is_valid;
}


void controller_set_encoder_valid(bool is_valid) {
    is_encoder_valid = is_valid;
}
