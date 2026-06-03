#include "motor_calibration.h"
#include "bldc_math.h"
#include "config.h"
#include "control_ll.h"
#include "encoder.h"
#include "foc.h"
#include "mathutil.h"
#include "open_loop_current_controller.h"
#include "printf.h"
#include "stm32g4xx_hal.h"
#include "util.h"
#include "stm32g4xx_hal_hrtim.h"
#include <math.h>

// ============== PHASE RESISTANCE =====================

/*
 * Phase resistance calibration:
 * - Lock motor to +alpha
 * - Ramp voltage with a predefined (very underdamped) PI controller
 * - See what voltage we settle at - WITH LOTS OF SAFETY CHECKS
 */

static volatile f32 resistance_calib_test_voltage = 0.f;
static volatile f32 resistance_calib_output_mod = 0.f;

static volatile f32 resistance_calib_current_ialpha_filt = 0.f;
static volatile f32 resistance_calib_current_ibeta_filt = 0.f;

static const f32 resistance_calib_current_filter = 80.f; // thank u odrive
static const f32 resistance_cal_ki = 2.f; // (V/s) / A

static const f32 current_meas_period = 1.f / ((f32)CURRENT_MEAS_FREQ);

static const u32 resistance_cal_length_cycles = (u32)(((f32)RESISTANCE_CAL_TIME / 1000.f) * LOOP_UPDATE_FREQ);

static volatile f32 motor_phase_resistance = 0.f;
//volatile bool motor_phase_resistance_valid = false;

static volatile u32 resistance_cal_n_loop_start = 0;
static volatile u32 resistance_cal_n_loop_end = 0;


void init_resistance_cal() {
    resistance_calib_test_voltage = 0.f;
    resistance_calib_output_mod = 0.f;
    resistance_calib_current_ialpha_filt = 0.f;
    resistance_calib_current_ibeta_filt = 0.f;

    u32 n_loops = controller_get_loop_count();
    resistance_cal_n_loop_start = n_loops;
    resistance_cal_n_loop_end = n_loops + resistance_cal_length_cycles;
}

static bool resistance_cal_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    ab_vec_t ab_currents = clarke_transform((svm_vec_t){.A = A_i, .B = B_i, .C = C_i});

    f32 alpha_current_err = MOTOR_CALIBRATION_CURRENT - ab_currents.alpha;
    resistance_calib_test_voltage += (resistance_cal_ki * current_meas_period) * alpha_current_err;

    f32 ialpha_filt_err = ab_currents.alpha - resistance_calib_current_ialpha_filt;
    resistance_calib_current_ialpha_filt += (resistance_calib_current_filter * current_meas_period) * ialpha_filt_err;

    f32 ibeta_filt_err = ab_currents.beta - resistance_calib_current_ibeta_filt;
    resistance_calib_current_ibeta_filt += (resistance_calib_current_filter * current_meas_period) * ibeta_filt_err;

    f32 vmod = resistance_calib_test_voltage * (1.f / ((2.f / 3.f) * Vbus));

    resistance_calib_output_mod = vmod;

    if (ab_currents.alpha < -1.0f) {
        return false;
    }

    if (vmod > 1.0f) {
        return false;
    }


    if (abs_f32(resistance_calib_current_ialpha_filt) > MOTOR_CURRENT_TRIP_LIMIT) {
        controller_add_errors(CTRLR_ERR_MOTOR_OVERCURRENT);
        return false;
    }
    if (abs_f32(resistance_calib_current_ibeta_filt) > MOTOR_CURRENT_TRIP_LIMIT) {
        controller_add_errors(CTRLR_ERR_MOTOR_OVERCURRENT);
        return false;
    }
    if (abs_f32(resistance_calib_test_voltage) > MAX_RESISTANCE_CAL_VOLTAGE) {
        controller_add_errors(CTRLR_ERR_MOTOR_RESISTANCE_OUT_OF_RANGE);
        return false;
    }

    return true;
}

bool resistance_cal_pwm_cb(svm_vec_t* mag) {
    ab_vec_t resistance_mod = {.alpha = resistance_calib_output_mod, .beta = 0.f};
    svm_vec_t svm_magnitudes = SVM(resistance_mod);
    *mag = svm_magnitudes;

    return true;
}

bool resistance_cal_encoder_cb(u16 _) {
    return true;
}

// Called at ~8kHz or so externally
void resistance_cal_exec() {
}

bool resistance_cal_is_complete() {
    return controller_get_loop_count() > resistance_cal_n_loop_end;
}

bool resistance_cal_finish(bool was_cancelled) {
    if (was_cancelled) return false;

    f32 _phase_resistance = resistance_calib_test_voltage / MOTOR_CALIBRATION_CURRENT;

    if ((_phase_resistance < MOTOR_PHASE_RESISTANCE_MIN) || (_phase_resistance > MOTOR_PHASE_RESISTANCE_MAX)) {
        controller_add_errors(CTRLR_ERR_MOTOR_RESISTANCE_OUT_OF_RANGE);
        //motor_phase_resistance_valid = false;
        return false;
    }

    printf("Motor Resistance: %.3f mΩ\r\n", _phase_resistance*1000.);
    //motor_phase_resistance_valid = true;
    motor_phase_resistance = _phase_resistance;
    return true;
}

//bool get_phase_resistance_valid() {
//    return motor_phase_resistance_valid;
//}

f32 get_phase_resistance() {
    return motor_phase_resistance;
}

control_law_t resistance_cal_ctrl_law = {
    .control_law_init = init_resistance_cal,
    .encoder_consumer = resistance_cal_encoder_cb,
    .current_consumer = resistance_cal_current_cb,
    .pwm_producer = resistance_cal_pwm_cb,
    .control_law_exec = resistance_cal_exec,
    .is_complete = resistance_cal_is_complete,
    .control_law_finish = resistance_cal_finish
};

// ============== PHASE INDUCTANCE =====================

const u32 inductance_cal_length_cycles = (u32)(((f32)RESISTANCE_CAL_TIME / 1000.f) * LOOP_UPDATE_FREQ);

volatile f32 inductance_calib_test_voltage = MAX_INDUCTANCE_CAL_VOLTAGE;

volatile f32 inductance_calib_output_mod = 0.f;

volatile f32 inductance_calib_last_ialpha = 0.f;
volatile f32 inductance_calib_delta_ialpha = 0.f;

static volatile f32 motor_phase_inductance = 0.f;

static volatile u32 inductance_cal_n_loop_start = 0;
static volatile u32 inductance_cal_n_loop_end = 0;

static volatile u32 inductance_cal_last_loop_count = 0;

void init_inductance_cal() {
    inductance_calib_output_mod = 0.f;
    inductance_calib_last_ialpha = 0.f;
    inductance_calib_delta_ialpha = 0.f;

    u32 n_loops = controller_get_loop_count();
    inductance_cal_n_loop_start = n_loops;
    inductance_cal_n_loop_end = n_loops + inductance_cal_length_cycles;
}

bool inductance_cal_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    ab_vec_t ab_currents = clarke_transform((svm_vec_t){.A = A_i, .B = B_i, .C = C_i});

    f32 sign = inductance_calib_test_voltage > 0.f ? 1.f : -1.f;
    inductance_calib_delta_ialpha += -sign * (ab_currents.alpha - inductance_calib_last_ialpha);
    inductance_calib_last_ialpha = ab_currents.alpha;

    inductance_calib_test_voltage *= -1.f;
    f32 inductance_calib_vfactor = 1.f / ((2.f / 3.f) * Vbus);
    inductance_calib_output_mod = inductance_calib_vfactor * inductance_calib_test_voltage;
    //inductance_calib_output_mod = inductance_calib_vfactor * MAX_INDUCTANCE_CAL_VOLTAGE;

    if (ab_currents.alpha > MOTOR_CURRENT_TRIP_LIMIT) return false;
    return true;
}


bool inductance_cal_pwm_cb(svm_vec_t* mag) {
    ab_vec_t inductance_mod = {.alpha = inductance_calib_output_mod, .beta = 0.f};

    svm_vec_t svm_magnitudes = SVM(inductance_mod);
    *mag = svm_magnitudes;
    return svm_output_valid(svm_magnitudes);
}

bool inductance_cal_encoder_cb(u16 _) {
    return true;
}

void inductance_cal_exec() {
}

bool inductance_cal_is_complete() {
    u32 _loop = controller_get_loop_count();
    inductance_cal_last_loop_count = _loop;
    return _loop > inductance_cal_n_loop_end;
}

bool inductance_cal_finish(bool was_cancelled) {
    if (was_cancelled) return false;

    f32 dt = (inductance_cal_last_loop_count - inductance_cal_n_loop_start) / LOOP_UPDATE_FREQ;
    f32 _phase_inductance = abs_f32(inductance_calib_test_voltage) / (inductance_calib_delta_ialpha / dt);

    if (_phase_inductance < MOTOR_PHASE_INDUCTANCE_MIN || _phase_inductance > MOTOR_PHASE_INDUCTANCE_MAX) {
        controller_add_errors(CTRLR_ERR_MOTOR_INDUCTANCE_OUT_OF_RANGE);
        return false;
    }

    printf("Motor Inductance: %.3f uH\r\n", _phase_inductance*1000.*1000.);
    motor_phase_inductance = _phase_inductance;

    return true;
}

control_law_t inductance_cal_ctrl_law = {
    .control_law_init = init_inductance_cal,
    .encoder_consumer = inductance_cal_encoder_cb,
    .current_consumer = inductance_cal_current_cb,
    .pwm_producer = inductance_cal_pwm_cb,
    .control_law_exec = inductance_cal_exec,
    .is_complete = inductance_cal_is_complete,
    .control_law_finish = inductance_cal_finish
};

f32 get_phase_inductance() {
    return motor_phase_inductance;
}
