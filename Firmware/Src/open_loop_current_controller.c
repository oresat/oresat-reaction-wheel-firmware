#include "open_loop_current_controller.h"
#include "bldc_math.h"
#include "mathutil.h"
#include "config.h"
#include "control_ll.h"
#include "encoder.h"
#include "util.h"

// ============== OPEN LOOP CURRENT CONTROL =====================

/*
 * Useful for basic open-loop motor spinning
 * Does not use encoder feedback
 * Takes just an alpha/beta current setpoint
 * TODO in the future - may want to just specify +d axis current + vel + end on distance
 */

static volatile f32 current_ctrl_kp = 0.f;
static volatile f32 current_ctrl_ki = 0.f;

static volatile ab_vec_t ialpha_beta_setpoint = {0.f, 0.f};
static volatile ab_vec_t ialpha_beta_measured = {0.f, 0.f};
static volatile f32 vvbus_meas = 0.f;
static volatile ab_vec_t v_current_ctrl_integral = {0.f, 0.f};

static volatile svm_vec_t abc_currents = {0.f, 0.f, 0.f};
static volatile f32 vvbus = 0.f;

static volatile f32 ibus = 0.f;

void init_open_loop_current_controller() {
    f32 phase_resistance = HARDCODE_PHASE_RESISTANCE;
    f32 phase_inductance = HARDCODE_PHASE_INDUCTANCE;

    f32 p_gain = CURRENT_CONTROLLER_BANDWIDTH * phase_inductance;
    f32 RL_pole = phase_resistance / phase_inductance;
    current_ctrl_kp = p_gain;
    current_ctrl_ki = RL_pole * p_gain;

    ialpha_beta_setpoint = (ab_vec_t){0.f, 0.f};
    ialpha_beta_measured = (ab_vec_t){0.f, 0.f};
    v_current_ctrl_integral = (ab_vec_t){0.f, 0.f};
    vvbus_meas = 0.f;
}

__ccmram bool open_loop_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    abc_currents = (svm_vec_t){A_i, B_i, C_i};
    vvbus = Vbus;
    ab_vec_t Ialpha_beta_currents = clarke_transform((svm_vec_t){A_i, B_i, C_i});
    ialpha_beta_measured = Ialpha_beta_currents;
    vvbus_meas = Vbus;
    return true;
}

__ccmram bool open_loop_pwm_cb(svm_vec_t* mag) {
    f32 vvbus_meas_ = vvbus_meas;
    ab_vec_t ialpha_beta_measured_ = ialpha_beta_measured;
    ab_vec_t ialpha_beta_setpoint_ = ialpha_beta_setpoint;
    ab_vec_t v_current_ctrl_integral_ = v_current_ctrl_integral;

    f32 p_gain = current_ctrl_kp;
    f32 i_gain = current_ctrl_ki;

    f32 mod_to_V = (2.f / 3.f) * vvbus_meas_;
    f32 V_to_mod = 1.f / mod_to_V;

    ab_vec_t iab_err = (ab_vec_t){
        ialpha_beta_setpoint_.alpha - ialpha_beta_measured_.alpha,
        ialpha_beta_setpoint_.beta - ialpha_beta_measured_.beta
    };
    ab_vec_t mod_ab = (ab_vec_t){
        V_to_mod * (v_current_ctrl_integral_.alpha + iab_err.alpha * p_gain),
        V_to_mod * (v_current_ctrl_integral_.beta + iab_err.beta * p_gain)
    };

    f32 mod_scalefactor = 0.8f * sqrt3_over_2 * 1.f / sqrt_f32(mod_ab.alpha * mod_ab.alpha + mod_ab.beta * mod_ab.beta);
    if (mod_scalefactor < 1.f) {
        mod_ab.alpha *= mod_scalefactor;
        mod_ab.beta *= mod_scalefactor;
        v_current_ctrl_integral_.alpha *= 0.99f;
        v_current_ctrl_integral_.beta *= 0.99f;
    }
    else {
        v_current_ctrl_integral_.alpha += iab_err.alpha * (i_gain * 1.f / ((f32)CURRENT_MEAS_FREQ));
        v_current_ctrl_integral_.beta += iab_err.beta * (i_gain * 1.f / ((f32)CURRENT_MEAS_FREQ));
    }
    ibus = mod_ab.alpha * ialpha_beta_measured_.alpha + mod_ab.beta * ialpha_beta_measured_.beta;

    dac_debug1((u32) clamp_f32(0.f,mod_ab.alpha * 4095.f,4095.f));


    svm_vec_t pwm_output = SVM(mod_ab);
    if (!svm_output_valid(pwm_output))
        controller_add_errors(CTRLR_ERR_MODULATION);
    *mag = pwm_output;

    v_current_ctrl_integral = v_current_ctrl_integral_;

    return true;
}

__ccmram bool open_loop_encoder_cb(u16 meas) {
    encoder_update(meas);
    return true;
}

__ccmram void open_loop_exec() {
}

__ccmram bool open_loop_is_complete() {
    return false;
}

bool open_loop_finish(bool was_cancelled) {
    return true;
}

__ccmram void set_open_loop_current_setpoint(ab_vec_t current_setpoint) {
    ialpha_beta_setpoint = current_setpoint;
}

ab_vec_t get_open_loop_currents() {
    return ialpha_beta_measured;
}

control_law_t open_loop_ctrl_law = {
    .control_law_init = init_open_loop_current_controller,
    .encoder_consumer = open_loop_encoder_cb,
    .current_consumer = open_loop_current_cb,
    .pwm_producer = open_loop_pwm_cb,
    .control_law_exec = open_loop_exec,
    .is_complete = open_loop_is_complete,
    .control_law_finish = open_loop_finish
};
