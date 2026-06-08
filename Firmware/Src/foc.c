//
// Created by solomon on 5/24/23.
//


#include "foc.h"
#include "bldc_math.h"
#include "motor_calibration.h"
#include "config.h"
#include "control_ll.h"
#include "encoder.h"
#include "foc_capture.h"
#include "mathutil.h"
#include "printf.h"
#include "swo_util.h"

bool foc_meas_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus);

bool foc_gen_pwm_cb(svm_vec_t *mag);

bool foc_read_encoder_cb(u16 enc_val);

//static volatile f32 current_ctrl_kp = 0.f;
//static volatile f32 current_ctrl_ki = 0.f;

static const f32 current_ctrl_kp = CURRENT_CONTROLLER_BANDWIDTH * HARDCODE_PHASE_INDUCTANCE;
static const f32 current_ctrl_ki =
        (HARDCODE_PHASE_RESISTANCE / HARDCODE_PHASE_INDUCTANCE) * CURRENT_CONTROLLER_BANDWIDTH *
        HARDCODE_PHASE_INDUCTANCE;

static const f32 foc_loop_period = 1.f / ((f32) CURRENT_MEAS_FREQ);

static volatile dq_vec_t idq_setpoint = {0.f, 0.f};
static volatile f32 motor_phase = 0.f;
static volatile f32 motor_vel = 0.f;
static volatile f32 vvbus_meas = 0.f;

static volatile ab_vec_t ialpha_beta_measured = {0.f, 0.f};
static volatile dq_vec_t idq_measured = {0.f, 0.f};
volatile dq_vec_t v_current_ctrl_integral = {0.f, 0.f};

static const f32 idq_measured_filter = 1.f;
static volatile f32 ibus = 0.f;

// Latest phase currents stashed by foc_meas_current_cb so foc_gen_pwm_cb
// can include them in the per-tick capture snapshot.
static volatile f32 last_Ia = 0.f, last_Ib = 0.f, last_Ic = 0.f;


void init_foc() {
    // TODO LOL
    //if (!is_phase_inductance_calibrated() || !is_phase_resistance_calibrated()) return false;
    //f32 phase_resistance = get_calibrated_phase_resistance();
    //f32 phase_inductance = get_calibrated_phase_inductance();

    f32 phase_resistance = HARDCODE_PHASE_RESISTANCE;
    f32 phase_inductance = HARDCODE_PHASE_INDUCTANCE;

    f32 p_gain = CURRENT_CONTROLLER_BANDWIDTH * phase_inductance;
    f32 RL_pole = phase_resistance / phase_inductance;
    //current_ctrl_kp = p_gain;
    //current_ctrl_ki = RL_pole * p_gain;

    printf("FOC Kp: %.5f, FOC Ki: %.5f\r\n", p_gain, RL_pole * p_gain);
    printf("FOC hardcode Kp: %.5f, FOC Ki: %.5f\r\n", current_ctrl_kp, current_ctrl_ki);

    idq_measured = (dq_vec_t) {0.f, 0.f};
    v_current_ctrl_integral = (dq_vec_t) {0.f, 0.f};
    ialpha_beta_measured = (ab_vec_t) {0.f, 0.f};
    idq_setpoint = (dq_vec_t) {0.f, 0.f};
    ibus = 0.f;
}

bool foc_meas_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    //debug_phA_current(A_i);
    //debug_phB_current(B_i);
    //debug_phC_current(C_i);

    last_Ia = A_i;
    last_Ib = B_i;
    last_Ic = C_i;

    ab_vec_t Ialpha_beta_currents = clarke_transform((svm_vec_t) {A_i, B_i, C_i});
    ialpha_beta_measured = Ialpha_beta_currents;
    vvbus_meas = Vbus;

    return true;
}

volatile f32 debug_Vq = 0.f;

// I fucking shit you not this is actually useful here
static __fn_inline __fn_const f32 Q_rsqrt(f32 number) {
    i32 i;
    f32 x2, y;
    const f32 threehalfs = 1.5f;

    x2 = number * 0.5F;
    y = number;
    i = *(i32 *) &y; // evil floating point bit level hacking
    i = 0x5f3759df - (i >> 1); // what the fuck?
    y = *(f32 *) &i;
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration
    // y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
}

bool foc_gen_pwm_cb(svm_vec_t *mag) {
    dq_vec_t idq_setpoint_ = idq_setpoint;
    f32 phase_ = motor_phase;
    f32 vel_ = motor_vel;
    f32 vvbus_meas_ = vvbus_meas;
    dq_vec_t v_current_ctrl_integral_ = v_current_ctrl_integral;
    ab_vec_t ialpha_beta_meas_ = ialpha_beta_measured;
    dq_vec_t idq_measured_ = idq_measured;

    //debug_Id_setpoint(idq_setpoint_.d);
    //debug_Iq_setpoint(idq_setpoint_.q);
    //debug_enc_phase_pos(phase_);
    //debug_enc_phase_vel(vel_);
    //debug_vbus(vvbus_meas_);
    //debug_Ialpha_meas(ialpha_beta_meas_.alpha);
    //debug_Ibeta_meas(ialpha_beta_meas_.beta);
    //debug_Id_meas(idq_measured_.d);
    //debug_Iq_meas(idq_measured_.q);

    //f32 current_phase = phase_ + vel_ * 1.f / ((f32) ENC_MEAS_FREQ);
    f32 current_phase = phase_;
    dq_vec_t idq = park_transform(ialpha_beta_meas_, current_phase);

    idq_measured_.d += idq_measured_filter * (idq.d - idq_measured_.d);
    idq_measured_.q += idq_measured_filter * (idq.q - idq_measured_.q);

    f32 mod_to_V = (2.f / 3.f) * vvbus_meas_;
    f32 V_to_mod = 1.f / mod_to_V;

    f32 p_gain = current_ctrl_kp;
    f32 i_gain = current_ctrl_ki;

    dq_vec_t Vdq_ff = {0.f, 0.f};

    if (FOC_USE_wL_FF) {
        Vdq_ff.d -= vel_ * HARDCODE_PHASE_INDUCTANCE * idq_setpoint_.q;
        Vdq_ff.q += vel_ * HARDCODE_PHASE_INDUCTANCE * idq_setpoint_.d;
    }
    if (FOC_USE_R_FF) {
        Vdq_ff.d += HARDCODE_PHASE_RESISTANCE * idq_setpoint_.d;
        Vdq_ff.q += HARDCODE_PHASE_RESISTANCE * idq_setpoint_.q;
    }

    if (FOC_USE_bEMF_FF) {
        Vdq_ff.q += vel_ * (2.f / 3.f) * (MOTOR_TORQUE_CONSTANT / (f32) MOTOR_NUM_POLEPAIRS);
    }

    dq_vec_t idq_err = {idq_setpoint_.d - idq.d, idq_setpoint_.q - idq.q};
    dq_vec_t mod_dq = {
            V_to_mod * (Vdq_ff.d + v_current_ctrl_integral_.d + idq_err.d * p_gain),
            V_to_mod * (Vdq_ff.q + v_current_ctrl_integral_.q + idq_err.q * p_gain),
    };

    f32 mod_scalefactor = 0.99f * sqrt3_over_2 * Q_rsqrt(mod_dq.d * mod_dq.d + mod_dq.q * mod_dq.q);
    if (mod_scalefactor < 1.f) {
        mod_dq.d *= mod_scalefactor;
        mod_dq.q *= mod_scalefactor;
        v_current_ctrl_integral_.d *= 0.99f;
        v_current_ctrl_integral_.q *= 0.99f;
    } else {
        v_current_ctrl_integral_.d += idq_err.d * (i_gain * 1.f / ((f32) CURRENT_MEAS_FREQ));
        v_current_ctrl_integral_.q += idq_err.q * (i_gain * 1.f / ((f32) CURRENT_MEAS_FREQ));
    }

    swo_print_f32(_Id_meas,idq_measured_.d);
    swo_print_f32(_Iq_meas,idq_measured_.q);
    swo_print_f32(_Iq_setpoint,idq_setpoint_.q);
    swo_print_f32(_Vq_integral,v_current_ctrl_integral_.q);


    //debug_Vd_integral(v_current_ctrl_integral_.d);
    //debug_Vq_integral(v_current_ctrl_integral_.q);
    //debug_mod_d(mod_dq.d);
    //debug_mod_q(mod_dq.q);

    f32 pwm_phase = phase_ + vel_ * 2.f / ((f32) ENC_MEAS_FREQ); // TODO <- why is this 2/?

    ab_vec_t mod_ab = inv_park_transform(mod_dq, pwm_phase);

    //debug_mod_alpha(mod_ab.alpha);
    //debug_mod_beta(mod_ab.beta);

    ibus = mod_dq.d * idq.d + mod_dq.q * idq.q;

    //debug_ibus(ibus);

    svm_vec_t pwm_output = SVM(mod_ab);

    //debug_svm_A(pwm_output.A);
    //debug_svm_B(pwm_output.B);
    //debug_svm_C(pwm_output.C);

    *mag = pwm_output;

    idq_measured = idq_measured_;
    v_current_ctrl_integral = v_current_ctrl_integral_;

    if (foc_capture_state == FOC_CAPTURE_RUNNING) {
        u32 idx = foc_capture_idx;
        foc_capture_buf[idx % FOC_CAPTURE_LEN] = (foc_capture_t){
            .loop_count = controller_get_loop_count(),
            .Ia = last_Ia, .Ib = last_Ib, .Ic = last_Ic,
            .Ialpha = ialpha_beta_meas_.alpha,
            .Ibeta  = ialpha_beta_meas_.beta,
            .Id_meas = idq.d,
            .Iq_meas = idq.q,
            .Id_set  = idq_setpoint_.d,
            .Iq_set  = idq_setpoint_.q,
            .motor_phase = phase_,
            .motor_vel   = vel_,
            .Vd_int = v_current_ctrl_integral_.d,
            .Vq_int = v_current_ctrl_integral_.q,
            .mod_d = mod_dq.d,
            .mod_q = mod_dq.q,
            .mod_alpha = mod_ab.alpha,
            .mod_beta  = mod_ab.beta,
            .svm_A = pwm_output.A,
            .svm_B = pwm_output.B,
            .svm_C = pwm_output.C,
            .Vbus  = vvbus_meas_,
        };
        foc_capture_idx = idx + 1;  // monotonic; ring wrap is at the index
    }

    return true;
}

bool foc_read_encoder_cb(u16 enc_val) {
    encoder_update(enc_val);
    posvel_t phase_posvel = encoder_get_phase();
    motor_phase = phase_posvel.pos;
    motor_vel = phase_posvel.vel;
    //posvel_t enc_posvel = encoder_get_pos_vel();
    //debug_enc_pos(enc_posvel.pos);
    //debug_enc_vel(enc_posvel.vel);

    return true;
}

void foc_exec() {
}

bool foc_is_complete() {
    return false;
}

bool foc_finish(bool was_cancelled) {
    return true;
}

void set_iq_setpoint(f32 iq) {
    idq_setpoint.d = 0.f;
    idq_setpoint.q = iq;
}

dq_vec_t get_idq_measured() {
    return idq_measured;
}

f32 get_foc_ibus() {
    return ibus;
}

f32 get_foc_vbus() {
    return vvbus_meas;
}

control_law_t foc_control_law = {
        .encoder_consumer = foc_read_encoder_cb,
        .current_consumer = foc_meas_current_cb,
        .pwm_producer = foc_gen_pwm_cb,
        .control_law_exec = foc_exec,
        .is_complete = foc_is_complete,
        .control_law_init = init_foc,
        .control_law_finish = foc_finish
};
