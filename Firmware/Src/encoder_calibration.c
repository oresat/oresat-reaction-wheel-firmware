#include "encoder_calibration.h"
#include "bldc_math.h"
#include "config.h"
#include "control_ll.h"
#include "encoder.h"
#include "foc.h"
#include "mathutil.h"
#include "open_loop_current_controller.h"
#include "printf.h"
#include "util.h"
#include <math.h>

// Since both dir and phase cal involve recording encoder position and setting open-loop current angles
// We can unify the functions and implementations
// Here are variables used for both cal phases

static volatile f32 last_encoder_pos = 0.f; // Local cache of encoder position
static volatile f32 lockin_phase_angle = 0.f; // Phase angle in rads
static volatile f32 lockin_current = 0.f; // Phase current in amps

__ccmram bool encoder_cal_encoder_cb(u16 angle) {
    encoder_update(angle);
    return true;
}

// Passthrough to open loop control law
__ccmram bool encoder_cal_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    return open_loop_ctrl_law.current_consumer(A_i, B_i, C_i, Vbus);
}

__ccmram bool encoder_cal_pwm_cb(svm_vec_t *mag) {
    return open_loop_ctrl_law.pwm_producer(mag);
}


// =================== Encoder Dir Find ==================
// Record initial encoder position, spin +1 rev, record new encoder position
// If the encoder angle doesn't match the electrical angle, flip the encoder direction


static volatile u32 dir_cal_n_loop_start = 0;
static volatile u32 dir_cal_n_loop_end = 0;

static const u32 dir_cal_n_loops_total = (u32) ((ENCODER_DIR_CAL_ROTATION_DIST / ENCODER_DIR_CAL_ROTATION_VEL) *
                                                LOOP_UPDATE_FREQ);
static const u32 dir_cal_n_loop_lockin = (u32) (1.f * LOOP_UPDATE_FREQ); // Lockin start direction for one second

static volatile f32 dir_cal_enc_pos_start = 0.f;

static volatile f32 last_enc_pos = 0.f;

void encoder_dir_cal_init() {
    encoder_set_direction(1.f);
    encoder_init();
    open_loop_ctrl_law.control_law_init();

    u32 _n_loop_start = controller_get_loop_count();
    u32 _n_loop_end = _n_loop_start + dir_cal_n_loops_total + dir_cal_n_loop_lockin;
    dir_cal_n_loop_start = _n_loop_start;
    dir_cal_n_loop_end = _n_loop_end;
}

__ccmram void encoder_dir_cal_exec_cb() {
    u32 n_loops = controller_get_loop_count() - dir_cal_n_loop_start;

    if (n_loops == dir_cal_n_loop_lockin)
        dir_cal_enc_pos_start = encoder_get_multiturn_pos_raw(); // Capture initial position

    f32 percent_complete = ((f32) (n_loops - dir_cal_n_loop_lockin) / (f32) dir_cal_n_loops_total);
    f32 phase_pos = n_loops < dir_cal_n_loop_lockin
                    ? 0.f
                    : MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32 *
                      percent_complete * ENCODER_DIR_CAL_ROTATION_DIST;

    f32 lockin_mod = (n_loops < dir_cal_n_loop_lockin) ? clamp_f32(0.f, 2.f * (f32) n_loops / (f32) dir_cal_n_loop_lockin, 1.f) : 1.f;

    dq_vec_t dq_lockin = {.d = lockin_mod * LOCKIN_SPIN_CURRENT, .q = 0.f};
    ab_vec_t ab_lockin = inv_park_transform(dq_lockin, phase_pos);

    if (controller_get_loop_count() >= (dir_cal_n_loop_end - 3))
        __asm__("nop");

    set_open_loop_current_setpoint(ab_lockin);
}

__ccmram bool encoder_dir_cal_is_complete() {
    return controller_get_loop_count() >= dir_cal_n_loop_end;
}

bool encoder_dir_cal_finish(bool was_cancelled) {
    open_loop_ctrl_law.control_law_finish(was_cancelled);
    if (was_cancelled) return false;

    f32 enc_distance = encoder_get_multiturn_pos_raw() - dir_cal_enc_pos_start;

    f32 enc_distance_max = ENCODER_DIR_CAL_ROTATION_DIST * (1.f + ENCODER_DIR_CAL_ROTATION_TOLERANCE);
    f32 enc_distance_min = ENCODER_DIR_CAL_ROTATION_DIST * (1.f - ENCODER_DIR_CAL_ROTATION_TOLERANCE);

    f32 enc_distance_abs = abs_f32(enc_distance);

    if (enc_distance_abs < enc_distance_min || enc_distance_abs > enc_distance_max)
        return false;

    encoder_set_direction(
            enc_distance > 0 ? 1.f : -1.f
    );

    printf("Distance %.3f\r\n", enc_distance);
    char buf[64];
    u32 len = snprintf(buf, 64, "ENCODER CALIB DIRECTION: %.3f\r\n", enc_distance > 0 ? 1.f : -1.f);
    uart_transmit_buf(COMMS_UART_HANDLE, buf, len);

    return true;
}

control_law_t encoder_dir_cal_ctrl_law = {
        .control_law_init = encoder_dir_cal_init,
        .encoder_consumer = encoder_cal_encoder_cb,
        .current_consumer = encoder_cal_current_cb,
        .pwm_producer = encoder_cal_pwm_cb,
        .control_law_exec = encoder_dir_cal_exec_cb,
        .is_complete = encoder_dir_cal_is_complete,
        .control_law_finish = encoder_dir_cal_finish
};


// ==================== Encoder Phase Find ====================
// Record encoder phase while spinning some amount up-down (to balance friction effects)
// Calculate phase offset to align the motor phase with encoder phase

static volatile u32 phase_cal_n_loop_start = 0;
static volatile u32 phase_cal_n_loop_end = 0;

static const u32 phase_cal_n_loops_per_leg = (u32) ((ENCODER_PHASE_CAL_ROTATION_DIST / ENCODER_PHASE_CAL_ROTATION_VEL) *
                                                    LOOP_UPDATE_FREQ);
static const u32 phase_cal_n_loops_spin = 2u * phase_cal_n_loops_per_leg;

// Commanded cruise speed during the constant-velocity portion of each leg, in electrical rad/s (for
// the velocity gate). The trapezoid's ramps mean the peak (cruise) speed must exceed the nominal
// rev/s so the leg still covers ROTATION_DIST in the same total time — i.e. heat-neutral vs. the old
// position-triangle, which cruised at exactly ROTATION_VEL.
static const f32 phase_cal_cruise_evel = (ENCODER_PHASE_CAL_ROTATION_VEL / (1.f - ENCODER_PHASE_CAL_RAMP_FRAC)) *
                                         (f32) MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32;
static const f32 phase_cal_vel_gate = ENCODER_PHASE_CAL_VEL_GATE_FRAC * phase_cal_cruise_evel;

static const u32 phase_cal_n_loop_lockin = (u32) (1.f * LOOP_UPDATE_FREQ); // Lockin start direction for one second

// Accumulate (commanded - measured) electrical-phase error separately per sweep direction so the two
// means can be averaged explicitly in finish (cancels friction even if the gated sample counts differ).
static volatile f32 error_accum_fwd = 0.f;
static volatile u32 error_count_fwd = 0;
static volatile f32 error_accum_rev = 0.f;
static volatile u32 error_count_rev = 0;

static const f32 erad_total = MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32 * ENCODER_PHASE_CAL_ROTATION_DIST;
static const f32 erad_start = 0;

static volatile bool first_run = true;

// Guaranteed to be called loop-synchronously with exec/is_complete/etc
void encoder_phase_cal_init() {
    encoder_init();
    open_loop_ctrl_law.control_law_init();
    if (first_run) {
        printf("first\r\n");
        encoder_set_phase_offset(0);
        first_run = false;
    }

    u32 _n_loop_start = controller_get_loop_count();
    u32 _n_loop_end = _n_loop_start + phase_cal_n_loops_spin + phase_cal_n_loop_lockin;
    phase_cal_n_loop_start = _n_loop_start;
    phase_cal_n_loop_end = _n_loop_end;

    error_accum_fwd = 0.f;
    error_count_fwd = 0;
    error_accum_rev = 0.f;
    error_count_rev = 0;

    //erad_start = wrap_pm_pi(0 - erad_total / 2);
}

// Normalized trapezoidal motion profile. `u` in [0,1] is the fraction of a leg's elapsed time; `a` in
// (0,0.5) is the fraction of the leg spent in each (accel / decel) ramp. Returns the fraction of the
// leg's distance covered, [0,1]: velocity ramps 0->peak over [0,a], holds over [a,1-a], ramps back to
// 0 over [1-a,1]. Peak normalized velocity is 1/(1-a) so the area (distance) integrates to exactly 1.
static __fn_inline f32 trapezoid_pos(f32 u, f32 a) {
    f32 vpk = 1.f / (1.f - a);
    if (u < a)
        return vpk * u * u / (2.f * a);   // accel ramp: ½·(vpk/a)·u²
    if (u <= 1.f - a)
        return vpk * (u - 0.5f * a);      // cruise
    f32 w = 1.f - u;                       // decel ramp (mirror of accel)
    return 1.f - vpk * w * w / (2.f * a);
}


__ccmram void encoder_phase_cal_exec_cb() {
    u32 n_loops = controller_get_loop_count() - phase_cal_n_loop_start;

    f32 phase_pos;

    // End of lock-in: seed a coarse one-point offset so the fine correction below stays small and
    // wrap-safe. The bidirectional sweep then refines it (see encoder_phase_cal_finish).
    if (n_loops == phase_cal_n_loop_lockin)
        encoder_set_phase_offset(encoder_get_pos_vel().pos);

    if (n_loops < phase_cal_n_loop_lockin) {
        phase_pos = erad_start;
    } else {
        u32 s = n_loops - phase_cal_n_loop_lockin;          // index within the spin phase
        bool outbound = (s < phase_cal_n_loops_per_leg);
        u32 s_leg = outbound ? s : (s - phase_cal_n_loops_per_leg);
        f32 u = clamp_f32(0.f, (f32) s_leg / (f32) phase_cal_n_loops_per_leg, 1.f); // 0..1 within leg

        // Trapezoidal velocity profile (fix 3): finite acceleration instead of the old position-
        // triangle's instantaneous velocity reversals, so we stop impulse-exciting the rotor/field
        // resonance. Outbound covers 0->DIST, inbound DIST->0.
        f32 leg_frac = trapezoid_pos(u, ENCODER_PHASE_CAL_RAMP_FRAC);
        f32 pos_mechanical = ENCODER_PHASE_CAL_ROTATION_DIST * (outbound ? leg_frac : 1.f - leg_frac);
        f32 pos_electrical = pos_mechanical * MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32;
        phase_pos = wrap_pm_pi(pos_electrical);

        // Accumulate the phase error only in the constant-velocity cruise, moving in the expected
        // direction at speed (fix 2): rejects the accel/decel ramps, the v~=0 turnaround stiction
        // band, and post-step ringing. Keep the two directions separate (fix 4). If dir-cal hasn't
        // run, the measured phase moves the "wrong" way and the gate rejects everything -> finish bails.
        posvel_t enc = encoder_get_phase();
        f32 leg_sign = outbound ? 1.f : -1.f;
        bool in_cruise = (u >= ENCODER_PHASE_CAL_RAMP_FRAC) && (u <= 1.f - ENCODER_PHASE_CAL_RAMP_FRAC);
        bool vel_ok = (leg_sign * enc.vel) > phase_cal_vel_gate;

        if (in_cruise && vel_ok) {
            f32 error = wrap_pm_pi(phase_pos - enc.pos);
            if (outbound) {
                error_accum_fwd += error;
                error_count_fwd++;
            } else {
                error_accum_rev += error;
                error_count_rev++;
            }
        }
    }

    f32 lockin_mod = (n_loops < phase_cal_n_loop_lockin)
                     ? clamp_f32(0.f, 2.f * (f32) n_loops / (f32) phase_cal_n_loop_lockin, 1.f)
                     : 1.f;

    dq_vec_t dq_lockin = {.d = lockin_mod * LOCKIN_SPIN_CURRENT, .q = 0.f};
    ab_vec_t ab_lockin = inv_park_transform(dq_lockin, phase_pos);

    set_open_loop_current_setpoint(ab_lockin);
}

__ccmram bool encoder_phase_cal_is_complete() {
    return controller_get_loop_count() >= phase_cal_n_loop_end;
}

// Not guarenteed to be called loop-synchronously with exec/is_complete/etc
bool encoder_phase_cal_finish(bool was_cancelled) {
    open_loop_ctrl_law.control_law_finish(was_cancelled);
    if (was_cancelled) return false;

    // Need clean cruise samples in both directions, otherwise friction can't be cancelled. Bailing
    // here (rather than applying a one-sided result) is also the failure signature when encoder
    // dir-cal hasn't run: the velocity gate then rejects every sample.
    if (error_count_fwd == 0 || error_count_rev == 0) {
        printf("Phase cal failed: no cruise samples (fwd=%u rev=%u) - run dir cal first?\r\n",
               (unsigned) error_count_fwd, (unsigned) error_count_rev);
        return false;
    }

    // Average the two directions' means (fix 4): mean_fwd = offset + friction, mean_rev = offset -
    // friction, so 0.5*(mean_fwd + mean_rev) cancels friction regardless of unequal sample counts.
    f32 mean_fwd = error_accum_fwd / (f32) error_count_fwd;
    f32 mean_rev = error_accum_rev / (f32) error_count_rev;
    f32 phase_offset = 0.5f * (mean_fwd + mean_rev);

    f32 phase_offset_mechanical = (phase_offset / encoder_get_direction()) / (2.f * M_PI_f32 * MOTOR_NUM_POLEPAIRS);

    // Apply it (fix 1). Sign: encoder_get_phase() ~ E*(pos - offset)*dir with E = 2pi*polepairs, so
    // d(phase)/d(offset) = -E*dir; nudging offset by -error/(E*dir) drives the mean error to zero.
    encoder_set_phase_offset(encoder_get_phase_offset() - phase_offset_mechanical);

    printf("Phase cal: offset %.5f rad-e (%.6f rev-m) | fwd %.5f rev %.5f | n=%u/%u\r\n",
           phase_offset, phase_offset_mechanical, mean_fwd, mean_rev,
           (unsigned) error_count_fwd, (unsigned) error_count_rev);

    return true;
}


control_law_t encoder_phase_cal_ctrl_law = {
        .encoder_consumer = encoder_cal_encoder_cb,
        .current_consumer = encoder_cal_current_cb,
        .pwm_producer = encoder_cal_pwm_cb,
        .control_law_exec = encoder_phase_cal_exec_cb,
        .is_complete = encoder_phase_cal_is_complete,
        .control_law_init = encoder_phase_cal_init,
        .control_law_finish = encoder_phase_cal_finish
};

#define ENC_VAL_POINTS_LEN 12
#define ENC_VAL_POINTS_START -M_PI_f32
#define ENC_VAL_POINTS_END M_PI_f32
#define ENC_VAL_LOOPS_PER_POINT 32000

static volatile f32 encoder_validation_points[ENC_VAL_POINTS_LEN + 1];
static volatile u8 current_point = 0;
static volatile u32 point_loop_start = 0;

typedef struct val_datapoint {
    f32 motor_phase;
    f32 encoder_phase;
    f32 encoder_pos;
} val_datapoint_t;

static volatile val_datapoint_t datapoints[ENC_VAL_POINTS_LEN];

void encoder_validation_init() {
    //encoder_init();
    open_loop_ctrl_law.control_law_init();
    for (u8 i = 0; i < ENC_VAL_POINTS_LEN; i++) {
        encoder_validation_points[i] = ENC_VAL_POINTS_START + (f32) i * (ENC_VAL_POINTS_END - ENC_VAL_POINTS_START) /
                                                              ((f32) ENC_VAL_POINTS_LEN);
        datapoints[i] = (val_datapoint_t) {0.f, 0.f, 0.f};
    }
    encoder_validation_points[ENC_VAL_POINTS_LEN] = encoder_validation_points[ENC_VAL_POINTS_LEN -
                                                                              1]; // handle the few cycles before fsm catches we're done
    current_point = 0;
    point_loop_start = controller_get_loop_count();
}


__ccmram void encoder_validation_exec_cb() {

    if (controller_get_loop_count() - point_loop_start > ENC_VAL_LOOPS_PER_POINT) {
        datapoints[current_point] = (val_datapoint_t) {.motor_phase=encoder_validation_points[current_point],
                .encoder_phase = encoder_get_phase().pos,
                .encoder_pos = encoder_get_pos_vel().pos};
        current_point++;
        point_loop_start = controller_get_loop_count();
    }

    if (current_point >= ENC_VAL_POINTS_LEN) return;

    dq_vec_t dq_lockin = {.d = LOCKIN_SPIN_CURRENT, .q = 0.f};
    ab_vec_t ab_lockin = inv_park_transform(dq_lockin, encoder_validation_points[current_point]);

    set_open_loop_current_setpoint(ab_lockin);
}

__ccmram bool encoder_validation_is_complete() {
    return current_point >= ENC_VAL_POINTS_LEN;
}

bool encoder_validation_finish(bool was_cancelled) {
    open_loop_ctrl_law.control_law_finish(was_cancelled);
    if (was_cancelled) return false;
    for (u8 i = 0; i < ENC_VAL_POINTS_LEN; i++) {
        char buf[128];
        //u32 len = snprintf(buf, 64, "ENCODER CALIB DIRECTION: %.3f\r\n", enc_distance > 0 ? 1.f : -1.f);
        u32 len = snprintf(buf,128,"%d\tmotor phase: %.3f\tencoder phase: %.3f\tencoder pos: %.3f\r\n",
               i, datapoints[i].motor_phase, datapoints[i].encoder_phase, datapoints[i].encoder_pos
        );
        uart_transmit_buf(COMMS_UART_HANDLE, buf, len);
        uart_transmit_buf(DEBUG_UART_HANDLE, buf, len);
    }
    return true;
}

control_law_t encoder_validation_ctrl_law = {
        .encoder_consumer = encoder_cal_encoder_cb,
        .current_consumer = encoder_cal_current_cb,
        .pwm_producer = encoder_cal_pwm_cb,
        .control_law_exec = encoder_validation_exec_cb,
        .is_complete = encoder_validation_is_complete,
        .control_law_init = encoder_validation_init,
        .control_law_finish = encoder_validation_finish
};


static volatile u32 shitty_cal_start;
static const u32 shitty_cal_n_loops = (u32) (4 * LOOP_UPDATE_FREQ);
static volatile f32 enc_val_cal = 0.f;

void encoder_single_point_cal_init() {
    shitty_cal_start = controller_get_loop_count();
    encoder_init();
    //encoder_set_phase_offset(0.f);
    open_loop_ctrl_law.control_law_init();

    dq_vec_t dq_lockin = {.d = LOCKIN_SPIN_CURRENT, .q = 0.f};
    ab_vec_t ab_lockin = inv_park_transform(dq_lockin, -M_PI_f32 / 2.f);
    ab_lockin = inv_park_transform(dq_lockin, 0.);

    set_open_loop_current_setpoint(ab_lockin);
}

__ccmram void encoder_single_point_cal_exec_cb() {
    if (controller_get_loop_count() - shitty_cal_start > shitty_cal_n_loops) return;
    enc_val_cal = encoder_get_pos_vel().pos;
    //enc_val_cal = encoder_get_phase().pos;
}

__ccmram bool encoder_single_point_cal_is_complete() {
    return (controller_get_loop_count() - shitty_cal_start) > shitty_cal_n_loops;
}

bool encoder_single_point_cal_finish(bool was_cancelled) {
    open_loop_ctrl_law.control_law_finish(was_cancelled);
    if (was_cancelled) return false;
    encoder_set_phase_offset(enc_val_cal);
    printf("Fuck you: %.3f\r\n", enc_val_cal);
    return true;
}


control_law_t encoder_single_point_cal_ctrl_law = {
        .encoder_consumer = encoder_cal_encoder_cb,
        .current_consumer = encoder_cal_current_cb,
        .pwm_producer = encoder_cal_pwm_cb,
        .control_law_exec = encoder_single_point_cal_exec_cb,
        .is_complete = encoder_single_point_cal_is_complete,
        .control_law_init = encoder_single_point_cal_init,
        .control_law_finish = encoder_single_point_cal_finish
};
