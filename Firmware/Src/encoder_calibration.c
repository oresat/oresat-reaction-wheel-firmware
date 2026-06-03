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

bool encoder_cal_encoder_cb(u16 angle) {
    encoder_update(angle);
    return true;
}

// Passthrough to open loop control law
bool encoder_cal_current_cb(f32 A_i, f32 B_i, f32 C_i, f32 Vbus) {
    return open_loop_ctrl_law.current_consumer(A_i, B_i, C_i, Vbus);
}

bool encoder_cal_pwm_cb(svm_vec_t *mag) {
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

void encoder_dir_cal_exec_cb() {
    u32 n_loops = controller_get_loop_count() - dir_cal_n_loop_start;

    if (n_loops == dir_cal_n_loop_lockin)
        dir_cal_enc_pos_start = encoder_get_multiturn_pos_raw(); // Capture initial position

    f32 percent_complete = ((f32) (n_loops - dir_cal_n_loop_lockin) / (f32) dir_cal_n_loops_total);
    f32 phase_pos = n_loops < dir_cal_n_loop_lockin
                    ? 0.f
                    : MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32 *
                      percent_complete * ENCODER_DIR_CAL_ROTATION_DIST;

    f32 lockin_mod = (n_loops < dir_cal_n_loop_lockin) ? clamp_f32(0.f, 2 * n_loops / dir_cal_n_loop_lockin, 1.f) : 1.f;

    dq_vec_t dq_lockin = {.d = lockin_mod * LOCKIN_SPIN_CURRENT, .q = 0.f};
    ab_vec_t ab_lockin = inv_park_transform(dq_lockin, phase_pos);

    if (controller_get_loop_count() >= (dir_cal_n_loop_end - 3))
        __asm__("nop");

    set_open_loop_current_setpoint(ab_lockin);
}

bool encoder_dir_cal_is_complete() {
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

static const u32 phase_cal_n_loops_spin = (u32) (2.f *
                                                 (ENCODER_PHASE_CAL_ROTATION_DIST / ENCODER_PHASE_CAL_ROTATION_VEL) *
                                                 LOOP_UPDATE_FREQ);

static const u32 phase_cal_n_loop_lockin = (u32) (1.f * LOOP_UPDATE_FREQ); // Lockin start direction for one second

static volatile f32 error_accum = 0.f;
static volatile u32 error_accum_count = 0;

static volatile f32 enc_pos_start = 0.f;
static volatile f32 enc_pos_end = 0.f;

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

    error_accum = 0.f;
    error_accum_count = 0;

    //erad_start = wrap_pm_pi(0 - erad_total / 2);
}

const u32 capture_data_every = (u32) (LOOP_UPDATE_FREQ / 1000); // 1ms sampling
volatile static u32 n = 0;


void encoder_phase_cal_exec_cb() {
    u32 n_loops = controller_get_loop_count() - phase_cal_n_loop_start;

    f32 enc_pos = encoder_get_multiturn_pos_raw();

    f32 phase_pos;


    if (n_loops == phase_cal_n_loop_lockin) {
        enc_pos_start = enc_pos;
        encoder_set_phase_offset(encoder_get_pos_vel().pos);
    }

    if (n_loops == phase_cal_n_loops_spin + phase_cal_n_loop_lockin)
        enc_pos_end = enc_pos;

    if (n_loops < phase_cal_n_loop_lockin)
        phase_pos = erad_start;

    else {
        u32 n_loops_spin_phase = n_loops - phase_cal_n_loop_lockin;

        f32 percent_complete = (f32) n_loops_spin_phase / (f32) phase_cal_n_loops_spin;
        f32 travel_dist;

        if (percent_complete < 0.5f)
            travel_dist = 2 * percent_complete;
        else
            travel_dist = 2 * (1.f - percent_complete);

        f32 current_pos_mechanical = travel_dist * ENCODER_PHASE_CAL_ROTATION_DIST;
        f32 current_pos_electrical = current_pos_mechanical * MOTOR_NUM_POLEPAIRS * 2.f * M_PI_f32;
        phase_pos = wrap_pm_pi(current_pos_electrical);

        if ((++n) == capture_data_every) {
            f32 enc_phase = encoder_get_phase().pos;
            f32 elec_phase = phase_pos;
            f32 error = wrap_pm_pi(elec_phase - enc_phase);
            error_accum += error;
            error_accum_count++;
            n = 0;
        }
    }

    f32 lockin_mod = (n_loops < phase_cal_n_loop_lockin)
                     ? clamp_f32(0.f, 2 * n_loops / phase_cal_n_loop_lockin, 1.f)
                     : 1.f;

    dq_vec_t dq_lockin = {.d = lockin_mod * LOCKIN_SPIN_CURRENT, .q = 0.f};
    ab_vec_t ab_lockin = inv_park_transform(dq_lockin, phase_pos);

    set_open_loop_current_setpoint(ab_lockin);
}

bool encoder_phase_cal_is_complete() {
    return controller_get_loop_count() >= phase_cal_n_loop_end;
}

// Not guarenteed to be called loop-synchronously with exec/is_complete/etc
bool encoder_phase_cal_finish(bool was_cancelled) {
    open_loop_ctrl_law.control_law_finish(was_cancelled);
    if (was_cancelled) return false;

    f32 phase_offset = error_accum / (f32) error_accum_count;


    f32 phase_offset_mechanical = (phase_offset / encoder_get_direction()) / (2.f * M_PI_f32 * MOTOR_NUM_POLEPAIRS);
    printf("Fine-tuned phase offset: %.5f rad electrical, %.5f rad mechanical\r\n", phase_offset,
           phase_offset_mechanical);
    //encoder_set_phase_offset(encoder_get_phase_offset() - phase_offset_mechanical);
    //printf("Accum: %.3f, Phase offset: %.3f\r\n", enc_pos_accum, phase_offset);

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


void encoder_validation_exec_cb() {

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

bool encoder_validation_is_complete() {
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

void encoder_single_point_cal_exec_cb() {
    if (controller_get_loop_count() - shitty_cal_start > shitty_cal_n_loops) return;
    enc_val_cal = encoder_get_pos_vel().pos;
    //enc_val_cal = encoder_get_phase().pos;
}

bool encoder_single_point_cal_is_complete() {
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
