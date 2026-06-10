#include "closed_loop_controller.h"

#include "encoder.h"
#include "foc.h"

static volatile f32 torque_setpoint = 0.f;
static volatile f32 vel_setpoint = 0.f;


static volatile f32 vel_integrator = 0;

__ccmram bool closed_loop_encoder_consumer(u16 meas) {
    return foc_control_law.encoder_consumer(meas);
}

__ccmram bool closed_loop_current_consumer(f32 A_i, f32 B_i, f32 C_i, f32 V_vbus) {
    return foc_control_law.current_consumer(A_i, B_i, C_i, V_vbus);
}

__ccmram bool closed_loop_pwm_producer(svm_vec_t *mag) {
    return foc_control_law.pwm_producer(mag);
}

__ccmram void torque_ctrl_exec() {
}

__ccmram bool torque_ctrl_is_complete() {
    return false;
}

void torque_ctrl_init() {
    init_foc();
    torque_setpoint = 0.f;
}

bool torque_ctrl_finish(bool was_cancelled) {
    return true;
}

__ccmram void set_torque_setpoint(f32 torque_nm) {
    torque_setpoint = torque_nm * encoder_get_direction();
    f32 current_setpoint = torque_setpoint / MOTOR_TORQUE_CONSTANT;
    set_iq_setpoint(
            clamp_f32(
                    -MOTOR_CURRENT_CMD_LIMIT,
                    current_setpoint,
                    MOTOR_CURRENT_CMD_LIMIT
            )
    );
}


void set_pos_setpoint(f32 pos) {
}


control_law_t torque_ctrl_control_law = {
        .encoder_consumer = closed_loop_encoder_consumer,
        .current_consumer = closed_loop_current_consumer,
        .pwm_producer = closed_loop_pwm_producer,
        .control_law_exec = torque_ctrl_exec,
        .is_complete = torque_ctrl_is_complete,
        .control_law_init = torque_ctrl_init,
        .control_law_finish = torque_ctrl_finish
};

void vel_ctrl_init() {
    torque_ctrl_init();
    vel_integrator = 0;
    vel_setpoint = 0;
}

__ccmram void vel_ctrl_exec() {
    f32 torque = 0;

    f32 vel_est = encoder_get_pos_vel().vel;

    f32 vel_err = vel_setpoint - vel_est;

    f32 vel_integrator_ = vel_integrator;

    vel_integrator_ += VEL_INTEGRATOR_GAIN * SECONDS_PER_LOOP * vel_err;
    vel_integrator_ = clamp_f32(-VEL_INTEGRATOR_LIMIT,vel_integrator_,VEL_INTEGRATOR_LIMIT);

    torque += vel_err * VEL_GAIN;
    torque += vel_integrator_;

    vel_integrator = vel_integrator_;

    set_torque_setpoint(torque);
}

__ccmram bool vel_ctrl_is_complete() {
    return false;
}

__ccmram void set_vel_setpoint(f32 vel) {
    vel_setpoint = vel;
}

bool vel_ctrl_finish(bool was_cancelled) {
    return true;
}

control_law_t vel_ctrl_control_law = {
        .encoder_consumer = closed_loop_encoder_consumer,
        .current_consumer = closed_loop_current_consumer,
        .pwm_producer = closed_loop_pwm_producer,
        .control_law_init = vel_ctrl_init,
        .control_law_exec = vel_ctrl_exec,
        .is_complete = vel_ctrl_is_complete,
        .control_law_finish = vel_ctrl_finish
};

static volatile f32 ramped_vel_setpoint = 0.f;

void vel_ramp_ctrl_init() {
    vel_ctrl_init();
    set_vel_setpoint(0.f);
    ramped_vel_setpoint = 0.f;
}

void set_vel_ramp_setpoint(f32 _ramped_vel_setpoint) {
    ramped_vel_setpoint = _ramped_vel_setpoint;
}

const f32 MAX_VEL_DELTA_PER_LOOP = VEL_RAMP_RATE*SECONDS_PER_LOOP;

__ccmram void vel_ramp_ctrl_exec() {
    f32 vel_setpoint_ = vel_setpoint;
    f32 ramped_vel_setpoint_ = ramped_vel_setpoint;
    if (vel_setpoint_ < ramped_vel_setpoint_) {
        if (ramped_vel_setpoint_ - vel_setpoint_ < MAX_VEL_DELTA_PER_LOOP)
            vel_setpoint_ = ramped_vel_setpoint_;
        else vel_setpoint_ += MAX_VEL_DELTA_PER_LOOP;
    } else if (vel_setpoint_ > ramped_vel_setpoint_) {
        if (vel_setpoint_ - ramped_vel_setpoint_ < MAX_VEL_DELTA_PER_LOOP)
            vel_setpoint_ = ramped_vel_setpoint_;
        else vel_setpoint_ -= MAX_VEL_DELTA_PER_LOOP;
    }

    set_vel_setpoint(vel_setpoint_);
    vel_setpoint = vel_setpoint_;

    vel_ctrl_exec();
}

__ccmram bool vel_ramp_ctrl_is_complete() {
    return false;
}

bool vel_ramp_ctrl_finish(bool was_cancelled) {
    vel_ctrl_finish(was_cancelled);
    return true;
}

control_law_t vel_ramp_ctrl_control_law = {
        .encoder_consumer = closed_loop_encoder_consumer,
        .current_consumer = closed_loop_current_consumer,
        .pwm_producer = closed_loop_pwm_producer,
        .control_law_init = vel_ramp_ctrl_init,
        .control_law_exec = vel_ramp_ctrl_exec,
        .is_complete = vel_ramp_ctrl_is_complete,
        .control_law_finish = vel_ramp_ctrl_finish
};