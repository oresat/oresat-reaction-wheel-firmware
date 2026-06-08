#pragma once

#include "typedefs.h"

typedef enum fsm_state {
    FSM_STATE_NONE = 0,
    FSM_STATE_IDLE = 1,
    FSM_STATE_SYSTEM_ERROR = 2,
    FSM_STATE_CONTROLLER_ERROR = 3,
    FSM_STATE_TORQUE_CONTROL = 4,
    FSM_STATE_VEL_CONTROL = 5,
    FSM_STATE_POS_CONTROL = 6,
    FSM_STATE_MOTOR_RESISTANCE_CAL = 7,
    FSM_STATE_MOTOR_INDUCTANCE_CAL = 8,
    FSM_STATE_ENCODER_DIR_CAL = 9,
    FSM_STATE_ENCODER_OFFSET_CAL = 10,
    FSM_STATE_ENCODER_TEST = 11,
    FSM_STATE_OPEN_LOOP_CONTROL = 12,
    FSM_STATE_CLEAR_ERRORS = 13,
    FSM_STATE_ENCODER_VALIDATION = 14,
    FSM_STATE_SHITTY_OFFSET_CAL = 15,
    FSM_STATE_VEL_RAMP_CONTROL=16,
} fsm_state_t;

typedef enum fsm_status {
    FSM_STATUS_BUSY = 0,
    FSM_STATUS_OK = 1,
    FSM_STATUS_INVALID_STATE_TRANSITION = 2,
} fsm_status_t;

void fsm_init();

void fsm_request_state(fsm_state_t new_state);

fsm_state_t fsm_get_current_state();

fsm_status_t fsm_get_current_status();

void fsm_exec();

bool fsm_get_last_state_exit_ok();

typedef struct state_name {
    const char string[32];
} state_name_t;

#define tmp(enum_name) ((state_name_t) {#enum_name})
static const state_name_t state_names[] = {
        tmp(FSM_STATE_NONE),
        tmp(FSM_STATE_IDLE),
        tmp(FSM_STATE_SYSTEM_ERROR),
        tmp(FSM_STATE_CONTROLLER_ERROR),
        tmp(FSM_STATE_TORQUE_CONTROL),
        tmp(FSM_STATE_VEL_CONTROL),
        tmp(FSM_STATE_POS_CONTROL),
        tmp(FSM_STATE_MOTOR_RESISTANCE_CAL),
        tmp(FSM_STATE_MOTOR_INDUCTANCE_CAL),
        tmp(FSM_STATE_ENCODER_DIR_CAL),
        tmp(FSM_STATE_ENCODER_OFFSET_CAL),
        tmp(FSM_STATE_ENCODER_TEST),
        tmp(FSM_STATE_OPEN_LOOP_CONTROL),
        tmp(FSM_STATE_CLEAR_ERRORS),
        tmp(FSM_STATE_ENCODER_VALIDATION),
        tmp(FSM_STATE_SHITTY_OFFSET_CAL),
        tmp(FSM_STATE_VEL_RAMP_CONTROL)
};
#undef tmp
