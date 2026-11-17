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
    FSM_STATE_BACKEMF_CAL = 17,
    // Cal-store "RPC" states (CLEAR_ERRORS pattern: no control law, action fires on the exit
    // transition, auto-returns to IDLE; result lands in procedure_result via last_state_exit_ok)
    FSM_STATE_SAVE_CALIBRATION = 18,
    FSM_STATE_ERASE_CALIBRATION = 19,
    FSM_STATE_RESTORE_CALIBRATION = 20,
    FSM_STATE_BACKEMF_CAL_BIDIR = 21, // fwd + reverse coast capture in one run (gold-standard fit)
    FSM_STATE_SELF_TEST = 22, // desk bring-up self-test; report on the debug UART (self_test.h)
} fsm_state_t;

typedef enum fsm_status {
    FSM_STATUS_BUSY = 0,
    FSM_STATUS_OK = 1,
    FSM_STATUS_INVALID_STATE_TRANSITION = 2,
} fsm_status_t;

void fsm_init();

void fsm_request_state(fsm_state_t new_state);

// True for states whose entry arms closed-loop FOC off the encoder dir/offset — they refuse
// to enter (and SDO ingestion refuses to request) without valid cal. See encoder_cal_valid().
bool fsm_state_requires_encoder_cal(fsm_state_t state);

fsm_state_t fsm_get_current_state();

fsm_status_t fsm_get_current_status();

void fsm_exec();

bool fsm_get_last_state_exit_ok();

// Executes a parked cal-store save/erase/restore (states 18/19/20). Call from the foreground
// loop — the ops block on flash for tens of ms and must not run in ISR context.
void fsm_run_pending_calstore_op();

// Runs a parked self-test report (state 22) and latches its result into procedure_result.
// Foreground only: it prints synchronously over the debug UART and reads the encoder over SPI.
void fsm_run_pending_self_test();

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
        tmp(FSM_STATE_VEL_RAMP_CONTROL),
        tmp(FSM_STATE_BACKEMF_CAL),
        tmp(FSM_STATE_SAVE_CALIBRATION),
        tmp(FSM_STATE_ERASE_CALIBRATION),
        tmp(FSM_STATE_RESTORE_CALIBRATION),
        tmp(FSM_STATE_BACKEMF_CAL_BIDIR),
        tmp(FSM_STATE_SELF_TEST)
};
#undef tmp

// Bounds-checked name lookup — the raw array indexed with a host-supplied state value reads
// past the table (host can write any u8 to OD 0x4004).
static inline const char* fsm_state_name(fsm_state_t state) {
    u32 i = (u32) state;
    if (i >= sizeof(state_names) / sizeof(state_names[0]))
        return "FSM_STATE_OUT_OF_RANGE";
    return state_names[i].string;
}
