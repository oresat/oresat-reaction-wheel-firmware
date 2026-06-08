#include "fsm.h"

#include <stdlib.h>

#include "can_comms.h"
#include "closed_loop_controller.h"
#include "control_ll.h"
#include "encoder.h"
#include "encoder_calibration.h"
#include "foc_capture.h"
#include "motor_calibration.h"
#include "open_loop_current_controller.h"
#include "printf.h"

static volatile fsm_state_t fsm_state;
static volatile fsm_status_t fsm_status;

static volatile fsm_state_t fsm_requested_state;

void fsm_exit_state(fsm_state_t exit_state, bool cancelled);

void fsm_enter_state(fsm_state_t enter_state);

static volatile bool fsm_last_state_exit_ok = false;

void fsm_init() {
    fsm_state = FSM_STATE_IDLE;
    fsm_requested_state = FSM_STATE_NONE;
    fsm_status = FSM_STATUS_OK;
}

void fsm_request_state(fsm_state_t new_state) {
    fsm_requested_state = new_state;
}

fsm_state_t fsm_get_current_state() {
    return fsm_state;
}

fsm_status_t fsm_get_current_status() {
    return fsm_status;
}

static __fn_inline control_law_t* get_ctrl_law_for_state(fsm_state_t state) {
    switch (state) {
        default:
        case FSM_STATE_CLEAR_ERRORS:
        case FSM_STATE_NONE:
        case FSM_STATE_IDLE:
        case FSM_STATE_SYSTEM_ERROR:
        case FSM_STATE_CONTROLLER_ERROR:
            return NULL;
        case FSM_STATE_TORQUE_CONTROL:
            return &torque_ctrl_control_law;
        case FSM_STATE_VEL_CONTROL:
            return &vel_ctrl_control_law;
        case FSM_STATE_POS_CONTROL:
            return &pos_ctrl_control_law;
        case FSM_STATE_MOTOR_RESISTANCE_CAL:
            return &resistance_cal_ctrl_law;
        case FSM_STATE_MOTOR_INDUCTANCE_CAL:
            return &inductance_cal_ctrl_law;
        case FSM_STATE_ENCODER_DIR_CAL:
            return &encoder_dir_cal_ctrl_law;
        case FSM_STATE_ENCODER_OFFSET_CAL:
            return &encoder_phase_cal_ctrl_law;
        case FSM_STATE_ENCODER_TEST:
            return &encoder_test_control_law;
        case FSM_STATE_OPEN_LOOP_CONTROL:
            return &open_loop_ctrl_law;
        case FSM_STATE_ENCODER_VALIDATION:
            return &encoder_validation_ctrl_law;
        case FSM_STATE_SHITTY_OFFSET_CAL:
            return &encoder_single_point_cal_ctrl_law;
        case FSM_STATE_VEL_RAMP_CONTROL:
            return &vel_ramp_ctrl_control_law;
    }
}


// TODO cache volatiles
void fsm_exec() {

    // Check to see if we've errored out
    if (controller_get_errors() != CTRLR_ERR_NONE && fsm_state != FSM_STATE_CONTROLLER_ERROR && fsm_requested_state != FSM_STATE_CLEAR_ERRORS) {
        fsm_exit_state(fsm_state, true);
        //fsm_state = FSM_STATE_CONTROLLER_ERROR;
        fsm_enter_state(FSM_STATE_CONTROLLER_ERROR);
    }

    // Check to see if our running control law has finished, and we need to exit the state
    else if (fsm_state != FSM_STATE_IDLE && !controller_is_armed() && controller_is_control_law_done()) {
        fsm_exit_state(fsm_state, false);
        //fsm_state = FSM_STATE_IDLE;
        fsm_enter_state(FSM_STATE_IDLE);
    }

    // If we're going from IDLE to a new state
    else if (fsm_requested_state != FSM_STATE_NONE && fsm_state == FSM_STATE_IDLE) {
        fsm_exit_state(fsm_state, false);

        fsm_enter_state(fsm_requested_state);
        //fsm_state = fsm_requested_state;

        fsm_requested_state = FSM_STATE_NONE;
    }

    // If we're trying to move from a non-idle state to a non-idle state
    else if (fsm_requested_state != FSM_STATE_NONE && fsm_state != FSM_STATE_IDLE) {
        fsm_exit_state(fsm_state, true);
        //fsm_state = FSM_STATE_IDLE; // We'll enter the new one on the next FSM loop
        fsm_enter_state(FSM_STATE_IDLE); // We'll enter the new one on the next FSM loop
    }

    // If we're trying to clear errors
    else if (fsm_requested_state == FSM_STATE_NONE && fsm_state == FSM_STATE_CLEAR_ERRORS) {
        fsm_requested_state = FSM_STATE_IDLE;
    }
}

void fsm_exit_state(fsm_state_t exit_state, bool cancelled) {
    printf("Exiting state %s, cancelled %s\r\n", state_names[exit_state].string, cancelled ? "TRUE" : "FALSE");
    switch (exit_state) {
        case FSM_STATE_CLEAR_ERRORS:
            controller_clear_errors();
        case FSM_STATE_NONE:
        case FSM_STATE_IDLE:
        case FSM_STATE_SYSTEM_ERROR:
        case FSM_STATE_CONTROLLER_ERROR:
            fsm_last_state_exit_ok = true;
            break;
        case FSM_STATE_TORQUE_CONTROL:
        case FSM_STATE_VEL_CONTROL:
        case FSM_STATE_POS_CONTROL:
        case FSM_STATE_MOTOR_RESISTANCE_CAL:
        case FSM_STATE_MOTOR_INDUCTANCE_CAL:
        case FSM_STATE_ENCODER_DIR_CAL:
        case FSM_STATE_ENCODER_OFFSET_CAL:
        case FSM_STATE_ENCODER_TEST:
        case FSM_STATE_OPEN_LOOP_CONTROL:
        case FSM_STATE_ENCODER_VALIDATION:
        case FSM_STATE_SHITTY_OFFSET_CAL:
        case FSM_STATE_VEL_RAMP_CONTROL:
            controller_disarm(); // Should be handled in the controller but we do it here anyways
            foc_capture_stop();  // freeze ring buffer for post-run readout
            fsm_last_state_exit_ok = controller_get_control_law()->control_law_finish(cancelled);
            break;
        default: ;
    }
}

void fsm_enter_state(fsm_state_t enter_state) {
    printf("Entering state %s\r\n", state_names[enter_state].string);
    control_law_t* ctrl_law = get_ctrl_law_for_state(enter_state);

    if (ctrl_law != NULL) {
        set_tpdo_send_rate(100);
        controller_set_control_law(ctrl_law);
        controller_arm();
    } else {
        set_tpdo_send_rate(5000);
    }

    fsm_state = enter_state;
}

bool fsm_get_last_state_exit_ok() {
    return fsm_last_state_exit_ok;
}
