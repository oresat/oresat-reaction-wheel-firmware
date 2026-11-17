#include "fsm.h"

#include <stdlib.h>

#include "inverter.h"

#include "backemf_cal.h"
#include "cal_store.h"
#include "can_comms.h"
#include "closed_loop_controller.h"
#include "config.h"
#include "control_ll.h"
#include "encoder.h"
#include "encoder_calibration.h"
#include "foc_capture.h"
#include "motor_calibration.h"
#include "open_loop_current_controller.h"
#include "printf.h"
#include "self_test.h"

static volatile fsm_state_t fsm_state;
static volatile fsm_status_t fsm_status;

static volatile fsm_state_t fsm_requested_state;

void fsm_exit_state(fsm_state_t exit_state, bool cancelled);

void fsm_enter_state(fsm_state_t enter_state);

static volatile bool fsm_last_state_exit_ok = false;

// Cal-store RPC op parked by fsm_exit_state (TIM17 ISR), executed by
// fsm_run_pending_calstore_op() in the foreground loop.
static volatile fsm_state_t calstore_op_pending = FSM_STATE_NONE;

// SELF_TEST entered without its bridge half (boot shunt-offset cal failed, arming would be
// refused): it then behaves like an action state and auto-returns to IDLE.
static volatile bool self_test_report_only = false;

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
        case FSM_STATE_POS_CONTROL:         // stub: the law struct was all-NULL pointers — never
                                            // install/arm it (requests are rejected at ingestion)
        case FSM_STATE_SAVE_CALIBRATION:    // cal-store RPC states: no law, never armed;
        case FSM_STATE_ERASE_CALIBRATION:   // action runs in fsm_exit_state on the way back
        case FSM_STATE_RESTORE_CALIBRATION: // to IDLE (CLEAR_ERRORS pattern)
            return NULL;
        case FSM_STATE_TORQUE_CONTROL:
            return &torque_ctrl_control_law;
        case FSM_STATE_VEL_CONTROL:
            return &vel_ctrl_control_law;
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
        case FSM_STATE_BACKEMF_CAL:
            return &backemf_cal_ctrl_law;
        case FSM_STATE_BACKEMF_CAL_BIDIR:
            return &backemf_cal_bidir_ctrl_law;
        case FSM_STATE_SELF_TEST:
            return &self_test_ctrl_law;
    }
}


// States whose entry arms closed-loop FOC off the encoder dir/offset. The backemf-cal states
// spin up closed-loop bootstrapping off the EXISTING cal, so they're gated too — the runbook
// order (9 dir-cal → 15 single-point → 21 backemf) satisfies this naturally.
bool fsm_state_requires_encoder_cal(fsm_state_t state) {
    switch (state) {
        case FSM_STATE_TORQUE_CONTROL:
        case FSM_STATE_VEL_CONTROL:
        case FSM_STATE_VEL_RAMP_CONTROL:
        case FSM_STATE_BACKEMF_CAL_BIDIR:
            return true;
        case FSM_STATE_BACKEMF_CAL:
            // Hand-spin mode (target 0) floats the bridge and only logs — pre-cal bench use.
            return BACKEMF_CAL_SPIN_TARGET != 0.f;
        default:
            return false;
    }
}

// TODO cache volatiles
void fsm_exec() {

    // A request for the state we're already in is an idempotent no-op. Hosts replay commands
    // (and the OD write path now forwards every download) — don't cancel and re-init a
    // running law for a repeat of itself.
    if (fsm_requested_state != FSM_STATE_NONE && fsm_requested_state == fsm_state)
        fsm_requested_state = FSM_STATE_NONE;

    // While errors are latched, CLEAR_ERRORS is the only honorable request — DROP anything
    // else rather than leave it pending. A pending request would otherwise bounce the FSM
    // IDLE<->CONTROLLER_ERROR forever, one CANopen-visible transition per tick
    // (flight-review P1-1). Exemption: CLEAR_ERRORS itself auto-requests IDLE to finish its
    // job — don't eat that.
    if (controller_get_errors() != CTRLR_ERR_NONE &&
        fsm_state != FSM_STATE_CLEAR_ERRORS &&
        fsm_requested_state != FSM_STATE_NONE &&
        fsm_requested_state != FSM_STATE_CLEAR_ERRORS) {
        printf("FSM: dropping request %s while errors latched — send CLEAR_ERRORS first\r\n",
               fsm_state_name(fsm_requested_state));
        fsm_requested_state = FSM_STATE_NONE;
    }

    // Check to see if we've errored out. CLEAR_ERRORS is exempt both as the pending request
    // and as the CURRENT state: it clears the errors on its exit transition, and re-trapping
    // it here used to park the FSM in CONTROLLER_ERROR with errors==0 (flight-review P1-1
    // mode C — the documented auto-return-to-IDLE never ran when it mattered).
    if (controller_get_errors() != CTRLR_ERR_NONE && fsm_state != FSM_STATE_CONTROLLER_ERROR
        && fsm_state != FSM_STATE_CLEAR_ERRORS && fsm_requested_state != FSM_STATE_CLEAR_ERRORS) {
        fsm_exit_state(fsm_state, true);
        fsm_enter_state(FSM_STATE_CONTROLLER_ERROR);
    }

    // Check to see if our running control law has finished, and we need to exit the state
    else if (fsm_state != FSM_STATE_IDLE && !controller_is_armed() && controller_is_control_law_done()) {
        fsm_exit_state(fsm_state, false);
        fsm_enter_state(FSM_STATE_IDLE);
    }

    // If we're going from IDLE to a new state
    else if (fsm_requested_state != FSM_STATE_NONE && fsm_state == FSM_STATE_IDLE) {
        if (calstore_op_pending != FSM_STATE_NONE || self_test_report_pending()) {
            // A parked cal-store op hasn't executed yet (foreground, lands within tens of
            // ms): HOLD the request — do nothing this tick, re-evaluate next. Entering a law
            // state now would let a deferred RESTORE swap dir/offset/LUT under an armed
            // 32 kHz loop, and entering another cal-store state would double-park the op
            // slot (review F1/F2). A parked self-test report is held the same way: it reads
            // the encoder over SPI (needs the loop disarmed) and owns procedure_result.
        } else if (get_ctrl_law_for_state(fsm_requested_state) != NULL && !inverter_get_calibrated()
                   && fsm_requested_state != FSM_STATE_SELF_TEST) {
            // (SELF_TEST is exempt: it enters report-only and says which channel is bad.)
            // Synchronous refusal: the fast-loop CALIBRATION_INVALID check can never fire if
            // the HRTIM doesn't tick — the exact failure the boot offset-cal timeout detects
            // — so arming would otherwise sit armed-and-inert with the bridge enabled and no
            // supervision (review minor).
            printf("FSM: refusing %s — inverter offset cal invalid\r\n",
                   fsm_state_name(fsm_requested_state));
            controller_add_errors(CTRLR_ERR_INVERTER_CALIBRATION_INVALID);
            fsm_requested_state = FSM_STATE_NONE;
        } else if (fsm_state_requires_encoder_cal(fsm_requested_state) && !encoder_cal_valid()) {
            // Defense in depth: SDO ingestion already aborts these requests. Internal callers
            // land here — refuse loudly (the error bit forces a CAN-visible CONTROLLER_ERROR).
            printf("FSM: refusing %s — encoder dir/offset cal not valid\r\n",
                   fsm_state_name(fsm_requested_state));
            controller_add_errors(CTRLR_ERR_NOT_CALIBRATED);
            fsm_requested_state = FSM_STATE_NONE;
        } else {
            fsm_exit_state(fsm_state, false);
            fsm_enter_state(fsm_requested_state);
            fsm_requested_state = FSM_STATE_NONE;
        }
    }

    // If we're trying to move from a non-idle state to a non-idle state: transit IDLE; the
    // IDLE->target branch above runs on the next tick.
    else if (fsm_requested_state != FSM_STATE_NONE && fsm_state != FSM_STATE_IDLE) {
        fsm_exit_state(fsm_state, true);
        fsm_enter_state(FSM_STATE_IDLE);
        if (fsm_requested_state == FSM_STATE_IDLE)
            fsm_requested_state = FSM_STATE_NONE; // satisfied — don't bounce through IDLE twice
    }

    // Action states (clear-errors / cal-store RPCs): auto-request IDLE; the action itself fires
    // in fsm_exit_state on the resulting transition.
    else if (fsm_requested_state == FSM_STATE_NONE &&
             (fsm_state == FSM_STATE_CLEAR_ERRORS ||
              fsm_state == FSM_STATE_SAVE_CALIBRATION ||
              fsm_state == FSM_STATE_ERASE_CALIBRATION ||
              fsm_state == FSM_STATE_RESTORE_CALIBRATION ||
              (fsm_state == FSM_STATE_SELF_TEST && self_test_report_only))) {
        fsm_requested_state = FSM_STATE_IDLE;
    }
}

void fsm_exit_state(fsm_state_t exit_state, bool cancelled) {
    printf("Exiting state %s, cancelled %s\r\n", fsm_state_name(exit_state), cancelled ? "TRUE" : "FALSE");
    switch (exit_state) {
        case FSM_STATE_CLEAR_ERRORS:
            controller_clear_errors();
            break;
        case FSM_STATE_NONE:
        case FSM_STATE_IDLE:
        case FSM_STATE_SYSTEM_ERROR:
        case FSM_STATE_CONTROLLER_ERROR:
            // Transit states do NOT touch fsm_last_state_exit_ok: exiting IDLE used to force
            // it true, destroying a failed procedure's result the moment the host commanded
            // the next step (flight-review RE-2). procedure_result now latches the outcome of
            // the last PROCEDURE (law or cal-store state) until the next one runs.
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
        case FSM_STATE_BACKEMF_CAL:
        case FSM_STATE_BACKEMF_CAL_BIDIR:
            controller_disarm(); // Should be handled in the controller but we do it here anyways
            foc_capture_stop();  // freeze ring buffer for post-run readout
            {
                // NULL-guard: an unpopulated law would branch to address 0 here — HardFault →
                // IWDG reboot, remotely triggerable by a single SDO state write.
                ll_control_law_finish_t finish = controller_get_control_law()->control_law_finish;
                fsm_last_state_exit_ok = (finish != NULL) ? finish(cancelled) : false;
            }
            break;
        // Cal-store RPC actions: parked for the FOREGROUND to execute, not run here. A page
        // erase blocks ~22 ms; at TIM17 priority that masked the CAN tick AND SysTick (frozen
        // HAL_GetTick = the HAL's own flash timeout could never expire) for the whole op
        // (flight-review P2-7). Motor is guaranteed disarmed (these states never arm; armed
        // states exit through IDLE first). procedure_result reads false until the op lands a
        // few tens of ms later — hosts poll it after commanding 18/19/20.
        case FSM_STATE_SAVE_CALIBRATION:
        case FSM_STATE_ERASE_CALIBRATION:
        case FSM_STATE_RESTORE_CALIBRATION:
            if (calstore_op_pending != FSM_STATE_NONE)
                printf("FSM: cal-store op %s superseded before it ran\r\n",
                       fsm_state_name(calstore_op_pending));
            fsm_last_state_exit_ok = false; // pessimistic until the foreground op completes
            calstore_op_pending = exit_state;
            break;
        // Self-test: bridge half (if it ran) is over; the report half is parked for the
        // foreground, which latches procedure_result when it finishes.
        case FSM_STATE_SELF_TEST:
            controller_disarm(); // idempotent; the report-only path never armed
            self_test_exit(cancelled);
            fsm_last_state_exit_ok = false; // pessimistic until the report lands
            break;
        default: ;
    }
}

void fsm_enter_state(fsm_state_t enter_state) {
    printf("Entering state %s\r\n", fsm_state_name(enter_state));
    control_law_t* ctrl_law = get_ctrl_law_for_state(enter_state);

    if (enter_state == FSM_STATE_SELF_TEST) {
        self_test_report_only = !self_test_prepare(); // false: boot shunt-offset cal is bad
        if (self_test_report_only)
            ctrl_law = NULL;
    }

    if (ctrl_law != NULL) {
        set_tpdo_send_rate(100);
        controller_set_control_law(ctrl_law);
        controller_arm();
        if (enter_state == FSM_STATE_SELF_TEST)
            self_test_after_arm(); // float the bridge before the first fast-loop tick
    } else {
        // No law for this state: also drop the previous law and its latched done-flag. A stale
        // control_law_is_done (only ever cleared by installing a law) otherwise re-fires the
        // law-finished branch from CONTROLLER_ERROR/IDLE forever — the autonomous
        // IDLE<->CONTROLLER_ERROR ping-pong a failed R/L cal used to trigger (flight-review
        // P1-1 mode B).
        controller_clear_control_law();
        set_tpdo_send_rate(5000);
    }

    fsm_state = enter_state;
}

bool fsm_get_last_state_exit_ok() {
    return fsm_last_state_exit_ok;
}

// Foreground half of the cal-store RPC states (see fsm_exit_state). Blocking flash ops are
// fine here: the fast loop, FSM tick and CAN tick all preempt the foreground.
void fsm_run_pending_calstore_op() {
    fsm_state_t op = calstore_op_pending;
    if (op == FSM_STATE_NONE)
        return;

    bool ok = false;
    switch (op) {
        case FSM_STATE_SAVE_CALIBRATION:    ok = cal_store_save(); break;
        case FSM_STATE_ERASE_CALIBRATION:   ok = cal_store_erase(); break;
        case FSM_STATE_RESTORE_CALIBRATION: ok = cal_store_restore(); break;
        default: break;
    }
    fsm_last_state_exit_ok = ok;

    // Clear only if unchanged: the FSM may have parked a NEW op while we were blocked in the
    // flash driver — that one still needs to run on the next pass.
    if (calstore_op_pending == op)
        calstore_op_pending = FSM_STATE_NONE;
}

void fsm_run_pending_self_test() {
    if (!self_test_report_pending())
        return;
    // Latch the result BEFORE clearing pending: the FSM holds new requests until the clear, so
    // no later procedure can overwrite procedure_result in between.
    fsm_last_state_exit_ok = self_test_run_report();
    self_test_report_clear();
}
