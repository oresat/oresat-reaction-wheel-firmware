# Control-law / FSM-state cheat sheet

Per-state reference for the FSM (`Firmware/Inc/fsm.h`). Each non-trivial state installs a
`control_law_t` (the fast-loop plug-in; see `CLAUDE.md` → "control-law plug-in contract") via
`get_ctrl_law_for_state()` in `fsm.c`, arms the controller, and runs at the 16 kHz HRTIM tick.

How a state runs, in one breath: host writes `0x4004` → `process_sdo_vals()` calls
`fsm_request_state()` → FSM tick (`fsm_exec`, 1 kHz) installs the law + `controller_arm()` →
fast loop calls the law's `control_law_init` once, then `encoder_consumer` / `current_consumer` /
`control_law_exec` / `pwm_producer` every tick → when `is_complete()` returns true (or any error
is raised) the controller disarms and the FSM returns to `IDLE`, calling `control_law_finish(cancelled)`.
`fsm_last_state_exit_ok` (the bool returned by `finish`) is published as `procedure_result` in TPDO1.

Setpoints arrive via `0x4005` (`setpoint_1`/`setpoint_2`). **`process_sdo_vals()` routes
`setpoint_1` by the *current* FSM state** — so you must request the state first, then write the
setpoint. `setpoint_2`, `reserved_1`, `reserved_2` are read but currently unused (no-ops).

## Summary

| State (enum) | # | Control law | `setpoint_1` | Completes? | Result / output |
|---|---|---|---|---|---|
| `FSM_STATE_IDLE` | 1 | none | — | — | TPDO rate drops to 5 s |
| `FSM_STATE_TORQUE_CONTROL` | 4 | `torque_ctrl_control_law` | torque [N·m] | never | holds torque |
| `FSM_STATE_VEL_CONTROL` | 5 | `vel_ctrl_control_law` | velocity [rev/s] | never | holds velocity (PI) |
| `FSM_STATE_POS_CONTROL` | 6 | `pos_ctrl_control_law` | (pos) | — | **STUB — faults out** |
| `FSM_STATE_MOTOR_RESISTANCE_CAL` | 7 | `resistance_cal_ctrl_law` | — | after `RESISTANCE_CAL_TIME` (3 s) | prints R (mΩ) |
| `FSM_STATE_MOTOR_INDUCTANCE_CAL` | 8 | `inductance_cal_ctrl_law` | — | after ~3 s (see note) | prints L (µH) |
| `FSM_STATE_ENCODER_DIR_CAL` | 9 | `encoder_dir_cal_ctrl_law` | — | after 1 rev + 1 s lock-in | sets encoder direction |
| `FSM_STATE_ENCODER_OFFSET_CAL` | 10 | `encoder_phase_cal_ctrl_law` | — | after 4-rev up/down sweep | coarse offset only (see note) |
| `FSM_STATE_ENCODER_TEST` | 11 | `encoder_test_control_law` | — | never | runs estimator, PWM = 0 |
| `FSM_STATE_OPEN_LOOP_CONTROL` | 12 | `open_loop_ctrl_law` | — (not on CAN) | never | α/β current via code only |
| `FSM_STATE_CLEAR_ERRORS` | 13 | none | — | immediate | clears `controller_errors` |
| `FSM_STATE_ENCODER_VALIDATION` | 14 | `encoder_validation_ctrl_law` | — | after 12 points × 2 s | dumps table over UART1+UART2 |
| `FSM_STATE_SHITTY_OFFSET_CAL` | 15 | `encoder_single_point_cal_ctrl_law` | — | after 4 s lock-in | **applies** single-point offset |
| `FSM_STATE_VEL_RAMP_CONTROL` | 16 | `vel_ramp_ctrl_control_law` | target velocity [rev/s] | never | velocity, slew-limited |
| `FSM_STATE_BACKEMF_CAL` | 17 | `backemf_cal_ctrl_law` | — (target in config.h) | spin+settle+coast (or hand-spin timeout) | logs coast capture for offline offset fit; see `backemf-coastdown-implementation.md` |
| `FSM_STATE_SAVE_CALIBRATION` | 18 | none (action on exit) | — | immediate | live offset/dir → flash cal page; result → `procedure_result` |
| `FSM_STATE_ERASE_CALIBRATION` | 19 | none (action on exit) | — | immediate | erases the cal page |
| `FSM_STATE_RESTORE_CALIBRATION` | 20 | none (action on exit) | — | immediate | flash cal page → live offset/dir (also auto-runs at boot) |
| `FSM_STATE_BACKEMF_CAL_BIDIR` | 21 | `backemf_cal_bidir_ctrl_law` | — (target in config.h) | fwd leg → drain → reverse leg | one-run fwd+rev coast capture — the gold-standard dataset for the offset fit |
| `FSM_STATE_NONE`/`SYSTEM_ERROR`/`CONTROLLER_ERROR` | 0/2/3 | none | — | — | error/sentinel states |

Closed-loop laws (torque/vel/pos/vel-ramp) delegate their `encoder_consumer`/`current_consumer`/
`pwm_producer` to `foc_control_law` (`foc.c`); only their `exec` differs.

## Per-state detail & gotchas

**Torque (4).** `setpoint_1` is torque in N·m. `set_torque_setpoint()` multiplies by encoder
direction, divides by `MOTOR_TORQUE_CONSTANT`, and clamps the resulting `iq` to
±`MOTOR_CURRENT_CMD_LIMIT` (5 A). `id` setpoint is forced to 0.

**Velocity (5).** `setpoint_1` is rev/s. PI on velocity error (`VEL_GAIN`, `VEL_INTEGRATOR_GAIN`,
integrator clamped to `VEL_INTEGRATOR_LIMIT`) produces a torque command fed to the torque path.

**Velocity-ramp (16).** Like velocity, but `setpoint_1` is the *target* and the internal velocity
setpoint slews toward it at `VEL_RAMP_RATE` (200 rev/s²) per loop. Note: in `process_sdo_vals` the
`VEL_RAMP_CONTROL` case is missing a `break;` before `default` — harmless today (default is empty)
but fragile.

**Position (6).** `pos_ctrl_control_law` is a file-scope struct that is never initialized
(`closed_loop_controller.c:115`) → all function pointers NULL, and `set_pos_setpoint()` is empty.
Requesting this state arms a law that produces no PWM, so the fast loop raises
`CTRLR_ERR_PWM_TIMING_UPDATE_MISSING` and disarms almost immediately. **Treat as not implemented.**

**Resistance cal (7).** Locks to +α, ramps voltage via a slow integrator until phase current hits
`MOTOR_CALIBRATION_CURRENT` (2 A); after `RESISTANCE_CAL_TIME` reports `R = V/I`. Bounds-checked
against `MOTOR_PHASE_RESISTANCE_{MIN,MAX}`; out-of-range or overcurrent → error + disarm.

**Inductance cal (8).** Injects an alternating ±voltage on α and measures the current slope to get
L. **Gotcha:** its duration uses `RESISTANCE_CAL_TIME` (3 s), not `INDUCTANCE_CAL_TIME` (1 s) —
`inductance_cal_length_cycles` is derived from the wrong constant (`motor_calibration.c:153`).

**Both R/L cals are diagnostic only.** Results are stored in module statics and printed, but the
FOC current controller uses `HARDCODE_PHASE_RESISTANCE`/`HARDCODE_PHASE_INDUCTANCE` from `config.h`
(`init_foc()` ignores the measured values). Calibrating does not change closed-loop behavior.

**Encoder dir cal (9).** 1 s lock-in, then open-loop spins +1 rev at 0.5 rev/s. On finish, checks
the encoder moved `ENCODER_DIR_CAL_ROTATION_DIST` ±10 %; sets encoder direction ±1 from the sign.
Out-of-tolerance → `finish` returns false (procedure_result = 0) but no error flag.

**Encoder offset/phase cal (10).** Spins 4 rev up-and-down, accumulating electrical-vs-encoder phase
error. Sets a **coarse** offset at lock-in (`encoder_set_phase_offset(pos)` during exec). The
**fine-tuned** correction is computed and printed but **not applied** — the line that would apply it
is commented out (`encoder_calibration.c:245`). So this state currently only does the coarse set.

**Single-point "shitty" offset cal (15).** Locks at electrical angle 0 for 4 s, then sets the phase
offset to the measured position. Unlike state 10, this one **does** apply the offset. This is the
practical offset-cal path today.

**Encoder validation (14).** Steps the open-loop vector through 12 angles from −π to π, dwelling
~2 s each (`ENC_VAL_LOOPS_PER_POINT = 32000` ticks), recording motor-vs-encoder phase, then dumps a
table over both `COMMS_UART_HANDLE` (USART1) and `DEBUG_UART_HANDLE` (USART2). Diagnostic.

**Open-loop (12).** Pure α/β current PI with no encoder feedback. **Its setpoint is not wired to
CAN** — only `set_open_loop_current_setpoint()` from code sets it, so over CAN this state just holds
zero current. Used as the primitive that the encoder-cal laws drive.

**Encoder test (11).** Runs the estimator on every tick with zero PWM output. Never completes — use
to watch the estimator without energizing the motor.

**Clear errors (13).** The only way to reset `controller_errors`. On *exit* it calls
`controller_clear_errors()`, then the FSM auto-requests `IDLE`. Any nonzero error elsewhere forces
`FSM_STATE_CONTROLLER_ERROR` until you pass through here.

## Telemetry while running

Entering any control-law state bumps all four TPDOs to a 100 ms event timer; `IDLE` drops them to
5 s. Motor/bus fields in TPDO2/TPDO3 are only populated in the four closed-loop control states
(torque/vel/pos/vel-ramp); in calibration/test states they read 0. Temperatures (TPDO4) are always
live. See `docs/can-interface.md` for the wire format.
