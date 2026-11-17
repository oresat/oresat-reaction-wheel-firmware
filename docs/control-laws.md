# Control-law / FSM-state cheat sheet

Per-state reference for the FSM (`Firmware/Inc/fsm.h`). Each non-trivial state installs a
`control_law_t` (the fast-loop plug-in; see `CLAUDE.md` → "control-law plug-in contract") via
`get_ctrl_law_for_state()` in `fsm.c`, arms the controller, and runs at the 32 kHz HRTIM tick.

How a state runs, in one breath: host writes `0x4004` → the OD write extension (can_comms.c; validates, SDO-aborts bad values, always applies) calls
`fsm_request_state()` → FSM tick (`fsm_exec`, 1 kHz) installs the law + `controller_arm()` →
fast loop calls the law's `control_law_init` once, then `encoder_consumer` / `current_consumer` /
`control_law_exec` / `pwm_producer` every tick → when `is_complete()` returns true (or any error
is raised) the controller disarms and the FSM returns to `IDLE`, calling `control_law_finish(cancelled)`.
`fsm_last_state_exit_ok` (the bool returned by `finish`) is published as `procedure_result` in TPDO1.

Setpoints arrive via `0x4005` (`setpoint_1`/`setpoint_2`). **The OD write extension routes
`setpoint_1` by the *current* FSM state** — so you must request the state first, then write the
setpoint. `setpoint_2`, `reserved_1`, `reserved_2` are read but currently unused (no-ops).

## Summary

| State (enum) | # | Control law | `setpoint_1` | Completes? | Result / output |
|---|---|---|---|---|---|
| `FSM_STATE_IDLE` | 1 | none | — | — | TPDO rate drops to 5 s |
| `FSM_STATE_TORQUE_CONTROL` | 4 | `torque_ctrl_control_law` | torque [N·m] | never | holds torque |
| `FSM_STATE_VEL_CONTROL` | 5 | `vel_ctrl_control_law` | velocity [rev/s] | never | holds velocity (PI) |
| `FSM_STATE_POS_CONTROL` | 6 | — | (pos) | — | **STUB — request rejected at SDO ingestion (used to install an all-NULL law and HardFault on exit)** |
| `FSM_STATE_MOTOR_RESISTANCE_CAL` | 7 | `resistance_cal_ctrl_law` | — | after `RESISTANCE_CAL_TIME` (3 s) | prints R (mΩ) |
| `FSM_STATE_MOTOR_INDUCTANCE_CAL` | 8 | `inductance_cal_ctrl_law` | — | after ~3 s (see note) | prints L (µH) |
| `FSM_STATE_ENCODER_DIR_CAL` | 9 | `encoder_dir_cal_ctrl_law` | — | after 1 rev + 1 s lock-in | sets encoder direction |
| `FSM_STATE_ENCODER_OFFSET_CAL` | 10 | `encoder_phase_cal_ctrl_law` | — | after 4-rev up/down sweep | coarse seed **+ fine offset applied** (see note) |
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
| `FSM_STATE_SELF_TEST` | 22 | `self_test_ctrl_law` (+ foreground report) | — | ~0.13 s armed, then the report | per-check PASS/FAIL on the debug UART; overall → `procedure_result` (see below) |
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
setpoint slews toward it at `VEL_RAMP_RATE` (20 rev/s², config.h) per loop
(`MAX_VEL_DELTA_PER_LOOP`, `closed_loop_controller.c:129`). On (re-)arm the ramp target is seeded to
the *measured* velocity (via `vel_ctrl_init`), so it holds speed until the host writes a setpoint.

**Position (6).** Not implemented, and rejected before it can run: `fsm_state_requestable()` omits
`POS_CONTROL`, so a `0x4004` request for `6` is answered with an SDO abort (`0x0609 0030`,
`can_comms.c:315-321,331`). Even internally, `get_ctrl_law_for_state()` returns NULL for it
(`fsm.c:61`), so no law is ever installed or armed. (Historically it installed an all-NULL file-scope
`pos_ctrl_control_law` struct that produced no PWM and HardFaulted on exit — that struct and
`set_pos_setpoint()` are now gone.) **Treat as not implemented.**

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
error **only during the constant-velocity cruise of each leg** (the trapezoidal accel/decel ramps and
the turnaround stiction band are gated out), keeping the two directions separate. Seeds a **coarse**
one-point offset at the end of lock-in (`encoder_set_phase_offset(pos)`, `encoder_calibration.c:234`),
then in `finish` averages the two directions' means to cancel friction and **applies** the fine result
(`encoder_calibration.c:311`), setting `offset_cal_valid`. If it lacks clean cruise samples in both
directions — the signature of dir-cal (9) not having run first — it **rolls back** to the previous
offset + validity and returns false. So this state now does a full coarse+fine offset cal (the apply
that used to be commented out is live).

**Single-point "shitty" offset cal (15).** Locks at electrical angle 0 for 4 s, then sets the phase
offset to the measured position and marks it valid. A single-point cal — coarser than state 10's
bidirectional friction-cancelled fit, but fast and robust; it's the bootstrap offset step in the
standard commissioning runbook (`9 → 15 → 21`, see `can-interface.md`). Both 10 and 15 now apply.

**Encoder validation (14).** Steps the open-loop vector through 12 angles from −π to π, dwelling
~2 s each (`ENC_VAL_LOOPS_PER_POINT = 2·LOOP_UPDATE_FREQ = 64000` ticks @ 32 kHz), recording motor-vs-encoder phase, then dumps a
table over both `COMMS_UART_HANDLE` (USART1) and `DEBUG_UART_HANDLE` (USART2). Diagnostic.

**Open-loop (12).** Pure α/β current PI with no encoder feedback. **Its setpoint is not wired to
CAN** — only `set_open_loop_current_setpoint()` from code sets it, so over CAN this state just holds
zero current. Used as the primitive that the encoder-cal laws drive.

**Encoder test (11).** Runs the estimator on every tick with zero PWM output. Never completes — use
to watch the estimator without energizing the motor.

**Self-test (22).** Desk bring-up check of a bare board (`self_test.c`; operator guide in
`board-bringup.md`, host runner `can_console/bringup.py`). Added 2026-11-17, **bench-untested;
thresholds are design guesses.** Two halves:

1. *Bridge half* — a control law, armed. `fsm_enter_state` floats the bridge right after
   `controller_arm()`, and the law's init floats it again on its first tick. Bridge off for
   2 + 100 ms: it accumulates shunt ADC midpoints, Vbus min/mean/max, per-phase phase-voltage
   mean and swing (PHx_V, rank 2), and encoder motion (wrapped deltas; counted as "rotating"
   only when |net| ≥ ½ path, so magnet-less noise doesn't count). It stops there
   (`bridge_off` FAIL, drive skipped) if a phase swings more than 0.5 V p-p (back-EMF), the
   encoder sees > 2 rev/s, or a phase's mean is above 0.3 × Vbus (a leak to the bus: the 5 %
   step would shoot through a shorted high side). A static offset below that is allowed: the
   high-side driver's bias current flows through the divider when all FETs are off. Shunt
   selection is switched off for the run and restored at exit. With all duties equal it would
   rebuild phase A as −(B+C), and a common edge kick would then read as current to the 15 A trip.
   Otherwise it sets **all three legs to the same duty**: 5 % for 12 ms, then 95 % for 12 ms.
   A motor at rest sees 0 V. Each step allows 2 ms to settle, then 10 ms of measurement:
   phase V as a fraction of Vbus (raw `JDR2/JDR1(Vbus)` ratio, same divider), Vbus minimum,
   and the differential-mode phase current, taken from the raw samples with the 3-phase mean
   removed (all edges share one duty, so a common switching artifact cancels). It floats the
   bridge and stops if Vbus drops below 0.8 × the bridge-off mean, or if the current goes above
   5 A in the 95 % step. In the 5 % step the edges fall inside the current-sample burst, so
   only the sag abort and the fast loop's 15 A trip cover it. The pass bands are 5 % step < 25 % of bus, 95 % step > 65 % of bus, current at 95 % ≤ 1 A
   (at 5 % the edges sit on the current sample, so that step is info only), and Vbus ≥ 0.95 ×.
   It is skipped (report-only, no arming) when the boot shunt-offset cal was rejected or timed
   out; the FSM exempts SELF_TEST from that arming gate for this.
2. *Report half* — parked by `self_test_exit()` (FSM exit), run by
   `fsm_run_pending_self_test()` in the foreground, disarmed. It checks the clock (HSE → PLL),
   option bytes, node-ID strap vote margin, unclamped VREF+ (2.9–3.1 V pass, outside 2.5–3.4 V
   fail), thermistors and die (−20…85 °C, ±15 °C of the median), MA732 register 0xE readback
   == `MA732_FILTER_WINDOW_REG` (bounded HAL-free SPI under PRIMASK) plus the 0x1B field flags
   (info), FDCAN TEC/REC/EP/BO, the bridge-half results, and latched errors. Info lines cover
   reset cause/boot count/crash record, BOOT0 level, encoder cal and fast-loop high-water.
   Output: `SELFTEST BEGIN …`, one `SELFTEST <PASS|FAIL|WARN|SKIP|INFO> <id>: <text>` line per
   check, `SELFTEST END PASS|FAIL (n fail, m warn)`. Overall pass = no FAIL lines.

`procedure_result` stays 0 until the report finishes (tens of ms after the state returns to
IDLE). The FSM holds new requests until then, the same way it does for a parked cal-store op.
Host tools should wait for `SELFTEST END` on the UART, or read `procedure_result` ≥ 0.4 s
after IDLE.

**Clear errors (13).** The only way to reset `controller_errors`. On *exit* it calls
`controller_clear_errors()`, then the FSM auto-requests `IDLE`. Any nonzero error elsewhere forces
`FSM_STATE_CONTROLLER_ERROR` until you pass through here.

## Telemetry while running

Entering any control-law state bumps all four TPDOs to a 100 ms event timer; `IDLE` drops them to
5 s. The `motor_current`/`bus_voltage`/`bus_current` fields in TPDO2/TPDO3 are only populated in the
three closed-loop control states (torque/vel/vel-ramp); elsewhere they read 0 — but `motor_vel`
(TPDO2) is live in *every* state (armed → encoder PLL, disarmed → foreground sampler). Temperatures
(TPDO4) are always live. See `docs/can-interface.md` for the wire format.
