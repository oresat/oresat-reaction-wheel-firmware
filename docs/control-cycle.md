# One control cycle, end to end

A precise trace of the 32 kHz fast loop and how the other timing domains interleave with it. Read
`CLAUDE.md` → "three concurrent timing domains" first for the high-level map; this doc drills into
the fast loop and the producer/consumer flag machine that is easy to misread.

## Interrupt priorities (who preempts whom)

Set explicitly at the top of `app_main()` (lower number = higher priority):

| Pri | IRQ | Role |
|---|---|---|
| 0 | `HRTIM1_TIMA_IRQn` | fast control loop (32 kHz) — preempts everything |
| 1 | `TIM1_TRG_COM_TIM17_IRQn` | FSM tick (1 kHz) |
| 2 | `TIM4_IRQn` | CANopen tick (1 kHz) |
| 3 | `TIM7_DAC_IRQn`, `FDCAN1_IT0/1`, `DMA1_Channel1` | aux ADC, CAN RX/TX, DMA |

So the fast loop can interrupt the FSM and CAN ticks mid-execution. Anything the fast loop shares
with them must tolerate that (see `docs/footguns.md` → volatile shared state).

## The tick: `hrtim_repetition_callback()` (`control_ll.c:243`)

Fires every `PWM_CYCLES_PER_ISR` PWM periods — `PWM_SWITCHING_HZ 96000 / PWM_CYCLES_PER_ISR 3` = **32 kHz, 31.25 µs budget**.
Registered as the HAL repetition callback by `controller_arm()`; *unregistered* by
`controller_disarm()`, so the loop only runs while armed. In order:

1. **`debug_hrtim_isr(true)`** — raises a scope/SWO marker for the whole ISR (timing instrumentation).
2. **`n_loop_count++`** — the monotonic loop counter. Calibrations measure durations by differencing
   it (`controller_get_loop_count()`), so it is the fast loop's clock.
3. **One-shot init** — if `control_law_needs_init`, call `control_law.control_law_init()` once. This
   is how a freshly installed law initializes *inside* the loop (not from the FSM domain).
4. **`__MA702_FAST_ANGLE_READ_START()`** — drops nCS and writes the SPI DR to launch the angle read.
   Non-blocking: the conversion happens while step 5 runs, hiding ~SPI latency under the ADC work.
5. **`adc_update_callback()`** (`control_ll.c:109`) — the current path (≈5 µs):
   - Validates inverter calibration + that last cycle's currents were consumed (see flag machine).
   - Reads injected ADC raw (3 shunts + Vbus), subtracts per-phase offset cal, applies
     `SHUNT_AMP_CONDUCTANCE` and Vbus divider, scaling counts by the measured VREF+ (3.0 V reference,
     read through VREFINT on the aux ADC; the code calls it `vvcc`).
   - Removes common-mode (`zero = A+B+C`, subtract `zero/3`).
   - **Trips:** over/under-Vbus, and `|I|² > MOTOR_CURRENT_TRIP_LIMIT²`. Any error → `controller_disarm()` + `goto fail`.
   - Publishes `last_phase_currents` / `last_vbus_voltage`, sets `is_currents_updated = true`, then
     calls `control_law.current_consumer(A,B,C,Vbus)` if armed.
6. **Cache `A_i/B_i/C_i`** from `last_phase_currents` (these feed the commented-out DAC dump; marked
   `__maybe_unused`).
7. **`__MA702_FAST_ANGLE_READ_FINISH()`** — spins on SPI RXNE, reads the 14-bit angle, raises nCS.
   Result → `current_angle_dma_raw`. (`encoder_read_success_dma = true` is hard-coded — the SPI read
   is assumed to have succeeded; see footguns.)
8. **`encoder_update_callback()`** (`control_ll.c:212`) — feeds `current_angle_dma_raw` to
   `control_law.encoder_consumer()`, which calls `encoder_update()` (the PLL estimator). Same
   flag/disarm pattern as currents.
9. **`control_law.control_law_exec()`** — the law's per-tick work, if any. For FOC laws this is empty
   (the real work is in the consumers/producer); for the closed-loop wrappers it computes the torque
   command (`vel_ctrl_exec`, etc.).
10. **`pwm_update_callback()`** (`control_ll.c:68`) — the PWM path (≈4 µs):
    - Calls `control_law.pwm_producer(&out)`; if it returns true, latches `out` as the new timings
      and marks them valid.
    - Validates `svm_output_valid()`; missing-update or invalid → `CTRLR_ERR_*` → `controller_disarm()`.
    - If still armed, writes HRTIM compares via `hrtim_pwm_set_dcycle_f32()` and clears
      `is_pwm_timings_updated`.
11. **Completion check** — if `control_law.is_complete()` returns true, `controller_disarm()` and set
    `control_law_is_done = true`. The FSM tick later observes this and returns to `IDLE`.
12. **`debug_hrtim_isr(false)`** — drops the ISR marker.

## The producer/consumer flag machine (read this twice)

Three pairs of `volatile` flags track, each cycle, *both* "is there fresh data" and "did someone use
it". They exist so a misbehaving control law (one that silently stops consuming currents or producing
PWM) is caught and disarms the motor rather than running on stale values.

- `is_currents_updated` / `is_currents_valid`
- `is_encoder_updated` / `is_encoder_valid`
- `is_pwm_timings_updated` / `is_pwm_timings_valid`

Per cycle for the current path:
1. *Start of `adc_update_callback`*: if `is_currents_updated` is **still true** from last cycle,
   nobody consumed it → `CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING`.
2. Fresh data produced → `is_currents_updated = true`.
3. `current_consumer(...)` runs; **if it returns true, `is_currents_updated = false`** (consumed).

So **consumers/producers return `true` on success ("I handled it")**, and the controller clears the
"updated/missing" flag on a true return. ⚠️ This is the *opposite* polarity of the typedef comment at
`control_ll.h:76` ("return bool of if there's any error"). Trust the code: `foc_meas_current_cb`,
`open_loop_current_cb`, etc. all `return true` on success. A consumer that returns **false** leaves
the flag set, which trips `*_USAGE_MISSING` on the *next* tick → disarm. (Several cal consumers also
`controller_add_errors()` directly before returning false.)

## Crossing domains

- **State change:** host SDO → OD write extension `od_write_requested_state()` (SDO-server/foreground context, validates + always applies) → `fsm_request_state()` → FSM tick
  installs the law + `controller_arm()` → fast loop runs it. The fast loop never picks a law itself.
- **Setpoint:** OD write extension `od_write_setpoint()` (validates finite/range, applies on every download) → `set_*_setpoint()` writes a `volatile` the fast
  loop reads next tick. No locking; a torn float is possible but practically harmless here.
- **Disarm is unilateral:** any non-zero `controller_errors` anywhere in the chain calls
  `controller_disarm()` immediately, which unregisters this very callback. The FSM tick later
  reconciles to `CONTROLLER_ERROR`.
- **Telemetry out:** `update_tpdo_od_vals()` (foreground) snapshots fast-loop state into the OD.

## Timing budget sanity

~62.5 µs/tick. The instrumented costs (`adc` ≈5 µs, `pwm` ≈4 µs, plus SPI/encoder/FOC) leave wide
margin, which is why the SPI read is started early (step 4) and finished late (step 7) rather than
blocking. If you add work here, watch the `debug_hrtim_isr` marker on a scope — overrun silently
starves the FSM/CAN ticks (they're lower priority).
