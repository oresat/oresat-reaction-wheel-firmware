# Footguns

Cross-cutting traps that bite at runtime and aren't obvious from any single file. Per-state stubs and
calibration quirks live in `docs/control-laws.md`; the fast-loop walk is in `docs/control-cycle.md`.
File:line refs were current when written — verify, this is a moving codebase.

## Concurrency & timing

**The fast loop (pri 0) preempts the FSM and CAN ticks.** Shared state is `volatile` but unlocked.
Single-writer-per-field is mostly respected, but multi-word values (the `svm_vec_t` timings, f32
setpoints, `u32 controller_errors`) can be read torn. In practice the torn reads here are benign
(a one-cycle-stale float), but don't *assume* atomicity — if you add a multi-field shared struct that
must be consistent, guard it (disable the HRTIM IRQ briefly, or double-buffer).

**`printf` is blocking, and the FSM transition path calls it from inside the 1 kHz FSM ISR.**
`_putchar` → `uart_transmit_one` spins on TXE then TC (`util.h`). `fsm_enter_state`/`fsm_exit_state`
each `printf` a line (`fsm.c:121,151`), and `control_law_finish()` runs there too — so **every state
transition blocks the FSM ISR for the whole UART transmit** (milliseconds at typical baud). During
that, the CAN tick (pri 2) and foreground starve; only the fast loop (pri 0) preempts. Worse,
`encoder_validation_finish` / `encoder_dir_cal_finish` do multi-line `uart_transmit_buf` dumps from
this ISR context. Fine for bench bring-up; do **not** add `printf` to the fast loop, and be aware
state changes hiccup CAN.

**Changing TPDO rate resets CAN communication.** `set_tpdo_send_rate()` calls
`canopen_app_resetCommunication()` (`can_comms.c:207`), and `fsm_enter_state` calls it on *every*
state entry (100 ms for control states, 5000 ms otherwise). So each state transition briefly drops
and re-inits the CANopen comm layer — expect a heartbeat/TPDO blip on every transition.

**Setpoints are edge-triggered and state-routed.** `process_sdo_vals` only applies `setpoint_1` when
it *changes*, and routes it by the state active at write time. Request the state first, then write the
setpoint; re-writing the same value is a no-op. See `docs/can-interface.md`.

## Sensors & control

**The encoder SPI read is assumed to succeed.** `encoder_read_success_dma = true` is hard-coded in
the fast loop (`control_ll.c:278`, literally commented "this is a joke"). A failed/garbage MA702 read
isn't detected — it feeds straight into the FOC angle. If FOC ever goes unstable with no error flag,
suspect the encoder read before the controller.

**The encoder estimator goes unstable past ½-turn error.** `encoder.c:61`: "IF POS_ESTIMATE EVER
DIVERGES MORE THAN 0.5 TURNS FROM POS THEN THIS SHIT WILL GO UNSTABLE." The wrap math assumes the
prediction tracks within ±0.5 rev. Very high acceleration, a missed-sample burst, or a bad first-run
seed can break that assumption. `is_first_run` reseeds pos and zeroes velocity on (re)entry — make
sure `encoder_init()` is called before arming if the rotor may have moved.

**The consumer/producer return convention is inverted vs. its comment.** `control_ll.h:76` says
"return bool of if there's any error," but the implementation treats **`true` = success/consumed**.
Returning `false` leaves the watchdog flag set and trips `*_USAGE_MISSING` next tick. Detail in
`docs/control-cycle.md` → flag machine.

**Watch for integer division in a float context generally.** The lock-in soft-start ramps in
`encoder_calibration.c` used `2 * n_loops / lockin` with `u32` operands (integer division → the
"ramp" was a step at 50%); fixed to float arithmetic. clang-tidy's `bugprone-integer-division` (live
via the LSP) catches this class — heed it on any `u32 / u32` feeding a `f32`.

**Measured R/L is never used by FOC.** `init_foc` uses `HARDCODE_PHASE_RESISTANCE/INDUCTANCE`
(`config.h`); resistance/inductance cal only print. Tuning the current loop means editing `config.h`,
not running cal. See `docs/control-laws.md`.

## Hardware / peripheral

**HRTIM dead-time: don't trust the HAL constant names.** Dead-time is set in generated
`Core/Src/hrtim.c:202-208` (`HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV8`, rising/falling value `0xC`). The
HAL's `..._DIV8`/`..._MUL8` naming and comments are misleading about the actual t_DTG. Compute the
real dead-time from the RM §28 dead-time formula (see `resources/INDEX.md` → reference manual), not
the symbol name, before trusting or changing it. Getting this wrong shoot-throughs the half-bridge.

**Option bytes are rewritten and the MCU reset on every boot if they mismatch.** `app_main` →
`check_option_bytes_ok()`; on mismatch it `write_option_bytes()` + `HAL_NVIC_SystemReset()`. If the
*expected* option-byte values are ever wrong, this is a reboot loop that also wears flash. Touch
`bootloader.c`'s expected values with care.

**The on-chip USART bootloader is bypassed deliberately** ("FUCKS OUR SHIT UP"). Boot-pin handling is
custom; don't re-enable the ST bootloader path expecting it to work.

## Build / tooling

**A clean CMake build proves very little.** No `-Wall`/`-Wextra` in the build, and no tests. Run
`./lint.sh` (clean baseline — any output is yours) before calling app-code changes done. `-Wformat` is
only active under lint (via `printf.h` attributes), so format-string bugs won't show in a normal build.

**`OD/OD.{c,h}`, `CMakeLists.txt`, and `Core/` are generated.** Hand-edits get overwritten — change
the source (EDS / `.ioc` / `CMakeLists_template.txt`) and regenerate. See `CLAUDE.md`.

**Use the custom typedefs/printf, not libc.** `u8`/`f32`/etc. (`typedefs.h`) and `printf` from
`printf.h` (not `<stdio.h>`). Deliberately-unused debug locals get `__maybe_unused` so lint stays clean.
