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

**`printf` from ISRs is deferred, not blocking — but it can DROP.** Since 2026-06: `printf`
formats into a stack buffer and routes through `console_write()` (`console.c`). From ISR context
the message is memcpy'd into a 2 KB RAM ring (≈1 µs under PRIMASK) and the foreground loop pumps
it to the UART — FSM-transition prints no longer stall the TIM17 ISR (~1 ms each before), and the
old in-ISR `init_foc` gain prints (~31 dropped fast-loop ticks at every arm) moved to boot. The
trades: **a full ring silently drops the whole message** (counted in `console_dropped`,
SWD-visible; the pump prints `[console: N dropped]` after the fact), lines truncate at 255 chars,
and ISR output is delayed by up to one foreground pass. Thread-mode `printf` (boot, foreground)
is still synchronous. If the box wedges before the pump runs, the un-drained text — plus the last
~2 KB of console history — is recoverable over SWD: `tools/console_tail.sh`. Call
`console_flush()` before any `NVIC_SystemReset` you add, or the last lines die with the reset.
Two deliberate exceptions still write the UART raw from TIM17 ISR context:
`encoder_validation_finish` / `encoder_dir_cal_finish` bulk-dump via `uart_transmit_buf`
(end-of-procedure data export; would flood the ring) — multi-ms FSM stalls by design; don't add
new ones. A `printf` in the fast loop is now *survivable* (it used not to be) but still costs
format time (~1-3 µs) and will flood/drop at 32 kHz — use `foc_capture` for per-tick data.

**Nothing below NVIC preempt-priority 2 may touch CANopen state.** The CO_LOCK critical sections
are BASEPRI(2) masks, not PRIMASK (so the prio-0 fast loop and prio-1 FSM tick stay live during
the TPDO pass — `CO_driver_target.h`). That only holds because the foreground and the prio-2 TIM4
tick are the *only* CANopen-touching contexts. `set_tpdo_send_rate()` from the FSM ISR keeps this
rule by parking a pending value that `canopen_process()` applies through the OD interface — don't
add a CANopen call to TIM17/HRTIM context, and don't "simplify" the pending-rate handoff away.
(The old implementation ran a full `canopen_app_resetCommunication()` from TIM17 on every state
entry: TX purge + boot-up frame + SDO server reset per transition.)

**Host commands flow through OD write extensions, not polling.** 0x4004/0x4005/0x4007 handlers
(`can_comms.c`) fire on every SDO download regardless of value, validate (SDO abort on bad
state / non-finite setpoint / cal-not-valid), and route setpoints by the state at write time.
The old edge-trigger ("rewrite the same value = silent no-op") is gone — but setpoint routing is
still write-time, so hosts should confirm `0x4000` state before writing setpoints. Adding a new
command object? Give it an extension; don't reintroduce a foreground compare-against-shadow.

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

**Option bytes are rewritten (and the MCU resets, inside `HAL_FLASH_OB_Launch`) on boot
mismatch — but at most 3 attempts per power-on.** `app_main` → `check_option_bytes_ok()` now
covers nBOOT0/nBOOT1/nSWBOOT0 **and BOR_LEV (level 4, ~2.8 V)**; the attempt counter lives in a
RAM2 cell (`crash_record.h`), and after 3 failed writes the wheel flies with mismatched OBs
rather than reboot-looping forever. Touch `bootloader.c`'s expected values with care — and note
OB_Launch *is* the reset: code after it only runs on write failure.

**RAM2 diagnostic cells (0x20017FA8–0x20017FFF) are shared contracts — and the app's stack sits
directly below them.** Crash record, boot counter, flash-ECC NMI latch, OB attempt counter
(`Firmware/Inc/crash_record.h`) plus the bootloader's RCSR mailbox at 0x20017FF8
(`bootloader/Src/main.c`). The app's MSP starts at `_estack = top-of-RAM2 − 0x58`
(STM32G474VEHX_FLASH.ld) precisely to keep the first stack frames from overwriting these cells
— if you add a cell, grow that reservation in the SAME change, and keep the mailbox
address/magic in sync between the two projects. (The bootloader's stack is in the CCM alias —
unaffected.)

**The flash-ECC NMI handler must stay a return-not-spin.** `NMI_Handler` clears ECCD/ECCD2,
latches ECCR into the RAM2 cell, and *returns* — the faulting cal-page read completes with
garbage and the blob CRC rejects it. Re-introducing a `while(1)` there re-creates the
boot-loop brick (flight-review P0-3): IWDG reset → same read next boot → NMI forever. cal_store
brackets every page read with the latch (clear → read → check) and erases the page on a hit.

**The on-chip USART bootloader is bypassed deliberately** ("FUCKS OUR SHIT UP"). Boot-pin handling is
custom; don't re-enable the ST bootloader path expecting it to work.

**The app is relinked to run behind the bootloader at `0x08008000` — via two edits to *generated*
files that a CubeMX/CubeIDE regen silently reverts.** (1) `STM32G474VEHX_FLASH.ld` FLASH origin =
`0x8008000`, LENGTH `480K` (not `0x8000000`/512K); (2) `Core/Src/system_stm32g4xx.c` — uncomment
`#define USER_VECT_TAB_ADDRESS` (~line 110) **and** set `VECT_TAB_OFFSET 0x00008000U` (~line 125), so
`SystemInit` does `SCB->VTOR = 0x08008000`. The VTOR write is gated behind `USER_VECT_TAB_ADDRESS`, so
editing the offset *alone* is a no-op that still builds clean and even passes `objdump -h` (vector LMA
is linker-set) — then dispatches every IRQ through the bootloader's table at runtime. Verify the real
thing: `arm-none-eabi-objdump -d --disassemble=SystemInit …` must store `0x08008000` to VTOR. Full
checklist in `docs/bootloader-integration.md`; working reference is `oresat-acs-fw-v7`.

## Build / tooling

**A clean CMake build proves very little.** No `-Wall`/`-Wextra` in the build, and no tests. Run
`./lint.sh` (clean baseline — any output is yours) before calling app-code changes done. `-Wformat` is
only active under lint (via `printf.h` attributes), so format-string bugs won't show in a normal build.

**`OD/OD.{c,h}`, `CMakeLists.txt`, and `Core/` are generated.** Hand-edits get overwritten — change
the source (EDS / `.ioc` / `CMakeLists_template.txt`) and regenerate. See `CLAUDE.md`.

**Use the custom typedefs/printf, not libc.** `u8`/`f32`/etc. (`typedefs.h`) and `printf` from
`printf.h` (not `<stdio.h>`). Deliberately-unused debug locals get `__maybe_unused` so lint stays clean.

**Float NaN checks must use `f32_finite_bits()` (mathutil.h), not compares.** Every optimized
build is `-Ofast` = `-ffinite-math-only`: GCC may legally fold `x != x` and negated range checks
into forms that PASS NaN. The bit-pattern helper is fold-proof. Load-bearing users: the SDO
setpoint gate (a laundered NaN setpoint used to command full negative current via
`clamp_f32(lo, NaN, hi) == lo`), `blob_valid()` in cal_store, the backemf fit gates.

**CubeMX/CubeIDE regeneration is RETIRED for this repo (2026-06 decision).** Generated files
(`Core/`, `CMakeLists.txt`, `OD/OD.c`) are now hand-owned and contain manual edits *outside* the
USER CODE markers (fdcan.c retransmission/SJW/priorities, tim.c prescalers, OD.c 0x1014, it.c
fault handlers). A regen would silently revert flight fixes — don't.

**Optimized builds run the fast loop from CCM SRAM — set breakpoints there in *hardware*.** In every
non-Debug build the HRTIM ISR path is placed at `0x1000_0000` (`__ccmram` / `CCMRAM_HOT_FUNCS`) and
shadowed from flash by a copy loop in `startup_stm32g474vehx.s`. GDB **software** breakpoints set in
that code are overwritten by the boot copy and silently never fire — use `hbreak`, or
`gdb_breakpoint_override hard` in `openocd.cfg`. CCM is linked at `0x1000_0000` (not the
`0x2001_8000` SRAM alias) precisely so the Cortex-M4 FPB, which only reaches `< 0x2000_0000`, can set
HW breakpoints there. The Debug build keeps this code in flash so ordinary breakpoints work — **debug
from Debug, fly from optimized.** The copy loop and the `PREFETCH_ENABLE` bit live in *generated* files
(`startup_*.s`, `stm32g4xx_hal_conf.h`) that a CubeMX regen reverts. Full reference: `docs/ccmram.md`.
