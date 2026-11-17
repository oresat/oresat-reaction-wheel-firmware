# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

OreSat ACS (Attitude Control System) reaction-wheel motor controller firmware. Bare-metal C on STM32G474VEHX (Cortex-M4F, 144 MHz), targeting four reaction wheels on a CANopen bus. Drives a BLDC motor via field-oriented control (FOC) using the on-chip HRTIM, three-shunt current sensing, and an MA702 SPI magnetic angle encoder.

## Build & flash

The project is dual-managed: STM32CubeIDE (`.cproject`, `.mxproject`) and CMake (`CMakeLists.txt`). Most local development uses the CMake out-of-tree build directories already on disk (CLion-style):

```
cmake-build-relwithdebinfo # -Ofast -g — BENCH DEFAULT (build.sh/flash.sh/dump tools default here)
cmake-build-debug          # -Og -g — lint flag source; CANNOT run the fast loop armed (below)
cmake-build-release        # -Ofast
cmake-build-minsizerel     # -Os
```

**At the 32 kHz control rate the armed fast loop does not fit a Debug build** (`-Og`,
flash-resident; measured 2026-06-09: optimized+CCM tick = 72% of the 31.25 µs budget, Debug
is at/over 100%) — the priority-0 ISR saturates, SysTick (the sole IWDG feeder) starves, and
the board watchdog-reboots ~512 ms after arming. Bench on RelWithDebInfo; use Debug only for
unarmed work or after dropping the loop to 16 kHz (`PWM_CYCLES_PER_ISR` + `RepetitionCounter`).
Keep `cmake-build-debug` configured anyway — `./lint.sh` reads its compile flags.

Build:
```bash
cmake --build cmake-build-relwithdebinfo -j
# or, if reconfiguring:
cmake -S . -B cmake-build-relwithdebinfo -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build cmake-build-relwithdebinfo -j
```

Build artifacts: `cmake-build-*/oresat-acs-fw-v8.{elf,hex,bin,map}`. Toolchain is `arm-none-eabi-gcc` (must be on PATH). Linker script: `STM32G474VEHX_FLASH.ld`.

Flash via OpenOCD with the bundled config (uses ST-Link SWD):
```bash
openocd -f openocd.cfg -c "program cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf verify reset exit"
```

### Two-image build/flash (bootloader + app)

This repo also vendors the board's UART bootloader at `bootloader/` — a separate,
self-contained CMake project (its own toolchain, linker script and FPU ABI) that
is **not** part of the app build above. Two scripts at the repo root drive both
images at once:

```bash
./build.sh [Debug|Release|RelWithDebInfo|MinSizeRel]   # builds app + bootloader/
./flash.sh                                             # flashes BOTH over SWD, one OpenOCD session
```

`flash.sh` programs the bootloader at `0x08000000` and the app at `0x08008000`
(each at the address baked into its ELF; they don't overlap). The single-image
commands above still work for app-only build/flash. See the **Bootloader** note
under "Conventions" and `docs/bootloader-integration.md`.

### Pinned toolchain, CI releases, release flashing

- `./toolchain.sh` fetches Arm GNU Toolchain 14.3.Rel1 into `.toolchain/` (no root, Linux
  x86_64/aarch64). `build.sh` uses it whenever it is installed; `TOOLCHAIN_DIR=<bin dir>` in
  the environment overrides, and an empty `TOOLCHAIN_DIR=` forces whatever is on PATH. A
  `.toolchain/` that exists without the pinned version in it (old version, unfinished
  download) is an error, not a fallback to PATH. `build.sh` starts a build dir over when it
  was configured with a different compiler. CI builds releases with the pinned toolchain.
  CLion builds are unaffected (they use PATH).
- `.gitlab-ci.yml`: a pushed tag builds both images, runs `tools/dist.sh`, uploads `dist/` to
  the generic package registry as `oresat-acs-fw/<tag>/<file>`, and creates a GitLab release.
  A pipeline started by hand from the web UI builds only.
- `./flash-release.sh <tag>` needs only OpenOCD + curl: downloads and checksum-verifies a
  release, backs the flash up to `backups/`, clears write protection (and read protection
  level 1), sets the option bytes the firmware expects, erases everything except the cal
  page (`--erase-cal` wipes that too; if the page is not blank it demands a yes at a terminal
  that `-y` does not cover), programs + verifies the `.hex` files, and confirms the app booted. The release ELFs land in `releases/<tag>/` for the `tools/*.sh` scripts.
- The OpenOCD config is `openocd.cfg` (was `st_nucleo_g4.cfg`); a bare `openocd` in the repo
  root picks it up.

### CMake/CubeMX regeneration is RETIRED (2026-06 decision)

`CMakeLists.txt`, `Core/`, and `OD/OD.c` were historically auto-generated (CubeIDE / CubeMX / CANopenEditor). **Regeneration is permanently retired for this repo** — too much hand-owned state would be clobbered. The generated files now carry deliberate manual edits *outside* the USER CODE markers (marked `/* manual edit */`: `fdcan.c` retransmission/SJW/IRQ priorities, `tim.c` prescalers, `stm32g4xx_it.c` fault handlers, `OD.c` 0x1014). Treat every file in the tree as hand-owned; do not run CubeMX/CubeIDE/CANopenEditor regeneration.

There are no tests in this project — it is target firmware. "Build passes" is the primary correctness signal; behavior is verified on hardware.

### Warning checks (`./lint.sh`)

The CMake build runs **without** `-Wall`/`-Wextra` (and globs vendor HAL/CMSIS-DSP/CANopenNode into one target, which would bury app warnings if it did). `./lint.sh` compiles each `Firmware/Src/*.c` with `-Wall -Wextra` using the exact flags from `compile_commands.json`, giving real coverage on the code you own. Run it before declaring app-code changes done — **the baseline is clean (0 warnings), so any output is something you introduced.** Deliberately-unused debug locals are marked `__maybe_unused` (`typedefs.h`); `printf`/`snprintf` carry format attributes (`printf.h`) so `-Wformat` arg/format mismatches are caught here too (the normal build, lacking `-Wformat`, ignores them).

### Editor/LSP

A gitignored root `compile_commands.json` symlink → `cmake-build-debug/` plus `.clangd` (strips arm-gcc-only flags clang rejects) make command-line clangd / editor LSP work against the cross-compile flags. CLion uses its own indexer and ignores both.

### Docs

- `docs/control-laws.md` — per-FSM-state cheat sheet (setpoint meaning, completion/error conditions, stubs/gotchas).
- `docs/control-cycle.md` — end-to-end trace of one 32 kHz fast-loop tick, interrupt priorities, and the producer/consumer flag machine.
- `docs/can-interface.md` — CANopen host interface (COB-IDs per wheel, TPDO wire format, SDO commands, example sequences).
- `docs/foc_capture.md` — per-tick FOC inner-loop snapshot mechanism for offline analysis (arm/stop/dump/decode via `tools/foc_capture.sh`, SWD-only — no CAN surface).
- `docs/footguns.md` — cross-cutting runtime/concurrency/hardware traps (blocking printf in the FSM ISR, assumed-good encoder read, dead-time naming, option-byte reflash, etc.).
- `docs/bootloader-integration.md` — how this app boots behind the `bootloader/` image: memory map, the `0x08008000` relink + `SCB->VTOR` relocation, option bytes, IWDG continuity.
- `docs/ccmram.md` — CCM SRAM hot-path placement: the fast loop runs from `0x1000_0000` in optimized builds (`__ccmram`/`CCMRAM_HOT_FUNCS`), Debug stays in flash, debug CCM with hardware breakpoints.
- `docs/adc-sampling-tuning.md` — scope-free calibration of the ADC trigger position (`ADC_TRIG_CMP4_TICKS`): CMP4-sweep equivalent-time sampling over SWD (`tools/cmp4_sweep.sh` + `cmp4_sweep_analyze.py`).
- `docs/shuntsel-delay-calibration.md` — scope-free measurement of the duty→sample pipeline lag (`SHUNT_SEL_DELAY_TICKS`, worklog Q2-A): torque-step + foc_capture (`tools/shuntsel_delay.sh` + `shuntsel_delay_analyze.py`).
- `docs/swo-pc-sampling.md` — statistical CPU profiler: hardware DWT PC sampling over SWO, live per-function "top" via `tools/orbtop.sh` (orbuculum submodule at `tools/orbuculum`). Zero firmware cost, safe while armed; all config poked from OpenOCD.
- `docs/flight-review-2026-06.md` — full-codebase flight-readiness review (2026-06-09, vs. commit `ebdba3d`): verified P0/P1 findings (node-ID strap defect, unreachable OV trip, ECC-NMI boot loop, bootloader no-validity-check jump, FSM error ping-pong, edge-trigger SDO trap table), what was checked and found clean, and a prioritized fix punch list (statuses + bench-verification checklist at the bottom). Consult before touching the FSM ladder, command ingestion, fault handlers, cal_store, or the bootloader — and update its punch list as findings are fixed.
- `docs/review-fixes-change-log.md` — what the 2026-06-10 overnight fix session actually changed, grouped as commit-sized units with files + behavior deltas; includes the known-open list. The companion to the review's punch list.
- `docs/can-protocol-change-log.md` — host-facing summary of the 2026-06-10 bus-behavior changes (command semantics, aborts, EMCY, timing) + the can_console update list.
- `docs/errata-todo.md` — open firmware bugs (E1…) and requested changes (T1…) found after the flight review. Put new RW firmware errata and TODOs here, not in the OreSat design review. Delete or mark an item when it's fixed.
- `docs/board-bringup.md` — operator guide (written for a non-firmware user) for desk bring-up of a bare board: `can_console/cli.py bringup` (guided steps + per-board log) and the FSM self-test, state 22 (`Firmware/Src/self_test.c`; reference in `docs/control-laws.md`).

Consult these before changing FSM/control-law, the fast loop, or CAN/OD code.

## Source layout

- `Core/{Inc,Src}` — originally CubeMX-generated HAL/LL peripheral init and ISRs (`main.c`, `stm32g4xx_it.c`, per-peripheral `*.c`) — **now hand-owned** (regen retired; carries manual flight fixes).
- `Firmware/{Inc,Src}` — application code (everything custom lives here). Entry point is `app_main()`, called from `main()` after HAL init.
- `Drivers/STM32G4xx_HAL_Driver` — vendor HAL/LL.
- `Drivers/CMSIS` — Cortex-M / device headers.
- `Drivers/libtrace` — trace-port helpers.
- `DSP/` — CMSIS-DSP (used for filters, controllers, fixed-point math via `dsp/controller_functions.h`).
- `CANopenSTM32/CANopenNode` + `CANopenNode_STM32` — vendored CANopenNode stack and STM32 driver glue. Build excludes `CANopenNode/example` and `CANopenNode_STM32/OD*`.
- `OD/{OD.c,OD.h}` — Object Dictionary, originally generated from `CANopenNode_STM32/DS301_profile.eds` (CANopenEditor; `.xpd` is the editor project) — **now hand-owned** (regen retired; carries a manual 0x1014 EMCY fix). The application-specific `0x4000`–`0x4007` block is already defined.
- `oresat-acs-fw-v8.ioc` — STM32CubeMX project. Pin assignments, clock tree, peripheral parameter init.
- `stm32g474.svd` — register definitions for debug.
- `resources/` — STM32G474 datasheet, STM32G4 reference manual, HRTIM cookbook (PDFs), with companion `.toc.md` outlines and `.txt` text dumps. See `resources/INDEX.md` for the query workflow.
- `bootloader/` — the board's UART bootloader (formerly the standalone `oresat-acs-bootloader-v8`), vendored here as a self-contained subproject (its own CMake/`.ioc`/linker script). Flashed to `0x08000000`, runs before this app; **not** part of the app's CMake build — build/flash both via the root `build.sh` / `flash.sh`. See the **Bootloader** note under "Conventions".

## Reference manuals (resources/)

When a question is about STM32G4 hardware behavior — register fields, peripheral wiring, timing constraints, electrical limits, HRTIM topologies — consult `resources/` before guessing. Three PDFs, indexed for offline lookup:

- `stm32g474_datasheet.pdf` (236 p) — pinout, electrical limits, peripheral availability.
- `stm32g4_reference_manual.pdf` (2140 p) — register definitions, peripheral behavior. The authoritative source.
- `stm32_hrtim_cookbook.pdf` (84 p) — HRTIM application patterns (this firmware uses HRTIM heavily).

For each PDF there is a `<name>.toc.md` (markdown outline with PDF page numbers) and `<name>.txt` (page-marked text dump). Workflow:

1. **Section is known** → open the relevant `.toc.md`, find the page, Read the PDF directly with `pages="<N>-<M>"` (PDF pages are 1-based, max 20 per call). Use this for figures, register-bit diagrams, timing charts.
2. **Searching for a term** → grep the `.txt` with the page-aware awk pattern documented in `resources/INDEX.md`:
   ```bash
   awk '/^=== PAGE / {p=$3} /PATTERN/ {print "p" p ": " FILENAME ":" NR ": " $0}' resources/<name>.txt
   ```
   That gives `p<page>: <file>:<line>: <match>` so you can jump straight from grep hit to PDF page.

`resources/build_index.py` regenerates the indexes if PDFs are added or replaced.

## Architecture: three concurrent timing domains

The system is structured around three independent ISR domains plus a foreground loop. Understanding which domain code runs in is essential before changing it.

1. **Fast control loop — HRTIM1 repetition ISR @ 32 kHz** (`PWM_SWITCHING_HZ=96000` / `PWM_CYCLES_PER_ISR=3`).
   Entry: `hrtim_repetition_callback()` in `Firmware/Src/control_ll.c`. Each tick:
   1. Kick off MA702 SPI angle read (non-blocking start via direct-register macro `__MA702_FAST_ANGLE_READ_START()`).
   2. Run `adc_update_callback()` — reads injected ADC (3-shunt phase currents + Vbus), applies offset cal & shunt-amp gain, runs over/undervoltage/overcurrent trips.
   3. Finish SPI read, `encoder_update_callback()` — feed angle into encoder estimator.
   4. Call active `control_law.control_law_exec` / `current_consumer` / `encoder_consumer` / `pwm_producer`.
   5. `pwm_update_callback()` validates SVM output and writes new HRTIM compares.
   The fast loop is the only place that arms/drives PWM. If `controller_get_errors()` is non-zero anywhere in this chain, `controller_disarm()` is called immediately.

2. **FSM tick — TIM17 IRQ @ ~1 kHz** (`CONTROLLER_FSM_TIM_HANDLE`).
   Routed through `controller_run_state_machine()` → `fsm_exec()`. The FSM picks a `control_law_t*` for the requested state via `get_ctrl_law_for_state()` and installs it; the fast loop then runs that law.

3. **CANopen tick — TIM4 IRQ @ ~1 kHz** (`CAN_TIM_HANDLE`).
   Calls `canopen_1khz_isr()` → `canopen_app_interrupt()` (CANopenNode internals).

4. **Foreground loop — `app_main()`'s `while(1)`** (`HAL_Delay(1)`-paced).
   Strokes the IWDG liveness gate, pumps the console ring, calls `update_tpdo_od_vals()`, `canopen_process()` (NMT/SDO/PDO processing + pending-TPDO-rate apply + EMCY edges), `process_sdo_vals()` (deferred reboot), `fsm_run_pending_calstore_op()` (flash save/erase/restore parked by the FSM), `fsm_run_pending_self_test()` (state 22's UART report, parked by the FSM), and the 5 s CCM SEU scrub. Host command *ingestion* happens in OD write extensions (`can_comms.c`) inside the SDO server, not by polling.

`TIM7` triggers the auxiliary ADC (thermistors, VVCC sense); timekeeping is the Cortex SysTick (which is also the sole IWDG feeder — gated on a foreground heartbeat, see `Core/Src/iwdg.c`). See `Firmware/Inc/periphmap.h` for the canonical handle-to-role mapping; always reference these symbolic names rather than `htimN` directly.

## Architecture: control-law plug-in contract

The fast loop is generic — it only knows the `control_law_t` struct (`Firmware/Inc/control_ll.h`). Each operating mode (torque / velocity / position control, motor R/L cal, encoder dir/phase cal, open-loop, validation, etc.) provides its own `control_law_t` instance:

```c
typedef struct {
    ll_encoder_consumer_t encoder_consumer;       // bool(u16 enc_val)
    ll_phase_current_consumer_t current_consumer; // bool(f32 A,B,C, f32 Vbus)
    ll_pwm_timings_producer_t pwm_producer;       // bool(svm_vec_t* out)
    ll_control_law_exec_t control_law_exec;       // void(void)
    ll_control_law_is_complete_t is_complete;     // bool(void)
    ll_control_law_init_t control_law_init;       // void(void)
    ll_control_law_finish_t control_law_finish;   // bool(bool cancelled)
} control_law_t;
```

Returning `false` from a consumer/producer signals an error and the controller will disarm. `is_complete` returning true ends the procedure; the FSM then returns to `IDLE`.

Owners of `control_law_t` instances:
- `closed_loop_controller.c` — `torque_ctrl_control_law`, `vel_ctrl_control_law`, `pos_ctrl_control_law`, `vel_ramp_ctrl_control_law` (delegate consumers/producer to `foc_control_law`).
- `foc.c` — `foc_control_law` (Clarke/Park/SVM core).
- `motor_calibration.c` — `resistance_cal_ctrl_law`, `inductance_cal_ctrl_law`.
- `encoder_calibration.c` — `encoder_dir_cal_ctrl_law`, `encoder_phase_cal_ctrl_law`, `encoder_validation_ctrl_law`, `encoder_single_point_cal_ctrl_law`.
- `encoder.c` — `encoder_test_control_law`.
- `open_loop_current_controller.c` — `open_loop_ctrl_law`.
- `self_test.c` — `self_test_ctrl_law` (state 22; plus a foreground report half, see `self_test.h`).

## Architecture: FSM and error model

`fsm_state_t` (`Firmware/Inc/fsm.h`) enumerates states 0–22 (`IDLE`, the control modes, calibration/test routines, the cal-store action states 18–20, `SELF_TEST` 22, error states, `CLEAR_ERRORS`). External requests arrive via `fsm_request_state()` (called from CANopen SDO handler) — the FSM tick reconciles current vs. requested state, calls `control_law_finish` on the outgoing law and `control_law_init` on the incoming one, and tracks `fsm_last_state_exit_ok` for the host to read.

Errors are bit-flag `CONTROLLER_ERRORS_t` accumulated via `controller_add_errors()`. Any nonzero error forces `FSM_STATE_CONTROLLER_ERROR`; only `FSM_STATE_CLEAR_ERRORS` resets them. Decoding helpers: `STRINGIFY_CONTROLLER_ERRORS()` in `util.h`.

Per-state reference (what each state does, its setpoint interpretation, completion/error conditions, and known stubs/gotchas) is in `docs/control-laws.md`.

## Architecture: CAN identity and OD

CAN node ID is **hardware-strapped** at boot from DEBUG1/2/3 GPIOs (1000-sample majority vote in `app_main.c`), mapping to one of `CANID_WHEEL{1..4}` (`0x3C/0x40/0x44/0x48` from `config.h`). Bus runs at 1 Mbit/s on FDCAN1. Four TPDOs are configured.

OD entries the firmware reads/writes (see `can_comms.c`):
- `0x4000` rxw_ctrl_stat — current FSM state, error bitmask, procedure result (TPDO).
- `0x4001` rxw_motor_vel_current — velocity, iq (TPDO).
- `0x4002` rxw_bus_volt_current — bus current, bus voltage (TPDO).
- `0x4003` rxw_temp — three temperature readings (TPDO).
- `0x4004` rxw_requested_state — host-writable target FSM state.
- `0x4005` rxw_setpoint — `setpoint_1`, `setpoint_2` (interpretation depends on state).
- `0x4006` rxw_reserved — boot diagnostics, read-only by convention (reset cause | crash kind | boot count; last-fault PC — bit-packed u32s, see `docs/can-interface.md`).
- `0x4007` rxw_reboot — write-nonzero-to-reboot (disarm + flush; SDO response egresses before the reset).

`update_tpdo_od_vals()` is the single fan-out from internal state to the OD; the fan-in is the OD **write extensions** on 0x4004/0x4005/0x4007 (always-apply, validating, SDO-abort on bad input — `init_od_extensions()` in `can_comms.c`). Wire-level detail (per-wheel COB-IDs, TPDO byte layout, SDO command sequences, setpoint routing) is in `docs/can-interface.md`.

## Coordinate frames and motor math

Three vector types in `bldc_math.h`, used throughout:
- `svm_vec_t {A,B,C}` — three-phase ABC duty cycles or currents.
- `ab_vec_t {alpha,beta}` — stationary two-phase (Clarke).
- `dq_vec_t {d,q}` — rotor-synchronous (Park; needs electrical angle).

Transforms: `clarke_transform`, `park_transform(ab, phase)`, `inv_park_transform(dq, phase)`, `SVM(ab)` (with third-harmonic injection). `svm_output_valid` is what triggers `CTRLR_ERR_MODULATION` / `CTRLR_ERR_PWM_TIMING_INVALID`.

Pole pairs: `MOTOR_NUM_POLEPAIRS = 6`. Encoder CPR: `ENCODER_CPR = 16384` (14-bit MA702). Motor electrical angle = encoder mech angle × 6, modulo 2π.

All tunables (current limits, calibration voltages/times, R/L bounds, controller bandwidths, voltage trip thresholds, FOC feedforward enables) live in `Firmware/Inc/config.h`. Treat changes there as system-affecting.

## Conventions specific to this codebase

- **Custom typedefs** in `Firmware/Inc/typedefs.h`: use `u8`/`u16`/`u32`/`u64`/`i*`/`f32`/`f64`, not `uint8_t` etc., in `Firmware/`. Same file defines function-attribute macros: `__fn_const`, `__fn_pure`, `__fn_flatten`, `__fn_hot`, `__fn_leaf`, `__fn_noreturn`, `__fn_used`, `__fn_inline` (= `inline + always_inline`). `__ccmram` places data in CCM RAM. Apply these on hot-path functions for register-pressure / locality wins.
- **Direct register access on hot paths.** ISRs and the fast loop bypass the HAL where it's slow: see GPIO writes via `BSRR`/`BRR` in `util.h`'s `gpio_debug*`, the `__MA702_FAST_ANGLE_READ_START/_FINISH` SPI macros, and DAC writes in `util.h`'s `dac_debug*`. Don't replace these with `HAL_GPIO_WritePin` / `HAL_SPI_Transmit` etc. — the timing budget at 32 kHz won't tolerate it.
- **Custom printf, deferred from ISRs** (`Firmware/Src/printf.c` + `console.c`) over `DEBUG_UART_HANDLE` (USART2, 921600). Use `printf` from `Firmware/Inc/printf.h`, not the libc one. ISR-context printf lands in a 2 KB RAM ring (never blocks; full ring drops the message, `console_dropped` counts) drained by `console_pump()` in the foreground loop; thread-context printf is synchronous. Call `console_flush()` before any reset. Postmortem ring dump: `tools/console_tail.sh`. `blank_console()` macro clears the terminal. See the printf entry in `docs/footguns.md`.
- **Bootloader.** This app is flashed at `0x08008000` and runs *behind* a separate custom bootloader — vendored in this repo at `bootloader/` and built standalone (root `build.sh`/`flash.sh`) — at `0x08000000`. That bootloader — not this app — owns the boot decision: it reads BOOT0 (PB8) as a GPIO and either jumps to this app (which then relocates its vector table to `0x08008000` via `SCB->VTOR`; see `system_stm32g4xx.c` + `STM32G474VEHX_FLASH.ld`) or runs a UART DFU loader (ST OpenBootloader on USART1) for field reflash. The on-chip ST *system-memory* bootloader is deliberately bypassed (`app_main.c` comment: "FUCKS OUR SHIT UP"), and the app's own `jump_to_bootloader()` path is now commented out (that logic lives in the separate bootloader). Option bytes are still verified/written on every boot via `check_option_bytes_ok()` / `write_option_bytes()`; if they need updating, the firmware writes them and reboots. See `docs/bootloader-integration.md`.
- **`assert` is custom** (`util.h`) — calls `user_assert_fail(file, line)`, not libc abort.
- **Volatile shared state.** The fast loop, FSM tick, and foreground each touch shared state (`controller_errors`, `last_phase_currents`, `is_currents_updated`, etc.). These are typed `volatile`; respect that and don't assume single-reader semantics.
