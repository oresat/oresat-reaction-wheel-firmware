# CCM SRAM hot-path placement

The 32 kHz HRTIM fast loop runs from **CCM SRAM** (`0x1000_0000`) in optimized builds. CCM executes
with zero wait states and is immune to flash-cache misses, so the control ISR chain has lower and far
more *deterministic* latency than running from flash (which leans on the ART prefetch + I-cache and
still jitters on a miss). Background: `docs/control-cycle.md` (the tick), the debug trap in
`docs/footguns.md`.

## The gate: Debug = flash, optimized = CCM

`__ccmram` (`Firmware/Inc/typedefs.h`) expands to `__attribute__((section(".ccmram")))` **only when
`CCMRAM_HOT_FUNCS` is defined**, which CMake sets for every build type except Debug (see the
`Release`/`RelWithDebInfo`/`MinSizeRel` branches in `CMakeLists_template.txt` — and the generated
`CMakeLists.txt`):

| build dir | opt flags | `__ccmram` | hot path runs from |
|---|---|---|---|
| `cmake-build-debug` | `-Og -g` | no-op | flash — normal SW breakpoints / stepping |
| `cmake-build-relwithdebinfo` | `-Ofast -g` | section | CCM `0x1000_0000` (debug w/ HW breakpoints) |
| `cmake-build-release` | `-Ofast` | section | CCM |
| `cmake-build-minsizerel` | `-Os` | section | CCM |

So you **debug from flash** (Debug build) and **fly from CCM** (optimized build). For an on-target CCM
build with symbols, flash the `cmake-build-relwithdebinfo` ELF.

## What's placed in CCM

Everything reachable per-tick from `hrtim_repetition_callback`, tagged `__ccmram` at the definition
(grep the live set with `grep -rn __ccmram Firmware/Src`):

- fast-loop core + the four callbacks — `control_ll.c`
- FOC + Clarke/Park/SVM + sincos — `foc.c`, `bldc_math.c`, `mathutil.c` (`sincos_f32`/`sincos_cmsis`)
- encoder estimator — `encoder.c`
- every control law's per-tick consumers / producer / exec / is_complete — `closed_loop_controller.c`,
  `motor_calibration.c`, `encoder_calibration.c`, `open_loop_current_controller.c`, and the test law
- per-tick ADC / inverter accessors — `inverter.c`, `aux_adc.c`

Placed by the **linker** instead (vendor code, can't annotate — and **unconditional**, so it's in CCM
even in Debug): `arm_sin_cos_f32` and its `sinTable_f32` 513-entry LUT. Init / finish / setup paths and
off-tick getters stay in flash. Inline helpers (`__fn_inline`: `Q_rsqrt`, `hrtim_pwm_set_dcycle_f32`,
`swo_print_f32`, `wrap_pm_pi`, …) fold into their tagged callers and need no tag.

Budget: ~8.7 KB of 32 KB used (RelWithDebInfo) — lots of headroom.

## Mechanism

- **`STM32G474VEHX_FLASH.ld`** — `MEMORY` splits the former single 128 K `RAM` into `CCMRAM`
  (32 K @ `0x1000_0000`), `RAM` (80 K SRAM1), `RAM2` (16 K). The `.ccmram` output section is placed
  **before `.text`** (so its object-file `.text`/`.rodata` pulls win over the generic `*(.text*)` /
  `*(.rodata*)` catch-alls) and mapped `>CCMRAM AT> FLASH`: VMA in CCM, LMA in flash.
- **`startup_stm32g474vehx.s`** — a copy loop (same scheme as the `.data` copy) shadows `.ccmram` from
  its flash LMA (`_siccmram`) into CCM (`_sccmram`.._eccmram`) before `main()`.
- **`stm32g4xx_hal_conf.h`** — `PREFETCH_ENABLE 1U` turns on the flash prefetch buffer (helps the
  flash-resident remainder; I-cache/D-cache were already on by reset default + HAL_Init).

## Debugging CCM code — use hardware breakpoints

CCM code is copied from flash at boot, so **GDB software breakpoints set in it are overwritten by the
startup copy and silently never fire**. Use **hardware** breakpoints for CCM functions: the
Cortex-M4 FPB can break on any address `< 0x2000_0000`, which is exactly why CCM is linked at
`0x1000_0000` and *not* the `0x2001_8000` SRAM alias. Either `hbreak <fn>` per breakpoint, or
uncomment `gdb_breakpoint_override hard` in `st_nucleo_g4.cfg` for the session (leave it off for the
Debug build — there are only 6 FPB comparators, and flash debugging uses them automatically). The
Debug = flash gate exists precisely so you only need this when chasing something in the CCM build.

## Maintenance / gotchas

- **A CubeMX/CubeIDE regen reverts two of these files.** `startup_stm32g474vehx.s` (copy loop) and
  `stm32g4xx_hal_conf.h` (prefetch) are generated — a regen silently drops both. The `.ccmram`
  section + `MEMORY` split live in `STM32G474VEHX_FLASH.ld`, also generated but already hand-maintained
  for the bootloader. `CMakeLists.txt` is safe because the gate is mirrored in `CMakeLists_template.txt`.
- **Adding a hot function:** tag its definition `__ccmram`. Verify with
  `arm-none-eabi-nm cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf | grep ' <fn>$'` → expect `1000xxxx`.
- **SRAM1 is now 80 K, not 128 K.** `.data`+`.bss` must fit there (currently ~39 K, incl. the 22 K
  `foc_capture_buf`). `RAM2` (16 K) is reserved/unused — available if SRAM1 ever gets tight.
- Composes with the bootloader memory map (FLASH `0x8008000`): the `.ccmram` LMA sits after `.text` in
  the relocated flash, and the boot copy runs in the app's own `Reset_Handler`. See
  `docs/bootloader-integration.md`.
