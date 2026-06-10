# SWO PC-sampling profiler (`tools/orbtop.sh`)

A statistical "top" for the MCU: where is the CPU actually spending its time?
The Cortex-M4's DWT periodically emits the current program counter as a trace
packet; the packets go out the SWO pin, orbuculum decodes them against the ELF,
and `orbtop` shows a live per-function histogram.

Sampling happens **entirely in hardware** — zero CPU cost, nothing halted, the
IWDG keeps getting fed. It is safe to attach to an armed, spinning motor.
Bench-verified 2026-06-09 against a live VEL_RAMP run at the full control rate.

## Quick start

```bash
./tools/orbtop.sh                 # profiles cmake-build-relwithdebinfo ELF
```

One-time setup (the orbuculum suite is a submodule at `tools/orbuculum`):

```bash
git submodule update --init tools/orbuculum
meson setup tools/orbuculum/build tools/orbuculum -Dlibdwarf:werror=false
ninja -C tools/orbuculum/build
```

(`-Dlibdwarf:werror=false` is required: the vendored libdwarf 0.7.0 subproject
trips `unused-but-set-variable` under `-Werror` on current GCC. Host deps, all
in Arch repos: meson, ninja, libusb, zeromq, ncurses, libelf.)

Example, captured with the fast loop armed (VEL_RAMP):

```
 16.6%   2919  hrtim_repetition_callback
 14.9%   2615  HAL_Delay              <- foreground idle
 12.9%   2259  HAL_GetTick            <- foreground idle
  6.6%   1154  adc_update_callback
  6.2%   1097  foc_gen_pwm_cb
  5.3%    938  HAL_HRTIM_IRQHandler   <- HAL dispatch overhead before our callback
  4.5%    786  arm_sin_cos_f32
  4.1%    715  pwm_update_callback
  4.0%    700  encoder_update
  ...
```

## Knobs (environment variables)

| Var | Default | Meaning |
|---|---|---|
| `BUILD_DIR` / `ELF` | `cmake-build-relwithdebinfo` | which image's symbols to use — **must match what's flashed** |
| `SAMPLE_DIV` | 16 | PC sample rate = 144 MHz / 1024 / `SAMPLE_DIV` (16 → 8.8 kHz, 8 → 17.6 kHz, min 1) |
| `SWO_HZ` | 2000000 | SWO bit rate. 2 MHz works on any ST-Link; the bench STLINK-V3 is happy at 6–12 MHz (pick a divisor of 144 MHz) |
| `CPU_HZ` | 144000000 | HCLK feeding the TPIU prescaler; only change if the clock tree changes |
| `EXC` | 0 | 1 = also enable DWT exception entry/exit trace and pass `-E` to orbtop |

Extra CLI args pass through to `orbtop` (`-l` per-line aggregation, `-I <ms>`
interval, `-c <n>` row cutoff, `-j -` JSON output).

**Bandwidth budget:** a PC sample packet is 5 bytes ≈ 50 SWO bits. Default
8.8 kHz × 50 ≈ 440 kbit/s of the 2 Mbit/s link — comfortable. `SAMPLE_DIV=8`
doubles that; still fine. `EXC=1` adds ~3 packets per interrupt entry/exit —
at a 32 kHz fast loop that alone is ~2 Mbit/s, so raise `SWO_HZ` first.
Health check: the "Samples" line should equal sample-rate × interval (17577
per 2 s at defaults). If it's persistently low, the link is overrunning.

## How it works

```
DWT (PCSAMPLENA, POSTCNT)        hardware PC sampler
  └─> ITM (TXENA)                packetizes DWT output
       └─> TPIU (NRZ/UART, formatter off, /72 prescaler)
            └─> PB3 TRACESWO ──> ST-Link SWO capture
                 └─> OpenOCD  (tcp :3443, raw SWO server)
                      └─> orbuculum mux (tcp :3402, ORBFLOW)
                           └─> orbtop -e <elf>
```

Everything is configured **from the debugger side** — no firmware
participation, so it attaches to whatever is already running. The script's
OpenOCD pokes, in order, after `tpiu enable` (which itself sets
`DBGMCU_CR.TRACE_IOEN` via the `stm32g4x.cfg` pre-enable hook):

| Poke | What |
|---|---|
| `GPIOB MODER3 01→10, AFRL3=0` | PB3 back to AF0/TRACESWO — firmware's `MX_GPIO_Init` boots it as a plain GPIO output (`.ioc` label "SWO") |
| `ITM_LAR = 0xC5ACCE55` | unlock ITM |
| `ITM_TCR = 0x0001000D` | TraceBusID 1, **TXENA** (forward DWT packets), SYNCENA, ITMENA |
| `DWT_CTRL` | CYCCNTENA + CYCTAP(/1024) + POSTPRESET + SYNCTAP + **PCSAMPLENA** |

`ITM_TER` (stimulus port enables) is **deliberately left at 0**: `foc.c` calls
`swo_print_f32()` on four channels every tick, which at 16–32 kHz would be
2.6–5 Mbit/s — saturating SWO and drowning the PC samples. If you want those
channels (decode with `orbcat`), raise `SWO_HZ` well above the combined rate
and add an OpenOCD poke `mww 0xE0000E00 0x0000000F`. The old firmware-side
`swo_setup()` is dead code and configures **Manchester** SWO, which ST-Link
cannot capture — don't resurrect it for this.

## Reading the numbers

- Percentages are fractions of *all CPU time* (samples are unconditional).
  `HAL_Delay`/`HAL_GetTick` together ≈ the foreground idle fraction; the
  fast-loop ISR budget is roughly everything else.
- **Inlining:** at `-Ofast` most small helpers are folded into callers — a
  sample lands in whichever function's code the PC is inside *after* inlining.
  `arm_sin_cos_f32` showing up separately means it really wasn't inlined.
  Use `-l` (per-line) to see hot spots inside big functions.
- **CCM-resident code resolves correctly**: the ELF's symbol addresses are the
  CCM run addresses (VMA), which is what the sampled PC contains.
- `HAL_HRTIM_IRQHandler` time is pure dispatch overhead before
  `hrtim_repetition_callback` is reached — measurable, and a known cost.

## Gotchas

- **The session owns the ST-Link.** `flash.sh`, gdb, or a second OpenOCD can't
  run concurrently. Profile after flashing.
- **Reset undoes everything.** A reboot re-runs `MX_GPIO_Init` (PB3 back to
  GPIO-out) and clears DWT/ITM. Restart the script after any reset/reflash.
- **ELF must match the flash.** Stale symbols produce confidently wrong tables.
- Logs land in `/tmp/orbtop-openocd.log` and `/tmp/orbtop-mux.log`.
- The mux can feed other clients simultaneously (`orbcat`, `orbstat`,
  `orbprofile` for callgrind-style output — all in `tools/orbuculum/build/`).
  To skip the mux entirely: `orbtop -s localhost:3443 -p ITM -e <elf>`.
