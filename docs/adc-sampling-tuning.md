# ADC sample-point tuning, scope-free (CMP4 sweep)

How to calibrate the injected-ADC trigger position (`ADC_TRIG_CMP4_TICKS`, HRTIM Timer A CMP4)
with **no oscilloscope** — just the ST-Link, the wheel, and a power supply. The trick: the thing
being tuned is itself a sampler. Holding the PWM duty perfectly constant and sweeping the trigger
across the period while recording what the ADC reads reconstructs the intra-period current
waveform — CMP4 + the ADC is an equivalent-time sampling oscilloscope, read out over SWD.

## Why this exists

The 4× injected-oversampling burst should be *centered* on the counter crest (counter =
`PWM_CNTR` = 24000 = the temporal middle of the up-down period, mid null-vector, where phase
current ≈ its average). The trigger must therefore fire `D` ticks before the crest, where
`D` = trigger latency + S&H aperture centroid + 1.5·Tconv (burst centroid). The latency term was
**assumed** (~42 ns) when `ADC_TRIG_CMP4_TICKS = 20768` was first derived; this procedure
**measures** the whole of `D` empirically and removes every assumption:

- Each phase's switching edge sits at a *known* counter tick: `E = (1 − duty) · 24000` (up-slope).
- The edge's artifact (INA kick + slope break) *appears* in the swept reconstruction at
  `X* = E − D` — the sample leads the trigger by exactly the delay being measured.
- So `D = E − X*`, averaged over the three phase edges, and **CMP4 = 24000 − D**. The
  4×-oversampling staircase smear is symmetric about the burst centroid, so the gradient
  centroid of the smeared step *is* the burst centroid — no analytic correction terms survive.

## Moving parts

| Piece | Where | Notes |
|---|---|---|
| Live CMP4 knob | `adc_trig_cmp4_ticks` (`control_ll.c`) | Seeded from `ADC_TRIG_CMP4_TICKS` (`config.h`); fast loop applies on change, clamped ≥64 ticks from 0/24000. The literal in generated `hrtim.c` only matters until first arm. |
| Fixed-duty mode | `open_loop_fixed_mod_{enabled,alpha,beta}` (`open_loop_current_controller.c`) | Bypasses the open-loop current PI: constant `SVM({α,β})` duty. **Required** — a closed loop servoes the *sampled* current flat and nulls the waveform being measured. Sticky across law init (set while IDLE, then enter the state). Clamped to `OPEN_LOOP_FIXED_MOD_LIMIT` (0.10 ≈ 5.4 A worst case). β ≠ 0 separates the B/C duties so all three edges are distinct fiducials. |
| Sweep accumulator | `adcsweep` struct (`control_ll.c`) | Host zeroes block, writes `req_n=N`; ISR accumulates N ticks of **pre-CM-removal** currents + Vbus (CM removal would smear each phase's edge artifact across all three); host polls `count==N`, reads 36 B. |
| Orchestrator | `tools/cmp4_sweep.sh run` | Single OpenOCD session, **no halt anywhere** (MEM-AP background access). Checks IDLE → disables shunt selection (restores after) → sets fixed mod → requests `OPEN_LOOP_CONTROL` via direct `fsm_requested_state` write → sweeps → returns to IDLE → restores everything. Needs OpenOCD ≥ 0.12 (`read_memory`). |
| Analyzer | `tools/cmp4_sweep_analyze.py` | Loads the dump dir, extracts `D` per edge, recommends `ADC_TRIG_CMP4_TICKS`, plots waveform + variance map. `--selftest` runs it on synthetic data with known D. |

## Procedure

1. **Build + flash a RelWithDebInfo image** containing the hooks (this branch):
   ```bash
   cmake --build cmake-build-relwithdebinfo -j
   openocd -f openocd.cfg -c "program cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf verify reset exit"
   ```
   **Not Debug** — the sweep arms the fast loop, and Debug (`-Og`, flash-resident) can't run it
   at 32 kHz: the ISR overruns its budget, SysTick (the IWDG feeder) starves, and the board
   watchdog-reboots ~0.5 s after arming — i.e. mid-sweep. The sweep script reads symbol
   addresses from the ELF (default `cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf`, override
   with `ELF=...`) — flashed image and ELF must match.
2. **Power at the bench supply**, current limit ~3 A at 8 V. Wheel must be in IDLE (fresh boot
   is fine; boot-time offset cal must have run, i.e. no error state).
3. **Run the sweep** (rotor will twitch once as it locks to the commanded vector — it holds a
   locked position for the whole sweep, ~2 A in the windings, sub-watt heating):
   ```bash
   tools/cmp4_sweep.sh run            # ~118 points × 32 ms dwell ≈ 30–60 s
   ```
   Defaults: `START=300 STOP=23700 STEP=200 DWELL=1024 MODA=0.0375 MODB=0.012` (env-overridable).
4. **Analyze**:
   ```bash
   python3 tools/cmp4_sweep_analyze.py cmp4_sweep_out/
   ```
   Prints the per-edge delay table + `RECOMMENDED: #define ADC_TRIG_CMP4_TICKS <n>u` and writes
   `cmp4_sweep_out/cmp4_sweep.png`. **Eyeball the plot**: dotted lines = true edge positions,
   dashed = where they appear; all three D's should agree within ~±100 ticks, and the
   recommendation should sit in the variance map's quiet plateau around the crest.
5. **Apply**: set `ADC_TRIG_CMP4_TICKS` in `Firmware/Inc/config.h`, rebuild, reflash. (The
   `hrtim.c` literal can stay — it's overridden at first arm.)
6. **Verify** (optional but cheap): re-run the sweep — the variance minimum should now be
   centered on the new value — and/or A/B the real objective: steady spin in `VEL_CONTROL`,
   `tools/foc_capture.sh arm/stop/dump`, compare `Iq_meas` noise before/after. The PSU's
   front-panel ammeter vs. TPDO3 bus-current telemetry is an independent cross-check.

## Safety / gotchas

- **Never halt the MCU while armed** — HRTIM free-runs with frozen compares during halt
  (`docs/foc_capture.md` "Dump halt safety" computes the ~40 A consequence). Everything here is
  non-halting background MEM-AP; keep it that way if you modify the script.
- The fixed-duty step is applied **without soft-start** — the rotor snaps to the vector at sweep
  start (≈2 A). Fine for a bench wheel; don't run it on flight hardware with a fragile load.
- `open_loop_fixed_mod_enabled` is **sticky** (deliberately not cleared by the law's init).
  The script clears it on teardown; if you abort mid-sweep, clear it manually before using
  `OPEN_LOOP_CONTROL` normally, or you'll get fixed duty instead of current control.
- Shunt selection is disabled during the sweep (it would rebuild the phase under test from the
  other two and erase the artifact). The script saves/restores the runtime flag.
- The accumulator taps **pre-CM-removal** currents, so the sweep sees each phase's raw artifact;
  the running control law (and everything else) still gets the normal processed currents.
- Sweep CSV-equivalents live in `cmp4_sweep_out/` as raw 36-B dumps (gitignored via `*.bin`);
  `meta.txt` records mod/dwell/range for the analyzer.

## Interpreting a weird result

- **Edges not found / D's disagree wildly** — raise `MODA` (bigger duty split → bigger, better-
  separated kicks), or widen the analyzer window (`--lead`). Check `duties.bin` actually shows
  three distinct duties (β must be nonzero).
- **Recommendation far from ~20800** — plausible if the assumed trigger latency was wrong, but
  >±1000 ticks (±217 ns) deserves suspicion: check the staircase width in the plot is
  ~3·1920 ticks (the 4× burst signature) — if it's a single sharp step, the oversampler isn't on.
- **Variance plateau not centered on the recommendation** — the fiducial fit and the noise map
  disagree; trust neither, look at the waveform plot, and consider sweeping with `STEP=50` around
  the crest region for finer resolution.
