# Errata and TODOs

Open firmware bugs (E) and requested changes (T) found after the 2026-06 flight review (`docs/flight-review-2026-06.md`). Short file refs such as `config.h:89` are under `Firmware/Inc/` or `Firmware/Src/`. Line numbers were current on 2026-11-03 (master `1116027`). When an item is fixed, delete it or mark it fixed with the commit.

## Errata

### E1. Bumpless velocity entry aliases above ~100 rev/s
Follow-on to flight-review P2-1. Found 2026-11-03.
- `vel_ctrl_init` and `vel_ramp_ctrl_init` seed the setpoint from `get_disarmed_vel()` (`closed_loop_controller.c:73`, `:120-123`).
- That value comes from `update_disarmed_velocity()` (`can_comms.c:182-235`). It differences two absolute encoder reads at least 5 ms apart and wraps the change to ±0.5 rev. Above 0.5 rev / 5 ms = 100 rev/s it aliases, and the limit is lower if the foreground loop runs slow.
- Result: re-entering VEL or VEL_RAMP on a wheel spinning faster than ~100 rev/s seeds a wrong speed and brakes at the 5 A clamp. A mid-spin reboot is the case the seed exists for. The regen goes into the bus, and OV (8.6 V) is only 0.2 V above a full 2S pack (E3).
- 0x4001 `motor_vel` reports the same aliased value while disarmed, so a coast-down can't be measured over CAN above ~100 rev/s.
- Fix: sample at ≥2 kHz while disarmed. No-load speed is ~740 rev/s at 8.4 V, so dt must stay under 0.68 ms.

### E2. Bus-voltage trips act on one unfiltered sample
Found 2026-11-03.
- `control_ll.c:213` scales one raw ADC sample per 32 kHz tick. `:251-254` compare it with the OV and UV thresholds. There is no filter and no count.
- One noise spike, or a dip of a few µs, disarms the wheel.
- Fix: trip only after a few consecutive samples (~0.1–0.3 ms). OV must still trip fast enough to protect the bridge.

### E3. OV margin above a full pack is 0.2 V
Found 2026-11-03.
- `DC_BUS_OVERVOLTAGE_THRESHOLD_V` is 4.3 × 2 = 8.6 V (`config.h:8`). A full 2S pack is 8.4 V.
- Full braking at top speed returns ~17–21 W, about 2–2.5 A, into the bus. At full charge, any source impedance above ~80 mΩ (pack + harness + EPS path) trips OV.
- If the battery path blocks reverse current (ideal diode or eFuse), only the bus caps absorb the regen, and OV trips at once.
- Question: does the EPS accept regen current? Set the OV threshold, or add a regen limit, to match the answer. Today there is no regen limit (E4).

### E4. `CTRLR_ERR_IBUS_OVERCURRENT` has the wrong name, and nothing limits bus current
Found 2026-11-03.
- `control_ll.c:264-269` compares the phase-current norm with `MOTOR_CURRENT_TRIP_LIMIT` (15 A) and raises bit 7, IBUS_OVERCURRENT. Bit 8, MOTOR_OVERCURRENT, is the matching name.
- `IBUS_OVERCURRENT_THRESHOLD_A` and `IBUS_OVERREGEN_THRESHOLD_A` (`config.h:13-14`) are never used, so nothing limits bus draw or regen. Flight-review P2-3 already called the bus-current protection dead code.
- Fix: raise bit 8 here. This changes what the host sees, so update cli.py and oresat-configs at the same time. Decide whether flight needs a bus or regen limit (E3).

### E5. cli.py is out of date with the firmware
Found 2026-11-03. **cli.py part fixed 2026-11-17 (v1.1.0):** bit 22 mirrored, `VEL_RAMP_RATE = 20.0`, and the old edge-trigger nudges removed. The nudge wrote `0` to 0x4004 before any repeat request, which current firmware answers with an SDO abort. oresat-configs still lacks bit 22.
- It has no error bit 22, `CTRLR_ERR_NOT_CALIBRATED` (`control_ll.h:58`). The enum stops at bit 21 (`can_console/cli.py:111`), so bit 22 prints as `UNKNOWN(0x400000)`. oresat-configs has the same gap (design review `sections/05-rw.md`, open question 6).
- `VEL_RAMP_RATE = 200.0` (`can_console/cli.py:154`) is stale. Firmware uses 20 rev/s² (`config.h:89`, 200 → 20 in e956f1b on 2026-06-09, before every tag). So `spin` reports "never reached" above ~111 rev/s while the wheel is still ramping, and `stop` can go to IDLE before the wheel slows down. If T1 lands, cli.py should read the rate back from the wheel instead of hardcoding it.

### E6. A reboot while the wheel spins shorts the motor for ~0.6 s with no current trip
Found 2026-11-17 by reading the code. Not seen on the bench.
- **Priority: low (Solomon, 2026-11-17).** Upstream e-fuses and current limits bound it. "Not great", but not a flight-safety problem. Fix it when convenient.
- At every boot, `inverter_amp_offset_correction()` (`inverter.c`, called from `app_main`) runs `inverter_start()`. That enables all six bridge outputs at 50/50/50 % duty for 20000 samples (~625 ms). It does this before the fast loop exists, so nothing watches the current.
- Equal duty on all legs is the zero vector. On a turning motor, that shorts the windings: hard braking, with current set by back-EMF / |R + jωL|. A rough estimate from `config.h` R/L and the 740 rev/s no-load speed is tens of amps peak near top speed. That is an inference, not a measurement.
- Mid-spin reboots are expected: watchdog, fault handler, 0x4007, CCM scrub, brownout. E1's bumpless seed exists for exactly this case.
- Second effect: the AC current lands in the offset average. The cal may then be biased, or rejected (±50 mV), which refuses arming until the next reboot.
- Possible fix (proposal): before the offset cal, check for rotation (two encoder reads, as `update_disarmed_velocity` does). If the wheel turns, wait for it to coast down, or calibrate with the bridge floated (`hrtim_outputs_disable()`, timers running). The floated cal misses the switching-induced offset the 50 % run captures. The self-test's `adc_mid` line prints both, so the bench can show how big that difference is.

## TODOs

### T1. Feature request: ramp rate settable over CAN
Requested by Max (PSAS), 2026-11-03.
- Today `VEL_RAMP_RATE` is compile-time (`config.h:89`). `closed_loop_controller.c:129` folds it into the const `MAX_VEL_DELTA_PER_LOOP`. No OD entry reaches it.
- Why: body torque in VEL_RAMP is J × ramp rate, and the velocity loop absorbs drag. A settable rate gives ADCS the "angular acceleration mode" Max asked for. Torque mode doesn't: it is open-loop Kt·iq with nameplate Kt and no drag feedforward.
- Smallest change:
  - Make `MAX_VEL_DELTA_PER_LOOP` a static volatile, initialized from `VEL_RAMP_RATE` at boot.
  - Add `set_vel_ramp_rate()` and declare it in `closed_loop_controller.h`. Reject non-finite and ≤ 0 values, and bound it (the 5 A clamp gives ~140 rev/s² at J ≈ 7.3e-6 anyway).
  - Route the write either through 0x4005 sub 2 (`setpoint_2`, stored but unused, `can_comms.c:346`) or through a new OD entry. A new entry is cleaner, but it adds to the OD divergence from oresat-configs (design review open question 6), so oresat-configs needs the same change.
  - Add a cli.py command and a line in `docs/can-interface.md`. The value resets to the default at reboot.
- Do T2 in the same change.

### T2. The velocity-loop integrator clamp is below the drag at high speed
2026-11-03.
- `VEL_INTEGRATOR_LIMIT` is 1 A·Kt ≈ 1.31 mNm (`config.h:93`). Drag at top speed is ~1.5–1.8 mNm. That's an upper bound from the bench power numbers.
- At top speed the integrator saturates. The P term (`VEL_GAIN`, 1 mNm per rev/s, `config.h:91`) covers the rest, which leaves ~0.2–0.5 rev/s of steady speed error. The offset itself is harmless, but a saturated integrator can't follow changes in drag.
- Fix: raise the clamp to ~2–3 A·Kt (2.6–3.9 mNm).
- No change needed: the integrator corner is Ki/Kp = 1 rad/s ≈ 0.16 Hz (`config.h:91-92`), so drag is trimmed out over a few seconds. That's fine for ADCS.
- Optional: J·α feedforward in VEL_RAMP. Without it, each ramp starts with a ~1 s lag of ~0.9 rev/s (at 20 rev/s²) while the integrator builds up the J·α term. It's a cleanup, not a flight need. It needs a J value, and J is only estimated today: Max has 7.1–7.3e-6 kg·m² for the brass wheel alone, without the rotor or magnet, and `docs/encoder-lut-design.md:203` still guesses 2e-5.

### T3. Make the SWD variable read/modify tooling good
Solomon, 2026-11-03.
- Today `tools/swd_sdo.sh` covers only state, setpoint, status and reboot. It copies the `od_write_*()` checks by hand ("keep it in sync"), and it assumes the ELF matches the flashed image.
- Gaps:
  - No generic read/write of any global by name, with the type taken from the ELF (f32/u32/bool, struct members).
  - Compile-time `static const` tunables have no symbol to poke, because the compiler folds them in: `VEL_RAMP_RATE`, `VEL_GAIN`, `VEL_INTEGRATOR_LIMIT`, the bus thresholds. Live tuning needs them as runtime variables, which overlaps with T1–T2.
  - No automatic check that the ELF matches the flashed image.

### T4. Live data capture for Max (pinned, not now)
Solomon, 2026-11-17. Parked on purpose; don't start it without asking.
- Max asked for "live profiling". Solomon's read: he wants something like `foc_capture` and doesn't know how to ask for it. Step one is showing him how to use what exists: `tools/foc_capture.sh` + `plot_foc_capture.py` (per-tick capture, SWD-only), and `cli.py monitor` for the slow CAN telemetry.
- Ideas for later:
  - Choose at runtime what gets captured or plotted, instead of a fixed record.
  - Stream over SWO in real time at some decimation.
  - And/or keep a RAM buffer for full-rate, non-real-time capture.
