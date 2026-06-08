# FOC capture — per-tick scope of the inner loop

A one-shot, every-tick snapshot of FOC inner-loop state. Use it for current-PI tuning, encoder
phase verification, modulation-saturation hunting, deadtime-distortion analysis — anything where
SWO bandwidth is too tight to stream all the variables you want.

The capture lives in `Firmware/Inc/foc_capture.h` + `Firmware/Src/foc_capture.c`; the write site is
at the end of `foc_gen_pwm_cb()` in `Firmware/Src/foc.c`. Dump tooling is in `tools/`.

## What gets captured

Each fast-loop tick (16 kHz, when armed) writes one `foc_capture_t` to the buffer. Layout, in order
— **must stay in sync with `tools/decode_foc_capture.py`'s `DTYPE`**:

| Field | Type | Unit | Source |
|---|---|---|---|
| `loop_count` | u32 | — | `controller_get_loop_count()` at capture time; monotonic, ISR-incremented. Use to align across runs. |
| `Ia`, `Ib`, `Ic` | f32 ×3 | A | Phase currents *after* zero-sum common-mode subtraction (`control_ll.c:164-167`). Stored from `foc_meas_current_cb()`. |
| `Ialpha`, `Ibeta` | f32 ×2 | A | Clarke output. |
| `Id_meas`, `Iq_meas` | f32 ×2 | A | Park output (this tick's measurement, not filtered). |
| `Id_set`, `Iq_set` | f32 ×2 | A | Current setpoint (d-axis is always 0 in normal FOC; q-axis from torque controller). |
| `motor_phase` | f32 | rad | Electrical angle used for *measurement* Park this tick. |
| `motor_vel` | f32 | rad/s | Electrical angular velocity from encoder estimator. |
| `Vd_int`, `Vq_int` | f32 ×2 | V | PI integrator state at end of this tick. |
| `mod_d`, `mod_q` | f32 ×2 | (0..1) | PI output normalized by `mod_to_V = 2/3·Vbus`. |
| `mod_alpha`, `mod_beta` | f32 ×2 | (0..1) | Inverse Park output (modulation in α/β frame). |
| `svm_A`, `svm_B`, `svm_C` | f32 ×3 | (0..1) | SVM output → fed straight to `hrtim_pwm_set_dcycle_f32()`. |
| `Vbus` | f32 | V | Bus voltage at sample time. |

Total **88 bytes per sample**. Buffer is `FOC_CAPTURE_LEN = 256` samples = **22.5 KB**, **16 ms at
16 kHz** (giving a couple ms of margin past the typical 10 ms tuning window).

## How to use it

### 1. Arm the capture

Edge-triggered via OD `0x4006.reserved_1` (any nonzero f32 that differs from the previous value
arms a new run; see `can_comms.c:process_sdo_vals`). From a CANopen host:

```
SDO download 0x4006:01 = 1.0   # arms; idx resets to 0, state → FOC_CAPTURE_ARMED
```

If you want to re-arm later in the same session, write a *different* value (e.g. `2.0`, then `1.0`,
then `2.0`, …) — `process_sdo_vals` only acts on changes.

### 2. Wait for it to fill

256 samples × 62.5 µs/tick = **16 ms**. After that:
- `foc_capture_state == FOC_CAPTURE_DONE` (= 2)
- `foc_capture_idx == 256`

You can poll either via OpenOCD memory read, or just wait 20+ ms before dumping. State and index
live next to the buffer:

```
$ arm-none-eabi-objdump -t cmake-build-debug/oresat-acs-fw-v8.elf | grep foc_capture
... O .bss 00000001 foc_capture_state    # 1-byte enum
... O .bss 00000004 foc_capture_idx
... O .bss 00005800 foc_capture_buf      # 22528 bytes
```

### 3. Dump from target

```
$ tools/foc_capture_dump.sh
foc_capture_buf @ 0x20001f74, size 22528 bytes -> foc_capture.bin
wrote foc_capture.bin
```

The script looks up the buffer's address + size from the ELF symbol table, halts the target via
OpenOCD, dumps `dump_image`, resumes. **Briefly halts the target** (~few hundred ms) — fast loop
will resume from a fresh ISR after resume.

Override defaults:
```bash
tools/foc_capture_dump.sh path/to/oresat-acs-fw-v8.elf my_capture.bin
OPENOCD_CFG=other_iface.cfg tools/foc_capture_dump.sh
```

### 4. Decode + plot

```bash
$ python3 tools/decode_foc_capture.py foc_capture.bin > foc.csv
$ head -2 foc.csv
loop_count,Ia,Ib,Ic,Ialpha,Ibeta,Id_meas,Iq_meas,Id_set,Iq_set,motor_phase,motor_vel,Vd_int,Vq_int,mod_d,mod_q,mod_alpha,mod_beta,svm_A,svm_B,svm_C,Vbus
40213,0.123,-0.456,0.333,...
```

The decoder trims trailing all-zero records (zero `loop_count` ⇒ unused slot), so it's safe to dump
mid-capture if you don't want to wait.

Quick matplotlib recipe:
```python
import pandas as pd, matplotlib.pyplot as plt
df = pd.read_csv('foc.csv')
t = (df.loop_count - df.loop_count.iloc[0]) / 16000.0  # seconds since first sample
fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot(t, df.Iq_meas, label='Iq meas'); ax[0].plot(t, df.Iq_set, '--', label='Iq set'); ax[0].legend()
ax[1].plot(t, df.Ia, t, df.Ib, t, df.Ic); ax[1].set_ylabel('phase A/B/C [A]')
ax[2].plot(t, df.svm_A, t, df.svm_B, t, df.svm_C); ax[2].set_ylabel('SVM A/B/C')
plt.show()
```

## Cost in the fast loop

When `foc_capture_state != FOC_CAPTURE_ARMED` the entire hook is one branch (~2 cycles ≈ 14 ns).
While armed: 88-byte struct copy ≈ 22 word writes + index update + bounds check ≈ **~175 ns** per
tick, < 0.3 % of the 62.5 µs ISR budget. Has not been observed to perturb FOC behavior.

## When you add or remove a captured field

Three places must move together — the compiler will catch the C side, but the Python side is on the
honor system:

1. `Firmware/Inc/foc_capture.h` — add the f32 to `foc_capture_t` (keep all fields 4-byte for no
   padding). New field count → new total size (88 → 92 → ...).
2. `Firmware/Src/foc.c` — set the field in the designated-init at the bottom of `foc_gen_pwm_cb`.
3. `tools/decode_foc_capture.py` — add the field to `DTYPE` *in the same order*, then update the
   `assert DTYPE.itemsize == 88` to the new size. This assert is the trip wire: if the dtype gets
   out of sync, the Python loud-fails on import. If you leave it in sync but in the wrong order,
   you get silently-misaligned values — **so add fields at the end of both the struct and the dtype,
   never in the middle**.

## Bench workflow checklist

For a current-PI tuning session, the canonical sequence:

1. Build + flash.
2. Bring the wheel up to the desired test state via SDO (`0x4004` = e.g. `4` for TORQUE_CONTROL,
   then `0x4005:01` = small Iq setpoint).
3. Let it settle.
4. Step the setpoint (write a new `setpoint_1`), then *immediately* SDO-write `0x4006:01 = 1.0` to
   arm the capture.
5. Wait ≥ 20 ms.
6. `tools/foc_capture_dump.sh`.
7. `python3 tools/decode_foc_capture.py foc_capture.bin > step.csv`, plot, look at the step
   response of `Iq_meas` vs `Iq_set`, integrator wind-up in `Vq_int`, modulation saturation in
   `mod_d`/`mod_q` or in the `0.99 * sqrt3_over_2 * |mod|⁻¹` scalefactor (deducible from comparing
   `mod_d/q` to `mod_alpha/beta`).

For deadtime / waveform analysis: same recipe, but step into a steady spin at the speed of
interest and capture during steady-state. The 22 SVM/mod/phase-current traces should reveal any
6th-harmonic deadtime distortion as 6 cycles per electrical revolution in `Ia/Ib/Ic`.

## Limitations / known footguns

- **One-shot only.** No ring buffer mode; once the buffer fills, captures stop until re-armed. If
  you want pre-trigger context (capture the *N* ms *before* a fault), the hook needs to become a
  ring + a separate "snapshot the head pointer" trigger. Not implemented yet.
- **No timestamping beyond `loop_count`.** Wall-clock alignment to host events requires the host to
  know the wheel's tick rate (16 kHz) and the offset of `loop_count` at the moment it sent the arm
  command. In practice "arm + step setpoint simultaneously" works fine; for tighter sync you'd need
  a clock-sync mechanism that doesn't exist here.
- **Capture is at the end of `foc_gen_pwm_cb`**, after the integrator updates and modulation clamp.
  The PI integrator state captured is *post-update for this tick*. If you want to compare against
  the pre-update value, look at the previous sample's `Vd_int`/`Vq_int`.
- **No native FSM hookup yet.** Re-arming is only via SDO. Could be triggered from `controller_arm`
  or on first error if you want pre-fault context — easy addition.
- **Edge-triggered SDO.** Writing the same numeric value to `0x4006:01` twice does *not* re-arm.
  Alternate values (e.g. 1 → 2 → 1 → 2) or use any monotonically increasing value.

## See also

- `docs/control-cycle.md` — where in the tick the capture write happens.
- `docs/can-interface.md` — SDO mechanics for the arm trigger; OD `0x4006:01` was "reserved" before
  this doc, now it's the capture-arm.
- `Firmware/Inc/foc_capture.h` — the source of truth for struct layout.
