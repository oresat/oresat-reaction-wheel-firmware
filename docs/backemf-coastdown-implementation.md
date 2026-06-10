# Back-EMF coast-down calibration — firmware implementation plan

> **Status: IN PROGRESS** (plan captured 2026-06-08; build started 2026-06-09).
> **→ See §10 for live as-built status & next steps.** In short: the hi-Z output split and the
> phase-voltage accessor are in-tree (build/lint clean); enabling injected rank 2 is a pending
> CubeMX `.ioc` edit (§3.1); the coast control law (§4) is the next firmware piece.
>
> Companion to **`docs/backemf_coastdown_calibration.md`** (the *method & theory* — why
> `v_d → 0` at open circuit, the `delta = atan2(s·v_d, s·v_q)` closed form, the
> `delta = Δ_true + k·ω_e` regression). **This** doc is *how to build it in this codebase*:
> the inverter-off mechanism, the phase-voltage ADC change, the coast control law, the capture
> path, and how the same machinery also produces the encoder LUT
> (**`docs/encoder-lut-design.md`**).
>
> The hardware is already provisioned for it (see §1).

---

## TL;DR

- **The board is already built for this.** Each current-sense ADC has a second, unused channel
  on that phase's voltage divider (`PHA_V`/`PHB_V`/`PHC_V` on ADC1/ADC3/ADC2 — same ADCs as the
  shunts). They're pin-assigned but not in the injected sequence. Adding them as **injected
  rank 2** gives three phase voltages sampled in parallel (synchronously across phases).
- **You do not need to rearchitect the control law.** "Inverter off / hi-Z" already exists — it's
  what `inverter_stop()` does (no gate-enable pin exists; off = HRTIM outputs inactive). The
  *only* problem is that disarm bundles **output-off + counter-stop + ISR-unregister** together.
  Split out an **outputs-only** disable that leaves the HRTIM counters, ADC triggers, and the
  16 kHz repetition ISR running, and the fast loop keeps sampling voltages + encoder while the
  bridge floats.
- The coast measurement runs as a normal phased `control_law_t`: spin up (delegate to FOC +
  velocity loop), then disable outputs and log `(θ_m, ω_e, v_a, v_b, v_c)` during the coast,
  feeding the loop's validators **benign dummies** (a valid SVM, consumed currents).
- **Build v1 as log-only + offline analysis** (matches the method doc §4.4, your bench/SWD
  workflow, and the `foc_capture` precedent). Most of it is writable with no hardware in the loop.
- **Synergy:** the same coast capture yields the **scalar offset** (regress to the `ω_e = 0`
  intercept) *and* the **encoder LUT** (back-EMF gives a true electrical angle per sample → the
  encoder-vs-true error map). The back-EMF map is a *better* LUT source than the constant-velocity
  fit in `encoder-lut-design.md` because it needs no constant-velocity assumption (§7).

---

## 0. Relationship to the other docs

| Doc | Role |
|---|---|
| `backemf_coastdown_calibration.md` | The **method**: derivation, `delta` formula, regression, pitfalls. Provider-agnostic. |
| **this doc** | The **implementation** on STM32G474 + this firmware: HRTIM/ADC/control-law/capture. |
| `encoder-lut-design.md` | The **angle-error LUT**. §7 here shows the back-EMF coast supersedes its constant-velocity capture method. |
| `control-laws.md`, `control-cycle.md` | The control-law plug-in contract + fast-loop walk this must live inside. |
| `foc_capture.md` | The ring-capture/SWD-dump pattern the §5 capture path mirrors. |

The **scalar back-EMF offset** replaces the static d-axis lock as the *precision* offset step;
the existing (now bidirectional, applied) phase-cal becomes the **rough bootstrap** that gets the
motor spinning. See `encoder_phase_cal_*` in `encoder_calibration.c`.

---

## 1. Hardware inventory — what the board already gives us

### 1.1 Phase-voltage senses (the enabler)

All three exist, each on the **same ADC** as that phase's shunt, and all currently **unused**
(pin-assigned in the `.ioc`, but `InjectedNbrOfConversion = 1` so only the shunt on rank 1 is
sampled; `inverter_get_adc_raw` reads `JDR1`, `inverter.c:216`).

| Phase | Shunt (rank 1, JDR1) | Phase voltage (to add) | ADC handle |
|---|---|---|---|
| A | `PA0` PHA_I, ADC1_IN1 | `PA2` **PHA_V, ADC1_IN3** | `hadc1` (`PHASE_A_ADC_HANDLE`) |
| B | `PB1` PHB_I, ADC3_IN1 | `PE9` **PHB_V, ADC3_IN2** | `hadc3` (`PHASE_B_ADC_HANDLE`) |
| C | `PA1` PHC_I, ADC2_IN2 | `PA7` **PHC_V, ADC2_IN4** | `hadc2` (`PHASE_C_ADC_HANDLE`) |
| Bus | — | `PE14` VBUS_V, ADC4_IN1 | `hadc4` (`VBUS_ADC_HANDLE`) |

Because the three voltages live on three different ADCs all triggered off the same HRTIM injected
trigger (`ADC_EXTERNALTRIGINJEC_HRTIM_TRG2`), they convert **in parallel → mutually
synchronous**, satisfying §5's "sample all three simultaneously."

### 1.2 The bridge / hi-Z

Six HRTIM outputs, complementary pairs: `TA1/TA2` = PHA_H/L (`PA8/PA9`), `TB1/TB2` = PHB_H/L
(`PA10/PA11`), `TE1/TE2` = PHC_H/L (`PC8/PC9`). **There is no gate-driver enable GPIO** — the
only off-mechanism is the HRTIM output stage. `hrtim_pwm_stop()` already sets all six to
`HRTIM_OUTPUTLEVEL_INACTIVE` (`pwm.c:78-83`) → both FETs off → phase floats. So **true open
circuit is already achievable**; we just need it *without* also stopping the timers (§2).

### 1.3 Safety we already have

`adc_update_callback` raises `CTRLR_ERR_VBUS_OVERVOLTAGE` at `DC_BUS_OVERVOLTAGE_THRESHOLD_V`
(8.6 V) every tick (`control_ll.c:205`). That **is** the over-voltage abort the method doc asks
for during coast (a freewheeling motor pumps the bus through the body diodes if spun too fast).
It disarms on trip. Keep the coast speed under the body-diode bound (method doc §5) so it's a
backstop, not a normal exit.

---

## 2. Core insight: decouple "outputs off" from "loop stopped"

This is the crux, and it's a small change rather than a teardown.

### 2.1 What disarm bundles (and why it kills the measurement)

`controller_disarm()` (`control_ll.c:353`) does:
1. `inverter_stop()` → `hrtim_pwm_stop()` → `WaveformCountStop(A,B,E)` **+** `WaveformOutputStop(6)`,
2. unregister the repetition-event ISR (`HAL_HRTIM_TIMxUnRegisterCallback`).

Stop the counters and unregister the ISR and you lose the HRTIM ADC triggers *and* the 16 kHz
loop — no more sampling. So "disarm and keep reading" is exactly what we **can't** do.

### 2.2 What we need instead

Keep the counters running (ADC injected triggers keep firing), keep the repetition ISR
registered (fast loop keeps running), but **disable only the six output stages** so the bridge
floats. HRTIM separates these natively — `WaveformOutputStop` (output pins) is independent of
`WaveformCountStop` (counters), and the HRTIM ADC-trigger/repetition events come from the timer
compare events, **not** the output stage.

Add to `pwm.c` (factored out of the existing start/stop):

```c
// Float the bridge (all 6 FETs off) WITHOUT touching the timers/ISR/ADC triggers.
__fn_inline void hrtim_outputs_disable(void) {
    HAL_HRTIM_WaveformOutputStop(&hhrtim1,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    // (optional belt-and-suspenders: SetOutputLevel(... INACTIVE) on the 6, like hrtim_pwm_stop)
}
__fn_inline void hrtim_outputs_enable(void) {
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, /* same 6 outputs */);
}
```

`WaveformOutputStop` takes effect immediately (no `SoftwareUpdate` needed to float the bridge).
The per-tick `hrtim_pwm_set_dcycle_f32()` writes in `pwm_update_callback` keep running but are
inert while outputs are off (they only set compare registers).

### 2.3 Satisfying the control-law contract while coasting

The fast loop, while `controller_armed`, will disarm if the producer/consumers misbehave
(`pwm_update_callback` validates the SVM → `CTRLR_ERR_MODULATION`/`_PWM_TIMING_INVALID`;
`adc_update_callback` wants currents consumed/valid/in-limit). A coast law feeds it harmless
values:

- `pwm_producer` → `{0.5, 0.5, 0.5}` — a valid SVM; outputs are off so the compares don't matter.
- `current_consumer` → `return true` — consumes the (≈0) currents; no overcurrent, raw ≈ midscale
  2048 so validity passes.
- `encoder_consumer` → `encoder_update()` as normal.

So we **stay armed** (loop alive, consumers firing) with the bridge floated. No infra changes
beyond §2.2.

---

## 3. Phase-voltage acquisition (ADC injected rank 2)

### 3.1 The change

Add the `PHx_V` channel as **injected rank 2** on ADC1/ADC3/ADC2 (`InjNumberOfConversion = 2`),
keeping the shunt on rank 1. Read it from `JDR2`.

> **⚠ Verified constraint (the gotcha that picks the method):** these ADCs have
> `Init.ScanConvMode = ADC_SCAN_DISABLE` (`adc.c:61,151,234`). `HAL_ADCEx_InjectedConfigChannel`
> then takes its "scan disabled" branch — *"Injected sequence length set to 1 channel; parameter
> InjectedNbrOfConversion is discarded"* — and **silently ignores any rank > 1**. So you cannot
> just append a rank from a `USER CODE` block or a runtime call; **`ScanConvMode` must be
> `ADC_SCAN_ENABLE`** (which forces a re-`HAL_ADC_Init`). That re-init touches the safety-critical
> current-sense path, so don't do it blind — drive it from the `.ioc`.

Ways, in order of cleanliness:

1. **CubeMX `.ioc` edit + regen** (now clearly the right path). Setting the injected group to 2
   conversions makes CubeMX flip `ScanConvMode = ENABLE`, generate the correct two-rank
   `HAL_ADCEx_InjectedConfigChannel` sequence, *and* set the `PHx_V` pins to analog — all the
   pieces the manual routes have to get right by hand. `Core/` is generated anyway (`CLAUDE.md`).
2. **Runtime reconfig in app code** (e.g. extend `inverter_adc_init`): set
   `ScanConvMode = ENABLE`, re-`HAL_ADC_Init`, then config regular rank 1 + injected ranks 1&2,
   before `HAL_ADCEx_InjectedStart`. Survives regen, but re-inits a safety-critical ADC — only
   with bench validation that rank-1 current sensing is unchanged.
3. **Hand-edit the generated `MX_ADCx_Init`** (set `ScanConvMode`, `NbrOfConversion`, add the
   rank). Works, but a regen reverts it — like the existing 4× oversampling edit (`adc.c:114`).

Recommend (1). Make it **permanent** (always sampled) — negligible cost, and the phase voltages
become available for other diagnostics (sensorless bring-up, dead-time distortion). The accessor
`inverter_get_phase_voltages_raw()` (reads `JDR2`) is already in `inverter.c`; it returns
stale/zero until rank 2 is actually in the sequence.

### 3.2 Sample-time & oversampling caveats

- The shunt rank uses **2.5-cycle** sample time (`inverter.c` even flags this as marginal for the
  INA240 source). The `PHx_V` dividers are higher-Z; **give rank 2 a longer injected sample
  time** (e.g. 47.5 cy, as Vbus/ADC4 already uses) so it settles. Sample time is per-rank
  (`InjectedSamplingTime`), so this doesn't touch the shunt.
- **Injected oversampling is shared across ranks** (it's an ADC-wide setting; currently 4×, shift
  2). Rank 2 inherits 4× — good for back-EMF SNR, but it ~doubles injected-sequence time. Rough
  budget: rank1 (2.5+12.5)cy×4 ≈ 1.7 µs, rank2 (47.5+12.5)cy×4 ≈ 6.7 µs at ~36 MHz ADC clock.
  Both ADCs run in parallel, and the 62.5 µs ISR has room — **but verify `JDR2` is ready before
  `exec` reads it.**
- **Escape hatch if timing is tight:** read `JDR2` *one tick late* (use the previous tick's
  voltage). That's a fixed encoder↔voltage skew, which the method folds into the slope `k` and
  cancels in the intercept. So it costs nothing for the offset (and a constant phase for the LUT,
  removed bidirectionally).

### 3.3 Accessor & scaling

```c
__ccmram void inverter_get_phase_voltages_raw(u32 *va, u32 *vb, u32 *vc) {
    *va = (&PHASE_A_ADC_HANDLE)->Instance->JDR2;
    *vb = (&PHASE_B_ADC_HANDLE)->Instance->JDR2;
    *vc = (&PHASE_C_ADC_HANDLE)->Instance->JDR2;
}
```

Scaling: the `PHx_V` divider ratio is **unknown from the `.ioc`** (check the schematic — likely
distinct from `VSNS_VOLTAGE_DIVIDER`). Good news: **the offset angle is ratiometric**, so the
gain doesn't bias `Δ_true` — only **channel-to-channel matching** matters (mismatch distorts the
αβ circle) and **filter group-delay** (lands in `k`). You only need the absolute scale for the
SNR gate and any Vbus-vs-back-EMF comparison. Sample each channel's **zero** with the system
de-energized (method doc §5).

### 3.4 Time alignment

The encoder is read via SPI (started at ISR top, finished mid-ISR, `control_ll.c:291,313`); the
voltage is the injected conversion at the HRTIM trigger point. Fixed skew within the tick → folds
into `k`. During coast there's **no PWM switching**, so the sample *instant within the cycle*
doesn't matter for accuracy (no ripple to dodge) — only the encoder↔voltage skew, handled as
above.

---

## 4. The coast control law

A new phased `control_law_t` (e.g. `backemf_cal_ctrl_law` in `encoder_calibration.c` or a new
`backemf_cal.c`) + a new FSM state (e.g. `FSM_STATE_BACKEMF_OFFSET_CAL`). Most of it **reuses
existing FOC + velocity machinery**; only the coast phase is new.

### 4.1 Phases

```
SPINUP_FWD  -> FOC velocity control to +ω_cal (outputs ON)
COAST_FWD   -> hrtim_outputs_disable(); settle ~1-2 ms; log until |ω_e| < ω_min
SPINUP_REV  -> hrtim_outputs_enable(); FOC velocity control to -ω_cal     [recommended]
COAST_REV   -> hrtim_outputs_disable(); settle; log
DONE        -> is_complete() = true; finish() stops capture (v1) / regresses+applies (v2)
```

### 4.2 Behavior per callback (single `phase` flag read by all)

| Callback | SPINUP_* | COAST_* |
|---|---|---|
| `encoder_consumer` | `encoder_update()` | `encoder_update()` |
| `current_consumer` | delegate → `foc_control_law.current_consumer` | `return true` (consume; I≈0) |
| `pwm_producer` | delegate → `foc_control_law.pwm_producer` | `return {0.5,0.5,0.5}` (inert) |
| `control_law_exec` | run velocity PI → `set_torque_setpoint()` (≡ `vel_ctrl_exec`) | read V_abc(`JDR2`)+enc+ω → log; manage coast timing/transition |
| outputs | enabled | **disabled (hi-Z)** |

`control_law_init` → `init_foc()` + reset phase to `SPINUP_FWD` + zero the capture ring. The
spin-up is literally the existing `vel_ctrl` path (`closed_loop_controller.c:73`,
`set_torque_setpoint`), so the only genuinely new code is the coast logging + the phase machine.

### 4.3 Transitions

- **SPINUP→COAST:** when `|ω_e| ≥ ω_cal` (or after a fixed spin-up time), in `exec` set
  `phase = COAST_*`, call `hrtim_outputs_disable()`, stamp `coast_start_loop`. Same tick's
  `pwm_producer` (runs later, `control_ll.c:321`) then returns benign — fine.
- **Settle:** ignore the first ~1–2 ms of coast (let switching transients / diode ringing die)
  before logging — same idea as the phase-cal cruise gate.
- **COAST→SPINUP_REV:** `hrtim_outputs_enable()`, resume velocity control with negative setpoint.
- Gate logging on `ω_min ≤ |ω_e| ≤ ω_max` and optionally a min voltage-vector magnitude (method
  doc §4.2).

### 4.4 Bootstrap dependency

Spin-up needs a **rough** offset + correct **direction** already set (FOC won't spin efficiently
otherwise). Run the existing dir-cal + (improved) phase-cal first. The coast-down then does the
precision work and is insensitive to how rough the bootstrap was. If even rough FOC spin-up is
unreliable through cogging, fall back to **open-loop I/f** ramp for spin-up only (the measurement
doesn't care how it got spinning).

---

## 5. Capture path (v1: log-first, analyze offline)

Mirror `foc_capture` (`Firmware/{Inc,Src}/foc_capture.*`, `tools/foc_capture.sh`): a small
arm/run/stop ring written from the coast `exec`, dumped over SWD. **Do not** reuse the FOC ring
(it stores *electrical* angle, 16 ms deep) — this needs raw mechanical angle + the three phase
voltages over a multi-second coast.

Per-sample (`backemf_capture_t`):

| Field | Type | Notes |
|---|---|---|
| `loop_count` | u32 | align/timebase |
| `theta_m` | f32 | mechanical angle (or raw 0..1) |
| `omega_e` | f32 | signed electrical speed |
| `v_a, v_b, v_c` | f32 ×3 | phase voltages (raw or scaled) |

≈ 24 B/sample. **Decimate** (e.g. 1–2 kHz) — the offset regression needs only hundreds–low
thousands of samples spanning the speed range. ~1024–2048 samples = 24–48 KB; fits free RAM (the
build reports ~40 KB free; 16 KB **RAM2** is available but too small alone). For the **LUT** use
(§7) you want full angular coverage over many revs → size up / decimate-by-angle and note the
RAM cost. Drive arm/stop/dump over SWD exactly like `foc_capture.sh`; auto-stop on FSM exit.

Offline keeps the tricky math off the MCU and lets you *see* the data before trusting it (method
doc §4.4) — and since HW testing is deferred here, you can write all of this now and analyze later.

---

## 6. Offline analysis — `tools/backemf_cal_fit.py`

Mirror `tools/decode_foc_capture.py`. Steps (all per method doc §2–§4):

1. Unwrap ring; scale voltages; subtract per-channel zero.
2. Clarke (amplitude-invariant) → `v_alpha, v_beta`. **Plot the αβ circle vs encoder angle** —
   the first validation (clean circle ⇒ scaling/phase-order/filter-match OK).
3. Per sample: `theta_e = p·theta_m − theta_off_rough`; Park → `v_d, v_q`;
   `delta = wrap_pi(atan2(s·v_d, s·v_q))`, `s = sign(omega_e)`.
4. Pool fwd+rev; **regress `delta` vs signed `omega_e`** → intercept `Δ_true` (the offset
   correction) + slope `k` (lumped delay diagnostic). Plot it — should be a straight line.
5. Forward-only vs reverse-only intercept agreement (≤ ~1°) as a sanity gate.
6. Emit the offset; the **empirical sign check** (method doc §7) is mandatory once.

---

## 7. LUT unification (the big synergy)

The same coast capture builds the **encoder LUT**, and does it *better* than the constant-velocity
method in `encoder-lut-design.md`:

- The back-EMF vector gives the **true electrical angle directly, per sample**:
  `(v_alpha, v_beta) = psi·omega·(−sin θ, cos θ)` ⇒ `theta_true_e = atan2(v_beta, v_alpha) − π/2`
  (adjust for `sign(omega)`). **No constant-velocity assumption** — so cogging speed ripple is
  irrelevant *in principle*, not just suppressed by inertia.
- Per sample you then have `(theta_meas from encoder, theta_true from back-EMF)`. Binning
  `wrap(theta_true − theta_meas)` by **mechanical** measured angle across the coast (which sweeps
  all positions) **is** the encoder error map `e(θ)` → the LUT.
- The speed-dependent delay `k` shifts the whole map by `k·ω`; the **bidirectional** coast
  removes it (fwd/rev shift opposite, average aligns — same mechanism that gives the offset its
  intercept).

So: **one capture → scalar offset (regress to intercept) + LUT (bin angle error by position).**
Caveats: back-EMF SNR fails at the low-speed coast tail (gate it out; rely on higher-speed passes
through each position bin), and the high end is capped by the body-diode bound. Within that window
the angle reference is absolute and clean.

**Recommendation:** make this the unified encoder-cal path and supersede §3–§4 of
`encoder-lut-design.md` (its runtime LUT *apply*, table format, persistence, and `encoder_update`
integration in §5 all still stand — only its *capture/fit* method is replaced by this).

---

## 8. Integration points (file:line)

- `pwm.c` — add `hrtim_outputs_enable/disable()` (factor from `hrtim_pwm_start` `:23-27` /
  `hrtim_pwm_stop` `:59-83`). Per-tick `hrtim_pwm_set_dcycle_f32` `:120`.
- `control_ll.c` — fast-loop order `:291-321`; `pwm_update_callback` `:86` (SVM validate/disarm);
  `adc_update_callback` `:131` (currents + `:205` over-voltage); `controller_disarm` `:353`
  (the double-stop hazard, §9); `controller_arm` `:335`.
- `inverter.c` — `inverter_get_adc_raw`/`JDR1` `:216`; add `inverter_get_phase_voltages_raw`
  (`JDR2`); ADC trigger map comment `:40-44`; `inverter_start/stop` `:272/:279`.
- `Core/Src/adc.c` + `oresat-acs-fw-v8.ioc` — injected rank 2 on ADC1/3/2 (§3.1). Generated;
  prefer the `.ioc`.
- `closed_loop_controller.c` — reuse `vel_ctrl_exec` `:73` / `set_torque_setpoint` `:40` for
  spin-up; `init_foc` via `torque_ctrl_init` `:31`.
- `encoder.c` — `encoder_get_phase` `:95` convention (`phase = wrap(2π·p·(pos − offset))·dir`,
  `:103`); offset is mechanical-turns × `dir` (your phase-cal's rad↔turns conversion ports over).
- `fsm.c` / `fsm.h` / `docs/control-laws.md` — register the new state.
- `foc_capture.*` + `tools/` — the capture/dump/decode pattern to clone.

---

## 9. Gotchas / footguns (codebase-specific)

- **Disarm double-stop — verified benign.** If the coast disabled outputs and then an error
  trips, `controller_disarm` → `hrtim_pwm_stop` calls `WaveformOutputStop` on already-stopped
  outputs. Checked the HAL: it's just `ODISR |= mask` and always returns `HAL_OK`, so the
  `assert(... == HAL_OK)` won't fire. No special handling needed. (`hrtim_outputs_enable/disable`
  added in `pwm.c`; `enable` re-asserts `OENR`, which clears the matching `ODISR` disable.)
- **ADC ScanConvMode gates injected rank 2** (§3.1) — the HAL discards rank > 1 unless
  `ScanConvMode = ENABLE`. Enable via the `.ioc`; don't blind-edit the current-sense ADC init.
- **`JDR2` readiness** vs `exec` timing (§3.2). Verify, or read one tick late (skew → `k`).
- **Rank-2 settling:** the 2.5-cy shunt sample time is too short for the voltage divider — give
  rank 2 a longer sample time (§3.2).
- **Body-diode conduction** bounds coast speed from above (method doc §5): keep
  `ω_max < (V_bus − margin)/(√3·psi_m)`. The existing over-voltage trip (`:205`) is the backstop,
  but design under the bound so it doesn't fire normally (and you don't dump regen).
- **Cross-phase sampling instant:** the three current-sense ADCs trigger off different sub-timer
  compares (B3/F3/E3 per `inverter.c:40`). For coast (no switching) the instant barely matters,
  but confirm the compares are equal so the three voltages are mutually simultaneous.
- **Bootstrap/direction must precede** (§4.4). FOC spin-up needs a rough offset + correct dir, or
  use I/f.
- **Persistence is absent** (same as the LUT/offset story): `phase_offset`/`encoder_dir` are RAM
  statics (`encoder.c:25-26`), lost on reboot; only option bytes are flashed. The back-EMF offset
  lands in the same cal-flash-page decision as the LUT (see `encoder-lut-design.md` §5.6).
- **Keep `printf` out of the coast loop** — blocking UART in the fast path is a known footgun
  (`docs/footguns.md`); log to the ring, dump over SWD.

---

## 10. Phased plan & as-built status

**Status as of 2026-06-09** (all firmware + tooling written; bench validation is the live step):

1. ✅ **DONE — `hrtim_outputs_enable/disable()` split** (`pwm.c` + `pwm.h`). Floats the six
   outputs while leaving the HRTIM counters, repetition ISR, and ADC triggers running. Now
   implemented as **direct `ODISR`/`OENR` register writes** (not the HAL calls): `disable` is
   called from the fast-loop ISR, and the HAL version takes `__HAL_LOCK` — a held handle lock
   would make it silently no-op (`HAL_BUSY`) and the bridge would never float. Idempotent.
2. ✅ **DONE — phase-voltage acquisition.** Injected **rank 2 enabled by MANUAL EDITs in
   `Core/Src/adc.c`** (not the `.ioc` — superseding the earlier recommendation: the bench is
   available for immediate validation, and this repo already maintains MANUAL EDITs in generated
   files that any regen must re-apply, so one more flagged edit beats a regen that would clobber
   the oversampling + CMP4-trigger + RepetitionCounter edits). Per ADC: `ScanConvMode =
   ADC_SCAN_ENABLE` + `InjectedNbrOfConversion = 2` + a rank-2 block (PHA_V=ADC1_IN3,
   PHC_V=ADC2_IN4, PHB_V=ADC3_IN2; 47.5 cy sample time per the VBUS_V precedent). Sequence =
   300 ADC cyc = 8.3 µs < 10.4 µs trigger period (✓ with 4× OVSR). `inverter_get_phase_voltages_raw()`
   reads `JDR2`. **A CubeMX regen reverts all of it** — comments mark every site.
3. ✅ **DONE — coast control law + FSM state.** `Firmware/{Inc,Src}/backemf_cal.{h,c}`,
   `FSM_STATE_BACKEMF_CAL = 17` (enum + `state_names` + `get_ctrl_law_for_state` +
   `fsm_exit_state` case). Phases: SPINUP (delegates to `vel_ramp`/`foc` law structs) → CUT
   (float bridge, stamp metadata) → COAST (decimated logging, speed-gated) → DONE. Knobs in
   `config.h` (`BACKEMF_CAL_*`). **`BACKEMF_CAL_SPIN_TARGET = 0` = hand-spin mode**: no spin-up,
   bridge floats on arm, logs whatever motion appears — the zero-risk bring-up test.
4. ✅ **DONE — capture + tooling.** Linear (non-wrap) 1024×16 B buffer @ 500 Hz decimation
   (`backemf_capture_buf/idx` + `backemf_meta_*` globals), SWD-dumped by
   **`tools/backemf_capture.sh`** (status/dump/fit/grab; mirrors `foc_capture.sh`), fitted by
   **`tools/backemf_cal_fit.py`** (Clarke→Park→`delta`→regression, diagnostics PNG: v_abc,
   αβ hodograph, `delta`-vs-`ω_e` + fit, |v|-vs-|ω| linearity).
5. ◑ **IN PROGRESS (bench)** — see the checklist below. Then: sign check (method doc §7),
   repeatability, and only then on-target regression + write-back/persistence.
6. ☐ **TODO (after offset is solid)** — LUT extraction from the same capture (§7); fold into the
   LUT doc's runtime apply + persistence.

### Bench bring-up checklist & results (2026-06-09)

1. ✅ **Rank 1 untouched.** Boot offset-cal passed normally (6.6/6.0/5.4 mV) after the
   `ScanConvMode` flip; closed-loop behavior unchanged.
2. ✅ **Hand-spin test** (`BACKEMF_CAL_SPIN_TARGET = 0`): pipeline validated end-to-end. Caught a
   real workflow trap: **dir-cal is not persisted**, so the boot-default `encoder_dir=+1` made
   δ sweep at ~2× electrical rate (one-sided σ ≈ 1000s of degrees). The fit tool now auto-detects
   direction via a circular-resultant self-consistency check (R≈1 right dir, R≈0 wrong).
3. ✅ **Powered runs @ ±51 rev/s** (one-boot chain each: flash → state 9 → state 15 → state 17):
   - R(dir=−1) = **0.997** both runs; ψ_m = **172.9 µV·s** both runs (⇒ true Kv ≈ 5300, not the
     6300 nameplate); channel zeros matched ≤ 2 counts.
   - Static (state-15) cals: 0.612665 / 0.779597 turns — differ by exactly 1 electrical period
     (offsets are only meaningful **mod 1/p**); mod-1/6 they agree to 0.57° elec.
   - **Pooled bidirectional fit** (the required estimator — a one-sided band makes Δ/k collinear,
     each single run "measured" k≈26 µs as pure artifact): **Δ = −5.98° elec, k = +1.9 µs,
     σ = 4.6° elec, per-run intercepts −5.97°/−5.99° (0.02° agreement!)**.
   - **Suggested offset: 0.615435 turns** on run[0]'s baseline (≡ 0.115435 mod 1/6).
   - **Headline: the static-lock cal's cogging/friction bias is measured: +5.98° electrical**
     (≈1.0° mech, cos-loss 0.54%). It appears systematic (static cal repeats to ~0.6°), so a
     fixed bias constant on top of state-15 may be the flight answer — no persistence needed.
   - σ = 4.6° elec residual ≫ ~0.7° noise floor → that ripple is the **encoder eccentricity /
     LUT content**, already captured in these bins (~90 mech revs of coverage). Next analysis:
     bin residual vs mechanical angle → first LUT.
4. ✅ **Sign check CONFIRMED** (poke +5° elec via `poke-offset`, re-ran reverse): comparing the
   two same-direction one-sided runs at band center (the robust quantity), δ moved **+11.36° for
   a predicted +11.58°** (poke + static-cal baseline difference, mod 1 elec rev; dδ/d(offset) =
   +2πp for dir=−1 — the script's suggested update is negative feedback and nulls the error ✓).
   Absolute truth reproduced across boots to **0.23° elec** (poked run, pooled-k-corrected:
   0.115540 vs pooled 0.115435 mod-1/6). Write-back can be trusted.
5. ⚠ **DISCOVERY — the static cal is NOT a fixed bias.** Third boot's state-15 cal landed
   ≡ 0.115975 mod-1/6 = **+0.9° elec ABOVE truth**, vs runs 1–2 at **−5.98°/−5.41° BELOW** —
   a ~7° elec spread across three boots. Interpretation: with cogging comparable to lock-in
   stiffness, the elec-0 lock basin contains multiple cogging wells; rotor history selects one
   → static cal error is **multi-modal/history-dependent**, not a constant. Consequences:
   (a) the "state-15 + fixed bias constant" flight strategy is **dead**; (b) per-boot accuracy
   better than ~±4° elec **requires** the back-EMF measurement (or persistence of one) — i.e.
   the flash cal-store is now load-bearing, not a nicety. (c) It also explains why the original
   open-loop phase-cal struggled — the same wells.

### Decided 2026-06-09 (user): on-board fit + flash persistence + LUT

Run the whole pipeline on the wheel (no host in the loop) and persist results in a dedicated
flash page so cal survives reboots **and firmware updates**.

**BUILT 2026-06-09 — cal store + CAN-driven persistence states** (build/lint clean, bench-untested):
- `Firmware/{Inc,Src}/cal_store.{h,c}` — blob `{magic, ver, flags, phase_offset, encoder_dir,
  psi_m, lut[128] (reserved, zeroed), crc32}` = 536 B in the **last 2 KB page** (0x0807F800,
  bank 2 / page 127, DBANK runtime-verified). Save skips identical writes (wear), refuses
  invalid dir; restore is CRC- + sanity-gated.
- **FSM states 18/19/20** = `SAVE/ERASE/RESTORE_CALIBRATION` — CLEAR_ERRORS "RPC" pattern (no
  law, never armed, action on the exit transition, auto-return to IDLE, result →
  `procedure_result` on TPDO1). Plus **auto-restore at every boot** (`app_main`).
- Linker `FLASH LENGTH 480K → 478K` (MANUAL EDIT in the .ld, regen reverts) — no image can reach
  the cal page, so image-extent erases can't wipe it.
- **Bootloader erase audit (done):** OpenBL page-erase is host-page-list driven and
  CubeProgrammer's default download erases only written pages → cal survives both update paths
  (SWD `program` + UART DFU). A host-commanded **mass erase wipes the cal page AND the
  bootloader itself** (stock OpenBL has no self-protection) — never mass-erase in update
  procedures (was already true).
- **Stall-free flash ops:** app executes from bank 1 (95 KB ≪ 224 K bank-1 budget), cal page is
  bank 2 → dual-bank read-while-write, zero instruction stalls even in the FSM ISR. If the app
  image ever grows past 0x08040000, revisit.
- Commissioning over CAN: `9` (dir) → `15` or `17` (offset) → `18` (save); later boots
  auto-restore. See `docs/can-interface.md`.
- **FSM_STATE_BACKEMF_CAL_BIDIR (21)** — one-run fwd+rev capture (`backemf_cal_bidir_ctrl_law`,
  shared callbacks, separate init to avoid racing `fsm_state` from the fast-loop init): leg 0
  (sign of `BACKEMF_CAL_SPIN_TARGET`, capped at half the buffer) → **DRAIN** (still hi-Z, wait
  for |ω| below the gate so FOC re-engages gently at ~zero back-EMF, timeout-guarded) → leg 1
  (reversed) appends into the same buffer. The existing dump/fit tooling needs no changes — the
  pooled fit sees mixed-sign ω_e and does the gold-standard bidirectional fit automatically.
  State 17 (single-direction) unchanged.
- **Linker now caps the app at 224K = bank 1** (was 478K): makes the read-while-write guarantee
  *linker-enforced* — if the image ever outgrows bank 1 the build fails loudly instead of
  silently re-introducing flash-op stalls. 95K used = 2.3× headroom.

### Calibration runbook (fully on-board — pure CAN, no SWD required)

One-time per sensor: the first boot of this build writes the MA732 filter window
(`MA732 FW=119 != 51, writing sensor NVM... OK`).

Full calibration (wheel free to spin; host on CAN):

| # | Action | Wait for / check |
|---|---|---|
| 1 | `0x4004` = **9** (dir cal) | state back to IDLE, `procedure_result` = 1 |
| 2 | `0x4004` = **15** (bootstrap offset) | `procedure_result` = 1 (healthy FOC spin-up for #3) |
| 3 | `0x4004` = **21** (bidir back-EMF self-cal) | `procedure_result` = 1. UART (informational): `BEMF fit: D=… k=… n=…`, `LUT installed, …/128 bins`, `offset A -> B turns` (~15–60 s; drain dominates) |
| 4 | `0x4004` = **18** (save to flash) | `procedure_result` = 1 |
| 5 | Power-cycle | boot prints `cal_store: restored offset … dir … +LUT`; closed-loop runs with no recal |

State 21's finish now does the whole fit on-board over the frozen capture (same algorithm as
`tools/backemf_cal_fit.py`: record-mean channel zeros → 3-term Clarke → per-sample δ →
circular-mean-centered pooled regression) **and** builds + installs the 128-bin per-rev LUT from
the post-fit residual. Gated hard: ≥`BACKEMF_FIT_MIN_PER_DIR` samples in EACH direction, |Δ| and
|k| bounds, LUT coverage/amplitude bounds (failure keeps the previous LUT and/or returns
`procedure_result` = 0; the buffer stays SWD-dumpable for offline diagnosis either way).

Acceptance (once per wheel): re-run #3 after #5 — the printed `D` should be **≈ 0** (±~0.3° elec,
the measured cross-boot repeatability), and the new LUT should match the old (UART peak value
similar). The SWD tooling (`backemf_capture.sh` grab/fit/poke-offset) remains for diagnostics and
cross-checking the on-board fit against the offline one — worth doing once on the bench.
State 17 (single-direction) stays log-only: one-sided fits carry the measured Δ/k-collinearity
artifact and are never applied by firmware.

**BUILT 2026-06-09 — on-board fit + LUT (bench-untested):**
- **On-board fit in state 21's finish** (`backemf_fit_and_apply()`, `backemf_cal.c`): 4 passes
  over the frozen buffer (~ms-scale in the FSM tick, same blocking class as encoder_validation's
  dumps) → applies the offset correction live. Bounds in config (`BACKEMF_FIT_*`).
- **LUT end-to-end**: residual binned by raw mech angle → zero-meaned, empty bins
  circular-interpolated, amplitude-gated → `encoder_set_lut()`. Runtime apply is a wrap-safe
  lerp at the top of `encoder_update()` before the PLL (`ENC_LUT_N = 128`, all-zeros = no-op).
  `cal_store` saves/restores it (blob `flags` bit0 + `lut[]`, layout unchanged → old v1 blobs
  still valid); restore treats the blob as authoritative (clears the live LUT if absent).
  This implements `encoder-lut-design.md` §5's apply with the back-EMF capture method (its §3–4
  constant-velocity capture is superseded, as predicted by §7 of this doc).

**FIXED 2026-06-09 — trapezoid-wobble contamination of the LUT** (user observed "more ripple
with LUT on" — correct):
- δ assumes a sinusoidal back-EMF; with 5th/7th harmonics the αβ vector's angle **wobbles at
  6θ_e = 6·p = 36 cycles/mech-rev**, amplitude up to ~(e₅+e₇)/e₁ rad. The raw 128-bin LUT
  captured ~88% of that as FAKE encoder error → applying it injected real 6f_e ripple into FOC.
- Fix: **spatial-harmonic limit** — the table is Fourier-reconstructed keeping only harmonics
  **1..`BACKEMF_LUT_MAX_HARMONIC` (4)** cycles/rev (real encoder error is 1/rev eccentricity +
  small 2/rev; the wobble at 36/rev is rejected regardless of amplitude). This is exactly
  `encoder-lut-design.md` §3.5's defense, restored. Also subsumes zero-meaning (no DC term) and
  smooths bin noise/interp seams. **Re-run cal 21 after flashing to rebuild a clean table.**
- A/B verification: `tools/backemf_capture.sh lut-enable 0|1` toggles the runtime correction
  over SWD (table/blob untouched). Constant-speed `foc_capture` Iq spectra with LUT on/off:
  1/rev line should DROP with LUT on (eccentricity correction + sign both confirmed); a rising
  6f_e line with LUT on would indicate residual contamination. NB the on-board fit is
  deliberately LUT-independent (recomputes from raw), so re-running 21 canNOT validate the
  applied table — only this A/B can.

**VALIDATED 2026-06-09 — LUT A/B on the bench (nolut/ vs lut/ captures @ 50 rev/s):**
angle residual RMS 0.77° → **0.10° elec (8×)**, Iq ripple 343 → **105 mA (3.3×)**, the 1/rev
eccentricity velocity swing eliminated, **no 36/rev injection** (harmonic limit verified), 6f_e
Iq line unchanged (dead-time distortion, pre-existing). **Application sign empirically
confirmed** — a flipped sign would have doubled the residual instead. (The initial "more ripple"
impression was unmasking: removing the dominant slow 1/rev swell makes the same-size
high-frequency hash visually prominent.) Remaining nuance: the 8 ms window can't resolve
1–4 cyc/rev at 50 rev/s; a high-speed A/B pair (150–300 rev/s → 1–2.5 revs/window) would show
the 1/rev line directly and validate flight-speed playback.

**Still TODO:**
- High-speed LUT A/B pair (above); full-runbook end-to-end validation; one offline-vs-on-board
  fit cross-check via the SWD tooling; MA732 FW=51 σ recheck.
- **STANDING TODO (user-requested): trapezoidal back-EMF → torque-ripple compensation.** First
  step is free: extract 5th/7th harmonic content from the existing coast captures (fit-script
  `--harmonics` mode). Mitigation = dq 6th-harmonic current shaping; at flight speed 6f_e
  exceeds the 4 kHz current loop → voltage feedforward, not closed-loop tracking. Anti-cogging
  table rides the same blob (flags space reserved).

---

## 11. Open decisions

- **`ω_cal`** — high enough for back-EMF SNR and to span a speed range for the regression, under
  the body-diode bound. Needs `psi_m` (≈ `K_t` already known: `8.27/Kv ≈ 1.31 mN·m/A`) and Vbus.
- **`PHx_V` divider ratio + filter** — from the schematic; ratiometric for the angle, but needed
  for SNR gating and for matching/`k` understanding.
- **One state both directions, or two runs?** Bidirectional in one state is cleanest (method
  doc's gold standard); single-direction works for a short coast via the intercept.
- **Capture sizing / decimation** — offset needs little; LUT needs angular coverage over many
  revs (§5, §7).
- **ADC change vehicle** — `.ioc` regen (clean) vs hand-edit vs runtime reconfig (§3.1).
- **Permanent rank-2 sampling vs cal-only** — recommend permanent (cheap, broadly useful).
- **Persistence** — defer to the shared cal-flash-page decision.

---

## 12. References

- `docs/backemf_coastdown_calibration.md` — the method/theory (§§ referenced throughout).
- `docs/encoder-lut-design.md` — the LUT; §7 here supersedes its capture/fit method.
- `docs/foc_capture.md` — capture/SWD-dump pattern to mirror.
- `docs/control-laws.md`, `docs/control-cycle.md` — control-law contract + fast-loop walk.
- `docs/footguns.md` — blocking-printf, encoder-read-assumed-good, IWDG/flash.
- Code: `pwm.c`, `control_ll.c`, `inverter.c`, `closed_loop_controller.c`, `encoder.c`,
  `Core/Src/adc.c`, `oresat-acs-fw-v8.ioc`, `Firmware/Inc/periphmap.h`.
- External: Rinehart/Cascadia gamma-adjust, UltraZohm encoder-offset docs, Rahman & Hiti 2005
  (per the method doc's provenance).
