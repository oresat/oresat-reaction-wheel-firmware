# Encoder angle-error LUT — design & cogging-immune calibration

> **Status update 2026-06-09: IMPLEMENTED — via the back-EMF coast-down path, not this doc's
> capture method.** The runtime apply (§5.1-§5.2 here) is built as designed (`encoder.c`:
> `ENC_LUT_N`-bin wrap-safe lerp at the top of `encoder_update`, all-zeros no-op default), but
> the *capture/fit* is the back-EMF coast residual (absolute per-sample angle truth — strictly
> better than this doc's constant-velocity assumption, as §[supersession] in
> `backemf-coastdown-implementation.md` predicted), computed **on-board** in the bidir cal state
> (21) and persisted by `cal_store` (states 18/19/20 + boot auto-restore). This doc remains the
> reference for the *why* (cogging-immunity reasoning, spectral arguments, eccentricity
> background); for the *how it actually works now*, read
> **`docs/backemf-coastdown-implementation.md`** (incl. the calibration runbook).
>
> Original design note (2026-06-08) follows unmodified:
> Predecessor work — the scalar encoder phase-offset cal, now doing a gated bidirectional
> sweep and *actually applying* the averaged result — is done in-tree (see
> `encoder_calibration.c` → `encoder_phase_cal_*`, and §1.3 below). This document is the plan
> for the **position-indexed angle-correction table (LUT)** that comes next: the layer that
> kills the per-revolution encoder error a single scalar offset can't touch, while being
> immune to the motor's large cogging torque.
>
> Nothing here is wired up yet. It is a roadmap + the reasoning behind it, so it can be
> resumed without re-deriving everything.

---

## TL;DR

- A scalar phase offset corrects the *average* electrical alignment. It cannot correct the
  **once-per-mechanical-revolution eccentricity** of a magnetic encoder (MA702), which is
  usually the dominant residual angle error and varies per physical unit.
- The fix is a **LUT** `c(θ_meas)` applied in `encoder_update()`: `θ_corr = θ_meas + c(θ_meas)`.
- The hard part is **measuring** that map without the motor's large cogging torque biasing it.
  The key idea: **don't measure statically — measure while spinning fast.** A reaction wheel's
  large rotor inertia turns cogging from a *dominant static bias* (`∝ T_cog / T_lock`) into a
  *negligible dynamic ripple* (`∝ T_cog / (J·(N_c·ω)²)`). The improvement ratio
  `J·(N_c·ω)² / T_lock` is **independent of cogging amplitude** and is ~100–1000× for plausible
  wheel parameters.
- Stack three independent defenses against cogging: **(1) inertia × speed²** suppresses its
  amplitude; **(2) low-order harmonic fit** rejects it spectrally (eccentricity lives at
  spatial harmonic 1, cogging at ≥ `N_c` = LCM(slots, poles) ≥ 12); **(3) coast capture**
  removes controller/PWM contamination. Bidirectional capture is a *fourth, optional* refinement
  that cancels friction and measures sensor group delay — but **does not** cancel cogging.
- Recommended pipeline: **spin up (closed-loop) → cut torque → capture raw angle during the
  inertial coast → host-side polynomial detrend + low-harmonic fit → bake into a sampled
  table.** Run both directions if you care about MA702 group delay.

---

## 1. Background & motivation

### 1.1 The hardware

- Motor: 12-pole (`MOTOR_NUM_POLEPAIRS = 6`) BLDC reaction wheel. High Kv
  (`MOTOR_KV = 6300`), so a small torque constant `K_t = 8.27 / Kv ≈ 1.31 mN·m/A`.
- Encoder: MA702, 14-bit magnetic angle sensor on SPI, `ENCODER_CPR = 16384`. Magnetic
  encoders' intrinsic error is dominated by **eccentricity** — magnet/shaft not perfectly
  concentric with the die — which appears as a **once-per-mechanical-revolution** sinusoid in
  the reported angle. There is usually a smaller 2nd harmonic (sin/cos amplitude & quadrature
  mismatch) and minor higher-order INL.
- Fast loop: 16 kHz (`LOOP_UPDATE_FREQ`), the only place that reads the encoder and drives PWM.
- **Cogging is large relative to the max *continuous* current** — this is the whole reason the
  static calibration struggles, and the central constraint this design must respect.

### 1.2 Why a scalar offset is not enough

The current/predecessor calibration produces **one number**: the electrical phase offset
between the encoder's zero and the rotor's magnetic axis. That corrects the *mean* alignment.
But the encoder's reading error is a *function of mechanical angle* `e(θ_mech)` — predominantly
the 1/rev eccentricity. A scalar offset is the DC term of `e`; everything else (the 1/rev
ripple and friends) passes straight through into the electrical angle the FOC uses, showing up
as a once-per-rev torque-ripple / efficiency wobble. For a precision reaction wheel that error
is worth removing, and it is **per-unit** (each magnet/shaft assembly is different), so it can't
be a shared constant — it has to be measured per wheel.

### 1.3 What already exists (predecessor: scalar phase-offset cal)

`FSM_STATE_ENCODER_OFFSET_CAL` / `encoder_phase_cal_ctrl_law` in `encoder_calibration.c` was
reworked to:

1. Seed a coarse one-point offset at lock-in (`encoder_set_phase_offset(pos)`), then
2. Sweep the open-loop current vector out-and-back with a **trapezoidal velocity profile**
   (finite acceleration; no impulsive velocity reversals to ring the rotor/field resonance),
3. Accumulate `(commanded − measured)` electrical-phase error **only during the constant-velocity
   cruise**, gated on measured velocity (rejects accel/decel ramps, the v≈0 turnaround stiction
   band, and post-step ringing),
4. Keep the two sweep directions in **separate accumulators** and apply
   `offset = 0.5·(mean_fwd + mean_rev)` — bidirectional averaging cancels **friction lag**.

That fixes friction but is still only a **scalar**, and (because it's an open-loop static-ish
lock sweep) it averages cogging out into the single number rather than characterizing it. The
4 integer mechanical revolutions make the scalar unbiased by cogging/eccentricity (both integrate
to zero mean over integer revs), but it produces no *map*. The LUT is the next layer and is
**orthogonal**: the LUT carries the zero-mean ripple `e(θ) − mean`, the scalar offset carries the
mean. They compose (see §2.3).

> Note: `docs/control-laws.md` and `docs/footguns.md` still describe the *old* phase-cal
> (fine correction "commented out / not applied"). Those notes are now stale and should be
> refreshed when convenient — not part of this LUT work.

---

## 2. What the LUT corrects — and what it must not

Two position-periodic phenomena are constantly conflated; the entire design depends on keeping
them apart:

| | Encoder reading error `e(θ_mech)` | Cogging torque `T_cog(θ_mech)` |
|---|---|---|
| Nature | **Sensor geometry** error | **Mechanical** torque ripple |
| Spatial frequency | 1/rev (eccentricity) + small 2nd + minor higher | `N_c` = LCM(slots, poles) per rev, **≥ 12** for 12 poles |
| Depends on current/load/speed/dir? | No (pure geometry) | The torque doesn't, but its *effect on a measurement* does |
| Role here | **The thing the LUT represents** | **A confound in the measurement** |

Cogging does **not** corrupt the encoder reading. It corrupts your *knowledge of `θ_true`* the
moment you try to establish "truth" by balancing it against a motor torque (the static
lock-and-step trap, §3.1). Get that straight and the rest follows.

### 2.1 Index by *measured* angle

At runtime all you have is `θ_meas`. So the table is `c(θ_meas)` such that
`θ_corr = θ_meas + c(θ_meas) ≈ θ_true`. Index it by the **raw** measurement (the lookup variable
is the thing you actually have), not by true angle.

### 2.2 Zero-mean by construction

The LUT carries only the *shape* (ripple) of `e`. Its mean is, by definition, the scalar phase
offset, which is handled separately. Keep the table zero-mean so the two layers never fight and
so the all-zeros default (pre-cal) is a safe no-op.

### 2.3 Composition with the scalar offset

Runtime chain becomes:

```
raw  → +LUT (mechanical-angle ripple)  → PLL/estimator  → ×polepairs  → −phase_offset (scalar)  → electrical angle
```

The LUT de-ripples the mechanical angle *before* the electrical conversion and the scalar
offset. Because the LUT is zero-mean, the existing scalar cal is unaffected in expectation —
and actually becomes *more consistent*, since it now runs on a de-rippled angle. Build/apply
order is flexible (LUT is zero-mean), but conceptually: **LUT first, then scalar offset.**

---

## 3. The core problem: cogging immunity

This is the heart of the design and the question that motivated it: *how do we build the map
without cogging poisoning it?*

### 3.1 Static lock-and-step is maximally cogging-sensitive (the trap to avoid)

The obvious way to build a map — and what `encoder_validation_ctrl_law` scaffolds today — is to
lock the rotor at a sequence of commanded electrical angles with a d-axis current and record
`θ_meas` vs commanded at each. At a static lock with current `I` at commanded electrical angle
`φ_cmd`, the rotor settles at `φ_rotor` where the torque balance holds. With electrical lag
`δ = φ_cmd − φ_rotor`:

```
K_t · I · sin(δ)  =  T_cog(θ)  (+ friction)
⇒  for small δ:   δ ≈ T_cog / (K_t · I)  =  T_cog / T_lock      [electrical rad]
mechanical error = δ / polepairs
```

`T_lock = K_t·I` is **bounded by the continuous current limit** (heat). `T_cog` is *comparable
to* that on this motor. So `δ_cog` is large — the static map is dominated by cogging, exactly the
problem statement. **Avoid this method for the LUT.** (Bidirectional averaging at each point
cancels friction but **not** cogging — see §3.4.)

### 3.2 Spin instead: the inertia low-pass

Now spin the wheel at mean mechanical speed `ω` (rad/s) and let inertia `J` carry it. Equation
of motion:

```
J · θ̈_mech  =  T_motor  +  T_cog(θ_mech)  −  T_friction
```

Decompose cogging into spatial harmonics; the fundamental is at `N_c` cycles per mechanical
revolution. At constant spin (`θ_mech ≈ ω t`), harmonic `n` forces the rotor at *temporal*
frequency `Ω = n·N_c·ω`, which is far above the (very slow) velocity-loop bandwidth — so only
the inertia responds. For `J·θ̈ = T_n·cos(Ω t)` the steady angular deviation is:

```
Δθ_cog  ≈  T_cog / (J · (N_c · ω)²)        [mechanical rad]
```

The cogging-induced angle ripple scales as **1 / (J · ω²)**. The big flywheel inertia and a
healthy spin speed both crush it.

### 3.3 The improvement ratio is independent of cogging amplitude

```
   static error        T_cog / T_lock                 J · (N_c · ω)²
  ───────────────  =  ──────────────────────────  =  ─────────────────
   dynamic error       T_cog / (J·(N_c·ω)²)                T_lock
```

`T_cog` cancels — you don't even need to know it. Worked example (**plug in your real `J` and
`T_lock`; these are illustrative**):

```
J        = 2e-5  kg·m²        (assumed — TODO: measure/spec the wheel)
N_c      = 12                 (assumed slot/pole combo — TODO: confirm)
ω        = 5 rev/s = 31.4 rad/s
N_c·ω    = 377 rad/s
T_lock   = K_t · I ≈ 1.31e-3 · 4 ≈ 5.2e-3 N·m

ratio = J·(N_c·ω)² / T_lock = 2e-5 · 377² / 5.2e-3 ≈ 546×
```

So with these numbers, spinning is ~500× less cogging-sensitive than the static lock — and that
factor *grows* with `J`, `N_c`, and `ω²`. The very inertia that makes a reaction wheel painful
to bench-test is precisely what makes its encoder cleanly measurable at speed.

### 3.4 What bidirectional capture does and does **not** do

A subtlety worth pinning down, because it's tempting to assume direction-averaging fixes cogging
the way it fixes friction in the scalar cal. It does **not**.

- **Cogging — NOT cancelled.** The inertia response is even in `ω`. Forward `θ(t)=ωt`:
  forcing `cos(N_c·ω·t)`, response `θ_p = −cos(N_c θ)/(J(N_c ω)²)` as a function of *position*.
  Reverse `θ(t)=−ωt`: same time-forcing `cos(N_c·ω·t)`, response works out to the **identical**
  function of position. Averaging fwd+rev (both indexed by position) returns the same `Δθ_cog`,
  not zero. Cogging is handled by §3.2/§3.5, never by direction averaging.
- **Friction — cancelled (but moot).** Coulomb friction flips sign with velocity, so fwd/rev
  average cancels it. But in a *moving* capture friction is already benign: in closed-loop it's a
  ~constant compensating torque (constant offset), in coast it's a smooth deceleration (a trend)
  — both removed by the reference fit (§4). So bidirectional is belt-and-suspenders here.
- **MA702 group delay — cancelled, and *measured*.** A fixed sensor delay `τ` makes
  `θ_meas(t) = θ_true(t−τ) + e(θ_true(t−τ))`. At speed this shifts the *apparent* map by `+ω·τ`
  forward and `−ω·τ` reverse. Averaging the two aligns to the true phase; **half their difference
  = ω·τ measures the group delay.** This is the real reason to bother with bidirectional for the
  LUT. Caveat: if `τ` is significant, a map built at `ω_cal` is phase-shifted by `(ω_run−ω_cal)·τ`
  at other speeds — so build at a representative speed, or characterize and remove `τ`.

### 3.5 Spectral separation + low-harmonic fit (the "comb filter," done right)

Eccentricity lives at spatial harmonic **1** (±2, 3); cogging contamination lands at **≥ N_c**
(≥ 12). They don't overlap. So fit the captured residual to a Fourier series and **keep only
harmonics `1..K_h` with `K_h < N_c`**. This rejects cogging *regardless of amplitude* — the
robust, amplitude-independent guarantee that complements the inertia argument. It also yields a
compact, inherently-periodic, smooth representation (a handful of coefficients) to bake into the
table. This is the angle-domain realization of the comb/notch filter idea, with zero phase error
and no transients.

> Pick `K_h` from the *measured* spectrum (the fit tool plots it): you will literally see the
> n=1 eccentricity peak and the n=N_c cogging peak separated. Choose `K_h` between them. If you
> tell me the slot count, `N_c = LCM(slots, 12)` pins it analytically too.

---

## 4. Recommended method: spin-up → coast → low-harmonic fit

### 4.1 Why coast is the cleanest reference

The cleanest possible angle reference is **ballistic coast**: spin up, then cut motor torque and
record during the inertial coast-down. During coast there is **zero motor torque, zero controller
action, zero PWM ripple** injected into the rotor — the only forcing is inertia-suppressed cogging
(§3.2) and smooth friction. `θ_true(t)` is therefore a smooth low-order polynomial in time
(≈ quadratic if friction ≈ constant); the periodic residual about that polynomial **is** `e(θ)`.

Two further bonuses of coast:

- **No once-per-rev controller contamination.** In a *closed-loop* constant-speed capture, the
  velocity loop reads a velocity derived from the (still-uncorrected) encoder, whose 1/rev
  eccentricity ripple could drive a 1/rev torque → a 1/rev real angle deviation → aliasing into
  the very n=1 harmonic you're trying to measure. Coast eliminates this entirely. (In practice
  the loop is so slow — `VEL_GAIN = 0.002` — that at a 5 Hz once-rev ripple it barely responds,
  so closed-loop constant-speed is *also* fine; coast is just bulletproof.)
- **Deceleration decorrelates residual cogging across revs.** As `ω` drops, `Δθ_cog ∝ 1/ω²`
  changes rev-to-rev, so averaging across revs partially averages cogging down too — free extra
  rejection on top of §3.2/§3.5.

### 4.2 Procedure

1. **Spin up** to `ω_cal` using the existing velocity controller internals (`vel_ctrl_*` in
   `closed_loop_controller.c`).
2. **Settle** at `ω_cal` (let transients die).
3. **Arm a decimated capture**, then **cut to zero torque** (coast). Record `(raw_enc_pos)` —
   and optionally `loop_count` — for `K` full revolutions.
4. **Repeat both directions** (CW + CCW) if measuring/removing group delay (§3.4).
5. **Fit (host side):** detrend each capture against a smooth low-order polynomial in time; bin
   the residual by measured angle; average across revs; DFT and keep harmonics `1..K_h`; average
   the two directions; sample the fitted curve into the runtime table.

### 4.3 Role of each technique (summary)

| Technique | Kills | Mechanism |
|---|---|---|
| Spin fast on high-`J` wheel | cogging **amplitude** | inertia low-pass, `∝1/(Jω²)` (§3.2) |
| Low-harmonic fit (`K_h < N_c`) | cogging **spectrally** | eccentricity n=1 vs cogging n≥12 (§3.5) |
| Coast (zero torque) | controller/PWM contamination | no active torque at all (§4.1) |
| Deceleration across revs | residual cogging | decorrelation, averages down (§4.1) |
| Bidirectional | friction lag + **measures** sensor delay `τ` | sign flip vs even-in-ω (§3.4) |
| Polynomial detrend | friction (as a trend), mean speed | reference = smooth motion (§4.1) |

### 4.4 Iterative refinement (optional)

Once a first table is loaded, re-run the capture *with the LUT applied* and fit the **residual**;
add it to the table. Converges out any first-pass imperfection (e.g. small `K_h` truncation, mild
nonlinearity in the detrend). For the initial build, capture with the LUT **zeroed** so you're
fitting `e` itself, not a residual.

---

## 5. Architecture & integration

### 5.1 Runtime apply — `encoder_update()` (`encoder.c:60`)

Apply at the **top**, before `circular_delta`/PLL, so everything downstream (multiturn, phase,
velocity) benefits and the PLL smooths any table quantization:

```c
f32 enc_pos = (f32)meas / (f32)ENCODER_CPR;               // 0..1 raw
enc_pos = fast_fmod_01(enc_pos + enc_lut_lookup(enc_pos)); // index by RAW, add correction (turns)
// ... existing circular_delta / predict / PLL from here, unchanged ...
```

Notes:
- Index uses the **raw** measured position (the lookup variable you have at runtime).
- `fast_fmod_01` already exists (`encoder.c:53`) and is valid for inputs in `[-1, 2)`; the
  correction is tiny so it stays in range. Wrap-safe.
- The LUT also improves `multiturn_pos_raw` (→ better dir-cal) and the electrical phase (→ better
  FOC). All for free.
- **First-application transient:** loading a non-zero table mid-spin steps `enc_pos` by one
  sample. Load the table only when **idle** (not armed) to avoid a one-tick glitch into
  `circular_delta`.

### 5.2 Table format

```c
#define ENC_LUT_N 256u                 // power of two → index = top bits, free wrap mask
static f32 enc_lut[ENC_LUT_N];         // correction in TURNS (mechanical revs), zero-mean, periodic
                                       // default all-zeros = safe no-op before cal
```

Lookup (linear interp, wrap `N-1 → 0`), all in the `__ccmram` hot path, ~10–20 cycles —
negligible vs the 62.5 µs budget:

```c
static __fn_inline f32 enc_lut_lookup(f32 pos /*0..1*/) {
    f32 x = pos * (f32)ENC_LUT_N;
    u32 i = (u32)x & (ENC_LUT_N - 1u);
    f32 frac = x - (f32)(u32)x;
    f32 a = enc_lut[i];
    f32 b = enc_lut[(i + 1u) & (ENC_LUT_N - 1u)];
    return a + frac * (b - a);
}
```

256 × f32 = 1 KB. 256 bins over a 14-bit encoder is plenty for a 1/rev-dominated, smooth error;
could go to 128. Add `encoder_set_lut(const f32 *src)` / `encoder_clear_lut()` to `encoder.[ch]`.

### 5.3 Capture path — decimated ring

You **cannot** store raw at 16 kHz for multiple revolutions: 16 revs @ 5 rev/s ≈ 51 k ticks ×
~8 B ≈ 400 KB ≫ 80 KB RAM. So **decimate** to ~1 kHz (every 16th tick → ~200 samples/rev):

- Ring of `f32 raw_enc_pos` at fixed decimation. Time per sample is implicit
  (`start_loop_count + index·16` ticks), so storing `loop_count` per sample is optional (add it
  for robustness against missed ticks). ~4 B/sample.
- e.g. 4096 samples × 4 B = 16 KB → ~20 revs at 5 rev/s. Fits in free RAM (build reports ~40 KB
  RAM free + 16 KB unused **RAM2** — RAM2 is a natural home; keep CCM for hot code).
- Mirror the `foc_capture` mechanism (`Firmware/{Inc,Src}/foc_capture.*`, driven over SWD by
  `tools/foc_capture.sh`): a tiny state machine (arm/run/stop) and a struct-of-arrays ring.
  **Do not** reuse the FOC ring as-is: it stores *electrical* `motor_phase` (post-offset) and is
  only 256 samples / 16 ms — wrong quantity and far too short. This needs its own ring of *raw
  mechanical* angle.
- Dump over SWD, fit on host. (On-target binning + fit is possible later if you want field/at-boot
  recal without a probe, but host fit is the right call for bring-up — see §5.4.)

### 5.4 Host fit tool — `tools/encoder_cal_fit.py`

Mirror `tools/decode_foc_capture.py` + `foc_capture.sh`. Responsibilities:

1. Unwrap the ring, reconstruct `(t, raw_angle)`.
2. Unwrap angle to multiturn; **polynomial detrend** (fit `θ ≈ a + b·t + c·t²`), residual = `e + ripple`.
3. Bin residual by measured angle (mod 1 rev), average across revs.
4. **DFT / least-squares harmonic fit; plot the spectrum** (so you can see n=1 eccentricity vs
   the n=`N_c` cogging spike and choose `K_h`).
5. Average the two spin directions; report group delay `τ` from half their phase difference.
6. Emit a C table (`encoder_lut_data.h`) and/or a binary blob to flash (§5.6).

Fitting on the host keeps tricky least-squares off the MCU, lets you verify cogging rejection
visually, and matches the established bench/SWD workflow.

### 5.5 New FSM state + control law

- `FSM_STATE_ENCODER_LUT_CAL` in `fsm.h`; case in `get_ctrl_law_for_state()` (`fsm.c`).
- `encoder_lut_cal_ctrl_law` (likely in `encoder_calibration.c` alongside the others). Phased:
  **spin-up (closed-loop vel)** → settle → **arm capture + zero torque (coast)** → record `K`
  revs → stop capture → `is_complete`. Reuse `vel_ctrl` internals for spin-up; for the coast
  phase, drive zero PWM while continuing to read the encoder + write the capture ring (this is
  essentially `encoder_test_control_law` behavior + capture).
- Document it in `docs/control-laws.md` like the other cal states.

### 5.6 Persistence — note: **nothing is persisted today**

Checked the tree: `phase_offset`, `encoder_dir`, and the measured R/L are all **RAM statics**
(`encoder.c:25-26`), lost on every reboot. The only flash writes anywhere are **option bytes**
(`bootloader.c`). So today, every cal value evaporates on power-cycle. The HAL flash driver *is*
already compiled (used for option bytes), so the capability exists — there's just no cal-store
layer. Two paths for the LUT:

- **Bring-up / single bench unit:** generate `encoder_lut_data.h` and compile it in. Zero
  flash-write code. But it does **not** scale to 4 flight wheels with different eccentricity
  (would need 4 firmware images).
- **Right end-state — a small cal flash page.** Reserve one flash page (2–4 KB depending on the
  `DBANK` option bit) at the top of the 480 KB app region for a versioned, CRC'd struct:

  ```c
  typedef struct {
      u32 magic;           // sentinel, e.g. 'ACAL'
      u16 version;
      u16 _pad;
      u32 crc;             // over the rest
      f32 phase_offset;    // scalar offset (also fixes the not-persisted gap!)
      f32 encoder_dir;     // ±1
      f32 lut[ENC_LUT_N];  // 1 KB
  } encoder_cal_blob_t;    // ~1.05 KB → one page
  ```

  Write via `HAL_FLASH` (unlock → erase page → program double-words → lock); load + CRC-check in
  `app_main` and populate `encoder_set_*`. This is per-unit, and it **also fixes offset/dir
  persistence for free** — worth doing once for the whole cal story, not just the LUT.

  **Footgun:** a flash page **erase** stalls the core ~20–40 ms. The IWDG must be kicked around it
  (see `docs/footguns.md` on IWDG continuity) or the erase done in a safe, watchdog-aware window.
  Also keep the cal page out of the linker's app region (`STM32G474VEHX_FLASH.ld`) so a firmware
  flash doesn't wipe it — or accept that reflashing requires recal.

---

## 6. Parameters & open decisions

Decisions that materially change the build — capture before resuming:

- **Slot count → `N_c = LCM(slots, 12)`.** Sets where cogging lands and a safe `K_h`. (The fit
  tool shows it empirically regardless.)
- **Rotor inertia `J`** (and rough `T_cog`). Confirms the ~500× claim with real numbers and sets
  `ω_cal` for a target residual.
- **Target accuracy** (e.g. < 0.5° electrical?) → drives `K` (revs), `ENC_LUT_N`, `ω_cal`,
  `K_h`.
- **`ω_cal`** — high enough for strong inertia rejection, within safe wheel speed and the
  encoder/PLL tracking margin (16 kHz / `ω` gives samples/rev; at 5 rev/s that's 3200/rev — huge
  headroom).
- **Persistence appetite:** compile-in table (fast, one unit) vs the flash cal-store (more work,
  per-unit, fixes offset/dir persistence too).
- **Bidirectional?** Only needed if MA702 group delay matters at your speeds — cheap to include
  and it self-measures `τ`.
- **`ENC_LUT_N` and `K_h`** — 256 / a few harmonics are reasonable defaults; finalize from the
  measured spectrum.

---

## 7. Phased implementation plan

1. **Runtime LUT apply + table (all zeros), confirmed no-op.** Smallest, safest change; lands the
   hot-path edit and the `encoder_set_lut`/`clear_lut` API first. Verify build + `./lint.sh`
   clean and that behavior is unchanged with a zero table.
2. **Decimated capture ring + spin/coast cal state + `tools/encoder_cal_fit.py`.** Get a real
   residual spectrum off the bench wheel. This *empirically validates the cogging-immunity*: you
   should see the n=1 eccentricity cleanly separated from the n=`N_c` cogging, and can read off
   the actual amplitudes and group delay. **This data should drive the remaining choices.**
3. **Decide compile-in vs flash cal-store**, then wire table loading (and, if doing the flash
   page, fold in `phase_offset`/`encoder_dir` persistence).
4. *(Optional)* iterative refinement pass (§4.4); refresh stale phase-cal notes in
   `control-laws.md` / `footguns.md`.

Start with 1 + 2 against the bench wheel — the measured spectrum tells us `N_c`, eccentricity
amplitude, how much cogging actually leaks, and `τ`; everything after that is informed rather than
guessed.

---

## 8. Risks, footguns, caveats

- **Don't build the map statically.** Lock-and-step (incl. the existing `encoder_validation`
  scaffold) is the cogging-sensitive trap this whole design exists to avoid (§3.1).
- **Bidirectional ≠ cogging cancellation.** It's for friction/sensor-delay only (§3.4). Relying
  on it to remove cogging is the easy mistake.
- **Capture raw, or with LUT zeroed**, for the initial build — otherwise you fit a residual, not
  `e` (§4.4). The cal state should zero the LUT at entry (mirror how phase-cal seeds offset).
- **Load tables only when idle** to avoid a one-tick `circular_delta` glitch (§5.1).
- **Group-delay/speed dependence:** a map built at `ω_cal` is mildly phase-wrong at very different
  runtime speeds if `τ` is large; build at a representative speed or remove `τ` (§3.4).
- **PLL stability:** the estimator goes unstable past ½-turn tracking error
  (`encoder.c:59` warning). The LUT correction is tiny and applied to the input — no impact — but
  keep that invariant in mind if the apply point ever moves.
- **No persistence exists today** (§5.6) — the LUT (and the scalar offset) need a deliberate
  storage decision, or they vanish on reboot.
- **Flash erase vs IWDG** (§5.6) if you take the cal-page route.
- **MCU effort:** keep the *fit* on the host. On-target least-squares/DFT is avoidable complexity
  for bring-up.

---

## 9. References

**In-tree code:**
- `Firmware/Src/encoder.c` — estimator; runtime apply point is `encoder_update()` (`:60`),
  statics at `:25-26`, `fast_fmod_01` at `:53`.
- `Firmware/Src/encoder_calibration.c` — predecessor scalar phase-cal (`encoder_phase_cal_*`) and
  the `encoder_validation` scaffold to repurpose; home for the new cal law.
- `Firmware/Src/closed_loop_controller.c` — `vel_ctrl_*` (spin-up driver).
- `Firmware/{Inc,Src}/foc_capture.*` + `tools/foc_capture.sh` + `tools/decode_foc_capture.py` —
  the ring-capture + SWD-dump + host-decode pattern to mirror for the angle capture.
- `Firmware/Inc/config.h` — `MOTOR_NUM_POLEPAIRS`, `MOTOR_KV`, `ENCODER_CPR`, `LOOP_UPDATE_FREQ`,
  `ENCODER_PHASE_CAL_*`.
- `STM32G474VEHX_FLASH.ld`, `bootloader.c` — for the cal-page route.

**In-tree docs:**
- `docs/foc_capture.md` — the capture mechanism this borrows from.
- `docs/control-laws.md`, `docs/control-cycle.md` — FSM/control-law contract; where to register
  the new state. (Stale phase-cal note to refresh — §1.3.)
- `docs/footguns.md` — IWDG, encoder-read-assumed-good, estimator ½-turn instability.

**External / prior art (concept, not endorsements):** ODrive anticogging + encoder offset map,
VESC / Trinamic encoder-nonlinearity calibration, iC-Haus / AMS app notes on magnetic-encoder
eccentricity (1/rev) and harmonic correction. The common thread is the same as here: spin to
get a clean angle reference, fit low harmonics, store a per-unit map.
