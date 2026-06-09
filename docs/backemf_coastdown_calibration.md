# Back-EMF Coast-Down Encoder Offset Calibration — Implementation Guide

**Audience:** an engineer/agent implementing this in firmware (and an offline analysis script).
**Goal:** determine the constant offset between the position sensor's zero and the motor's electrical zero, so that the Park transform produces correct `(d, q)` alignment.

---

## 0. Why this method, and what it does/doesn't depend on

We spin the motor up with a rough offset, then **switch the inverter fully off** (all six switches open, gate drivers disabled) and let the rotor **coast** while logging the three phase voltages (the back-EMF) and the encoder angle. From that data we compute the true offset.

Read this first, because it determines what you can trust:

- During measurement the inverter is **OFF**. No PWM, no current loop, no voltage modulation. The correctness of the result therefore depends on **only three things**: (1) the phase-voltage ADC chain, (2) the encoder reading + its velocity, (3) the pole-pair count. It does **not** depend on the current controller, current sensors, or any modulation calibration. This is the main reason to prefer it when the current loop is not fully trusted.
- The current controller (or any crude method — six-step, open-loop I/f, a sloppy `Iq` command) is used **only to spin the motor up**. Any error in the spin-up is irrelevant, because the actual measurement happens *after* the inverter is disabled and the rotor is freewheeling.
- The computed error angle is **independent of speed and of magnet flux magnitude** (they cancel — see §2). So it is robust to: speed ripple from cogging, the motor warming up (magnet strength drifting), and the exact coast speed. This is precisely why it beats static d-axis locking for a high-cogging / high-friction motor: a static lock reports "wherever cogging + stiction let the rotor settle," whereas this measures the electrical relationship while the rotor sweeps through all positions.

---

## 1. Conventions — lock these down before writing any math

Most bugs in this procedure are sign/convention bugs. **Do not invent new conventions. Match whatever the existing firmware's Clarke/Park already use.** Then verify the final update sign empirically (§7). The derivation below assumes a specific, common convention; if the firmware differs, re-derive or just rely on the empirical sign check.

Assumed conventions in this document:

- **Electrical vs mechanical angle:** `theta_e = p * theta_m - theta_off`, where `p` = pole pairs, `theta_m` = mechanical angle from the encoder (rad), `theta_off` = the offset we are solving for (electrical rad). `theta_e` is the angle fed into Park. (If your firmware *adds* the offset, flip the sign everywhere — the empirical check will catch it.)
- **d-axis** is aligned with the rotor permanent-magnet flux (north pole). **q-axis** leads d by +90° electrical in the direction of positive rotation.
- **Clarke (amplitude-invariant, 2/3 factor):**
  ```
  v_alpha = (2/3) * (v_a - 0.5*v_b - 0.5*v_c)
  v_beta  = (2/3) * ( (sqrt(3)/2)*v_b - (sqrt(3)/2)*v_c )
  ```
  With this scaling, a balanced set of phase voltages of peak amplitude `V_pk` maps to a vector of magnitude `V_pk`. (Phase voltages may be measured phase-to-ground; the αβ transform rejects the common-mode/zero-sequence term — see §6.)
- **Park (matching the Clarke above):**
  ```
  v_d =  v_alpha*cos(theta_e) + v_beta*sin(theta_e)
  v_q = -v_alpha*sin(theta_e) + v_beta*cos(theta_e)
  ```
- **Electrical speed:** `omega_e = p * omega_m`, signed. Positive `omega_e` ⇒ `theta_e` increasing.
- **Pole pairs `p` must already be correct.** A wrong `p` makes `theta_e` advance at the wrong rate and the result is garbage. Confirm `p` during the bootstrap step or a separate pole-pair check.
- **Encoder counting direction** (a.k.a. sensor direction sign) must already be established, i.e. you must know that increasing commanded `theta_e` corresponds to increasing encoder count (or have a `-1` baked in). The offset is only meaningful once direction is pinned. If unknown, determine it in bootstrap (command a slowly rotating current vector, observe whether the encoder counts up or down).
- All angle arithmetic is **mod 2π**; use `atan2` and circular statistics; never linearly average raw angles across the ±π wrap.

---

## 2. The math — why the d-axis back-EMF goes to zero, and the exact error formula

At **true open circuit** (inverter off, no diode conduction — see §5), no current flows: `i_d = i_q = 0`, and `di/dt = 0`. The PMSM stator voltage equations in the dq frame,
```
v_d = R*i_d + L_d*(di_d/dt) - omega_e*L_q*i_q
v_q = R*i_q + L_q*(di_q/dt) + omega_e*L_d*i_d + omega_e*psi_m
```
collapse to
```
v_d = 0
v_q = omega_e * psi_m
```
So with the **correct** offset, the measured back-EMF lies entirely on the q-axis: `v_d = 0`, `v_q = omega_e*psi_m`. **Calibration target: drive `v_d` to zero** (equivalently, maximize `v_q`). This resolves the common "is it d or q?" confusion — it is the **d-axis voltage you null**, not q.

### Closed-form error angle (better than blind minimization)

You can compute the angle error *directly* per sample; you do not have to sweep/minimize. The rotor flux in the stationary frame is `lambda = psi_m * (cos theta, sin theta)`, so the open-circuit terminal voltage equals the back-EMF:
```
v_alpha = -psi_m * omega_e * sin(theta)
v_beta  =  psi_m * omega_e * cos(theta)
```
where `theta` is the *true* electrical angle. Suppose our transform uses `theta_hat = theta + delta` (i.e. our estimate is wrong by `delta`). Substituting into the Park equations above and simplifying:
```
v_d = psi_m * omega_e * sin(delta)
v_q = psi_m * omega_e * cos(delta)
```
Therefore, with sign-normalization by the speed direction `s = sign(omega_e)`:
```
delta = atan2(s * v_d, s * v_q)
```
Key consequences:
- `psi_m` and `omega_e` **cancel** — `delta` is independent of speed and flux. (Robust to cogging speed ripple and thermal drift of the magnets.)
- The `sign(omega_e)` normalization keeps the formula valid in **both** rotation directions (without it, reverse rotation puts the vector in the opposite quadrant and `atan2` returns `delta ± π`).
- `delta = theta_hat - theta_true`. With `theta_e = p*theta_m - theta_off`, a `delta` means the stored offset is wrong by `-delta`, so the correction is:
  ```
  theta_off_corrected = theta_off_current + delta
  ```
  **Verify this sign empirically (§7).** It depends on your Park sign and on whether your offset is subtracted or added.

---

## 3. Why measure in both directions (iron loss + filter/sampling delay)

Two effects perturb the measured `delta`, and both are **proportional to `omega_e` and flip sign with rotation direction**, while the genuine offset error is constant (even in `omega_e`):

1. **Sensing-chain time delay.** Any group delay `tau` in the voltage anti-alias/sense filter, plus ADC latency, plus skew between when the encoder is read and when the voltages are sampled, all act as a time delay. A delay makes the measured voltage vector point where the true vector was `tau` ago, i.e. shifted by `omega_e * tau`. Net effect: `delta_measured = Delta_true + omega_e * tau`.
2. **Iron/core loss.** Core loss introduces a small `omega`-dependent quadrature component that likewise shifts the apparent angle and flips sign with direction. To first order it adds to the same `omega_e`-proportional term.

So, to first order:
```
delta_measured(omega_e) = Delta_true + k * omega_e
```
linear in **signed** `omega_e`, where `k` lumps the filter group delay, sampling skew, and iron-loss contributions. The genuine offset error `Delta_true` is the value at `omega_e = 0`.

**This is the whole justification for the recommended estimator below:** a coast-down naturally sweeps `omega_e`, so regress `delta_measured` against signed `omega_e` and take the **intercept at `omega_e = 0`**. That extracts `Delta_true` with the speed-dependent error removed, and the slope `k` quantifies your lumped delay (useful as a diagnostic — `k` is roughly your effective sensing delay in seconds, expressed as electrical-rad error per electrical-rad/s).

Doing **both** directions is the gold standard: forward (`omega_e > 0`) and reverse (`omega_e < 0`) samples lie on the *same line* `delta = Delta_true + k*omega_e`, so you can pool them into one regression and the intercept is doubly well-constrained and immune to any residual even-in-`omega` artifact. A single direction with enough speed range still yields `Delta_true` via the intercept — important for a high-friction motor where each coast is short.

---

## 4. Algorithm

### 4.1 State machine (firmware)

```
BOOTSTRAP      -> get a rough offset + confirm p and encoder direction
SPINUP_FWD     -> accelerate to omega_meas_max (forward)
INVERTER_OFF   -> disable all gates; brief settle (let any transient die, ~1-2 ms)
COAST_FWD      -> log samples while |omega_e| in [omega_min, omega_max]
SPINUP_REV     -> accelerate to omega_meas_max (reverse)   [optional but recommended]
INVERTER_OFF
COAST_REV      -> log samples (reverse)
COMPUTE        -> regress, get intercept Delta, update offset
VERIFY         -> re-measure short; check repeatability and closed-loop Id~=0
```

`BOOTSTRAP` can be a simple bidirectional d-axis lock (approach the lock angle from both directions at the highest thermally-safe lock current and average — this is "good enough" to spin in velocity mode and to pin `p`/direction). It does **not** need to be accurate; the coast-down does the precision work.

### 4.2 Per-sample logging during COAST (the critical inner loop)

For each sample, captured with the encoder and all three ADCs **time-aligned** (§5):

```
theta_m  = encoder_angle_rad()           // mechanical
omega_e  = pole_pairs * encoder_speed()   // signed electrical speed
theta_e  = pole_pairs*theta_m - theta_off_current   // current (rough) offset

(v_a, v_b, v_c) = read_phase_voltages()   // volts, ADC-scaled, zero-corrected
clarke -> (v_alpha, v_beta)
park(theta_e) -> (v_d, v_q)

s = sign(omega_e)
delta_i = wrap_pi( atan2(s*v_d, s*v_q) )

if (omega_min <= abs(omega_e) <= omega_max) and (mag(v_alpha,v_beta) > v_snr_min):
    log( omega_e, delta_i )
```

Notes:
- Use the **current rough offset** when computing `theta_e` for logging. The regression then returns the *residual* error `Delta`, which you add to the rough offset. (Alternatively log with `theta_off = 0` and the regression returns the full offset directly. Either is fine — be consistent.)
- Gate on speed band (`omega_min`..`omega_max`) for SNR and to stay below the over-voltage/diode-conduction limit (§5). Discard the low-speed coast tail where SNR collapses.
- Optionally also gate on a minimum voltage-vector magnitude as a redundant SNR check.

### 4.3 COMPUTE

Pool forward + reverse samples and do one linear regression of `delta_i` vs **signed** `omega_e,i`:
```
delta_i ≈ Delta + k * omega_e,i
```
- `Delta` (intercept at `omega_e = 0`) = genuine offset error → `theta_off_new = theta_off_current + Delta` (sign per §7).
- `k` (slope) = lumped sensing delay; report it as a diagnostic. If `|k|*omega_max` is more than a few degrees, your filter/sampling delay is significant and worth reducing or compensating in the running controller too.
- **Sanity gates:** fit forward-only and reverse-only intercepts separately as a check; they should agree to within ~1°. Large disagreement (or a slope wildly different between directions) signals a pole-pair error, a direction-sign error, ADC scaling/offset asymmetry, or mismatched per-channel filters — fix that before trusting the number.

**Simple fallback (v1, if you don't want regression yet):** collect a narrow speed band at the *same* `|omega_e|` forward and reverse, take the circular mean of `delta_i` in each direction, and average the two means. This cancels the `k*omega_e` term at that speed but throws away the slope diagnostic and is more sensitive to the exact band. Prefer the regression.

### 4.4 Recommended bring-up workflow: log first, analyze offline

Because you have a coding agent and dedicated ADCs, **don't close any loop on day one.** First implement only the logging (BOOTSTRAP → SPINUP → INVERTER_OFF → COAST while streaming `(timestamp, theta_m, omega_e, v_a, v_b, v_c)` over your debug link). Do Clarke/Park/regression in a Python script offline. This lets you:
- visually confirm `v_alpha`/`v_beta` trace a clean circle vs encoder angle (validates ADC scaling, phase order, filter matching),
- confirm `v_d ≈ 0` after applying the computed offset,
- confirm the `delta` vs `omega_e` line is straight (validates the model and your timing),
before you ever write the offset back automatically. Once the offline result is solid and repeatable, port the regression into firmware.

---

## 5. Pitfalls (most failures live here)

**Open circuit must be genuine — beware body-diode rectification.** When the inverter is off, the FET body diodes form a passive rectifier. If the **peak line-to-line back-EMF** exceeds `V_bus + 2*V_diode`, the diodes conduct, current flows, the `R*i`/`omega*L*i` drops reappear and corrupt the measurement (and you dump regen into the DC bus). For sinusoidal back-EMF, `V_ll_peak ≈ sqrt(3) * V_phase_peak`, and `V_phase_peak = omega_e * psi_m` (= `|v_dq|` with amplitude-invariant Clarke). So keep
```
omega_max  <  (V_bus - margin) / (sqrt(3) * psi_m)
```
This bounds the coast speed from above. SNR bounds it from below. Pick a band comfortably inside both. Add an **over-voltage abort** (monitor `V_bus`) in case the rotor is spinning faster than expected when you cut the inverter.

**Floating neutral / common mode.** With three phase-to-ground dividers and FETs off, each measured node = (floating neutral) + (that phase's back-EMF). The neutral common-mode is rejected by the αβ (Clarke) transform's insensitivity to the zero-sequence component, so the math is fine — **provided** the three dividers are well matched and the ADC range accommodates the common-mode swing without clipping. Validate by checking that `v_a + v_b + v_c` is a small/consistent zero-sequence and that αβ traces a clean circle. If you sense line-to-line instead of phase-to-ground, use the corresponding transform.

**Per-channel filter mismatch.** The angle-error cancellation in §3 assumes the three voltage channels have **identical** group delay. Mismatched RC anti-alias filters inject an angle error that does not cleanly cancel and distorts the circle. Match the components (and ideally measure/trim).

**Synchronous sampling and encoder/voltage time-alignment.** Sample all three phase voltages simultaneously (or correct for inter-channel skew), and read the encoder at the *same instant* as the ADC conversion. Any skew `dt` adds `omega_e*dt` of angle error — it lands in the slope `k` and cancels in the intercept, but minimizing it tightens the fit and reduces reliance on the bidirectional cancellation.

**ADC zero/offset and gain.** Calibrate each voltage channel's zero (sample with the system de-energized so a true 0 V reads 0 counts) and its gain. A DC offset on `v_alpha`/`v_beta` biases `v_d`/`v_q` and hence `delta`. A gain error doesn't bias the *angle* (good — angle is ratiometric) but corrupts any voltage-magnitude SNR gate.

**Velocity estimate quality.** `omega_e` is used for the speed gate and is the regressor — it must be reasonably accurate and low-latency. Beware quantization/noise at the low end of the coast; that's another reason to gate out the low-speed tail.

**Pole-pair and direction errors dominate everything.** If `p` is wrong, `theta_e` advances at the wrong rate and nothing converges. If the encoder direction sign is wrong, the offset is meaningless. Pin both in bootstrap and let the forward/reverse intercept-agreement check flag residual mistakes.

**High friction ⇒ short coast window.** Your stated regime (high friction relative to max current) means the rotor may decelerate quickly after the inverter is cut, giving a brief measurement window. Mitigations: spin up to a higher (but still sub-over-voltage) speed to get more usable speed-range; log at a high sample rate to gather enough points; rely on the single-direction regression intercept (works from one coast); and, if needed, add inertia (a flywheel) during commissioning. The good news: because `delta` is speed-independent, even a short coast across a modest speed band yields a valid intercept.

**Cogging during coast does not bias the result.** Cogging causes torque ripple → speed ripple → an oscillating `omega_e`. Since `delta` is computed per-sample and is independent of `omega_e` magnitude, this ripple only adds scatter (removed by averaging/regression), not bias. This is the core advantage over static locking for your motor.

**Mechanical:** rotor must be free to coast — no brake engaged, no gravity/spring load biasing it, no heavy external coupling that the coast can't drive. Mass and light friction loads are fine.

**Safety:** a freewheeling motor is a generator. Ensure the gate-off is clean (no partial PWM, drivers actually disabled), expect bus voltage to rise during coast, and keep the over-voltage abort armed. Treat the DC link as live.

---

## 6. Quick reference — equations in one place

```
Conventions
  theta_e = p*theta_m - theta_off          (electrical angle into Park)
  omega_e = p*omega_m                       (signed)

Clarke (amplitude-invariant)
  v_alpha = (2/3)*(v_a - 0.5*v_b - 0.5*v_c)
  v_beta  = (2/3)*((sqrt(3)/2)*(v_b - v_c))

Park
  v_d =  v_alpha*cos(theta_e) + v_beta*sin(theta_e)
  v_q = -v_alpha*sin(theta_e) + v_beta*cos(theta_e)

Open-circuit truth (correct offset)
  v_d = 0 ;  v_q = omega_e * psi_m

Per-sample angle error  (s = sign(omega_e))
  delta = wrap_pi( atan2(s*v_d, s*v_q) )

Model across the coast
  delta(omega_e) = Delta_true + k*omega_e
  Delta_true = intercept at omega_e = 0   (the answer)
  k          = lumped sensing delay (diagnostic)

Update (verify sign empirically)
  theta_off_new = theta_off_current + Delta_true

Speed band
  omega_min  <=  |omega_e|  <  (V_bus - margin) / (sqrt(3) * psi_m)
```

---

## 7. Validation & acceptance

1. **Empirical sign check (do this once).** After computing an offset, deliberately corrupt the stored offset by +5° and run a short measurement. The regression intercept should move by **−5°** (negative feedback: the loop wants to remove the error you injected). If it moves by +5°, flip the sign of the update in §2/§6. This guards against any Park-convention or add-vs-subtract mismatch.
2. **Repeatability.** Re-run the full forward+reverse calibration a few times. Intercepts should agree to ~1° electrical (mirrors the field rule of thumb: if it varies more than ~1° between runs, raise measurement speed / fix timing / check ADC scaling before trusting it). VESC-style guidance: if it won't settle, you usually have an SNR or scaling problem, not a fundamentally wrong method.
3. **Forward/reverse intercept agreement.** As in §4.3 — disagreement points at `p`, direction, ADC asymmetry, or filter mismatch.
4. **Closed-loop confirmation.** With the new offset, run the motor in closed loop commanding `Id* = 0` at light load and modest speed; the *measured* `Id` should sit near zero. If `Id` is biased, the offset is off (or — separately — your current sensing is, which is exactly the part you don't fully trust, so corroborate against the coast-down value rather than the other way around).
5. **Cross-check vs bootstrap.** The coast-down offset should be within a few degrees of the rough bootstrap lock value. A wild disagreement almost always means a pole-pair or direction error upstream, not a bad coast-down.

---

## 8. Provenance / further reading

This `v_d → 0` open-circuit principle is the same one sensorless flux/PLL observers use (they hold the d-axis back-EMF at zero in steady state). The coast-down *commissioning* form of it is the documented gamma-adjust procedure in Rinehart Motion Systems (now Cascadia Motion / BorgWarner) EV traction inverters — spin above ~1000 RPM, command the inverter off, read the back-EMF-vs-resolver angle as it coasts, calibrating to +90° forward / −90° reverse (i.e. back-EMF on the q-axis), in both directions to cancel iron-loss and filter skew. The same `U_d = 0`, two-direction (forward/reverse symmetry) method is described in the UltraZohm open-source platform's encoder-offset documentation, and the underlying machine-model identification in Rahman & Hiti, "Identification of Machine Parameters of a Synchronous Motor," IEEE Trans. Industry Applications, 2005. Note that the mainstream open-source motor controllers (ODrive, VESC, SimpleFOC, moteus, TI sensored, ST) calibrate the offset with **static locking**, not coast-down — so this approach, while standard in automotive traction drives, is somewhat novel in the hobby/robotics ecosystem.
