# High-speed FOC worklog — lumpy current investigation

Living scratchpad for the "current looks lumpy/shitty at high speed" investigation
(LMG2100R044 GaN bridge). Pick-up notes, design sketches, and open questions so we
don't re-derive everything. Started 2026-06-08.

## Symptom

FOC works fine at low/moderate speed; phase current gets lumpy/irregular at high
speed. Question on the table: is switching or dead time wrong? **No** — see below.

## Diagnosis summary (ranked causes)

1. **Voltage-vector angular quantization (primary suspect).** SVM/`inv_park`/compares are
   recomputed once per fast-loop ISR and the HRTIM holds them flat for all cycles between
   rep events (`hrtim.c` `PreloadEnable` + `UpdateOnRepetition`, `RepetitionCounter`). So the
   applied voltage vector steps by `Δθ = ω_e · T_isr` and becomes a coarse polygon at high
   `f_e`. 96 kHz switching does **not** help — all cycles in an ISR share one compare set.

   Samples (= vector updates) per electrical rev, `f_e = RPM/10` (6 pole pairs):

   | RPM    | f_e    | @16 kHz | @32 kHz | @96 kHz (per-PWM) |
   |-------:|-------:|--------:|--------:|------------------:|
   | 10,000 | 1.0kHz | 16      | 32      | 96                |
   | 20,000 | 2.0kHz | 8       | 16      | 48                |
   | 30,000 | 3.0kHz | 5.3     | 10.7    | 32                |
   | 40,000 | 4.0kHz | 4       | 8       | 24                |

   Below ~10 updates/rev it gets visibly lumpy, and the non-integer ratio makes it wander
   ("shitty" vs clean ripple). → addressed by the 32 kHz change + (later) per-PWM updates.

2. **Current-sample CM corruption at high modulation.** ADC samples at the crest (`hrtim.c`
   CMP4=20768, ADROM_CREST), ~1.7 µs 4× oversample window. High speed → high modulation →
   the extreme-duty phase's switching edge marches into that window → INA296 CM kick corrupts
   that phase's reading. Rotates through phases at `6·f_e` → lumpy. → addressed by best-2-of-3
   shunt selection (this worklog, Q2).

3. **Minor / conditional:** voltage-ceiling saturation (no field weakening — `Id` hard-forced
   to 0 in `set_iq_setpoint`), empirical angle advance (`foc.c:184` `2/ENC_MEAS_FREQ`, has a
   "why 2?" TODO), all feed-forwards off (`config.h` FOC_USE_*).

## Dead time — RESOLVED, not the problem (don't re-litigate)

`hrtim.c` dead time uses HAL `..._PRESCALERRATIO_DIV8` whose **comment is inverted**
(`fDTG = fHRTIM/8` is wrong; RM0440 p891/p1028: `tDTG = 2^DTPRSC·(tHRTIM/8)`, DIV8 = DTPRSC 0).
So `tDTG = (1/144MHz)/8 = 0.868 ns`, `RisingValue/FallingValue = 0xC = 12` → **dead time ≈ 10.4 ns**.
Correct/slightly-generous for GaN (no reverse recovery); distortion ~0.1% of period and not
speed-dependent.

**VERIFIED 2026-06-09, three levels + datasheet cross-check:**
- Source (`hrtim.c:216-236`): DTPRSC=DIV8(=0), rising=falling=0xC, both signs POSITIVE,
  applied to TIMER A/B/E; `DeadTimeInsertion=ENABLED`.
- RM0440 (p891/p911/p1028): `tDTG = 2^DTPRSC × tHRTIM/8`; fHRTIM = 144 MHz (HSE16×18/2,
  AHB/APB2 ÷1) → tDTG = 868 ps → **tDTR = tDTF = 12 × 868 ps = 10.42 ns**.
- Live silicon (SWD): DTAR=DTBR=DTER=`0x000C000C`, OUTxR.DTEN=1 on all three timers;
  TIMxCR2.UDM=1 (center-aligned, 96 kHz period) — DTG clocks off the kernel clock, so the
  dead time is mode/CKPSC-independent.
- LMG2100R044 datasheet (SNOSDF9B §7.3/7.4): **NO internal overlap protection** — HI=LI=1
  turns BOTH FETs on ("will result in a shoot-through condition, which can permanently
  damage the device"). The HRTIM positive-sign dead time is the ONLY shoot-through barrier.
  Delay matching t_MON/t_MOFF = 2 ns typ / **8 ns max** → effective dead time at the FETs =
  10.42 ns ± mismatch: **~8–12 ns typical, ≥2.4 ns at the worst-case spec corner** — still
  non-overlapping everywhere. TI's own designs: 8 ns (datasheet §8.2.1 sample), 16.6 ns
  (TIDA-010936 motor drive); §7.3 explicitly targets "<10 ns" dead times. 3rd-quadrant loss
  during DT (V_SD≈2 V) ≈ tens of mW at our currents/96 kHz — negligible.
- Flight hardening candidate: the DTxR write-once LOCK bits (value+sign, rising+falling) are
  currently UNLOCKED — locking after config would make the only shoot-through barrier
  un-clobberable until reset.

NB: a naive reading of the HAL comment gives 667 ns — if anyone ever "fixes" the dead time
upward to that, that *would* cause distortion (but not damage). The DANGEROUS edit direction
is sign=NEGATIVE (deliberate overlap) or DT=0 — the gate driver provides no safety net.

## DONE

### 32 kHz fast loop (pending bench test — flashing tomorrow)
- `pwm.h` `PWM_CYCLES_PER_ISR 6u → 3u`; `hrtim.c` `RepetitionCounter 6-1 → 3-1` (shared
  `pTimeBaseCfg`, so TIMER_A/B/E all move together). **These two MUST stay equal** — no
  compile-time link. Switching freq unchanged (96 kHz); only ISR/control rate moved 16→32 kHz.
- All FOC/PLL/vel constants auto-rescale (derive from `LOOP_UPDATE_FREQ`/`SECONDS_PER_LOOP`/
  `CURRENT_MEAS_FREQ`). The angle advance stays "2 cycles" (now 62.5 µs absolute).
- **To revert:** set both back to 6 / 6-1.
- **Verify on HW:** scope `debug_hrtim_isr` pin high-time ÷ 31.25 µs = CPU frac (est ~45–50%);
  watch for re-entry if it nears the period. Re-sweep the advance `2` (`foc.c:184`). Compare
  lumpiness vs 16 kHz at the same RPM.
- **MEASURED 2026-06-09** (`isr_cycles_last/max`, DWT counters in control_ll.c, SWD-readable):
  - RelWithDebInfo (CCM hot path, -Ofast): steady-state tick ≈ **3246 cycles = 22.5 µs = 72%**
    of the 31.25 µs budget. Higher than the 45–50% estimate — headroom for Q1's per-PWM fast
    path is thinner than hoped; the DMA-compare variant looks more attractive than the
    96 kHz-ISR variant.
  - **Debug (-Og, flash-resident) does NOT fit at 32 kHz**: arming any FOC state saturates the
    pri-0 ISR → SysTick (pri 15, sole IWDG feeder) starves → IWDG reboot ~512 ms after arm.
    (This was the "enter VEL_RAMP → reboot" mystery; vel_ramp wasn't special — its exec adds
    <1% over torque/vel.) Bench optimized builds, or locally drop Debug to 16 kHz.
  - The **arm tick costs ~141k cycles (~1 ms ≈ 31 dropped ticks)**: control_law_init runs
    in-ISR and init_foc printfs two lines over blocking UART. Excluded from isr_cycles_max.
    **FIXED 2026-06-09**: ISR printf is now deferred (console.c ring + foreground pump; see
    docs/footguns.md) and the init_foc gain prints moved to boot (`foc_print_gains`) — the arm
    tick is now an ordinary tick. Captures no longer have the 1 ms post-arm gap.

### Best-2-of-3 shunt selection (this session — default OFF, A/B gated)
What: at high modulation, drop the CM-corrupted phase (smallest SVM magnitude → largest
compare = `PWM_CNTR·(1-mag)` → edge nearest crest sample) and reconstruct it from the trusted
two via KCL (`Ia+Ib+Ic=0`), instead of the symmetric `zero/3` common-mode removal.

Files:
- `config.h`: `FOC_USE_SHUNT_SELECTION` (default false, seeds runtime flag) + `SHUNT_SEL_DELAY_TICKS`.
- `control_ll.c`: `shunt_untrusted_hist[]` ring written in `pwm_update_callback` (argmin of SVM
  mags at duty-latch time); consumed in `adc_update_callback` `SHUNT_SEL_DELAY_TICKS` ticks later.
  Runtime flag `shunt_selection_enabled` (volatile, flip over SWD for live A/B). Debug global
  `last_shunt_drop_phase` (-1/0/1/2).
- `foc_capture` (`foc_capture.h`, `foc.c`, `tools/decode_foc_capture.py`): added `shunt_drop`
  field (struct 88→92 B) so alignment is verifiable offline.

Why a runtime flag + the ring stays warm even when disabled: lets you toggle live on the bench
without reflash and without a cold-ring transient.

**✅ Q2-A RESOLVED (measured 2026-06-09): pipeline alignment.** Currents are read live from
`JDR1` (`inverter_get_adc_raw`) while the current crest's conversion is still in flight, so the
sample reflects a duty written ~2 ISR ticks earlier. With the natural 1-tick call ordering (adc
runs before pwm in the ISR), `SHUNT_SEL_DELAY_TICKS = 1` (one history slot back) is the
predicted-correct value — **and the bench measurement agrees: multi-edge torque-step
(`tools/shuntsel_delay.sh`, docs/shuntsel-delay-calibration.md) measured K=2 with 1.6× fit
margin → DELAY=1.** (Side observation from the same fit: closed-loop current τ ≈ 9.5–11 ticks
vs the nominal 8 from `CURRENT_CONTROLLER_BANDWIDTH` — consistent with real L running ~25-35 %
above `HARDCODE_PHASE_INDUCTANCE`.) The stakes if it had been wrong: at high speed the SVM
sextant flips every ~2 ticks, so a 1-tick error ≈ 50 % wrong-phase drops. Re-measure if the loop
rate, ISR ordering, ADC timing, or CMP4 changes.

How to validate:
- **Direct measurement (feature OFF, do this first):** `tools/shuntsel_delay.sh run` +
  `shuntsel_delay_analyze.py` — torque-step while foc_capture records; counts the ticks between
  the duty command and the measured-current response (K), recommends `DELAY = K − 1`. See
  `docs/shuntsel-delay-calibration.md`.

Then confirm offline, from a `foc_capture` dump with the feature ON:
- For each row T, `shunt_drop` should equal `argmin(svm_A,svm_B,svm_C)` from row `T-2`
  (the duty written 2 ticks earlier). Scan a few DELAY values (0/1/2) and pick the one where
  reconstructed `Iq`/phase current is smoothest and the dropped phase tracks the extreme duty.
- On a scope: trigger on the ADC-trigger GPIO, watch the suspect (extreme-duty) phase's INA296
  output settle — confirms whether corruption is even present (the whole premise) and how much
  margin the in-line + INA296 choice actually buys.

Edge cases / notes:
- `shunt_untrusted_head` not reset on arm/disarm — first ~2 ticks after rearm read stale
  history, but modulation is ~0 at arm so harmless.
- `inverter_set_processed` (telemetry) still gets pre-reconstruction currents — left as-is.

## TODO / NEXT

### Q1 — per-PWM `inv_park` + `SVM` (the proper fix for cause #1)
Goal: keep the heavy current loop (Clarke→Park→PI) at the ISR rate, but update
`inv_park`+`SVM`+compares every PWM period (96 kHz) from the extrapolated angle. `mod_dq` is
slow (rotor-frame, ~DC); only the angle is fast. Three design axes:

1. **Spin the angle cheaply — phasor stepping.** Precompute one rotation `R(Δθ)`, `Δθ = ω_e·t_pwm`,
   in the slow tick; iterate `v_{k+1} = R(Δθ)·v_k` (complex multiply, no per-cycle sincos).
   Re-seed from the fresh `inv_park` each slow tick so magnitude drift can't accumulate.
   (CORDIC peripheral available if real sincos preferred.) Extrapolate to the cycle **midpoint**
   `(k + ½)` + existing transport advance.
2. **Deliver the update at 96 kHz:**
   - 96 kHz ISR + modulo (`if (k%N==0) current_loop(); always update_output();`). Simple, but
     M4F exception entry/exit ~0.3–0.5 µs *every* 96 kHz tick roughly doubles the cost (~6–10%
     CPU for the fast path alone).
   - DMA-streamed compares (elegant endpoint): slow tick precomputes N compare-triples into 3
     small circular buffers; one DMA channel per phase-timer copies the next triple to `CMPx1`
     on each period/rep event. ~zero per-cycle CPU, more setup.
3. **Hazards:** snapshot `mod_dq`/`ω_e` once at the top of the fast burst; compares latch at the
   period boundary before their cycle (so extrapolate to that cycle's crest); ADC still samples
   once per current-loop tick.

Recommendation: prototype the phasor-stepping fast path behind a config flag (A/B vs the held
vector). Worth it even after 32 kHz — it's the scalable lever (no extra ADC/PI cost). Sketch
first, gate it, measure CPU.

### Cleanup items found along the way (low priority)
- ~~`encoder_calibration.c:306` `ENC_VAL_LOOPS_PER_POINT 32000` hardcoded~~ **FIXED** (2026-06-08):
  now `(u32)(2.f * LOOP_UPDATE_FREQ)` = 2 s/point at any rate. (Found during the post-32 kHz delta
  review; the encoder-validation dwell had silently halved.)
- `inverter.c:31-45` comment block is stale (325 kHz, timer B/F CMP3, INA240) — doesn't match
  current HRTIM (timer A CMP4) or hardware (INA296). `ADC_POSTSCALER = 12-1` (`:54`) is vestigial
  (HRTIM postscaler is actually `0x0` in `hrtim.c`; conversions fire every crest).
- `config.h` `FOC_USE_DECOUPLING` is a **dead flag** — referenced nowhere (real decoupling is
  `FOC_USE_wL_FF`, which is off).
- `PWM_CYCLES_PER_ISR` (`pwm.h`) and `RepetitionCounter` (`hrtim.c`) are duplicated constants with
  no compile-time link — commented now, but a regen of `hrtim.c` from the .ioc would silently
  revert it. If 32 kHz becomes permanent, update the .ioc too.

## OPEN QUESTIONS (answer these to focus the work)
- **At what RPM / bus voltage does lumpiness start?** Tells us cause #1 (update-rate) vs #2
  (sampling) vs #3 (saturation). Still unanswered.
- Q2-A pipeline alignment (above) — the critical one for shunt selection.
- Are we voltage-saturated at the onset speed? Compare captured `mod_d/mod_q` magnitude vs the
  0.857 SVM limit (`foc.c:162`). If yes → fundamental (more bus / field weakening), not these fixes.
- Does the 32 kHz build hold timing on HW (CPU headroom)?
- LMG2100R044 datasheet min dead time vs our ~10.4 ns — confirm (almost certainly fine).
