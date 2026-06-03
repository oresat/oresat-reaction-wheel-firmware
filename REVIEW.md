# oresat-acs-fw-v8 — review notes

Three questions from the original developer (you), with deep-dive answers
referenced against:

- ST RM0440 — STM32G4 reference manual (`resources/stm32g4_reference_manual.pdf`)
- ST AN4539 — HRTIM cookbook (`resources/stm32_hrtim_cookbook.pdf`)
- The current source tree

PDF page numbers below refer to those PDFs.

---

## Baseline observations (read these first)

A few timing facts that will keep coming up:

- HRTIM1 kernel clock @ 144 MHz × DLL multiplier 32 = **4.608 GHz** counter
  resolution.
- Slave timers A/B/E configured as **up-down** (`UDM=1`), `PER=24000`.
  An up-down "lap" (valley→crest) is 24000 clocks; a full **switching cycle**
  (valley→crest→valley) is 48000 clocks → **96 kHz** actual switching freq.
- `pwm.h:#define PWM_FREQ 192000` is misleading — it's the per-direction
  period-event count (192k crests per second), not the switching frequency.
  Switching freq is 96 kHz. ENC_MEAS_FREQ math `192000/12 = 16000` happens to
  come out right because the user multiplied "events per second" by half the
  RCR.
- `HRTIM_TIM_ROM_CREST` + RCR=5 (`hrtim.c:148`) → repetition event fires every
  6 crests, i.e. once every 6 switching cycles → **HRTIM ISR runs at 16 kHz**.
- All ISRs set to NVIC priority `(0, 0)`. No preemption anywhere. (Issue 3l.)

---

## Question 1 — ADC trigger sync

> I was never quite able to get the ADC triggering (for current sense) to
> synchronize correctly. I'm currently using injected channels on the period
> of the HRTIM up-down count, so it kicks off the sampling at the center of
> the PWM. I wanted to only sample once per e.g. 6 PWM cycles (using RCR and
> normal ADC trigger), but it seems the ADC's RCR decrement got misaligned
> from the HRTIM interrupt's RCR. […] Maybe you could check my code and
> recommend a better solution? I think it would be nice to be able to (A)
> offset the ADC S&H start to a bit before the PWM center point, so the S&H
> latch is right at the center instead of a little after the center, and (B)
> maybe even do a burst sample via DMA and then average it later, to reduce
> ADC / current sense op amp noise. And also, am I even sampling at the middle
> of the PWM waveform now? Are my current readings just totally wrong lol?

Three subquestions: (i) is the postscaler issue real and what's the fix,
(ii) where is the sample currently happening, (iii) S&H pre-offset and burst.

### 1a. The postscaler issue is real, and unrelated to the bigger problem

The forum thread you linked is correct. The HRTIM ADC postscaler counter is
**not** the same hardware as the timer's repetition counter (RCR). The
postscaler lives inside the `HRTIM_ADCxR` trigger logic and counts trigger
*events*; the RCR lives inside the timer and counts rollover events. They are
both reset on chip reset, but **stop/start of the timer (`TxCEN`) only resets
the timer's RCR — the ADC trigger's postscaler keeps its state**. Result:
after every disarm/arm cycle, the postscaler offset relative to the RCR is
random.

So if you set timer A's RCR=5 (6 cycles per IRQ) and ADC postscaler=5 (6
events per ADC trigger), both will fire at 16 kHz, but *not necessarily at the
same crest within the 6-cycle window*. Worst case: ADC samples one cycle, ISR
runs five cycles later — current is stale.

The "fix" people suggest of resetting the postscaler doesn't really exist
(there's no reset bit for the AD1USRC postscaler counter once HRTIM is
enabled, only at MCR/timer-disabled state). So it's not just a software
cargo-cult — it's a hardware misfeature.

### 1b. Where the sample currently happens

> **Hardware update from author** (post-write of original analysis): shunts
> are **in-line** (not low-side), with **INA296** (not INA240). Author
> picked INA296 specifically because its CMRR settling time is fast enough
> to recover before the next sample window, and is OK eating the switching
> CM-kick because he doesn't sample around it.
>
> This invalidates the "biggest fix in the codebase" framing I had here
> originally. Updated analysis below.

Tracing the trigger:

- ADC trigger source for all four injected channels (phase A/B/C + Vbus):
  `HRTIM_ADCTRIGGEREVENTxx_TIMERA_PERIOD`  (`hrtim.c:64,74,84,94`).
- `HRTIM_TIM_ADROM_CREST` is set on Timer A (`hrtim.c:162`). Per RM
  §28.3.13 (p919, table 238 + p920) and RM fig 240/241 (p920–921):
  - `ADROM[1:0] = 10` → ADC-relevant rollover event = "crest mode" — **fires
    only when counter reaches `PER` (top of triangle)**.
  - `ADROM[1:0] = 01` would be valley.
- Net: ADC injected sample fires every up-down period at the **crest**
  (counter = PER = 24000), once per switching cycle (postscaler 0).

What's happening on the inverter at the crest, given output config is
`SetSource = ResetSource = TIMCMP1` (`hrtim.c:224–225`) + up-down +
`GreaterCMP1 = EQUAL` + `OUTROM = BOTH`:

- TA1/TB1/TE1 (high-side) is HIGH from when up-going counter crosses CMP1
  to when down-going counter crosses CMP1 → **HIGH around the crest**.
- TA2/TB2/TE2 (low-side) is the complement (with deadtime) → **HIGH around
  the valley**.

**For in-line shunts**: current flows through the shunt continuously (so
long as the motor is energized), and the half-bridge switching causes a
common-mode kick on the amp's inputs (Vphase swings 0↔Vbus at every CMP1
crossing). The two "settled" sample points per up-down cycle are the
**crest** (midpoint of the high-side-ON window, all three phases'
midpoints' centroid is at the crest) and the **valley** (midpoint of the
low-side-ON window). Both are equidistant from any switching edge, so for
in-line topology there's no fundamental crest-vs-valley preference.

Where it does matter:

- **Sample window asymmetry at extreme duty.** The simultaneous "all three
  phases in same state" window is `min(min duty, 1 − max duty) × period`
  centered on the crest/valley. For a balanced motor at low modulation
  (all phases near 50%), both crest and valley have ~T/2 ≈ 5 µs of clean
  window. At high modulation (e.g. all phases > 90% duty), the high-side
  ON window is wide and the low-side ON window collapses to ~10% × 10 µs =
  1 µs — sampling at the valley becomes risky, sampling at the crest is
  fine. Symmetric story at low modulation.
- **Cookbook §12 (3-phase PFC, p59-61, fig 39) recommends the valley** for
  *low-side* topology, where it's not just a noise question but a
  fundamental "shunt only sees current during low-side ON" question. That
  argument doesn't apply to in-line.
- **Cookbook fig 48 (p75)** shows the canonical "ADC trigger (counter
  roll-over)" annotation pointing at the valley. That's the up-down-mode
  default the AN documents, not necessarily the right choice for every
  topology.

So: **the current sampling at the crest is most likely fine for your
hardware**. Move on to other suspects (Q3a offset swap, Q3l priority,
deadtime distortion in §3w below, encoder phase calibration, current PI
tuning) before changing this.

The one nuance worth flagging: at very high modulation (Iq > MOTOR_CURRENT_CMD_LIMIT
and bus voltage low → mod_scalefactor active), all three phases will have
duty > 95%, the low-side ON window approaches 0, and the `min(min duty,
1 − max duty) × period` window at the *crest* becomes
`(1 − max duty) × period`. If max duty hits 99.5% (worst case post-clamp
in `pwm.c`), that window is 50 ns at the crest — and 1.74 µs at the valley.
Wait, that's backwards. Let me redo: at max duty 99.5%, the high-side ON
window is 99.5% of period = 10.35 µs centered on crest. So sampling at
the crest is well inside a 10 µs window. Fine. The asymmetry only bites
in the OPPOSITE direction (very low duty → tiny crest window). For BLDC
applications you're rarely there steady-state.

### 1c. The fix — use the master timer

Cookbook §12 spells out the canonical pattern. Adapted to your code:

1. **Reconfigure master timer** to be the system clock for the inverter:
   - `MasterPeriod = 24000` (same prescaler as slaves, MUL32; one master period
     = one slave up-down cycle).
   - `MasterRCR = 5` if you want the master fire every 6 switching cycles
     (matches your 16 kHz ISR).
   - Master is up-counting (it doesn't have an up-down mode). Master period
     event happens once per master period.
2. **Slave timers A/B/E**: keep up-down mode, but add a reset trigger from
   master:
   - `pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;`
   - This forces all three slaves to reset their counters to 0 (= valley)
     each time the master period rolls over. Master and slaves are now
     phase-locked.
3. **ADC trigger source**: change to
   `HRTIM_ADCTRIGGEREVENT13_MASTER_PERIOD` /
   `HRTIM_ADCTRIGGEREVENT24_MASTER_PERIOD`. Postscaler stays 0 (we're already
   dividing via MasterRCR).
   - Master period fires when slave counters are at the valley → ADC samples
     at low-side ON midpoint. ✓
4. **ISR source**: change `HAL_HRTIM_REPETITIONEVENTCALLBACK` registration to
   the master timer (`HRTIM_TIMERINDEX_MASTER`) instead of timer A. The
   master's RCR rollover is the same physical event that fires the ADC
   trigger, so they cannot drift.

That puts ADC-trigger and ISR on the same counter → the postscaler vs RCR
phase ambiguity is gone by construction.

If you don't want to do the full master-timer restructuring, the smaller fix
that gets you sampling at the valley is:

- Just flip `HRTIM_TIM_ADROM_CREST` → `HRTIM_TIM_ADROM_VALLEY` in the
  `HAL_HRTIM_RollOverModeConfig(... HRTIM_TIMERINDEX_TIMER_A ...)` call
  (`hrtim.c:161-163`).
- Leave ROM_CREST for `ROM[1:0]` (RCR/IRQ). They can be different —
  `xxROM[1:0]` is per-destination (table 238, RM p919).

This will move your sample point to the valley without restructuring. You
still have the postscaler-vs-RCR drift problem if you ever add a postscaler,
but you'll be sampling the right point and will get real currents.

### 1d. Pre-offsetting the trigger so the S&H *latches* at the center

Each ADC channel has `SamplingTime = 2.5` cycles (`adc.c:102`). At
`ClockPrescaler = SYNC_PCLK_DIV4` (`adc.c:57`) → ADC clock = SYSCLK/4 = 36
MHz → one ADC cycle = 27.8 ns → 2.5 cycles ≈ 70 ns. The S&H "latches" roughly
at the midpoint of the sample window, ~35 ns after trigger.

To sample *exactly at* the valley, fire the trigger 35 ns *before* the
valley. In HRTIM clocks: 35 ns × 4.608 GHz = **161 clocks** before counter=0
(or before counter=PER if you flip to crest).

Cleanest way to do this with the master timer setup above:

- Set `MasterCMP1 = MasterPeriod - 161 = 24000 - 161 = 23839`.
- Use `HRTIM_ADCTRIGGEREVENTxx_MASTER_CMP1` as the ADC trigger event,
  instead of `MASTER_PERIOD`.
- Keep ISR on master period (slightly later than ADC trigger), so the ISR
  reads JDR1 *after* the conversion completes.

Without master restructuring: do the same trick on timer A. Use
`TIMERA_CMP3` as the ADC trigger source (CMP3 isn't otherwise used in your
crossbar) and program CMP3 = 161 (in valley-mode interpretation, i.e. fire
when down-going counter hits 161, ~35 ns before reaching 0). Note: in
up-down mode, CMP3 will fire on both directions through 161 unless you mask
one with the `HRTIM_TIM_ADROM_*` setting. Set `ADROM_VALLEY` so only the
valley-side crossing counts as a rollover event for ADC purposes.

### 1e. Burst sampling / averaging

You don't need DMA + post-process. The G4 ADC has hardware oversampling
built into the injected path:

```c
// in adc.c MX_ADCx_Init, sConfigInjected:
sConfigInjected.InjecOversamplingMode = ENABLE;
sConfigInjected.InjecOversampling.Ratio        = ADC_OVERSAMPLING_RATIO_8;
sConfigInjected.InjecOversampling.RightBitShift= ADC_RIGHTBITSHIFT_3; // /8 → 12-bit result
sConfigInjected.InjecOversampling.TriggeredMode= ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
sConfigInjected.InjecOversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
```

This is exactly what `aux_adc.c` does for the slow-rate ADC5 (32x). For the
phase ADCs you're tighter on budget: at 8x ratio, 8 conversions × (2.5+12.5)
= 120 ADC cycles = 3.33 µs per sample. You have 10.4 µs of slack between
crests, so 8x fits fine; 16x would also fit (6.67 µs). The trigger pre-offset
in §1d should grow proportionally so the *midpoint* of the oversampled
window is at the valley.

**Recommendation**: 8x oversampling on injected channels is a 3 dB SNR win
(≈ √8 reduction in RMS noise) for cheap. Caveat: with the rotor moving fast,
oversampling smears the sample window across some electrical angle — at
600 Hz electrical (high speed) and 3.3 µs window, that's 3.3 µs × 600 Hz × 360°
≈ 0.7° of smearing. Negligible.

### 1f. Tactical summary for Q1 (revised, in-line topology)

In rough order of pain/value, do these:

| # | Change | Effort | Payoff |
|---|---|---|---|
| 1 | Pre-offset trigger by ~161 HRTIM clocks via TIMx_CMP3 (with ADROM masking one direction) so S&H latches *at* the crest center, not 35 ns past | small | Centers the sample on the cleanest part of the high-side ON window |
| 2 | Enable 8x injected oversampling on hadc1/2/3/4 (current sense) | small | −9 dB current-sense noise floor (√8); also widens the trigger pre-offset requirement to Tovs/2 |
| 3 | Fix the B/C offset swap in `OFFSET_CORRECTION_HRTIM_CB` (see §3a) | trivial | Per-phase DC bias goes away |
| 4 | Restructure HRTIM master/slave so master drives ADC trigger + ISR | medium | Postscaler-vs-RCR drift problem disappears by construction; opens the door to clean 1-of-N sample rates |
| 5 | (skip) Flip ADROM_CREST → ADROM_VALLEY | n/a | Not needed for in-line topology — crest is a valid sample point. |

---

## Question 2 — HRTIM start/stop sequence

> I'm currently going through this long elaborate cargo-culted routine to
> start/stop the HRTIM (in pwm.c), because otherwise the three phases would
> be misaligned, sometimes inverted phase, super weird. Could you check ST's
> literature (in the resources/ folder) and see if there's a "correct" way
> to do this?

Yes, the cookbook calls this out explicitly. Look at AN4539 §2.2 (p15-16),
specifically the note at the bottom of p16:

> *"A delay can be noticed at the PWM start-up between HRTIM_CHA1/HRTIM_CHA2
> and HRTIM_CHD1/HRTIM_CHD2 waveforms. This delay is normal, as the first
> update event (causing compare registers to take their programmed value)
> occurs only after the first counting period is elapsed. **Should the
> waveforms be started without this delay, it is possible to force a register
> update by software (using TASWU and TDSWU bits) to have all active compare
> registers contents updated at once.**"*

That is exactly your symptom (phase misalignment / inverted phase on first
boot of the timer). The fix is what you're already doing — `HAL_HRTIM_SoftwareUpdate`
right after enabling the counters — you just have a lot of redundant
ceremony around it.

### Why your current sequence works (and what's redundant)

`pwm.c:11-42` (start) does:

1. `hrtim_pwm_set_dcycle_f32(0.5f, 0.5f, 0.5f)` — write CMP1 preload.
2. `__HAL_HRTIM_SETCOUNTER(...) = 0` for A/B/E — force counters to valley.
3. `HAL_HRTIM_SoftwareUpdate(A|B|E)` — transfer preload to active. This is
   the "TASWU/TBSWU/TESWU all at once" trick from the cookbook note.
4. `HAL_HRTIM_WaveformOutputStart(...)` — sets `TxyOEN` (output → RUN, RM
   §28.3.14, p952).
5. `HAL_HRTIM_WaveformCountStart_IT(...)` — sets `TxCEN` to start counters.
6. `HAL_HRTIM_SoftwareUpdate(...)` — second update, redundant but harmless;
   ensures any preload writes between SWU calls also land.
7. `__HAL_HRTIM_SETCOMPARE(... COMPAREUNIT_3, PWM_CNTR_MAX_ALLOWED)` for
   A/B/E — this writes CMP3 preload. CMP3 is not used as a set/reset source
   in your crossbar config, and your ADC trigger is on `TIMERA_PERIOD`, not
   `TIMERA_CMP3` — so this write has no effect on the running PWM. It's just
   leftover from a debug session.
8. Final `HAL_HRTIM_SoftwareUpdate(...)` — also redundant.

`pwm.c:44-101` (stop) does roughly the inverse with three trailing
`HAL_HRTIM_SoftwareUpdate` calls — also overkill (one is enough; the others
do nothing).

### Why phases were going inverted before you added the ceremony

Per RM §28.3.14 (p952, Table 249):
- `TxyOEN = 0, TxyODS = 0` → output IDLE, level set by `IDLES[xy]` bit
  (default = INACTIVE).
- `TxyOEN = 1` → output RUN; level driven by the timing-unit crossbar.

If you flip `TxyOEN` to RUN *before* the active CMP1 has been programmed,
the crossbar sees CMP1 = whatever was in the preload register (your code
*does* write CMP1 = 12000 in `hrtim_pwm_setup`-equivalent path, but then
when you call `hrtim_pwm_start` again after a stop, the active CMP1 register
might have been clobbered by the `WaveformSetOutputLevel(... INACTIVE)`
calls in `_stop`, and the preload→active transfer doesn't happen until the
*next* period event after `TxCEN` is set, which is up to one full up-down
cycle away). During that window the output is whatever the set/reset
crossbar last latched, which depends on the counter direction at the
moment `TxCEN` was raised — sometimes giving an inverted-looking pulse.

Forcing a software update (`TASWU` etc.) immediately after starting the
counters propagates the preload→active transfer at once, so the very first
period sees the correct CMP1.

### Cleaner version

Cookbook's recommended pattern, ported to your function pointers:

```c
__fn_flatten void hrtim_pwm_start(void) {
    // 1. write 50% preload to all three CMP1s (current setting will be
    //    overwritten by the fast-loop on its first iteration anyway)
    hrtim_pwm_set_dcycle_f32(0.5f, 0.5f, 0.5f);

    // 2. force all three counters to valley so they start in lockstep
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, 0);

    // 3. enable outputs (TxyOEN: IDLE→RUN); crossbar starts driving pins
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);

    // 4. enable counters (TxCEN); they start ticking from 0
    HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,
        HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E);

    // 5. force preload→active transfer NOW so the very first period uses
    //    our 50% CMP1 instead of waiting one period (cookbook AN4539 §2.2 note)
    HAL_HRTIM_SoftwareUpdate(&hhrtim1,
        HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_E);
}

__fn_flatten void hrtim_pwm_stop(void) {
    HAL_HRTIM_WaveformCountStop(&hhrtim1,
        HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    // counters and outputs are now both off; gate drivers see IDLE level.
    // No need to twiddle compare values or run repeated SoftwareUpdates
    // — the next start() will reseed everything.
}
```

That's it. Eight lines on each side. The order matters:

- **start**: SET preload → SET counters to valley → enable outputs → enable
  counters → SoftwareUpdate. Outputs enabled before counters means the first
  edge of the very first PWM period is correct (instead of a stub of
  whatever the IDLE level was).
- **stop**: stop counters first → stop outputs. Stopping outputs first
  while counters are still running can leave the gate drivers in whatever
  state the crossbar last latched until you also stop the counter, which
  could be an asymmetric mid-pulse moment.

If you want belt-and-suspenders, you can also call
`HAL_HRTIM_WaveformSetOutputLevel(... HRTIM_OUTPUTLEVEL_INACTIVE)` in stop
to force-clear the IDLE level (useful if `IDLES[xy]` is set to ACTIVE for
some reason; not the case in your config — `pOutputCfg.IdleLevel = INACTIVE`
in `hrtim.c:227`).

---

## Question 3 — Deep code review

Walking the fast loop — `hrtim_repetition_callback` (`control_ll.c:243`) —
top to bottom, plus the FOC chain. Issues numbered for traceability.

### 3a. ADC offset calibration: B/C are swapped (real bug)

**`inverter.c:103-111` (`OFFSET_CORRECTION_HRTIM_CB`)**:

```c
ADC_DMA_buf.A_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
ADC_DMA_buf.B_raw = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
ADC_DMA_buf.C_raw = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
```

But `periphmap.h:22-24` says:

```c
#define PHASE_A_ADC_HANDLE hadc1   // OK
#define PHASE_B_ADC_HANDLE hadc3   // ← B is on hadc3
#define PHASE_C_ADC_HANDLE hadc2   // ← C is on hadc2
```

So during offset cal:
- `INA240_offset_correction.B_offset` is computed from `hadc2` = phase **C**'s
  raw reading.
- `INA240_offset_correction.C_offset` is computed from `hadc3` = phase **B**'s
  raw reading.

Then in `adc_update_callback` (`control_ll.c:130-151`):
- `B_raw` (correctly = phase B from `hadc3`) is offset-corrected with
  `B_offset_cal` (= phase C's actual offset).
- `C_raw` (correctly = phase C from `hadc2`) is offset-corrected with
  `C_offset_cal` (= phase B's actual offset).

Net error per phase = (other phase's offset) − (own offset). For two
INA240s the offset spread is typically a few mV; with `SHUNT_AMP_GAIN = -50`
and `SHUNT_VALUE_OHM = 0.002`, 5 mV of offset error → 50 mA of phase-current
DC bias, asymmetric between B and C. Visible on phase current waveforms as
a small DC offset between phases B and C, and as a small Iq bias / cogging
in FOC.

**Fix**: use the periphmap names. Either:

```c
ADC_DMA_buf.B_raw = HAL_ADCEx_InjectedGetValue(&PHASE_B_ADC_HANDLE, ADC_INJECTED_RANK_1);
ADC_DMA_buf.C_raw = HAL_ADCEx_InjectedGetValue(&PHASE_C_ADC_HANDLE, ADC_INJECTED_RANK_1);
```

or just delete `OFFSET_CORRECTION_HRTIM_CB` entirely and read from JDR1 in
the polling loop the same way `inverter_get_adc_raw` does (which uses the
correct mapping). The `ADC_FAKE_DMA_CB` (`inverter.c:85-96`) has the same
swap, but is never registered (offset cal uses the HRTIM rep CB, not the
ADC CPLT CB; runtime ADC reads use `inverter_get_adc_raw` which is correct).
So nuking `ADC_FAKE_DMA_CB` is fine.

### 3b. Sampling at CREST is fine for in-line shunts (revised)

Originally I called this the smoking gun. With author's clarification
(in-line shunts + INA296), it's not. See Q1 §1b (revised) for the
discussion. Still consider the trigger pre-offset (Q1 §1d) and oversampling
(Q1 §1e).

### 3c. Phase-current zero-sum subtraction

`control_ll.c:164-167`:

```c
f32 zero = A_i + B_i + C_i;
A_i -= zero / 3.f;
B_i -= zero / 3.f;
C_i -= zero / 3.f;
```

Forces `A+B+C = 0` by subtracting common-mode. KCL in the motor windings
guarantees this physically (motor is wye- or delta-connected, no neutral
current path), so any nonzero `A+B+C` in the reads is sensor noise or
residual offset, and subtracting it is a useful common-mode rejection.

With the *current* setup (swapped B/C offsets per §3a), there's a
deterministic non-zero common-mode that gets folded back equally into all
three phases — the offset swap leaks into all three currents instead of
just two. Fixing §3a removes the deterministic component; the zero-sum
subtraction then handles only random noise. **Keep this; just fix §3a.**

One subtle consequence: the `Inorm_squared` overcurrent check on line 184
uses `(A² + B² + C²)·2/3`. After zero-subtraction, this is the "true" rotor-
frame current magnitude. Without zero-subtraction, the check would be wrong
under any common-mode noise. Fine.

### 3d. PWM duty cycle inversion is correct, just confusing

`pwm.c:108-110`:

```c
a_dcycle = PWM_CNTR - a_dcycle;
```

With your set/reset = TIMCMP1 in up-down + amplitude-invariant duty
convention (input = high-side ON fraction):

- input dcycle 0.5 → CMP1 = 12000 → output HIGH from up-12000 to down-12000
  (around the crest) → 50% duty centered on crest → high-side ON 50%. ✓
- input dcycle 0.99 → CMP1 = 240 → output HIGH almost the whole period
  → high-side ON 99%. ✓
- input dcycle 0.01 → CMP1 = 23760 → output HIGH only briefly around the
  crest → high-side ON 1%. ✓

So the inversion is needed to map "input duty cycle" semantics to the
crossbar convention. Could be removed if you accept the inverted semantic
in callers, but that requires rewriting the SVM consumer — not worth it.

### 3e. SVM output convention vs. duty input

`bldc_math.c:81`: `svm_vec_t ret = {1.f - tA, 1.f - tB, 1.f - tC};`

Combined with `pwm.c`'s further inversion (3d), the two cancel: the `tA`
computed inside SVM ends up as the actual CMP1/PWM_CNTR fraction. The
double-flip is harmless but confusing — pick one convention. Mentioning so
that if you ever rewrite, you can collapse them.

### 3f. FOC transport-delay correction (the `2.f / ENC_MEAS_FREQ` comment)

`foc.c:174`:

```c
f32 pwm_phase = phase_ + vel_ * 2.f / ((f32) ENC_MEAS_FREQ); // TODO <- why is this 2/?
```

The `2.f` is correct, here's the reasoning. Pipeline at 16 kHz:

1. Cycle N: ADC samples at valley ⇒ `phase_now` is rotor angle at the valley.
2. Same cycle N, ISR runs at the next *crest* (about T/2 later in the
   up-down cycle, but conceptually "right after" the sample).
3. ISR computes new CMP1 values from sampled current + estimated phase, and
   writes them into preload registers.
4. Preload→active transfer happens at the next period event. With your
   `RepetitionUpdate = ENABLED` and `ROM = CREST`, that's the next crest →
   one ISR period N+1 later.
5. The new CMP1 is in active register through cycle N+1. The waveform
   centroid (the moment the d/q voltage actually applied to the motor is
   "centered") is at the next valley = the *next* sample point.

So between sampling at valley(N) and applying-at-valley(N+1) there's about
1 ISR period. The ISR runs once and does both the read at valley N and the
write effective at valley N+1. So the angle to use for the inverse Park is
`phase_at_valley(N+1)`. If `phase_` is the encoder estimate at valley N,
then `phase_at_valley(N+1) ≈ phase_ + vel_ * (1 ISR period)`, i.e. `1/ENC_MEAS_FREQ`,
not `2/ENC_MEAS_FREQ`.

Why 2? Because your encoder estimator (`encoder.c:79`) calls
`encoder_predict(PWM_RCR)` *internally* every update, which advances by
**another** 1 ISR period. So `motor_phase` (returned from `encoder_get_phase`)
is already `phase_at_valley(N+1)`. Then `+ vel * 1/ENC_MEAS_FREQ` gets you
to valley(N+2)... wait, that's only 1 forward, not 2.

Actually, re-reading the encoder estimator more carefully:

```c
posvel_t posvel_predict = encoder_predict(PWM_RCR); // predicts forward by 1 ISR period
posvel_predict.pos = fast_fmod_01(posvel_predict.pos);
f32 pos_err = circular_delta(enc_pos - posvel_predict.pos);  // error between predicted and measured
posvel_t posvel_est_corrected = {
    .pos = fast_fmod_01(posvel_predict.pos + enc_meas_period * pll_kp * pos_err),
    .vel = posvel_predict.vel + enc_meas_period * pll_ki * pos_err
};
```

So `posvel_est_corrected.pos` is the PLL's estimate of position **at the
sampled time** (not 1 period in the future). The predict-then-correct PLL
returns "current best estimate of rotor pos at the moment of measurement",
which is the valley of cycle N. So `motor_phase = phase_at_valley(N)`.

Then the FOC `pwm_phase = motor_phase + vel * 2*Tisr` aims at
`phase_at_valley(N+2)`, two periods ahead. That's *one more* than my back-of-
envelope. The extra period is probably accounting for the fact that the new
CMP1 takes effect on the *next* up-down cycle's crest (period event), not
the immediate one — there's another ~T/2 of the current cycle before the
update window closes plus the full next cycle.

So `2 * Tisr` is close to right. To be precise:

- Sample at valley N (counter = 0, going up).
- ISR fires at crest N (counter = PER, T/2 later).
- ISR completes within T/2; new CMP1 lands in preload.
- `RepetitionUpdate = ENABLED` + `ROM = CREST` + `RCR=5` means preload→active
  fires at every 6th crest. The next opportunity is *this* crest (the one
  triggering the ISR), but only if the ISR finishes before the crest event
  is registered for the next cycle... actually preload→active happens at
  rep-event boundaries, which for RCR=5 ROM_CREST means every 6 crests.
  Hmm. So actually the new CMP1 doesn't take effect until the NEXT rep
  rollover (6 PWM cycles later)? That can't be right or the loop would be
  hopelessly delayed.

Let me check this more carefully. Actually with `RepetitionUpdate = ENABLED`,
the preload→active transfer happens on **every** RCR rollover (= every
6 cycles for RCR=5). That's a once-per-ISR transfer. So:

- ISR runs once per RCR rollover (= once per 6 PWM cycles).
- ISR writes preload, returns.
- *Next* RCR rollover (6 cycles later, = 1 ISR period) fires preload→active
  AND triggers the next ISR.
- New CMP1 governs the PWM during the NEXT 6 cycles.
- The *centroid* of those 6 cycles is 3 PWM cycles past the rep boundary,
  but the average is just "active during the 6-cycle window starting at
  the next ISR".

So: sample at start of window N, write preload, transfer at start of window
N+1, applied during window N+1 with centroid at N+1.5 ≈ "1.5 ISR periods
after sample". Round to 2 for the Park transform argument: that's the
intuition behind `2.f / ENC_MEAS_FREQ`.

**Verdict**: `2.f` is approximately right. The comment "why is this 2/?"
can be removed — it's the transport-delay correction. Could refine to `1.5f`
with no real penalty.

The corresponding line **119** for the Park transform of measured currents
does `+ vel * 1.f / ENC_MEAS_FREQ`. By the same reasoning, the current
samples were taken at valley N, and `motor_phase` is the rotor at valley N.
Multiplying by 1/Tisr advances to valley N+1, which is wrong — Park should
use the rotor angle *at the moment the currents were sampled*, i.e.
`motor_phase` directly (no advance). So **line 119 should be `phase_` with
no advance term**. This is a small angular offset (~6 mrad at 100 rad/s_elec)
but it does add a fixed Iq cross-coupling at high speeds.

### 3g. `idq_measured_filter = 1.f` is dead code

`foc.c:41-42`: `static const f32 idq_measured_filter = 1.f;`

With gain 1, the LPF `idq_measured_.d += 1 * (idq.d - idq_measured_.d)`
collapses to `idq_measured_.d = idq.d`. And `idq_measured_` is then written
back to `idq_measured` but **only `idq` (the freshly-Park'd un-filtered
version) is used** in the modulation chain (`idq_err`, line 146). So the
"filtered" measurement is computed and then ignored.

Either delete the filter machinery or actually wire it through. If you
want filtering, set the filter coefficient = `2*pi*f_cutoff*Tisr` (e.g.
2π * 1000 Hz * 62.5 µs ≈ 0.39 for 1 kHz cutoff) and use `idq_measured_`
in `idq_err` instead of `idq`.

### 3h. `Q_rsqrt` is anachronistic on Cortex-M4F

`foc.c:84-98`: Quake III's fast inverse square root.

The M4F has hardware `VSQRT.F32` (≈14 cycles) and `VDIV.F32` (≈14 cycles).
`1.0f / sqrtf(x)` compiles to those two instructions = ~28 cycles, plus
single-precision-correct accuracy. `Q_rsqrt` with one Newton iteration is
~20 cycles but ~0.2% inaccurate. For motor control modulation scaling,
0.2% is fine, but you save nothing meaningful and lose a sanity check
(when `mod_dq.d² + mod_dq.q² → 0`, `Q_rsqrt` returns `1/√0 = ∞`; the
subsequent `if (mod_scalefactor < 1.f)` branch is taken, and you avoid
division by zero only because `0.99f * sqrt3_over_2 * inf = inf` and
`inf < 1.f` is false). It works, but a `if (mod_norm_sq > 1e-6f) ...`
guard is cleaner.

### 3i. `is_currents_updated` flag logic

`control_ll.c:117-118`:

```c
if (is_currents_updated)
    controller_add_errors(CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING);
```

Reads "if the flag is still true at the start of this ISR (i.e. nobody
consumed the current measurement from last cycle), error out". Then later,
line 200-201 re-sets the flag, and line 203-207 invokes the consumer which
*may* clear the flag if it returns true.

This is fine if the consumer is well-behaved (always returns true), but if
a consumer ever returns false, the flag stays true and you get a permanent
`CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING` once and then disarm. That can
turn a transient sensor glitch into a hard fault. Consider: clear the flag
at the *start* of every ISR, set it after `current_consumer` returns true,
and check it for downstream callers.

The same pattern repeats for `is_pwm_timings_updated` and
`is_encoder_updated`.

### 3j. Z-axis math precision: `Inorm_squared = 2/3 * (A² + B² + C²)`

`control_ll.c:184`: this is the squared-magnitude of the alpha-beta vector
*after* zero-subtraction (since A+B+C=0 implies α² + β² = 2/3·(A² + B² +
C²) under amplitude-invariant Clarke). Algebra checks out. ✓

### 3k. Encoder estimator clarity

`encoder.c:62-94`: the predict/correct PLL is correct in concept. Two notes:

- The comment "IF POS_ESTIMATE EVER DIVERGES MORE THAN 0.5 TURNS FROM POS
  THEN THIS SHIT WILL GO UNSTABLE" (line 61) is true and worth honoring.
  Initial-condition handling (line 67-74) bootstraps `posvel_estimate.pos`
  to the first measurement, so the divergence is ≤ 1 update period × peak
  velocity. At 60 rev/s mechanical (worst-case spin during cal) and 62.5 µs,
  divergence ≤ 60 × 62.5e-6 = 3.75 mturn. Far below 0.5 turns. ✓
- The PLL gains `pll_kp = 2*BW`, `pll_ki = (2*BW)²/4` give a critically
  damped pole at `BW`. With `ENCODER_ESTIMATOR_BANDWIDTH = 1000` and ISR
  at 16 kHz, `pll_kp * Tisr = 2000 / 16000 = 0.125`, well below the 1.0
  stability boundary of a discrete-time PLL gain. Good. Don't push BW above
  ~3 kHz with this update rate.

### 3l. NVIC priorities all (0,0) — preemption-impossible

`hrtim.c:320`: `HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 0, 0);`

And every other IRQ in `tim.c`/`fdcan.c`/`dac.c`/`dma.c` is also `(0, 0)`.
Same priority means *no preemption between them*. If a CANopen IRQ on TIM4
(1 kHz, runs `canopen_app_interrupt` which handles SDO/PDO machinery and
can take >50 µs in worst-case branches) is in flight, the HRTIM IRQ can't
preempt it — the next current-control cycle is delayed.

At 62.5 µs ISR period, even a 30-50 µs delay is a non-trivial phase shift
in the PWM update. You'd see this as occasional torque ripple correlated
with CANopen traffic or large SDO writes.

Fix: drop the priority of *everything* except HRTIM. With the default
`SCB->AIRCR.PRIGROUP = 3` (4 bits preempt, 4 bits sub), set:

```c
HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 0, 0);   // top
HAL_NVIC_SetPriority(TIM4_IRQn,        2, 0);   // CAN tick
HAL_NVIC_SetPriority(TIM7_DAC_IRQn,    3, 0);   // aux ADC
HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 2, 0); // FSM
HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,  3, 0);
HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn,  3, 0);
HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0); // ... etc
```

With these, HRTIM (preempt 0) can interrupt anything (preempt 2/3) and
finish its 5 µs of work; everything else runs round-robin.

To do this from CubeMX/CubeIDE: bump priorities under `Project Settings →
NVIC` so they regenerate into the Init code. Otherwise stick override calls
in `app_main` *after* the `Mx_*_Init` functions.

### 3m. Master timer is configured but unused

`hrtim.c:103-146`: the master timer is set up with period 0xFFDF (= 65503),
prescaler MUL32, continuous mode. That's **70.3 kHz** — not aligned with
slaves' 96 kHz switching freq. Master compare events (CMP1=CMP2=CMP3=CMP4
all = 0xFFDF) fire once per master period.

Master is enabled (it'll start running when MCEN is set), but nothing
listens to it — no slave has `ResetTrigger = MASTER_PER`, no ADC trigger
sources from it, no IRQ enabled.

This is wasted hardware. Either:
- Repurpose it (Q1 §1c — make it the ADC + ISR clock source).
- Or disable it (`pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT`,
  `MasterEnable = 0`) so the kernel clock isn't burning cycles for it.

### 3n. CubeMX-generated CMP3 = 0xFFDF on master

(Subset of 3m.) The master CMP1-4 are all set to `0xFFDF` = master period.
This is just CubeMX defaults. Harmless given that nothing reads them.

### 3o. RCR mismatch between header and init

`pwm.h:15` has `#define PWM_RCR 12`. `hrtim.c:148` sets the actual HW
RepetitionCounter to `6-1 = 5`. These mean different things — `PWM_RCR=12`
appears in the user's math `ENC_MEAS_FREQ = PWM_FREQ / PWM_RCR = 192000/12
= 16000`, which gives the right answer **only because** `PWM_FREQ=192000` is
also wrong-by-2x (real switching freq is 96 kHz). Two errors cancel.

Concretely: if you ever change one of these constants, the encoder
estimator's `enc_meas_period` math will silently break.

Fix: replace `PWM_FREQ` and `PWM_RCR` with derived constants:

```c
#define HRTIM_KCLK_HZ           144000000u
#define HRTIM_DLL_MUL           32u
#define HRTIM_TIMER_CLK_HZ      ((u64)HRTIM_KCLK_HZ * HRTIM_DLL_MUL)  // 4.608 GHz
#define HRTIM_TIMER_PERIOD      24000u
#define HRTIM_TIMER_RCR         5u    // (RCR + 1) PWM cycles per ISR

// derived
#define PWM_SWITCHING_HZ        ((u32)(HRTIM_TIMER_CLK_HZ / (2 * HRTIM_TIMER_PERIOD)))
                                                    // 96 kHz
#define PWM_ISR_HZ              (PWM_SWITCHING_HZ / (HRTIM_TIMER_RCR + 1))
                                                    // 16 kHz
```

and use `PWM_ISR_HZ` instead of `ENC_MEAS_FREQ`/`CURRENT_MEAS_FREQ`/etc.

### 3p. FSM `controller_get_loop_count` overflow

`control_ll.c:24` has `volatile u32 n_loop_count = 0;` incremented at 16 kHz.
`u32` wraps at 2³² / 16000 ≈ 74 hours. Cal procedures use `n_loop_start +
n_loop_total < n_loop_now` checks (e.g. `encoder_calibration.c:64`); after
74 hours of operation those wrap and a state-machine transition could
trigger spuriously. Probably fine for a cubesat (bus state machine resets
between operations), but worth noting.

### 3q. `controller_arm()` register/unregister pattern is sketchy

`control_ll.c:299-323`: every arm/disarm pair does
`HAL_HRTIM_TIMxRegisterCallback`/`UnRegisterCallback`. The HAL implementation
of these takes a non-trivial spinlock (`__HAL_LOCK(hhrtim)`) and does some
pointer juggling; doing it in the hot path is unnecessary. The callback
identity doesn't change between disarms — register it once at init, and
gate execution on `controller_armed` inside the callback (which you already
do for most logic).

### 3r. Default-trigger / fault inputs not configured

`pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE` (`hrtim.c:172`). The
HRTIM has dedicated FAULT inputs that can immediately push outputs to a
safe state in hardware on overcurrent / DESAT signals. Not using them puts
all the safety on the firmware error path (`controller_disarm`), which has
to wait for the next 16 kHz ISR. For a cubesat with 5 A motor currents,
that's tolerable; for the satellite reaction wheel application, you've
already got OD-level overcurrent thresholds, so this is more of a
robustness-against-software-fault concern.

### 3s. `printf` from FSM exit/enter is in a 1 kHz ISR

`fsm.c:121, 151`: `printf` runs from `fsm_exec`, which runs from
`controller_run_state_machine`, which runs from `TIM17` IRQ at 1 kHz. Each
`printf` blocks on UART TX (busy-wait `LL_USART_IsActiveFlag_TXE_TXFNF` in
`uart_transmit_one`, util.h:198). At 115200 baud, a 30-char log line takes
~2.6 ms — longer than the FSM tick period. This will stall the FSM and is
likely also why state changes feel laggy (if you've noticed that).

Either move state-change printfs to the foreground loop, or use a
non-blocking ring buffer + DMA.

### 3t. `state_names` array is referenced but not defined in shared header

`fsm.c:121` uses `state_names[exit_state].string`, which presumably comes
from `fsm.h`. Let me not chase this — if it builds, it builds.

### 3u. `swo_print_f32` calls in the fast loop

`foc.c:163-166`: `swo_print_f32` for four signals every 16 kHz ISR. Even
with ITM (SWO at high baud), four 32-bit ITM stimulus writes per cycle is
fine in budget, but if SWO is unconnected on the satellite version it just
writes to a register that never drains and might back-pressure. Worth
checking `ITM->TER` handling.

### 3v. Vbus is 47.5 ADC cycle sample (long), but on the same trigger as currents

`adc.c:336`: hadc4 (Vbus) has `InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5`
while phase ADCs are 2.5 cycles. All four are triggered together by the
same HRTIM event. The 47.5 cycle ADC4 sample window completes at ~1.67 µs
after trigger, while the phase samples complete at ~417 ns. If you're
reading JDR1 in the ISR within 417 ns of the trigger, the Vbus reading is
*last cycle's* value, not the current one — but since you read in the ISR
which fires at the next crest (T/2 = 5.2 µs after the valley trigger),
both have completed. So this is fine in practice. Just be aware if you ever
move the JDR1 reads earlier.

---

## Recommended action plan (revised for in-line + INA296)

Pick whichever subset matches your time budget.

**Tier 1 (real bugs / hard-real-time correctness):**

1. **Q3/3a**: fix the B/C handle swap in `OFFSET_CORRECTION_HRTIM_CB` and
   `ADC_FAKE_DMA_CB` (`inverter.c:87-110`). Real bug, ~50 mA per-phase
   DC bias asymmetric between B and C.
2. **Q3/3l**: bump non-HRTIM IRQ priorities to 2 or 3 so HRTIM can preempt.
   Fixes whatever current-loop jitter correlates with CANopen traffic.
3. **Q3/3f**: drop the `vel * 1/ENC_MEAS_FREQ` advance from `foc.c:119`
   (Park transform of measured currents) — leave only the `vel * 2/`
   advance for the inverse Park on line 174. Wrong sign on phase advance
   for the measurement-side Park.

**Tier 2 (cleanups, then look at waveform):**

4. **Q2**: simplify `hrtim_pwm_start`/`_stop` per the cookbook §2.2 pattern
   (the 8-line versions above).
5. **Q3/3g**: either delete `idq_measured_filter` machinery or actually
   use `idq_measured_` in `idq_err`. Currently dead code.
6. **Q3/3o**: replace `PWM_FREQ`/`PWM_RCR` with derived constants tied to
   the real HRTIM init values. Two existing errors cancel; future edits
   risk uncovering them.
7. **Q3/3s**: move FSM `printf`s out of the 1 kHz tick. They block on UART
   TX (~2.6 ms per line at 115200) inside an ISR.

After tier 1+2, scope the phase currents and decide whether the residual
distortion warrants tier 3.

**Tier 3 (architectural / low-yield without more data):**

8. **Q1/1d**: pre-offset ADC trigger by 161 HRTIM clocks via TIMx_CMP3 (or
   master CMP1) so S&H latches *at* the crest, not 35 ns after. Marginal
   gain; only matters if §3w (deadtime distortion) isn't dominating.
9. **Q1/1e**: enable 8x injected oversampling on hadc1/2/3. Wider trigger
   pre-offset window required.
10. **Q1/1c**: restructure HRTIM master/slave so master drives ADC trigger
    and ISR via `HRTIM_TIMRESETTRIGGER_MASTER_PER`. Eliminates postscaler-
    vs-RCR drift problem; only matters if you ever want 1-of-N sampling
    via postscaler.
11. **Q3/3r**: wire HRTIM fault inputs to your overcurrent comparator if
    one exists.

---

## §3w. Likely real cause of ugly current waveforms (with in-line clarified)

With the original "you're sampling on the wrong side of the PWM" theory off
the table, the candidate explanations for non-sinusoidal phase currents,
in rough likelihood order:

1. **Inverter dead-time distortion.** Your DTG config (`hrtim.c:202-210`):
   `Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8`,
   `Rising = Falling = 0xC = 12`.

   **HAL naming gotcha:** the HAL header comments
   (`stm32g4xx_hal_hrtim.h:1623-1630`) describe these prescalers backwards
   from reality. The constant names refer to *tick period* relative to
   `tHRTIM` (i.e. `DIV8` = "tick is tHRTIM/8" = SMALLER tick = FASTER DTG;
   `MUL8` = "tick is 8·tHRTIM" = LARGER tick = SLOWER DTG). The comments
   describe `fDTG` as if divided/multiplied — but `fDTG = (fHRTIM × 8) /
   2^DTPRSC` per RM §28.3.3 (p891), so `DIV8` (DTPRSC=0) is actually the
   *fastest* DTG (`fDTG = 8·fHRTIM`) and `MUL8` (DTPRSC=6) is the
   second-slowest (`fDTG = fHRTIM / 8`). Pick whichever interpretation,
   they contradict.

   Concretely on your 144 MHz kernel:

   | HAL constant | DTPRSC | tDTG | At DT=12 |
   |---|---|---|---|
   | `DIV8`  | 000 | **0.87 ns** | 10.4 ns |
   | `DIV4`  | 001 | 1.74 ns | 20.8 ns |
   | `DIV2`  | 010 | 3.47 ns | 41.7 ns |
   | `DIV1`  | 011 | 6.94 ns | 83.3 ns |
   | `MUL2`  | 100 | 13.9 ns | 167 ns |
   | `MUL4`  | 101 | 27.8 ns | 333 ns |
   | `MUL8`  | 110 | **55.6 ns** | **667 ns** ← current config |
   | `MUL16` | 111 | 111 ns | 1.33 µs |

   So your configured deadtime is **667 ns**. The chip can do down to
   0.87 ns/tick if you switch to `DIV8`. If 667 ns was *not* intentional
   (likely — it's plausibly a CubeMX default + the misleading HAL comment
   tricked you), the simplest fix is `Prescaler = DIV8` + `RisingValue =
   FallingValue = 12` → 10.4 ns deadtime.

   At 96 kHz switching, 667 ns is 6.4% of the switching period. During this
   window, neither switch is on — phase output floats and is pulled to 0
   or Vbus by the current direction (through the body diode / GaN reverse
   conduction). The effective duty cycle is therefore distorted by
   `±deadtime × sign(I_phase) / Tperiod`. This shows up as a 6th-harmonic
   ripple on the phase currents — the textbook "deadtime distortion" — and
   is *the* classic cause of non-sinusoidal waveforms in BLDC drives,
   especially at low modulation where the deadtime fraction dominates.

   Sanity check: scope the gate drives directly, measure actual overlap.
   If you see ~667 ns: check your FET datasheet for required deadtime; if
   GaN, you can probably drop to 20-50 ns. If you see ~10 ns: I missed
   something, please tell me.

   For software compensation (regardless of the deadtime value): deadtime
   feedforward in the FOC modulation stage. Add `±deadtime / Tperiod` to
   each phase duty based on `sign(I_phase)`. ODrive's source has a clean
   reference implementation.

2. **Encoder phase offset miscalibration.** `encoder_calibration.c` runs an
   open-loop spin and computes the offset between the rotor's mechanical
   angle and the assumed electrical angle. If the offset is off by even a
   few degrees, current that *should* be on the q-axis spills into the
   d-axis — which the d-axis controller then tries to push to zero,
   creating ripple. Check `encoder_set_phase_offset` is being called with
   a sensible value at startup; check that `encoder_get_direction` returned
   ±1 correctly.

   Worth running `FSM_STATE_ENCODER_VALIDATION` and looking at the printed
   table — the 12 datapoints should show a near-linear relationship between
   commanded electrical phase and measured encoder phase, with no slope
   discontinuities.

3. **Current PI bandwidth.** `CURRENT_CONTROLLER_BANDWIDTH = 4000.f`
   (`config.h:47`). Units are unclear from code (the code uses it as if
   it's rad/s in `current_ctrl_kp = BW * L`, which would give Kp = ω·L =
   the impedance at frequency BW; that's a textbook PI tuning target).
   At 4000 rad/s = 637 Hz cutoff, sampled at 16 kHz, that's BW/Fs = 0.04 —
   very stable, but slow to track. If it were 4000 Hz = 25 krad/s, BW/Fs =
   1.57 — way too aggressive, would chatter. The ambiguity in units is
   itself a smell. Try 1000 (the conservative interpretation if it's Hz).

4. **Modulation saturation overrun.** When mod_dq exceeds the SVM linear
   region (|mod| > sqrt(3)/2 ≈ 0.866 in your scaling), the modulator
   clamps to the boundary, the PI's anti-windup kicks in (`foc.c:156-157`),
   and the actual phase voltages get distorted. Visible on a clamp as a
   flat-topping of the current waveform. Easy test: scope current while
   varying torque setpoint — if distortion only appears at high setpoints,
   you're saturating. Solution: reduce torque setpoint, or run with
   feedforward disabled (you already have it disabled —
   `FOC_USE_DECOUPLING = true` but `FOC_USE_R_FF = FOC_USE_wL_FF =
   FOC_USE_bEMF_FF = false`).

5. **B/C offset swap (§3a).** Causes a small phase-asymmetric DC bias. Not
   "non-sinusoidal" so much as "off-center sinusoidal".

6. **`vel*1/Tisr` Park-of-measurement bug (§3f).** Adds a small
   speed-dependent angular shift to the d-q decomposition. Looks like a
   mild, speed-correlated cogging.

If I had to bet on one cause without more data, it'd be **#1 (deadtime
distortion)**. It's the textbook answer to "BLDC waveform isn't sinusoidal"
and your 667 ns deadtime is on the long side for modern FETs at 96 kHz.

---

## Open questions for you

- **Confirm deadtime.** Scope a single phase's high-side and low-side gate
  drives, measure overlap. Is it really 667 ns?
- **What does the current clamp trace actually look like?** Specifically:
  flat-topped, 6th-harmonic-ish ripple, or randomly noisy? Each of those
  points at a different cause from §3w.
- **CURRENT_CONTROLLER_BANDWIDTH units.** Hz or rad/s? Affects whether
  the existing tune is conservative or aggressive.
- **Anything you noticed during integration that I should be aware of**
  before recommending architectural changes? E.g. magic things you've
  added to keep CANopen happy, or board-level constraints I can't see
  from the code.
