# SHUNT_SEL_DELAY_TICKS measurement (worklog Q2-A), scope-free

Measures the duty→sample pipeline lag that best-2-of-3 shunt selection depends on — directly,
with the feature **OFF**, so the answer is ground truth rather than "which setting looks
smoothest". Companion to the smoothness-scan validation sketched in
`docs/high-speed-foc-worklog.md` (Q2-A); run this first, validate with that after.

## Principle

`foc_capture` records, per fast-loop tick, both the commanded SVM duties (`svm_*`, plus `Iq_set`)
and the measured currents (`Iq_meas`, `Ia/b/c`). The orchestrator applies **five alternating
setpoint edges** (~1 ms apart) while capturing:

- Each command tick is where `Iq_set` jumps (the PI's P-term moves the duties the same tick;
  latched to hardware at end-of-tick).
- The first *read* whose sample reflects a new duty starts the measured-current transition;
  **K** = ticks from command to that first affected read. The selection history ring is pushed
  at the end of each tick (slot `head` at tick M holds tick M−1's duty), so:

  ```
  SHUNT_SEL_DELAY_TICKS = K − 1
  ```

Predicted from timing analysis: K = 2 → DELAY = 1 (the config.h default — reasoned, not
measured, which is why this procedure exists).

**Why edges are averaged and the onset is *fitted*, not thresholded:** this is a closed-loop
setpoint step, so the current follows the current-controller bandwidth — τ = 1/4000 s ≈ **8
ticks** at 32 kHz — *not* the open-loop L/R (~1 tick). The first affected sample has only ~⅔
tick of new-duty exposure → ~2 % of the step, i.e. millamps, below single-sample noise. The
analyzer therefore aligns and averages all edges (each against its own local baseline,
normalized by its own size/sign) and grid-fits a delayed first-order rise over candidate onsets,
reporting the per-candidate SSE and an explicit **AMBIGUOUS** verdict when the margin is < 1.3×
(single-edge captures usually are; the 2026-06-09 bench runs demonstrated exactly this). The fit
also returns τ — it should land near 8 ticks; a much larger value suggests gain/cal problems.

## Procedure

Preconditions: RelWithDebInfo image flashed and matching the ELF (Debug IWDG-reboots when
armed), encoder calibration valid (this runs closed-loop `TORQUE_CONTROL`), wheel free to spin
(~50 ms of small torque, it will twitch), controller IDLE.

```bash
tools/shuntsel_delay.sh run                      # step 0.05 -> 0.30 A, capture, dump
python3 tools/shuntsel_delay_analyze.py shuntsel_step.bin
```

The analyzer prints the averaged response profile, the SSE-per-onset table with the fit margin,
fitted τ, and `RECOMMENDED: #define SHUNT_SEL_DELAY_TICKS <n>` — or an explicit `AMBIGUOUS`
verdict telling you to re-run rather than guess. Sanity checks built in: `loop_count` gap
detection, svm-moved-on-command-tick cross-check, and a warning if the answer differs from the
reasoned default. Repeat once to confirm — it's a 10 s run.

The orchestrator is non-halting throughout (SWD MEM-AP only), forces `shunt_selection_enabled=0`
for the measurement and restores it, zeroes the setpoint and returns to IDLE on completion or
abort. Tunables: `IQ_BASE`/`IQ_STEP` env vars.

## After the measurement

1. Set `SHUNT_SEL_DELAY_TICKS` in `Firmware/Inc/config.h` (compile-time; reflash).
2. Validate the *feature* per the worklog plan: enable selection (`shunt_selection_enabled=1`
   live over SWD, or flip `FOC_USE_SHUNT_SELECTION`), spin fast enough to reach high modulation,
   capture, and check (a) `shunt_drop` tracks the minimum-duty phase from the right history
   depth, (b) reconstructed `Iq`/phase currents are smoother than selection-off at the same
   operating point (A/B with the runtime flag — no reflash needed).
3. If K ever changes (loop rate, call ordering in the ISR, ADC timing/oversampling config,
   CMP4 position), re-run. The measurement is cheap; the failure mode of a stale DELAY is the
   feature dropping the *wrong* phase — worse than off.

## Cross-references

- `docs/high-speed-foc-worklog.md` — Q2-A context, why a wrong DELAY is worse than disabled.
- `docs/adc-sampling-tuning.md` — the CMP4 sweep that calibrated *where in the period* the
  sample lands; this doc calibrates *which tick's duty* that sample belongs to.
- `docs/foc_capture.md` — the capture mechanism both tools ride on.
