# Review-fixes change log — overnight session 2026-06-10

Every change made in the overnight fix session following the 2026-06-09 flight-readiness review
(`docs/flight-review-2026-06.md`). Owner decisions that scoped the session are in
`docs/flight_review_response.txt` plus two live answers: node-ID straps are **CAN_ADDR0/1
binary**, and overspeed/overtemp trips were **skipped** (NaN/range validation only).

**Verification status: build + lint only.** All three images build clean (app RelWithDebInfo +
Debug, bootloader via `./build.sh`), `./lint.sh` is at zero warnings, and two adversarial
review agents swept the full diff (their three blockers + seven minors are folded in below,
marked *[review-round]*). **Zero hardware time** — the 10-step bench-verification checklist at
the bottom of `docs/flight-review-2026-06.md` is the gate before trusting any of this.

Entries are grouped as **commit-sized units** (each is a coherent, independently-buildable
change; suggested ordering for a commit series is the order below — foundations first).
Review IDs (`P0-x`, `P1-x`, `F-x`, …) refer to `docs/flight-review-2026-06.md`.

---

## 1. Crash-record cells in RAM2 + linker reservation  *(foundation — everything later uses it)*

**Why:** No reset-cause/fault evidence survived to CAN (RE-4); several later fixes need
warm-reset-surviving state. *[review-round F-blocker]*: the first version put the cells inside
the live stack — `_estack` was top-of-RAM2−8 and app_main's first frames overlapped them
(confirmed by disassembly); fixed with an explicit linker reservation.

**Files:** `Firmware/Inc/crash_record.h` (new), `STM32G474VEHX_FLASH.ld`.

- New fixed-address cells at the top of RAM2 (survive NVIC/IWDG resets, lost on power cycle):
  `0x20017FC0` crash record (40 B: kind, CFSR/HFSR/BFAR/MMFAR, stacked PC/LR,
  controller_errors, aux), `0x20017FB8` boot counter, `0x20017FB0` flash-ECC NMI latch,
  `0x20017FA8` option-byte attempt counter. Bootloader RCSR mailbox at `0x20017FF8` unchanged.
- `_estack = ORIGIN(RAM2) + LENGTH(RAM2) - 0x58` — the stack may no longer grow into the cells.
- Contract documented in the .ld, crash_record.h, and a new footguns entry.

## 2. `f32_finite_bits()` NaN/Inf helper  *(foundation)*

**Why:** `-Ofast` = `-ffinite-math-only`; GCC may legally fold `x != x` and negated range
compares into forms that PASS NaN (P2-14). Every load-bearing NaN check needs a fold-proof
bit-pattern test.

**Files:** `Firmware/Inc/mathutil.h` (union-pun helper), consumed by entries 6/8/9.

## 3. Node ID from CAN_ADDR0/1 straps  *(P0-1)*

**Why:** The DEBUG1/2/3 vote read pins that `gpio.c` configures as push-pull outputs driven
LOW — every board enumerated as wheel 4 (`0x48`); four wheels on one bus = COB-ID collision
storm. The dedicated CAN_ADDR0/1 inputs (PB7/PB5) were never read.

**Files:** `Firmware/Src/app_main.c`.

- Reads CAN_ADDR0/1 with internal pull-ups (the .ioc left them NOPULL), 1000-sample majority,
  strap-to-GND = bit set: none = wheel 1, ADDR0 = wheel 2, ADDR1 = wheel 3, both = wheel 4.
- Raw high-counts printed at boot for marginal-strap diagnosis.
- **Behavior change: an unstrapped bench board now enumerates as wheel 1 (`0x3C`), not `0x48`.**

## 4. Bus-voltage thresholds  *(P0-2)*

**Files:** `Firmware/Inc/config.h`.

- `DC_BUS_UNDERVOLTAGE_THRESHOLD_V` 2.7 → **5.0 V** (2S at 2.5 V/cell = wiring/EPS fault), the
  screaming TODO resolved.
- OV note: flight boards must build with `IS_CORRECTED_VOLTAGE_DIVIDER` (owner-confirmed) —
  with the desk board's old divider the 8.6 V trip sits above the 8.40 V ADC ceiling and
  remains unreachable there.

## 5. Fast-loop arm/disarm hardening  *(P1-3, P2-5)*

**Files:** `Firmware/Src/control_ll.c`.

- `controller_arm()` clears `is_currents_updated`/`is_encoder_updated`: a stale flag (consumer
  returned false once, or an all-NULL law ran) used to trip `PHASE_CURRENT_USAGE_MISSING` on
  tick 1 of *every* subsequent arm — wheel torque-dead until reboot (P1-3).
- `controller_disarm()` writes ODISR first via the lock-free `hrtim_outputs_disable()` — the
  HAL teardown can silently no-op under handle-lock contention; the register write always lands.

## 6. FSM ladder rewrite + arm gating  *(P1-1, P1-2 part, review minors)*

**Why:** Three ways into a 1 kHz IDLE↔CONTROLLER_ERROR ping-pong (each transition resetting
CANopen comms under the old design); CLEAR_ERRORS parked in CONTROLLER_ERROR with errors==0;
no validation or cal interlocks anywhere.

**Files:** `Firmware/Src/fsm.c`, `Firmware/Inc/fsm.h`, `Firmware/Inc/control_ll.h` +
`Firmware/Inc/util.h` (new `CTRLR_ERR_NOT_CALIBRATED` bit 22; bit 21 `CTRLR_ERR_MODULATION`
was *missing from the stringifier table* — also fixed).

- Same-state requests are idempotent no-ops (no cancel/re-init blip — command replay is safe).
- While errors are latched, every request except CLEAR_ERRORS is **dropped** with a console
  note (a pending request used to bounce the FSM forever).
- Branch-1 exempts `state == CLEAR_ERRORS`, so the documented clear → auto-IDLE flow actually
  completes; requests are consumed in the non-idle transit branch (IDLE-bounce gone).
- Entering a NULL-law state calls `controller_clear_control_law()` — a latched
  `control_law_is_done` (e.g. a completed-then-failed R/L cal) can no longer self-oscillate.
- IDLE→new gating, in order: **hold** while a cal-store op is pending *[review-round F1 — a
  deferred RESTORE could otherwise swap dir/offset/LUT under a freshly-armed loop]*; refuse
  law states when the inverter offset cal is invalid *[review-round — the fast-loop check
  can't fire if the HRTIM never ticks]*; refuse cal-gated states without encoder cal
  (`CTRLR_ERR_NOT_CALIBRATED`, defense-in-depth behind the SDO gate).
- `state_names[]` indexing bounds-checked via `fsm_state_name()` (host-supplied values used to
  read past the table in ISR printf), used everywhere.

## 7. POS_CONTROL stub defused  *(P1-2)*

**Files:** `Firmware/Src/fsm.c`, `Firmware/Src/closed_loop_controller.c`,
`Firmware/Inc/closed_loop_controller.h`.

- `get_ctrl_law_for_state(POS_CONTROL)` returns NULL (the all-NULL `pos_ctrl_control_law`
  struct used to be installed and ARMED; its NULL `control_law_finish` was then called
  unguarded on exit → branch-to-0 → HardFault → IWDG reboot, one SDO typo away).
- The `control_law_finish` call site is NULL-guarded regardless; the dead law struct and
  `set_pos_setpoint` are deleted. State 6 is also rejected at SDO ingestion (entry 8).

## 8. OD write extensions: always-apply, validating command ingestion  *(§3 redesign, P1-4)*

**Why:** The old `process_sdo_vals` polled the OD against shadow copies — rewriting the same
state/setpoint was a silent no-op (the trap family you flagged), setpoints raced transitions
and poisoned the edge-trigger, NaN setpoints were laundered by `clamp_f32(lo, NaN, hi) == lo`
into **full negative current** with no error.

**Files:** `Firmware/Src/can_comms.c`, `Firmware/Inc/can_comms.h`,
`CANopenSTM32/CANopenNode_STM32/CO_app_STM32.h` (exports `CO`).

- `OD_extension_init` on 0x4004/0x4005/0x4007: every SDO download invokes its handler exactly
  once, regardless of value. The `last_sdo_*` shadow statics are deleted.
- 0x4004 validation: rejects 0 (sentinel), 2/3 (error states), 6 (stub), >21 with SDO abort
  `0x0609 0030`; rejects closed-loop/backemf states without valid encoder cal with
  `0x0800 0022`.
- 0x4005 validation: NaN/Inf (bit-pattern) and |x| > 1e5 rejected with `0x0609 0030`; routing
  by current state unchanged; a write that raced a transition can simply be re-sent.
- 0x4007: any nonzero reboots (was: exactly 1, silently else). Reboot is **deferred** to the
  foreground epilogue with a ~3 ms delay so the SDO response leaves the wire first — hosts now
  get a *confirmed* write instead of a guaranteed timeout. Disarm + console flush retained.
- The known missing `break` in the old setpoint switch went away with the switch.

## 9. Encoder-cal provenance flags, rollback, stale-FF zeroing  *(user req 1.3, P1-11, P2-13)*

**Files:** `Firmware/Src/encoder.c`, `Firmware/Inc/encoder.h`,
`Firmware/Src/encoder_calibration.c`, `Firmware/Src/backemf_cal.c`, `Firmware/Src/cal_store.c`.

- New dir/offset cal-validity flags: set by successful dir cal / phase cal / single-point /
  backemf apply / flash restore; snapshotted and rolled back with the values on failure.
- Dir cal: init snapshots the live direction before the +1 probe; failure/cancel restores it
  (a failed in-flight re-cal used to leave +1 live on a dir=−1 wheel → mirrored commutation,
  velocity loop positive feedback).
- Phase cal: same snapshot/rollback for the offset (exec applies a coarse seed mid-run).
- States 10/15 zero the live h6-FF coefficients when they move the offset (the coefficients
  are phase-referenced to the offset they were demodulated against; zeros are an exact no-op,
  state 21 re-fits).
- `cal_store_save()` refuses unless both flags are valid (the old `dir != ±1` heuristic never
  fired — the power-on default is +1); backemf fit gates check D/k finiteness by bit pattern.
- Single-point cal's `"Fuck you: %.3f"` print got a name.

## 10. Flash-ECC NMI policy + cal-page heal  *(P0-3)*

**Files:** `Core/Src/stm32g4xx_it.c`, `Firmware/Src/cal_store.c`.

- `NMI_Handler`: on ECCD/ECCD2, latch raw ECCR into the RAM2 cell, write-1-clear, **return**
  (the faulting read completes with garbage; the blob CRC rejects it). The old `while(1)` was
  a permanent boot loop: IWDG reset → same boot-time read → NMI forever, before CAN ever came
  up. Unknown NMI sources record + reset.
- `cal_store` brackets every page read (restore, save's compare, `cal_store_present`) with
  clear-latch → read → check-latch. On a hit during restore it **erases the page** (erased
  cells carry valid ECC → next boot reads clean) — but only when `BK_ECC` says the fault was
  in bank 2 *[review-round F6 — an unrelated bank-1 fault in the bracket window must not cost
  a healthy stored cal]*. Save's compare-bracket falls through to rewrite (which repairs ECC).
- `blob_valid()` NaN checks converted to `f32_finite_bits()`.

## 11. Fault handlers: bridge-off + crash record + reset  *(P1-9)*

**Files:** `Core/Src/stm32g4xx_it.c`, `Core/Src/main.c`.

- Common tail `fault_record_and_reset(kind, frame)`: `hrtim_outputs_disable()` FIRST (the
  HRTIM free-runs the last compares into a spinning rotor while the core is wedged — the old
  `while(1)` burned up to ~512 ms of uncontrolled drive waiting for the IWDG to do a blind
  reset), then CFSR/HFSR/BFAR/MMFAR + stacked PC/LR + controller_errors into the crash record,
  then `NVIC_SystemReset()`.
- `HardFault_Handler` is `__attribute__((naked))`: `mrs r1, msp` is guaranteed the first
  instruction at every optimization level *[review-round F7]* — verified in both ELFs.
- `Error_Handler` (app + bootloader): record + reset instead of `__disable_irq(); while(1)`
  (a permanent hang whenever the IWDG wasn't running yet).

## 12. IWDG foreground-liveness gate + bounded waits  *(P1-8, P2-4)*

**Files:** `Core/Src/iwdg.c`, `Firmware/Src/inverter.c`, `Firmware/Src/console.c`,
`Firmware/Inc/console.h`, `Firmware/Src/app_main.c` (heartbeat stroke), `Firmware/Src/util.c`.

- `HAL_IncTick` feeds the IWDG only if the foreground checked in within the last **2 s**
  (gate arms on the first `iwdg_foreground_alive()` stroke — the whole boot path stays
  unconditionally fed). Every host-facing service lives in the foreground; a wedged foreground
  with live ISRs used to be permanently uncommandable with the watchdog happily fed. Worst
  legitimate stall ≈ 65 ms → ~30× margin.
- `inverter_amp_offset_correction`: 2 s timeout on the HRTIM-flag spin (a dead HRTIM used to
  hang boot before CAN init, silently and watchdog-fed); offset sane-range now **±50 mV**
  (owner-specified) with a real early-return — out-of-range offsets are no longer installed
  (the old `assert(false); //return;` fell through and flew ~0.8 A/100-count biased currents),
  leaving `inverter_is_calibrated` false so arming is refused.
- Console TX spins bounded (~8 ms) with a self-healing wedge latch (`console_tx_timeouts`
  counter); `console_flush`'s TC wait bounded so no reset path can hang on a dead UART.

## 13. CANopen/FDCAN robustness  *(P1-5/6/7, P2-6, F8)*

**Files:** `Core/Src/fdcan.c`, `CANopenSTM32/CANopenNode_STM32/CO_driver_STM32.c`,
`CO_app_STM32.c`, `CO_driver_target.h`, `Firmware/Src/app_main.c` (DMA priorities).

- `AutoRetransmission = ENABLE` — DAR single-shot mode silently destroyed every frame that
  lost arbitration on the shared 4-wheel bus. `NominalSyncJumpWidth` 1 → 2.
- Bus-off recovery: `CCCR.INIT` cleared after a 100 ms back-off, polled outside the on-change
  gate (M_CAN does **not** auto-recover; one transient bus fault used to silence the node until
  power cycle).
- `CO_RESET_COMM` no longer calls `CO_delete` (was a use-after-free of the whole CANopen graph
  on every host NMT 0x82); `CO_RESET_APP` (NMT reset-node) now disarms + flushes like 0x4007.
- `CO_LOCK_*` macros: PRIMASK → **BASEPRI(preempt-prio 2)** via `__set_BASEPRI_MAX` — the
  4-TPDO serialization pass (10–25 µs) no longer blocks the 32 kHz fast loop or the FSM tick.
  New invariant (footguns): nothing below priority 2 may touch CANopen state.
- FDCAN IRQs prio 0 → 3 (MspInit used to silently overwrite app_main's setting after
  canopen_init); DMA1 channels 2–5 re-ranked 0 → 3 *[review-round F8]*.

## 14. TPDO rate changes without comm reset  *(§3.3)*

**Files:** `Firmware/Src/can_comms.c`, `Firmware/Src/fsm.c` (caller unchanged semantically).

- `set_tpdo_send_rate()` (TIM17 ISR) only parks a pending value; the foreground applies it via
  `OD_set_u16(…, odOrig=false)`, which routes through CANopenNode's PDO extension → live
  `eventTime_us` update + event-timer zero (TPDOs still burst promptly after transitions).
- The old implementation ran a full `canopen_app_resetCommunication()` from the FSM ISR on
  *every state entry*: TX queue purged, boot-up frame emitted, SDO server re-initialized under
  the foreground. All gone.

## 15. Cal-store flash ops moved to the foreground  *(P2-7)*

**Files:** `Firmware/Src/fsm.c`, `Firmware/Inc/fsm.h`, `Firmware/Src/app_main.c`.

- States 18/19/20 park the op; `fsm_run_pending_calstore_op()` executes it in the foreground
  loop and latches the result into `procedure_result`. A ~22 ms page erase used to run inside
  the TIM17 ISR, masking the CAN tick and SysTick (frozen HAL_GetTick = the HAL's own flash
  timeout could never expire).
- Host-visible: `procedure_result` reads 0 until the op lands (~tens of ms) — poll ~50 ms
  after commanding. The FSM holds new state entries while an op is pending (entry 6).

## 16. Telemetry & diagnostics  *(P2-18, RE-1/2/3/4/5)*

**Files:** `Firmware/Src/can_comms.c`, `Firmware/Src/fsm.c`, `OD/OD.c`,
`Firmware/Src/app_main.c`.

- **Velocity in every state**: armed → PLL (live in any law, including the backemf coast);
  disarmed → new ~200 Hz foreground SPI sampler (PRIMASK-bracketed, bounded spin, RX-FIFO
  drain *[review-round F4]*). A coasting wheel after a trip/cancel used to report vel = 0.
- `procedure_result` latches (transit states no longer overwrite it to 1 — a failed cal's
  result used to be destroyed the moment the host commanded the next step).
- **EMCY** on controller-error edges: per-wheel COB-ID (OD.c `0x1014` 0xBC → 0x80 so
  CANopenNode adds the node ID), raw `controller_errors` bitmask in bytes 4–7, logged in
  0x1003; re-emits when the mask *changes* mid-episode (reset+report pulse
  *[review-round minor]*), cleared-edge frame on recovery.
- **0x4006 = boot diagnostics** (read-only by convention; bit-packed u32s): reset-cause flags
  | crash kind | boot count, and last-fault PC. Boot banner prints LAST FAULT + boot count.
- `0x1018` identity serial from the silicon UID (duplicate-node-ID forensics; LSS-distinct).

## 17. Bumpless velocity-law entry  *(P2-1, review-round F-blocker)*

**Files:** `Firmware/Src/closed_loop_controller.c`, `Firmware/Src/can_comms.c` (+ header).

- `vel_ctrl_init`/`vel_ramp_ctrl_init` seed the setpoint/ramp from `get_disarmed_vel()` — the
  foreground sampler's estimate, ≤5 ms old at arm time. Entering VEL on a spinning wheel holds
  speed instead of full-authority braking toward an implicit zero. *[review-round]*: the first
  version read the PLL, which is frozen while disarmed and reads 0 after a mid-spin reboot —
  exactly the case that needs the seed most.

## 18. Boot order + option bytes  *(P2-8, P2-11, RE BOR)*

**Files:** `Firmware/Src/app_main.c`, `Firmware/Src/bootloader.c`, `Firmware/Inc/bootloader.h`.

- FSM timer starts **after** `canopen_init` (a transition in the gap would have called into
  the CANopen layer with `CO == NULL`).
- Option-byte enforcement now also pins **BOR_LEV = level 4 (~2.8 V)** — the factory ~1.7 V
  default let flash keep programming through a brownout, the exact torn-save trigger for the
  ECC trap. Write attempts bounded to 3 per power-on (RAM2 cell; fail-open with loud console
  output — the old behavior was an unbounded reboot loop wearing OB flash). Dead code after
  `HAL_FLASH_OB_Launch` removed (the launch *is* the reset); callers flush before writing.
  IWDG_SW deliberately left software-start (hardware-start fights bench halting).
- Dead app-side `jump_to_bootloader`/`System_DeInit` removed.

## 19. Bootloader image hardening  *(P0-4)*

**Files:** `bootloader/OpenBootloader_APP/Target/flash_interface.c`, `bootloader/Src/main.c`.

- `OPENBL_FLASH_JumpToAddress` validates the app vector first (SP within the 128K SRAM alias;
  PC thumb-bit set, inside the app image, ≠ 0xFFFFFFFF) and falls into the **DFU loop** on
  failure. Erased/half-written app used to mean MSP=0xFFFFFFFF → HardFault → IWDG → same jump
  → eternal ~512 ms reset loop that never offered DFU. A power-cut mid-reflash is now
  recoverable over UART with no PB8 action.
- `MX_IWDG_Init()` moved before `SystemClock_Config()` — a dead/slow crystal is now a 512 ms
  retry loop instead of a watchdog-less permanent hang; `Error_Handler` resets.

## 20. CCM SEU scrub  *(RE-7)*

**Files:** `Firmware/Src/app_main.c`.

- Every 5 s the foreground CRCs the CCM-resident fast-loop code against its ECC-protected
  flash LMA copy (hardware CRC unit, ~tens of µs). Mismatch → console line + flush +
  `fault_record_and_reset(CCM_SCRUB)` → the startup copy loop restores clean code.
  (`.ccmram` verified to contain only code + a const table — no false-positive source.)

## 21. Aux-ADC VDDA clamp  *(P2-9)*

**Files:** `Firmware/Src/aux_adc.c`.

- Raw VREFINT sample clamped to a 1300–1750-count plausibility band (nominal ~1530), falling
  back to 3.3 V. A zero sample was an integer divide-by-zero (currents/Vbus computed as 0 for
  the tick); a low-but-nonzero corruption overflowed the u32 scaling into garbage gains for
  both the loop and the trip math.

## 22. Landmine deletions + cosmetics  *(task 15, RE-10)*

**Files:** `Firmware/Src/util.c`, `Firmware/Inc/util.h`, `Firmware/Src/app_main.c`,
`Firmware/Inc/ma702.h`, `Firmware/Inc/periphmap.h`, `Firmware/Inc/control_ll.h`,
`Firmware/Inc/foc_capture.h`, `Firmware/Src/inverter.c`, `Core/Src/tim.c`.

- Deleted: `run_state()` (unbounded foreground busy-wait, zero callers), `micros()`/`delay_us`
  (micros stubbed to 0 → delay_us never terminated), the dead `ma702_fast_read_start/_finish`
  inlines (the dead `_finish` lacked the `>>2` shift — swapping it in would have scrambled
  every angle), stale `ENCODER_TIM_HANDLE`/`SYSTICK_HANDLE` defines, dead
  `controller_get_micros` prototype.
- TIM4/7/17 prescalers 144 → 143 (they ticked at 993 Hz, not 1 kHz).
- inverter.c header rewritten (described the v7 INA240/325 kHz/Timer-B board);
  foc_capture.h size comment corrected (256 × 92 B = 23 KB).

## 23. Documentation sweep  *(task 16)*

**Files:** `docs/can-interface.md` (command semantics, strap table, recovery sequence, 0x4006
encoding, EMCY, reboot-detection guidance), `docs/footguns.md` (2 entries rewritten — BASEPRI
invariant, OD-extension ingestion; 4 added — option-byte bounds, RAM2 cells/stack contract,
NMI return-not-spin, f32_finite_bits, **regen retired**), `docs/control-laws.md` +
`docs/control-cycle.md` + `docs/ccmram.md` + `CLAUDE.md` (32 kHz, extension ingestion, POS row,
VEL_RAMP_RATE 20 not 200), `docs/bootloader-integration.md` (§4.3 IWDG gate, §4.4 DFU
fallback), `docs/flight-review-2026-06.md` (punch list statuses + review addendum + bench
checklist), `CLAUDE.md` (**CubeMX/CubeIDE/CANopenEditor regeneration permanently retired** —
generated files now carry `/* manual edit */` flight fixes outside USER CODE markers).

---

## Known-open items (deliberate)

Owner-declined: overspeed/overtemp trips, hardware OCP (protected upstream), shunt-selection
leakage check + dead IBUS constants, MA702 read plausibility / fast-loop RXNE bound, bootloader
flash-write floor. Open minors: backemf-cancel µs OENR window (documented, fix risked a worse
regression), state-21 sub-result reporting / fit R gate / restore-bounds tightening
(bench-untested subsystem left minimally diffed), comms-loss-while-armed policy (needs an ops
decision), bootloader PB8 majority sampling + DFU inactivity timeout + OB write masking,
optional boot Vbus-plausibility gate. `can_console/cli.py` needs updating (NONE-nudge
unnecessary, bench default node `0x3C`, poll-state-before-setpoint, procedure_result timing).
