# Flight-readiness deep review — 2026-06-09

Full-codebase review of the app firmware, the vendored bootloader, the CANopen port glue, and
the hardware configuration (cross-checked against RM0440 / the G474 datasheet in `resources/`),
with two lenses: **(1) bugs that end the mission** — bricks, unrecoverable hangs, watchdog blind
spots, motor runaway, hardware-destroying conditions, states recoverable only by physical access —
and **(2) rough edges** — host-interface semantics that will bite operators, silent no-ops,
missing observability.

**How it was produced.** Eight parallel review passes (fast loop/FOC core; FSM/control laws;
CAN/OD interface; calibration + flash store; boot/infra/console/watchdog; hardware config vs.
reference manual; operator command sequences; bootloader subproject), followed by line-by-line
hand verification of every load-bearing claim in this document. Findings marked **verified**
were re-traced by hand in source; everything else was confirmed by at least two independent
passes with quoted code. Where a claim rests on an assumption that couldn't be checked from the
repo (schematic, GaN part number, EPS behavior), that's stated explicitly.

**Reference commit: `ebdba3d`** ("Add h5/h7 compensation, PC sampling"). File:line refs were
current when written — this is a moving codebase, verify before acting on them (same caveat as
`docs/footguns.md`).

**Severity scale:**

| Sev | Meaning |
|-----|---------|
| P0  | Plausible mission loss for a wheel: brick, boot loop, defeated protection, CAN identity collision |
| P1  | Wheel unrecoverable until power-cycle, remotely-triggered reboot/lockup, or serious uncommanded behavior |
| P2  | Transient fault, degraded protection fidelity, fragile-but-currently-safe structure |
| RE  | Rough edge: operator hazard, silent no-op, observability gap, doc drift |

Findings already documented in `docs/footguns.md` are not re-reported as discoveries; where one
appears here it's because the *flight consequence* needed assessment or a cheap mitigation exists.

---

## 1. P0 — brick class

### P0-1. Node-ID strap vote reads pins configured as push-pull outputs driven low — every board enumerates as wheel 4 ✅ verified

**Where:** `Firmware/Src/app_main.c:216-239` (vote), `Core/Src/gpio.c:60,79-83` (pin config),
`Core/Inc/main.h` (`CAN_ADDR0`=PB7, `CAN_ADDR1`=PB5), `docs/can-interface.md` (strap scheme).

**Mechanism.** `MX_GPIO_Init` configures DEBUG0–3 (PD4/PD2/PD7/PD3) as `GPIO_MODE_OUTPUT_PP`
with the output latch written LOW:

```c
HAL_GPIO_WritePin(GPIOD, DEBUG0_Pin|DEBUG2_Pin|DEBUG1_Pin|DEBUG3_Pin, GPIO_PIN_RESET);
...
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
```

The boot-time vote then reads IDR on DEBUG1/2/3 1000 times and maps **LOW = strapped**
(`debugN = avg < 500`). On STM32 the input Schmitt trigger stays connected in output mode, so
IDR reflects the real pin level — and a push-pull driver actively holding low (~25 Ω) always
reads low unless external hardware fights it (which would be a pin-damaging contention, not a
strap). Result: `debug1 && debug2 && debug3` is always true → `wheel_number = 4` →
`NODE_CANID = 0x48` on **every** board, regardless of strapping.

Meanwhile the v8 board has dedicated **`CAN_ADDR0`/`CAN_ADDR1` input pins** (PB7/PB5, configured
`GPIO_MODE_INPUT`/`GPIO_NOPULL` at `gpio.c:65-69`) that are referenced **nowhere** in
`Firmware/` — two bits, four wheels, clearly the intended strap source. The DEBUG-pin vote
appears to be v7 code reading the wrong pins; the same DEBUG pins double as the `gpio_debug*`
scope-timing outputs in `util.h`, which is incompatible with also being ID straps.

**Flight consequence.** Four wheels with identical node IDs = identical heartbeat/TPDO/SDO
COB-IDs. Arbitration ties between frames with identical IDs but different payloads are
guaranteed bit-error storms → error-passive escalation → bus-off cascades. The ACS bus is
unusable as a system. Every SDO command intended for "wheel 4" actuates all four wheels.

**Why the bench never showed it:** a single wheel just enumerates as 0x48 and the host tooling
uses whatever ID appears.

**Falsification test (5 seconds):** boot any board with no straps fitted — the banner prints
`We are wheel number 4`.

**Fix.**
1. Check the schematic: which nets does the flight harness actually strap?
2. If CAN_ADDR0/1: read those (2 bits → 4 wheels) and **add pulls** — they're currently NOPULL
   inputs and would float unstrapped.
3. If the DEBUG pins really are the straps: reconfigure DEBUG1–3 to input+pull-up for the vote
   window, restore output mode after.
4. Either way: publish the strap result and raw averages over CAN (see RE-5) so a mis-strap is
   diagnosable, and have the host do a heartbeat census before any actuation.

Cosmetic in the same block: `u32 debug1_avg = 0.f;` (float literal initializing a u32), and
DEBUG0 is written RESET before the vote but never read.

---

### P0-2. DC-bus overvoltage trip is unreachable; undervoltage trip is a bench placeholder ✅ verified

**Where:** `Firmware/Inc/config.h:5-9,25-33`, `Firmware/Src/control_ll.c:213,251-254`,
`Firmware/Inc/mathutil.h` (`VVCC_CORRECT`).

**Mechanism.** With the compiled divider (`IS_CORRECTED_VOLTAGE_DIVIDER` commented out →
5100/3300, ratio 0.39286), the conversion is:

```c
f32 Vbus = VVCC_CORRECT(Vbus_raw, vvcc) * 1 / VSNS_VOLTAGE_DIVIDER;
// = (counts/4095 × VDDA) / 0.39286
```

The ADC saturates at 4095 counts = VDDA at the pin, so the largest Vbus the firmware can ever
*compute* is `VDDA / 0.39286` ≈ **8.40 V at VDDA = 3.300 V**. The trip threshold is
`DC_BUS_OVERVOLTAGE_THRESHOLD_V = 4.3f * 2.f` = **8.6 V** — above the measurement ceiling. The
trip would need VDDA > 3.38 V, out of spec for the rail. (The 4× injected oversampling uses
shift-2, preserving 12-bit scale — verified against the ADC config — so there's no headroom
hiding in the oversampler.)

> **2026-10-26:** the ADC full scale is VREF+ = 3.0 V (external reference), not VDDA. The 3.3 V
> scale this section assumes is what the broken P2-9 clamp made the firmware use. Rev 8.1.1
> boards carry the 8.2k/2.7k divider, which makes the trip reachable; see the P2-9 note.

**Flight consequence.** Regen pumping the bus during deceleration — the exact event this trip
exists for — is invisible to the firmware. There is no other regen management (the
`IBUS_OVERREGEN_THRESHOLD_A` constant is dead code, see P2-12), so bus protection during
momentum dumps is entirely the EPS's problem. Additionally, a full 2S pack (8.4 V) sits at the
ADC rail, so Vbus telemetry saturates at top-of-charge even in normal operation.

**Compounding:** `DC_BUS_UNDERVOLTAGE_THRESHOLD_V = 2.7f; // TODO ************ REMEMBER TO
CHANGE THIS ************` (`config.h:9`). On an 8.4 V bus this only catches a near-dead supply;
brownout-induced FOC misbehavior (gain scaling through `V_to_mod` with sagging Vbus) won't trip.

**Both bus-voltage protections are effectively vestigial as compiled.**

**Fix.**
1. If flight boards have the 8200/2700 rework: define `IS_CORRECTED_VOLTAGE_DIVIDER` (ceiling
   becomes ≈13.3 V; the 8.6 V trip becomes meaningful).
2. If not: lower the OV threshold below the ceiling (≤8.2 V), **or** treat sustained near-rail
   counts (≥ ~4090) as an OV trip condition.
3. Set the UV threshold to a real 2S floor (resolve the TODO).
4. Add a boot-time plausibility check: measured Vbus outside a 5–9 V window → TPDO-visible flag
   + refuse to arm. This converts the silent 1.586× scale error of a board-rev/firmware mismatch
   (`PHASE_V_COUNTS_TO_VOLTS` additionally bakes in "phase dividers match VBUS divider") into a
   loud one before launch. As compiled-wrong-direction, the OV trip point silently becomes
   ~13.6 V and the current-loop gain mis-scales ~1.6×.

---

### P0-3. Flash-ECC double-bit error on the cal page → NMI `while(1)` → IWDG → permanent boot loop ✅ verified

**Where:** `Core/Src/stm32g4xx_it.c:83-92` (`NMI_Handler`), `Firmware/Src/cal_store.c:49-77`
(`stored()`/`blob_valid()`), `Firmware/Src/app_main.c:191` (`cal_store_restore()` at every
boot), `Core/Src/iwdg.c` (IWDG ~512 ms, running from boot).

**Mechanism.** `cal_store_restore()` runs unconditionally at every boot — **before**
`canopen_init()` (line 191 vs. 245) — and reads the cal page at 0x0807F800 directly:

```c
static const cal_blob_t *stored() { return (const cal_blob_t *) CAL_STORE_ADDR; }
```

`blob_valid()` then CRC-reads all 532 bytes. G474 flash has ECC; a **double-bit error raises
NMI on the read itself** (RM0440: ECCD → NMI; the handler must clear ECCD). The NMI handler is:

```c
void NMI_Handler(void) { while (1) { } }
```

NMI can't be preempted → SysTick (the sole IWDG feeder) starves → IWDG resets in ~512 ms → the
bootloader jumps back to the app → the same read at boot → NMI again. **Infinite reboot loop;
the wheel never reaches CAN; nothing in flight can clear the bad ECC** (the only cure is an
erase the firmware never gets to execute).

**How the page gets an invalid-ECC dword:**
- (a) Power loss / brownout / IWDG reset during the ~28 ms save window (erase ≤22 ms + 69
  double-word programs). An interrupted program or erase leaves cells with invalid ECC. **The
  CRC never gets a chance to protect you, because the read faults before the CRC runs.** This
  is precisely the event an in-flight cal save can collide with — and BOR_LEV is at the factory
  minimum (~1.7 V, see P2-15), so a slow brownout will happily corrupt a write in progress.
- (b) A radiation double-bit upset in that page any time over mission life.

The same trap is reachable in flight (not just at boot): `cal_store_save()`'s
`memcmp(stored(), ...)` (cal_store.c:132,154) and `cal_store_present()` read the page from
TIM17 context — an ECC hit there NMIs a running wheel (disarmed during cal states, so no motor
hazard, but it still reboots and then boot-loops).

**What's already right:** the blob format itself is solid — magic + version + CRC32 with the
CRC programmed in the final dword (front-to-back program order), semantic bounds on
offset/dir/LUT/FF. It protects against *logically* torn data. The ECC trap is upstream of all
of it.

**Fix (~20 lines, the single most important pre-flight change in the cal subsystem):**
implement a real NMI handler:
1. Read `FLASH->ECCR`. If `ECCD`/`ECCD2` set: record the failing address (`ADDR_ECC`, `BK_ECC`)
   in a noinit word, write-1-clear the flags, **return** — execution resumes past the faulting
   load with garbage data, which then fails `blob_valid()`'s CRC, and boot continues
   uncalibrated. Host re-cals over CAN with the existing runbook (9→15→21→18→save).
2. In `cal_store`, bracket page reads with a check of that fault flag; if set, treat the blob
   as invalid **and call `erase_page()`** to restore valid-ECC (erased) state.
3. Unknown NMI source (not flash ECC): `NVIC_SystemReset()` rather than spin — a reboot loop
   with a breadcrumb beats a silent one. (CSS is not enabled, SRAM parity is not enabled, so
   flash ECCD is currently the *only* NMI source on this configuration — verified.)
4. Bench test by cutting power mid-save. The save/restore flash path is flagged bench-untested
   in the worklog; this failure mode is the reason to test it.

This fix is also what makes the deliberate single-copy cal page (no A/B) acceptable: page death
degrades to "boot uncal + re-cal over CAN" instead of a brick.

---

### P0-4. Bootloader jumps to an invalid app with no validity check — eternal reset loop that never offers DFU ✅ verified

**Where:** `bootloader/OpenBootloader_APP/Target/flash_interface.c:137-153`
(`OPENBL_FLASH_JumpToAddress`), `bootloader/Src/main.c:136-147` (boot decision),
`bootloader/Src/stm32g4xx_it.c` (fault handlers = `while(1)`).

**Mechanism.** The PB8-low path jumps unconditionally:

```c
jump_to_address = (Function_Pointer)(*(__IO uint32_t *)(Address + 4U));
Common_SetMsp(*(__IO uint32_t *) Address);
jump_to_address();
```

No check of vector[0]/vector[1] — not even `!= 0xFFFFFFFF`. With app flash invalid (power loss
mid-reflash, radiation hit in the vector page, a bad image discovered after the strap was
released): MSP = 0xFFFFFFFF, branch to 0xFFFFFFFE → HardFault → bootloader's `while(1)` handler
→ IWDG (~512 ms) → reset → PB8 still low → jump again. **Infinite ~512 ms reset loop. DFU is
never offered because DFU entry is PB8-only** — `docs/bootloader-integration.md` explicitly
lists a soft entry trigger as not-implemented.

**Severity hinges on one unanswered hardware question: can anything on the spacecraft drive
PB8?** The reset loop re-samples PB8 every cycle, so if the C3/harness can pull it high,
recovery needs no power cycle and this is P1. If nothing can drive PB8 in flight, any
invalid-app state is a **dead wheel (P0)**. Resolving this is the most important pre-launch
question for the bootloader subsystem.

**Fix (~10 lines):** before jumping, validate `sp` in SRAM and `pc` in
`[0x08008000, 0x08080000)` with the Thumb bit set and ≠ 0xFFFFFFFF; on failure, fall into the
DFU loop regardless of strap. That decouples reflash recovery from the PB8 wire entirely.

**Companion bootloader items (full detail in §6):**
- **P1:** the DFU protocol accepts page erase/write targeting the bootloader's own pages 0–15
  (no software floor, no WRP) — one off-by-one in the C3's flasher page math self-destructs the
  bootloader (protocol *mass*-erase is already blocked via `MassErase = NULL`, which is good and
  worth keeping deliberate).
- **P1:** both images run `SystemClock_Config` (HSE-only, no CSS, no HSI fallback) **before**
  `MX_IWDG_Init`, with `Error_Handler` = `__disable_irq(); while(1)`. IWDG_SW is never
  programmed to hardware-start. A dead/slow-starting crystal at boot is a watchdog-less
  permanent hang — on every boot if the crystal is truly dead. (Runtime HSE death: the
  already-running IWDG resets within 512 ms… into the same boot-time hang.)

---

## 2. P1 — remotely-triggered lockups, reboots, uncommanded behavior

### P1-1. FSM error ping-pong: IDLE↔CONTROLLER_ERROR oscillation with a full CANopen comm reset per transition; CLEAR_ERRORS parks in a zombie error state ✅ verified by hand-trace

**Where:** `Firmware/Src/fsm.c:91-133` (`fsm_exec` branch ladder), `fsm.c:186,190`
(`fsm_enter_state` → `set_tpdo_send_rate`), `Firmware/Src/can_comms.c:203-210`
(`set_tpdo_send_rate` → `canopen_app_resetCommunication()`, immediately, in TIM17 ISR context),
`Firmware/Src/control_ll.c:444-462` (`control_law_is_done` lifecycle),
`Firmware/Src/motor_calibration.c:89,122,224` (errors added in `finish()`).

**The branch ladder** (paraphrased, in priority order):

1. `errors != 0 && state != CONTROLLER_ERROR && requested != CLEAR_ERRORS` → exit current,
   enter CONTROLLER_ERROR. **Does not consume `fsm_requested_state`.**
2. `state != IDLE && !armed && control_law_is_done` → exit, enter IDLE.
3. `requested != NONE && state == IDLE` → enter requested; `requested = NONE` (the **only**
   place a request is consumed).
4. `requested != NONE && state != IDLE` → exit, enter IDLE. **Does not consume the request.**
5. Action states with no request → auto-request IDLE.

Every `fsm_enter_state` — both law and non-law branches — calls `set_tpdo_send_rate()`, which
**re-initializes the entire CANopen stack** (`canopen_app_resetCommunication`: CAN module
disabled, `CO_CANopenInit` re-run, NMT re-init → boot-up frame emitted, TPDO timers reset) from
the TIM17 ISR.

**Failure mode A — request-driven ping-pong.** Wheel in CONTROLLER_ERROR, errors latched. Host
writes the most natural recovery value, IDLE (1) — or *any* state except CLEAR_ERRORS:

- Tick N: branch 4 → exit CONTROLLER_ERROR, enter IDLE. Request stays pending.
- Tick N+1: branch 1 (errors ≠ 0, state = IDLE, requested ≠ CLEAR_ERRORS) → exit IDLE, enter
  CONTROLLER_ERROR.
- Repeat forever, one transition (and one full CANopen comm reset, plus a boot-up frame and a
  TPDO burst) per 1 kHz FSM tick, on the bus shared by all four wheels and the C3.

The same loop starts if a host state-write merely *races* an error trip (write VEL_RAMP, OC
trips the same tick — request left pending, branch 1 shadows branch 3 forever).

**Failure mode B — autonomous ping-pong, no host input.** `control_law_is_done` is latched by
the fast loop when a law's `is_complete()` fires (`control_ll.c:372-375`) and is **only ever
cleared by `controller_set_control_law()`** — which only runs when entering a law state.
`controller_clear_control_law()` exists and has **zero callers** (grep-verified). The R/L cal
laws add `CTRLR_ERR_MOTOR_*_OUT_OF_RANGE` **inside `finish()`**, i.e. *after* is_done latched:

- Cal completes → branch 2 exits to IDLE (finish() adds the error on the way out).
- Branch 1: IDLE → CONTROLLER_ERROR.
- Branch 2 (is_done still true, still disarmed): CONTROLLER_ERROR → IDLE.
- Branch 1 → … forever.

Running R-cal on a wheel with a broken phase wire — exactly the diagnostic you'd run on a sick
wheel — starts a permanent comm-reset storm autonomously.

**Failure mode C — CLEAR_ERRORS parks in CONTROLLER_ERROR with errors == 0.** Host writes 13
from CONTROLLER_ERROR with errors latched:

- Branch 4 → exit CONTROLLER_ERROR, enter IDLE (request 13 keeps branch 1 suppressed).
- Branch 3 → enter CLEAR_ERRORS, request consumed to NONE.
- Branch 1 fires (errors still set — they clear only on **exit** of CLEAR_ERRORS; only
  `requested == CLEAR_ERRORS` is exempted, not `state == CLEAR_ERRORS`):
  `fsm_exit_state(CLEAR_ERRORS)` clears the errors via case fall-through… and then **enters
  CONTROLLER_ERROR**.
- Final state: CONTROLLER_ERROR, errors == 0, parked. The designed auto-return branch 5 is
  unreachable whenever there were real errors to clear — i.e. in the only case CLEAR_ERRORS
  exists for. (If `control_law_is_done` happened to be latched, branch 2 rescues it to IDLE;
  after a mid-run trip — the common case — it parks.)

A host that writes 13 and polls for `state == IDLE` hangs forever; re-writing 13 is a silent
no-op (edge trigger, §3). Recovery works only because writing any *different* state value exits
via branch 4 — undocumented, and it re-enters failure mode A if errors re-latch.

**Escape from the storm:** only a CLEAR_ERRORS SDO landing *through* ~1 kHz comm resets (the
SDO server is being re-initialized between request and response). Frames addressed to this node
during reset windows are ACKed by the *other three wheels* (so the host's CAN controller never
retransmits — see P1-7) and silently lost. If the storm's two comm resets per 2 ms overrun the
TIM17 budget, SysTick starves and the node IWDG-reboots — arguably the better outcome.

**Fix (~15 lines total, kills A+B+C):**
1. Branch 4 (and the branch-1 path): consume/clear `fsm_requested_state`.
2. Exempt `fsm_state == FSM_STATE_CLEAR_ERRORS` from branch 1 (or clear errors on CLEAR_ERRORS
   *entry* instead of exit).
3. Clear `control_law_is_done` on every state entry (call `controller_clear_control_law()` for
   NULL-law states).
4. Decouple TPDO rate from comm reset entirely — see §3.3.

---

### P1-2. Writing 6 (POS_CONTROL) to 0x4004 calls a NULL function pointer → HardFault → IWDG reboot of the wheel ✅ verified

**Where:** `Firmware/Src/closed_loop_controller.c:115`, `Firmware/Src/fsm.c:62-63,162,185-188`.

```c
control_law_t pos_ctrl_control_law;          // zero-initialized .bss — every pointer NULL
...
fsm_last_state_exit_ok = controller_get_control_law()->control_law_finish(cancelled);  // fsm.c:162 — no NULL check
```

`get_ctrl_law_for_state(POS_CONTROL)` returns the (non-NULL) *address* of the all-NULL struct,
so `fsm_enter_state` installs it and **arms**. The fast loop's per-pointer NULL guards mean no
torque (the bridge holds the inert 0.5/0.5/0.5 vector for ~2 ticks ≈ 62 µs) before the
missing-update checks trip and disarm. The next FSM tick takes the error branch →
`fsm_exit_state(POS_CONTROL, true)` → the **only unguarded law-pointer call in the system**
invokes `control_law_finish == NULL` → branch to 0 → HardFault → `while(1)` handler → SysTick
starves → IWDG reboot ~512 ms later, through the bootloader, multi-second CAN outage.

One errant SDO — state 6 is enum-adjacent to VEL_CONTROL (5) and documented as a real state —
remotely reboots a flight wheel. `docs/control-laws.md` says state 6 "faults out"; the actual
behavior is a crash.

**Fix:** NULL-guard the `control_law_finish` call at `fsm.c:162`, and make
`get_ctrl_law_for_state` return NULL for POS_CONTROL (reject like an action state) until the
law exists.

---

### P1-3. `is_currents_updated` latches once and poisons every future arm until reboot ✅ verified end-to-end

**Where:** `Firmware/Src/control_ll.c:173-174` (check), `:283-289` (the only clear),
`:391-407` (`controller_arm()` never resets it). Triggers: `closed_loop_controller.c:115`
(POS_CONTROL's NULL consumer), `motor_calibration.c:71-77,191` (cal consumers that
`return false` on out-of-bounds current/vmod).

**Mechanism.** Any armed tick that ends with `is_currents_updated == true` — the consumer
returned false, or the law has a NULL consumer — leaves the flag latched through the disarm.
On the next arm, tick 1 of `adc_update_callback`:

```c
if (is_currents_updated)                      // stale true from last time
    controller_add_errors(CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING);
...
if (controller_errors != CTRLR_ERR_NONE) { controller_disarm(); goto fail; }  // exits before…
...
is_currents_updated = true;                   // …this line, and before
if (ret_val) is_currents_updated = false;     // …the only line that ever clears it
```

The error fires **before** the only code path that could clear the flag, every time.
CLEAR_ERRORS clears the error bits but nothing clears the flag. **Every subsequent arm of any
state fails on its first tick**; the wheel can never produce torque again until the host writes
0x4007 (reboot) or the EPS power-cycles the card. The encoder path doesn't latch this way (its
consumer runs before its usage check) and the PWM path doesn't (arm re-seeds
`is_pwm_timings_updated = true`); the current path is the odd one out.

**Reproduction over CAN:** request state 6 → CONTROLLER_ERROR → CLEAR_ERRORS → request state 5
→ immediate CONTROLLER_ERROR with only `USAGE_MISSING`-class bits set; repeat forever. A failed
R/L cal abort does the same. Combined with P1-1's storm, one bad cal can take a wheel from
"healthy" to "torque-dead and spamming the bus" with no further input.

**Fix (one line):** `is_currents_updated = false;` in `controller_arm()` (plus
`is_encoder_updated = false;` for symmetry). Optionally also clear both in `controller_disarm()`.

---

### P1-4. NaN setpoint is laundered into −5 A full reverse torque; no setpoint validation anywhere; no overspeed or overtemperature trip ✅ verified (arithmetic hand-checked)

**Where:** `Firmware/Src/mathutil.c:164-176` (`clamp_f32`),
`Firmware/Src/closed_loop_controller.c:40-50` (`set_torque_setpoint`), `:97-99`
(`set_vel_setpoint` — no validation at all), `Firmware/Src/can_comms.c:164-181` (ingestion,
no `isfinite`), `Firmware/Src/foc.c:192-201` (saturation handled silently).

**The laundering.** `clamp_f32(lower, x, upper) = max_f32(min_f32(upper, x), lower)` with
comparison-based min/max:

- `min_f32(5, NaN)`: `(5 < NaN)` is false → returns **NaN**.
- `max_f32(NaN, −5)`: `(NaN > −5)` is false → returns **−5**.

So `set_torque_setpoint(NaN)` → `NaN / MOTOR_TORQUE_CONSTANT` = NaN → clamp → **−5.0 A**
`set_iq_setpoint`. A perfectly finite, in-range command; `svm_output_valid`'s NaN guard (which
is correctly written in must-prove-valid polarity) never sees a NaN because the laundering
happened upstream. One NaN f32 written to 0x4005:01 — one host-side divide-by-zero away — and
the wheel slams to sustained full negative torque with `errors == 0`, until a secondary trip
(OC at speed; OV is unreachable per P0-2) or host intervention dumps maximum momentum into the
spacecraft.

Aggravators, all verified:
- `NaN != NaN` is always true, so the edge-trigger compare **re-applies it every ~1 ms forever**
  instead of eating it like a normal value.
- VEL_CONTROL: `vel_err` = NaN → integrator clamps to −limit, `torque` = NaN →
  `set_torque_setpoint(NaN)` → same −5 A.
- VEL_RAMP: both ramp comparisons are false on NaN → setpoint frozen, **silently ignored** —
  the cross-state inconsistency makes this easy to miss in test.
- `set_vel_setpoint` accepts any finite magnitude too (no clamp): a 1e6 rev/s command
  degenerates to sustained max torque.

**No overspeed backstop exists in any mode.** At the voltage ceiling, `foc_gen_pwm_cb` scales
the modulation vector down and bleeds the integrator (`mod_scalefactor < 1` branch) — a *valid*
SVM output every tick, no `CTRLR_ERR_MODULATION`, no error of any kind. There is no velocity
trip anywhere (no config constant, no check — grep-verified). A latched torque command
accelerates the wheel to its physical maximum (MOTOR_KV = 6300 → tens of kRPM at 8 V) with zero
firmware objection. **No overtemperature trip either**: two board thermistors + die temp are
sampled at ~1 kHz and shipped on TPDO 0x4003, and nothing ever compares them to a limit — a
stalled rotor inside current limits cooks the board until ground notices.

**Fix.**
1. Validate at the single choke point (`process_sdo_vals`): reject non-finite setpoint_1
   (count/flag it), clamp to a state-appropriate range.
2. Optionally make `clamp_f32` NaN-safe; at minimum add the load-bearing
   `isfinite` checks with integer bit-pattern tests (see P2-19 on `-Ofast`).
3. Add `CTRLR_ERR_OVERSPEED` checked in the fast loop against `encoder_get_pos_vel().vel` and a
   config max; clamp `vel_setpoint`/`ramped_vel_setpoint` to it.
4. Add a thermistor trip through the existing error machinery (`CTRLR_ERR_OVERTEMP`).

---

### P1-5. Host NMT Reset-Communication (0x82) is a use-after-free of the whole CANopen object graph ✅ verified

**Where:** `CANopenSTM32/CANopenNode_STM32/CO_app_STM32.c:217-222` (CO_RESET_COMM path), `:120`
(first dereference).

```c
if (reset_status == CO_RESET_COMM) {
    CO_CANsetConfigurationMode((void*)canopenNodeSTM32);
    CO_delete(CO);                              // frees CO and every sub-object (heap; CO_USE_GLOBALS not defined)
    canopen_app_resetCommunication();           // first statement: CO->CANmodule->CANnormal = false;
```

This port diverged from upstream (which re-news the objects): it frees the graph, then
dereferences and re-initializes structures **inside freed newlib heap chunks**, corrupting the
free list on the first host-sent NMT 0x82. A second reset-communication double-frees. It works
until heap metadata bites — then a HardFault (bare `while(1)` → IWDG reboot) or silent
corruption — and NMT reset-communication is precisely the command a CANopen master sends as an
anomaly-recovery action. The heavily-exercised `set_tpdo_send_rate` → `resetCommunication` path
does *not* free and is structurally fine, so bench mileage proves nothing about the 0x82 path.

**Fix (one line):** remove the `CO_delete` (in-place re-init is supported and is what the other
path already does), or treat CO_RESET_COMM like CO_RESET_APP (flush + `NVIC_SystemReset`).

Related inconsistency: the CO_RESET_APP path (NMT 0x81) calls `HAL_NVIC_SystemReset()` directly
— no `controller_disarm()`, no `console_flush()` — while the 0x4007 handler does both
(`can_comms.c:193-197`). Functionally safe-ish (peripheral reset floats the bridge) but worth
mirroring.

---

### P1-6. FDCAN bus-off is never recovered — one bus-off permanently silences the wheel ✅ verified

**Where:** `CANopenSTM32/CANopenNode_STM32/CO_driver_STM32.c:~420` (`CO_CANmodule_process`);
no `HAL_FDCAN_ErrorStatusCallback` override anywhere in the tree.

On bus-off, the G4's M_CAN sets `CCCR.INIT` **by itself** and halts all bus activity; recovery
requires software to clear INIT (then the hardware waits 129 × 11 recessive bits per the
protocol). The driver merely mirrors `PSR.BO` into a status flag with the comment "we expect
that the controller is automatically handling the protocol exceptions" — true of bxCAN with
ABOM (the `#else` branch this was copied from), **false for FDCAN**. The BUS_OFF interrupt is
activated but lands in the empty weak HAL callback. The only code that ever calls
`HAL_FDCAN_Start` is `canopen_app_resetCommunication` — reachable solely via FSM transitions,
which are commanded over the now-dead bus.

**Flight consequence:** a transient bus short, connector chatter, or a babbling neighbor →
this wheel drops off CAN until something reboots it. If it was **armed**, there is no
comms-loss failsafe anywhere in `Firmware/` — the wheel holds its last setpoint indefinitely
with zero telemetry, uncommanded momentum until the C3 cycles power. (Accidental partial
mitigation: a controller fault while armed → CONTROLLER_ERROR entry → comm reset → FDCAN
restarted. Don't rely on it.)

**Fix:** in `CO_CANmodule_process` (or an `ErrorStatusCallback` override), on `PSR.BO`: back
off per policy, then `HAL_FDCAN_Start()` (clears INIT). Separately, **decide and document a
comms-loss policy for armed wheels** (e.g. ramp to zero / disarm after N seconds without host
traffic) — that's a flight-ops decision, but right now the implicit policy is "spin forever."

---

### P1-7. FDCAN automatic retransmission is disabled — every arbitration loss silently destroys a frame ✅ verified

**Where:** `Core/Src/fdcan.c:44` — `hfdcan1.Init.AutoRetransmission = DISABLE;` (M_CAN DAR mode).

Every frame is transmitted exactly once. On the flight bus (4 wheels + C3, four wheels' TPDOs
on identical nominal periods firing 4-frame bursts, heartbeats at 1 Hz), arbitration collisions
are routine — each loss is a *silently dropped* heartbeat / TPDO / SDO response / EMCY /
boot-up, not a delayed one. CANopen's machinery assumes CAN's retransmission semantics. The
host-visible symptom is random SDO timeouts, telemetry gaps, and false heartbeat-loss alarms
that look like flaky wheels — and a 2-node bench almost never collides, so it's invisible until
integration. DAR also accelerates error-passive entry (TEC +8 per unACKed attempt).

**Fix:** `AutoRetransmission = ENABLE` in the .ioc and regenerate. If DAR was chosen to bound
TPDO staleness, per-PDO inhibit/event times are the right tool.

---

### P1-8. The watchdog can't see the only context that can command the wheel; two real unbounded foreground spins exist ✅ verified

**Where:** `Core/Src/iwdg.c:66-70` (feed), `Firmware/Src/app_main.c:295+` (foreground loop),
`Firmware/Src/inverter.c:119-148` (boot offset cal), `Firmware/Src/console.c` (TX spins).

```c
void HAL_IncTick(void) {            // SysTick ISR, priority 15
  uwTick += uwTickFreq;
  IWDG->KR = IWDG_KEY_RELOAD;       // unconditional
}
```

The IWDG (LSI/4, reload 4095 → **~512 ms**, window off — verified) proves exactly one thing:
SysTick runs. That correctly catches ISR storms, spinning fault handlers, and the documented
Debug-build fast-loop saturation. But **everything the host can use to command or recover the
wheel runs in the foreground while(1)**: the SDO server, NMT processing (including Reset-Node —
the host's last-resort reboot), heartbeat production, EMCY, `process_sdo_vals` (state requests
and the 0x4007 reboot register). A wedged foreground with live ISRs is therefore *permanent*,
with a nasty signature: TPDOs keep transmitting from the TIM4 ISR — **with frozen payloads**
(`update_tpdo_od_vals` is foreground) — while heartbeats and SDO go dead. Recovery: power cycle
only.

Concrete unbounded foreground loops found:
- **`inverter_amp_offset_correction()`** at every boot, *before* `canopen_init`:
  `for (20000 samples) { while (!OFFSET_CORRECTION_FLAG); }` — waits on an HRTIM-ISR-set flag
  with no timeout (`inverter.c:142`). If the HRTIM repetition interrupt never fires (HRTIM
  start failure — note the arming `assert`s print-and-continue, P2-9; callback registration
  failure; clock anomaly), the wheel hangs **silently, CAN-less, watchdog-fed, on every boot**.
- **`console_tx_byte`** spins on TXE forever (every `console_pump` and every synchronous
  thread-mode printf); `console_flush` spins on TC. TXE physically can't stay 0 with a clocked,
  enabled USART — the trigger is SEU/fault class (UE or clock-enable bit flip) — but the spin
  is unbounded by construction.
- Latent: `run_state()` (`app_main.c:36-51`) busy-waits on an FSM state that an error diversion
  makes unreachable — currently only referenced by commented-out code; `delay_us()` never
  terminates because `micros()` returns 0 (`util.c:13-31`). Delete or fix both before someone
  revives them.

**Fix.**
1. Foreground-liveness-gated IWDG: foreground increments a counter each pass; `HAL_IncTick`
   reloads IWDG only if the counter advanced within the last N ms (N ≈ 2–5 s; generous grace
   period during boot, or stroke the counter inside known-long boot calibrations).
2. Bound the offset-cal wait (tick-budget per sample → on failure set
   `CTRLR_ERR_INVERTER_CALIBRATION_INVALID`, skip cal, **continue to CAN init** so the error is
   visible — the fast loop already refuses to run uncalibrated).
3. Timeout the UART TXE/TC spins (drop output, count it).

---

### P1-9. All fault handlers are bare `while(1)`: an armed bridge freewheels on the last PWM vector for up to ~512 ms, and no crash evidence survives ✅ verified

**Where:** `Core/Src/stm32g4xx_it.c:83-153` (NMI/HardFault/MemManage/BusFault/UsageFault, all
spin), `Core/Src/hrtim.c:175,242` (`FaultEnable = HRTIM_TIMFAULTENABLE_NONE`,
`FaultLevel = NONE`).

A fault while armed kills the fast loop, but **HRTIM autonomously keeps switching the bridge at
the last compare values** into a spinning PM rotor until the IWDG starves SysTick and resets
(≤512 ms; the peripheral reset then floats the outputs). During that window the software trips
are dead and there is no hardware backstop (P1-10). Nothing reads `SCB->CFSR/HFSR/BFAR` or the
stacked PC, so post-reboot the event is indistinguishable from any other IWDG reset
(MemManage/BusFault/UsageFault are never enabled in SHCSR, so three of the four handlers are
dead code that escalates to HardFault anyway).

**Fix (one common handler, ~30 lines):**
1. First statement: kill the bridge with the **lock-free raw register write** —
   `hrtim_outputs_disable()` already exists at `pwm.c:116-118` with exactly the right rationale
   ("the HAL version takes `__HAL_LOCK` and would silently no-op").
2. Snapshot CFSR/HFSR/BFAR/MMFAR + stacked PC/LR + `RCC->CSR` + a boot counter into a `.noinit`
   record (see RE-5).
3. `NVIC_SystemReset()` instead of spinning — saves up to 500 ms of uncontrolled drive and of
   outage. Same skeleton hosts the P0-3 NMI/ECCD logic.

---

### P1-10. There is no hardware overcurrent path at all — protection is software at 32 kHz, ~45 µs worst case, blind to shoot-through ✅ verified vs. .ioc + RM0440

**Where:** `Core/Src/hrtim.c:175,242`; no COMP instance exists in the .ioc at all (DAC1/DAC2
are debug-only); the only OCP is `Inorm² > (15 A)²` in `adc_update_callback`
(`control_ll.c:266-269`).

Numbers (loop confirmed 32 kHz: 96 kHz switching / RepetitionCounter 3):
- Worst-case reaction: ≈ 31.25 µs (tick) + ≤10.4 µs (crest sample staleness — JDR1 is read
  while the current crest's conversion is in flight) + execution ≈ **~45 µs from fault onset to
  gates-off**.
- Winding-level fault at Vbus 8.4 V with the hardcoded motor (L = 6.39 µH line-line,
  R = 0.196 Ω): τ ≈ 33 µs, I_ss ≈ 43 A → **~31 A flowing at disarm**. Within pulse SOA for
  larger GaN parts, outside it for 10–15 A-class parts — **part-dependent, not verifiable from
  the repo**.
- The 15 A trip itself *is* reachable (current full-scale ±16.5 A > threshold — unlike the Vbus
  trip, P0-2).
- Structural blind spots at any latency: a leg **shoot-through** never appears in an in-line
  *phase* shunt (bus→leg→GND path), and a phase-node short upstream of the shunt is likewise
  invisible. The sole shoot-through defense is the dead-time being correct — which it is
  (verified 10.42 ns from the RM0440 DTG formula, matching the GaN intent; the HAL constant
  name remains misleading, per the documented footgun).
- Disarm semantics are otherwise correct: ODISR → outputs take `IdleLevel = INACTIVE` with
  active-high polarity → both gates low, bridge coasts.

**Fix:** wire one of the seven free COMPs (DAC threshold from the existing shunt-amp outputs)
to an HRTIM FLTx input with `FaultLevel = INACTIVE` on all six outputs — cycle-by-cycle
hardware turn-off, zero loop-time cost. If board nets make that impossible for this rev,
*document the accepted exposure* (~45 µs / ~31 A) against the actual GaN part's pulse SOA so
it's a decision rather than an accident.

---

### P1-11. Failed or cancelled encoder dir-cal silently clobbers the live (flash-restored) direction with +1 ✅ verified

**Where:** `Firmware/Src/encoder_calibration.c:52-53` (`encoder_dir_cal_init` does
`encoder_set_direction(1.f)` unconditionally), `:90-113` (failure paths return false with no
restore), `:205-206` (phase-cal applies the coarse lock-in offset mid-procedure, no rollback).

On a wheel whose true direction is −1 (the bench wheel, per the worklog), an in-flight re-cal
attempt that fails tolerance or is cancelled leaves the live direction inverted relative to the
still-good flash blob. If ops then arms torque/velocity without noticing `procedure_result = 0`
(which is itself unreliable — RE-2), commutation is mirrored: torque sign inverts and the
velocity PI becomes **positive feedback** — uncommanded acceleration until the 15 A trip.
Recoverable over CAN (state 20 RESTORE_CALIBRATION, or reboot → auto-restore), but nothing
prompts that. The backemf law (state 21) already follows the correct discipline — fit from
frozen metadata, apply only after gates pass; the older cal laws don't.

**Fix:** snapshot dir/offset in init; restore on any `finish(cancelled=true)`/failure path;
commit live state only on success.

---

## 3. Host interface semantics — the edge-trigger family and friends

This is the section for the "writing the same state/setpoint twice is a no-op" class. The root
cause is a single pattern, and one redesign removes the whole family.

### 3.1 Mechanism ✅ verified

`process_sdo_vals` (`can_comms.c:144-198`) polls the OD once per foreground pass and acts on
**value change** against `static` shadow copies (`last_sdo_requested_state`, init IDLE;
`last_sdo_setpoint_1`, init 0.0). The FSM moves independently (auto-returns to IDLE, forced
error entries), the control laws zero their internal setpoints on entry
(`vel_ctrl_init`: `vel_setpoint = 0; vel_integrator = 0;`), and **nothing ever writes firmware
truth back to 0x4004/0x4005** (`set_sdo_requested_state()` exists at `can_comms.c:88` and is
dead code). So host and firmware drift, and the compare eats writes that the host has every
reason to believe are commands.

### 3.2 The trap enumeration

| # | Operator action | Expected | Actual (verified) | Hazard |
|---|---|---|---|---|
| 1 | Re-request the same procedure after it auto-returned to IDLE (write 9 again after a dir-cal) | re-runs | Eaten — `9 != 9`. `fsm_request_state` never called | Commissioning stalls; host waits forever for state 9 |
| 2 | Re-command VEL_CONTROL after an error forced the FSM out (stateless host replays last command) | re-enters | Eaten — compare baseline still holds 5 | Wheel stays in error/IDLE; host and firmware permanently disagree |
| 3 | Re-write the same setpoint after re-arm | resumes | Eaten — but `vel_ctrl_init` zeroed the internal setpoint | **Wheel sits at 0 rev/s in VEL_CONTROL, errors==0, while the host believes 2000.** A healthy-looking actuator no-op — the worst variant for ADCS |
| 4 | Write setpoint <1–2 ms after the state write (back-to-back SDOs, as the docs' example shows) | both apply | Routed by the **old** state at processing time (`fsm_get_current_state()` at `can_comms.c:166`) → dropped at `default:` → and still poisons the edge trigger, so the corrective rewrite of the same value is eaten too. Worse: routed-to-the-*outgoing* state means a vel setpoint can land as a torque command | Wheel never moves (or moves wrong), no error. Bench-invisible (single-node bus retransmits; 4-node bus ACKs and drops — see P1-7) |
| 5 | Read back 0x4004/0x4005 to check what the firmware is doing | firmware truth | Returns the host's own last write | An eaten write self-confirms |
| 6 | Write 0x4004 = 0 | reject/no-op | Silently invisible (0 aliases the FSM_STATE_NONE sentinel) — though it incidentally "un-poisons" the replay baseline | Accidental escape hatch, undocumented |
| 7 | Write 0x4004 = 22…255 | reject | Enters a zombie state: publishes `current_state=200` on TPDO1, no law, and `state_names[200]` reads kilobytes past the 22-entry array in an ISR printf | UB on a flight-commandable input; recoverable but nasty |
| 8 | Write 0x4004 = 2 or 3 (the error states) | reject | Entered directly; CONTROLLER_ERROR with errors==0 (same zombie as P1-1 mode C) | Confusing wire state |
| 9 | Cancel a procedure, read `procedure_result` | failure readable | Result=0 lives ~1 FSM tick before the IDLE-bounce rewrites it to 1 (see RE-2) | Host records false success for a non-applied cal |
| 10 | Write 0x4007 = 2 (or anything ≠1) to reboot | reboots | Silently ignored (`== 1` exact match) | Undocumented |
| 11 | Write setpoint_2 / reserved_1/2 | something | SDO ACKs, value tracked, nothing consumes it | Silent inert interface |
| 12 | Re-write the same state while it's running (idempotent replay) | no-op | Genuinely a harmless no-op (eaten before the FSM — no cancel/re-init blip) | The one safe case — by accident |

Also in this family: the missing `break` after `set_vel_ramp_setpoint`
(`can_comms.c:177-179`, falls into the empty `default:` — harmless today, documented, still a
landmine), and `-0.0 == 0.0` (writing −0.0 over 0.0 is a no-op) versus NaN (always-differs →
re-applies forever, P1-4).

**Current operational workaround** (already encoded in the `can_console` tool): nudge 0x4004
through a different value (NONE/IDLE) before re-asserting, and write a dummy setpoint before
the real one. It works; it's also exactly the kind of tribal knowledge that gets a flight wheel
stuck when someone scripts the host from the docs instead of the tool.

### 3.3 Every state entry resets CANopen comms, from the TIM17 ISR ✅ verified (documented footgun — consequence map)

`fsm_enter_state` → `set_tpdo_send_rate(100|5000)` → `canopen_app_resetCommunication()`,
synchronously, in interrupt context (`fsm.c:186,190`, `can_comms.c:203-210` — the comment
"will come into effect on next NMT RESET_COMMUNICATION" is stale; it resets immediately).
Traced consequences:

1. **Frames are lost unretrievably on the flight bus:** during the reset window, frames
   addressed to this node are link-layer ACKed by the *other three wheels*, so the host's CAN
   controller never retransmits (and with DAR, P1-7, not even unACKed frames retry). The host's
   immediately-following setpoint SDO is the most likely casualty. Single-wheel bench: the same
   frame NAKs and retries → invisible until integration.
2. `HAL_FDCAN_Stop` purges queued TX — the SDO *response* to the very state-write that caused
   the transition can die in the mailbox → host sees a timeout for a command that executed.
3. NMT re-initializes → a **CANopen boot-up frame on every transition** → boot-up is worthless
   for detecting real reboots (the standard mechanism).
4. Auto `CO_NMT_STARTUP_TO_OPERATIONAL` (verified in `CO_app_STM32.c:43-45`) — good for
   telemetry continuity, but it silently overrides any host-commanded PRE-OPERATIONAL/STOPPED
   state on the next transition.
5. TPDO event timers reset → all four TPDOs burst on the first 1 kHz tick after every
   transition (this is also why faults *are* announced within ~ms when the burst survives;
   under DAR a lost burst frame means up to 5 s of blindness in CONTROLLER_ERROR's 5000 ms
   rate).
6. Host-configured TPDO timers (0x180x:05 are SDO-writable) are clobbered on every transition —
   the host cannot durably configure telemetry rates.
7. Non-idle→non-idle commands transit IDLE → **two** comm resets ~1 ms apart.
8. The reset runs at TIM17 priority and can preempt foreground `CO_process` mid-SDO-transaction,
   re-initializing the SDO server under it. In-place re-init means this degrades to dropped
   transfers rather than memory corruption — but it's unsynchronized by design.

**Fix:** stop resetting comms to change a timer. Either write the event timers through the OD
interface (CANopenNode's PDO write extension applies `eventTime_us` live and zeroes the timer —
`CO_PDO.c` ~1039), deferred to the foreground via a flag rather than doing CANopen work in
TIM17 at all — or simply delete the rate switching and run 100 ms always: 4 TPDOs × 4 wheels at
100 ms ≈ 160 frames/s ≈ **2 % of the 1 Mbit bus**. Keep `resetCommunication` for actual NMT
use only.

### 3.4 The single redesign that kills the class

Replace polling-diff ingestion with **CANopenNode OD write extensions**
(`OD_extension_init` on 0x4004/0x4005/0x4007):

- Every SDO download invokes the handler exactly once, **regardless of value** — the entire
  eaten-write family (rows 1–5 above) disappears, and the `last_sdo_*` statics get deleted.
- Validate in the handler and **SDO-abort** invalid writes (state whitelist; `isfinite` + range
  clamp on setpoints; 0x4007 any-nonzero or documented) — the host gets a synchronous error
  instead of silence.
- Route setpoints by *target* state, or buffer the last setpoint and re-apply it in
  `control_law_init` on state entry — kills the transition-window race (row 4).
- Write FSM auto-transitions back to 0x4004 (the dead `set_sdo_requested_state` is exactly
  this) so readback is truthful.
- Same-state re-requests can then be filtered *in the FSM*, where actual state is known.

Until then, the interim host-side runbook (encode it in can_console and the docs):
always poll 0x4000:1 to confirm state before writing a setpoint; never write any state except
13 while errors ≠ 0; recovery = 13 → poll errors==0 → 1 → poll state → target state → poll →
dummy setpoint (0.0) → real setpoint; read `procedure_result` via SDO while still in IDLE and
*before* the next command; treat an SDO timeout after a 0x4004 write as expected (blind-retry
once).

---

## 4. P2 — protection fidelity, sensing, concurrency

### P2-1. Re-entering VEL_CONTROL on a spinning wheel commands full-authority braking to zero ✅ verified

`vel_ctrl_init` zeroes setpoint + integrator (`closed_loop_controller.c:67-71`); at 60 rev/s
the P-term alone (`vel_err × VEL_GAIN / MOTOR_TORQUE_CONSTANT` ≈ 45 A) saturates the ±5 A
clamp instantly — maximum regen braking commanded by no setpoint, with bus-pumping the OV trip
can't see (P0-2). The obvious corrective write (the same velocity that's still in OD 0x4005) is
a no-op (§3.2 row 3). **Fix (one line):** seed `vel_setpoint = encoder_get_pos_vel().vel` in
init (bumpless re-entry), or force-apply the stored setpoint on entry. (Torque mode re-arms at
zero — benign by the same mechanism.)

### P2-2. MA702 fast read: unbounded RXNE spin in the priority-0 ISR, plus the documented assumed-good data ✅ verified

`__MA702_FAST_ANGLE_READ_FINISH()` (`ma702.h:53-58`):
`while (__HAL_SPI_GET_FLAG(..., SPI_FLAG_RXNE) == false);` — no bound. A wedged SPI (SEU on
the peripheral-enable/clock bits, kernel-clock loss) freezes the prio-0 ISR mid-tick: **the
HRTIM keeps switching the last latched compares into a moving rotor with every software trip
dead** for the full ~512 ms until IWDG. Fix: bound the spin (~200 cycles ≈ 2× the SPI frame
time via DWT->CYCCNT); on timeout raise nCS, set `CTRLR_ERR_ENCODER_FAILURE`, let the existing
same-tick disarm run.

The data path (documented footgun, `control_ll.c:363` `encoder_read_success_dma = true; //
TODO this is a joke`): one EMI-corrupted 16-bit frame (MA702 has no CRC/parity) feeds the PLL
raw. With pll_ki = 4e6, a single ±0.5-turn glitch kicks the velocity estimate by up to
~62 rev/s and skews the electrical angle for the ~0.3–1 ms re-lock — at speed/load that can
cross the 15 A trip → uncommanded torque dropout into CONTROLLER_ERROR. Cheap mitigation the
predictor already enables: reject samples with `|circular_delta|` above a physical per-tick
bound (e.g. 0.05 turn/tick ≈ 1600 rev/s), coast the estimator one tick on prediction, count
rejections into a TPDO-visible diagnostic.

### P2-3. Shunt selection erases ground-fault visibility on the dropped phase; leakage/bus-current protection is dead code; experimental feature is default-ON ✅ verified

With `FOC_USE_SHUNT_SELECTION = true` (now default), the dropped phase's *measured* current is
discarded and rebuilt via KCL (`control_ll.c:236-249`) **before** the `Inorm²` trip — a
winding-to-chassis fault whose current returns outside the phase shunts is erased rather than
detected on that phase (the symmetric-CM fallback at least spreads it). The direct leakage
signature `A+B+C` is computed and thrown away. Meanwhile `MAX_CURRENT_LEAKAGE_AMOUNT`,
`IBUS_OVERCURRENT_THRESHOLD_A`, `IBUS_OVERREGEN_THRESHOLD_A` have zero references;
`CTRLR_ERR_PHASE_CURRENT_LEAKAGE`/`CTRLR_ERR_MOTOR_PHASE_LEAKAGE` and five other error bits
have no setters (ground decode tables should know they can never appear); and the trip that
does fire is misnamed `CTRLR_ERR_IBUS_OVERCURRENT` (it's motor phase current; no actual
bus-current computation exists — `ibus` is initialized and never written).

Also: `config.h:177`'s own comment still says "EXPERIMENTAL … Validate on a scope before
trusting it" while shipping it enabled. The bench-measured `SHUNT_SEL_DELAY_TICKS = 1` is
explicitly invalid if loop rate / CMP4 / oversampling change.

**Fix:** trip (or at least count) on `|A_i+B_i+C_i| > MAX_CURRENT_LEAKAGE_AMOUNT` using the raw
three readings *before* reconstruction — the data is in hand; implement-or-delete the IBUS
constants; rename the error bit; make the default-ON decision deliberately at flight freeze.

### P2-4. Inverter offset cal accepts out-of-range offsets and marks the inverter calibrated ✅ verified

`inverter.c:162-167`: offsets outside ±100 counts hit `assert(false); //return;` — the assert
prints and execution **falls through** to install the bogus offsets and set
`inverter_is_calibrated = true`. A degraded INA296 at boot flies with a systematic per-phase
current error (~0.8 A per 100 counts) baked into both the FOC loop and the trip math; the
runtime all-zero check doesn't catch it. The commented-out `return` is the lost intent —
restore it (leaving `inverter_is_calibrated = false` lets the existing
`CTRLR_ERR_INVERTER_CALIBRATION_INVALID` block arming with a CAN-visible cause).

### P2-5. `assert` is print-and-continue; disarm's HAL teardown is lockable ✅ verified

`user_assert_fail` (`util.c:8-10`) prints one line (into the ISR ring, which nobody reads in
flight) and returns. Consequences: `controller_arm()`'s
`assert(HAL_HRTIM_TIMxRegisterCallback(...) == HAL_OK)` failing would continue to
`inverter_start(); controller_armed = true;` — bridge enabled with **no fast-loop callback**,
i.e. no control law and no trips, and no error bit (all error sources live in the dead fast
loop). `hrtim_pwm_stop`'s six lockable HAL calls under a (microseconds-wide) `__HAL_LOCK`
collision would silently no-op the disarm. Today these are near-infallible register writes —
latent discipline, not a live bug — but the fix is cheap and doubles as P1-9's: make
`controller_disarm()` write `ODISR` **first**, unconditionally, via the existing lock-free
`hrtim_outputs_disable()`; make `user_assert_fail` latch a TPDO-visible flag (and in thread
context, flush + reset).

### P2-6. FDCAN interrupts actually run at priority 0 (tied with the fast loop), and CO_LOCK_OD masks ALL interrupts ✅ verified

`HAL_FDCAN_MspInit` (`fdcan.c:104-107`) sets the FDCAN IRQs to priority 0 — and it runs inside
`canopen_init` (app_main.c:245), **after** the intended prio-3 assignment at `app_main.c:62-63`
(which is therefore dead). With `StdFiltersNbr = 0` (no hardware filters), every frame from all
four wheels enters FIFO0 and is parsed + software-ID-matched at priority 0, ahead of (or
instead of) the motor tick. Independently, `CO_LOCK_OD` is `__disable_irq()` and is held across
the whole 4-TPDO serialization pass in the 1 kHz CAN tick (~10–25 µs of PRIMASK on
synchronized expiries). Against a 31.25 µs fast-loop period already at 72 %, both mechanisms
produce delayed/overrun control ticks — strictly worse on a busy flight bus than anything
benchable today, and a credible contributor to the open "lumpy current at high speed" thread.

**Interlock — don't fix blindly:** FDCAN at prio 0 is currently what keeps CAN RX alive during
the 20–30 ms TIM17-ISR flash erases (P2-7) and the multi-ms encoder-cal UART dumps. Re-rank
FDCAN to 3 (in the .ioc so MspInit writes it) **together with** moving the flash ops to the
foreground, and re-implement CO_LOCK_* with BASEPRI masking priorities ≥1 (TIM17 touches
CANopen state via `set_tpdo_send_rate`, so it must be masked; HRTIM never touches the OD and
can stay live). Then re-measure `isr_cycles_max` armed against a saturated bus.

### P2-7. Cal-store flash ops run in the TIM17 ISR: ~25–30 ms with the CAN tick masked, and the HAL's BSY timeout is inert there ✅ verified

`fsm_exit_state` runs `cal_store_save/erase/restore` in TIM17 context (`fsm.c:168-176`).
SysTick (prio 15) can't preempt TIM17 (prio 1), so `HAL_GetTick()` freezes during the op: the
HAL's nominal 1 s flash timeout never expires — a hardware-stuck BSY would spin in-ISR until
the IWDG resets the board (self-recovery, but the "timeout" is illusory). Normal-path cost
(~22 ms erase + ~6 ms program) blocks the 1 kHz CAN tick and IWDG feeding for the duration —
fine against the 512 ms budget, and RWW means no CPU stall (app is linker-confined to bank 1,
verified) — but TPDO/heartbeat timing slips, and the 3-deep RX FIFO survives only thanks to
the accidental prio-0 FDCAN (P2-6). The on-board fit (~10–20 ms of libm in the same ISR) is the
same class. **Fix:** move save/erase/restore to the foreground via an RPC flag from the FSM —
they have no timing requirement; states 18/19/20 already guarantee disarmed.

### P2-8. Option-byte enforcement: unbounded retry loop; BOR/IWDG_SW/SRAM_PE ride factory defaults ✅ verified

`app_main.c:113-122`: mismatch → write → reset, with no attempt counter. The three compared
bits (nBOOT0/nBOOT1/nSWBOOT0) are properly masked — no reserved-bit/silicon-lot trap, and
bit-for-bit consistent with the bootloader's host-commanded writes (no boot-time fight,
verified) — but a chip where the write won't stick = an infinite ~1 s reboot loop that consumes
OB flash endurance (~10 k) in hours and never reaches CAN. Untouched defaults: **BOR_LEV = 000**
(~1.7 V — thin for in-flight flash writes; a slow EPS brownout during a cal save is exactly the
P0-3 trigger), **IWDG_SW = 1** (software start — the P1-8/P0-4 pre-IWDG hang windows),
**SRAM_PE = 1** (parity off; on the G474 parity covers CCM + first 32 KB of SRAM1 — a policy
decision worth making consciously given the fast loop runs from CCM, RE-7). The enforcement
machinery already exists; add a noinit retry counter (fail-open after ~3 attempts — safe
provided the flight harness straps PB8 low), and pin BOR_LEV ≈ 2.8 V and IWDG_SW=0 in the same
pass. Also: everything after `HAL_FLASH_OB_Launch()` in `write_option_bytes`
(`bootloader.c:55-60`, including a stray `FLASH->CR |= FLASH_CR_OPTSTRT` after re-locking) is
unreachable on success and misleading — delete.

### P2-9. VDDA correction divides by a raw ADC count ✅ verified (math)

`aux_adc_conv_vdd_voltage_mV` computes `((VREFINT_CAL << 20) / vdd_raw) * 3000 >> 16`
(`aux_adc.c:64-65`), consumed every fast tick. `vdd_raw == 0` (DMA not yet run, SEU, aux-ADC
fault) → integer div-by-zero → 0 on M4 (DIV_0_TRP unset) → vvcc = 0 → all currents and Vbus
read 0 → spurious UV/measurement-missing disarm (safe direction, but an uncommanded torque
dropout). A corrupted low-but-nonzero sample (≲1214) overflows the u32 → garbage scale for both
the loop and the trip math for those ticks. **Fix:** clamp `vdd_raw` to a sane band
(~1300–1750 counts), fall back to nominal 3.3 V outside it.

> **2026-10-26: this fix was itself wrong, and is now corrected.** `vdd_raw` is the
> 16x-oversampled DMA value (~26400), so the 12-bit band rejected every sample, and the ADC
> reference is VREF+ = 3.0 V (an external reference; VDDA is 3.3 V). In v1.0.0-v1.0.2 every tick
> used 3.3 V: Vbus and the phase currents read 10% high (measured on the bench: 8.37 V reported
> for 7.62 V real), and the OV trip fired about 0.8 V early. Now `vdd_raw < 16384` is treated as
> garbage, the computed VREF+ must fall in 2.5-3.4 V, and the fallback is 3.0 V
> (`ADC_VREF_NOMINAL_mV`).

### P2-10. Error-mask pollution obscures root cause for remote diagnosis ✅ verified

A mid-tick disarm (e.g. genuine OC in `adc_update_callback`) skips the consumers for the rest
of that tick; `encoder_update_callback` then unconditionally adds
`CTRLR_ERR_ENCODER_READING_USAGE_MISSING` (`control_ll.c:311` runs with `is_encoder_updated`
still true), and next-tick missing-flags pile on before the FSM latches. Ground sees 2–4 bits
and must guess the causal one — and on a no-physical-access mission, error-mask forensics is
the only debugging channel. **Fix:** gate the bookkeeping checks on `controller_armed`, and/or
capture a `first_error` u32 alongside the accumulated mask and put it in TPDO1.

### P2-11. Boot ordering: FSM tick starts 40 lines before CANopen init ✅ verified

`HAL_TIM_Base_Start_IT(&CONTROLLER_FSM_TIM_HANDLE)` at `app_main.c:202`; `canopen_init` at
`:245`. Any FSM transition in that window (none today — errors can't be raised there yet, the
fast loop is unregistered) calls `canopen_app_resetCommunication()` with `CO == NULL`:
`CO->CANmodule` reads address 0 (the bootloader's vector table — readable), and the
`->CANnormal = false` store lands at an address derived from the bootloader's initial SP —
silent corruption, not a clean fault. One future boot-path edit (e.g. an error set during boot
cal) makes it live. **Fix:** start the FSM timer after `canopen_init`, or guard
`set_tpdo_send_rate`/`canopen_app_resetCommunication` with `if (CO == NULL) return;`.

### P2-12 … P2-16. Calibration pipeline (state 21 / cal store) — five findings

- **P2-12. State-21 partial success is invisible over CAN.** `backemf_fit_and_apply()` returns
  false only for sample-count or Δ/k bound failures; **LUT rejection** (empty-frac/amplitude
  gates) and **h6-FF rejection/skip** still apply the offset and return `procedure_result = 1`
  (`backemf_cal.c:432-445,503-513,520`). The implementation doc claims result=0 in these cases —
  the code only keeps-previous. A subsequent SAVE persists the mixed state with no indication.
  Fix: fold sub-step status into the result or surface an applied-bitmask
  (offset/LUT/FF) via a spare OD field so the runbook's acceptance check works over CAN.
- **P2-13. h6-FF coefficients aren't invalidated when the offset they're phase-referenced to
  changes.** Pass-5 rejection keeps the previous coefficients live under a *new* offset/LUT
  (and states 10/15 change the offset with blob-era coefficients live); a Δoffset of x rad-elec
  misphases the 6θ injection by 6x — at the 100 µV·s bound that's bounded ripple degradation,
  not a safety event, but a later SAVE persists the inconsistent pair. Fix: zero the live
  coefficients on pass-5 rejection and whenever `encoder_set_phase_offset` is driven outside
  state 21 — zeros are an exact no-op and strictly safer.
- **P2-14. `-Ofast` (`-ffinite-math-only`) can legally fold the negated-compare NaN guards** in
  `blob_valid` (`cal_store.c:58-72`, whose comments promise NaN rejection) and the fit bounds —
  and, for that matter, every float NaN guard in the codebase, including `svm_output_valid`.
  Realistic entry is a live NaN at save time being persisted CRC-valid and restored every boot.
  Not disassembled to confirm GCC does it *here* — treat as "verify or harden": check the
  objdump, or test NaN by integer bit-pattern (exponent mask) in `blob_valid` and the
  load-bearing guards. Cheap, one helper.
- **P2-15. The on-board bidir fit lacks the direction self-consistency gate the offline tool
  grew** (circular-resultant R ≈ 1 vs ≈ 0 — the exact failure the bench hit with wrong
  `encoder_dir`). Mostly theoretical on-board (closed-loop spin-up with a garbage angle map
  can't reach speed → timeout → no apply; |Δ| ≤ 46° bounds damage), but the ssin/scos sums are
  already accumulated — `R > ~0.5` is ~5 lines of defense-in-depth.
- **P2-16. SAVE_CALIBRATION's "refuse if never calibrated" guard doesn't fire.**
  `cal_store.c:125-130` checks only `encoder_dir != ±1` — the power-on default (+1) passes. The
  comment describes the right heuristic (`offset==0 && dir==+1` → nothing ran);
  `docs/can-interface.md` claims the refusal exists; the code doesn't implement it. A
  mis-ordered runbook on a fresh wheel persists factory-default cal as success, restored
  "successfully" every boot thereafter. Fix: implement the stated heuristic or an explicit
  `dir_cal_ran` flag.

Also noted (RE-class, in the same subsystem): DRAIN-timeout re-engage at up to coast speed
(`backemf_cal.c:224-242` "proceed anyway") starts leg 1 with full vel error → 5 A regen braking
pulse — check real coast-to-3.2-rev/s time against the 30 s budget on flight bearings in
vacuum, or seed the ramp from measured velocity; restore-time semantic bounds are ~10× looser
than the build gates (`cal_store.c:63-72` literals vs. `BACKEMF_LUT_MAX_ABS`/
`BACKEMF_FF_MAX_COEFF` — reuse the config constants); `staging.psi_m` is never populated; the
`nh >= 200` FF gate is a magic literal; no blank-check before erase and no rate limit on
host-commanded save/erase loops (page spec ~10 k cycles); `encoder_single_point_cal_init` has a
dead `inv_park_transform` store; a leftover `__asm__("nop")` scaffold sits in dir-cal.

### P2-17. MA732 sensor-NVM wear interaction ✅ verified

`ma732_ensure_filter_window()` (`ma702.c:71-84`) trusts a single register read; a garbage read
(dead sensor, SPI noise) mismatches → unconditional NVM write *that boot* (~1 k-cycle sensor
endurance, 20 ms each). Under any persistent reboot loop (IWDG, option-byte, P0-3) the sensor
NVM is consumed in under an hour. The call-site return is also discarded — a failed write
(wheel flying with FW=119 instead of 51, shifting the per-rev cal pattern several degrees at
speed) is invisible except as a boot UART line. Fix: require two consecutive identical reads
before writing; skip if readback is 0x00/0xFF; latch the result into a boot-status OD field.

### P2-18. Telemetry force-zeroed outside the four control states ✅ verified

`update_tpdo_od_vals` (`can_comms.c:124-132`) publishes vel/iq/Vbus/ibus only in
POS/VEL/TORQUE/VEL_RAMP. During BACKEMF_CAL the wheel spins to **−50 rev/s for up to ~80 s
while TPDO2 reads 0.0**; cancel mid-spin and a physically spinning wheel reports IDLE / vel 0 /
errors 0 while it coasts for many seconds. After any trip at speed, the wheel coasts with
near-full momentum while telemetry reports zero — the ADCS sees a momentum discontinuity it
can't verify, and operators can't time a safe re-arm. (Internally the estimate also goes stale:
the estimator only updates in the armed fast loop.) Fix: publish `encoder_get_pos_vel().vel`
unconditionally (flag staleness when disarmed), gate only `motor_current` if anything.

### P2-19. Bootloader protocol robustness (beyond P0-4) — from the bootloader pass

- **BOOT0/PB8 is sampled exactly once, no pull, no debounce** (`bootloader/Src/main.c:136`,
  NOPULL) — contrast the app's 1000-sample vote. A glitch at the sample instant enters DFU;
  the de-facto recovery is *accidental*: the detection phase never refreshes the IWDG
  (reconfigured to ~32.8 s in DFU), so it resets and re-samples — worst case ~33 s outage.
  **But** line noise that completes the 0x7F autobaud handshake lands in
  `OPENBL_USART_ReadByte`, which refreshes the IWDG in its wait loop **forever** → stuck in DFU
  until power cycle. Fix: majority-sample PB8; add an explicit inactivity timeout (no completed
  command in N s **and** app vector valid → jump), which also makes the IWDG-expiry behavior
  intentional.
- **Unmasked option-byte writes over DFU**: `OPENBL_OB_Write` writes host bytes straight to
  `FLASH->OPTR` + OPTSTRT. A bad OB write can set nBOOT0=0/nSWBOOT0=0 → the chip thereafter
  boots ROM system memory unconditionally → neither image ever runs again, and the ROM loader
  listens on AN2606 pins (PA9/PA10), not this board's PE0/PE1 — brick without SWD. RDP 1→0 via
  the protocol triggers full mass erase including the bootloader. Fix: force-mask the
  boot-critical bits before OPTSTRT; refuse RDP ≠ 0xAA.
- **Erase/program failures are ACKed** (stock ST behavior: erase status initialized to ACK and
  never set on HAL error; `HAL_FLASH_Program` return discarded). The C3 believes a failed
  reflash succeeded and resets into P0-4. Minimum zero-code fix: make read-back verify a
  mandatory step of the C3 reflash procedure; better, propagate errors to NACK.
- **`Common_EnableIrq()` before the jump** — interrupts are enabled, then the app's
  Reset_Handler runs .data/.bss init with the bootloader's stale VTOR until SystemInit. Safe
  today only because the bootloader never enables an NVIC source and SysTick is now properly
  killed (the "Stop crashing" fix is root-cause-correct, verified: CTRL/LOAD/VAL zeroed +
  PENDSTCLR, and the app's IWDG kick is deref-free by design). Replace with `__disable_irq()` +
  NVIC ICER/ICPR sweep; the app already does its own `__enable_irq()`.
- Operational caveats for the reflash runbook: protocol mass-erase NACKs (`MassErase = NULL` —
  keep, and document); the cal page (bank-2 page 127) **is** host-erasable — a reflash that
  touches it costs the encoder cal (plan re-cal or avoid the page); the reset-cause mailbox at
  0x20017FF8 is inside the DFU RAM-write window (spoofable — cosmetic); `USE_DEBUG_HANDLE` in
  `interfaces_conf.h` must ship commented (it is); a one-command extended-erase of many pages
  approaches the 32.8 s DFU IWDG with no mid-command refresh — add `OPENBL_IWDG_Refresh()` to
  the page-erase loop.

### P2-20. HSE clock bring-up (app side) ✅ verified

Same shape as the bootloader half of P0-4: `SystemClock_Config` (HSE-only PLL, no CSS, no
fallback) runs before `MX_IWDG_Init` in the generated `main()`, and `Error_Handler` is
`__disable_irq(); while(1)`. With IWDG inherited running from the bootloader, an app-side HSE
failure becomes a 512 ms reset loop into the bootloader's *watchdog-less* hang. Fix priority
follows the bootloader item: IWDG-before-clock (or OB hardware-start), Error_Handler → reset,
and optionally HSI16 fallback (CAN at 1 Mbit with SJW=1 is marginal on HSI — bump SJW, RE-9).

---

## 5. RE — observability, interface polish, latent landmines

### RE-1. No EMCY on controller faults; EMCY COB-ID baked to wheel 1's for all wheels ✅ verified

No `CO_errorReport` call exists anywhere in `Firmware/`; faults ride only on TPDO1 (burst on
transition — droppable under DAR — then 5000 ms in CONTROLLER_ERROR). Separately the generated
OD bakes `0x1014 = 0xBC` (the EDS's `$NODEID+0x80` was flattened at generation): CANopenNode
only node-offsets the stored value when it equals exactly 0x80, so wheels 2–4 would EMCY as
wheel 1 and simultaneous EMCYs collide on one arbitration ID. Fix: set 0x1014 default to 0x80
in the EDS and regenerate; call `CO_errorReport(CO->em, CO_EM_GENERIC_ERROR,
CO_EMC_DEVICE_SPECIFIC, controller_errors)` from the fan-out on error-entry transitions (and
`CO_errorReset` on clear); keep TPDO1 at 100 ms in CONTROLLER_ERROR.

### RE-2. `procedure_result` is clobbered to 1 by the next IDLE/error-state exit ✅ verified

`fsm_exit_state` sets `fsm_last_state_exit_ok = true` for IDLE/NONE/error exits (`fsm.c:144`).
A cancelled procedure's failure result survives exactly one FSM tick (the cancel path transits
IDLE); a completed-failed result is stable in IDLE but is destroyed the instant the host
requests the next state. One result register for 21 states, no sequence number, no state tag.
Fix: a latched `{last_completed_state, result, sequence}` OD object written only on law/action
exits; never touched by IDLE transitions.

### RE-3. No firmware-truth echo of setpoint or consumed request ✅ verified

0x4004/0x4005 read back the host's own last write; the internal
`vel_setpoint`/`torque_setpoint`/`ramped_vel_setpoint`/`idq_setpoint` are never exported. A
wheel "in VEL_CONTROL not tracking" is indistinguishable among eaten setpoint, stall, and load.
Fix: read-only OD object (or repurpose 0x4006) publishing the active internal setpoint and the
requested-state-as-consumed; map into TPDO2. (Pairs with the §3.4 redesign.)

### RE-4. Reset cause, crash records, and the console black box don't survive to CAN ✅ verified

The plumbing is half-built and good: the bootloader parks `RCC->CSR` in the RAM2 mailbox
(0x20017FF8, magic-tagged), `app_main.c:84-108` decodes IWDG/SOFT/BOR/PIN — then prints it to a
UART nobody can hear and consumes it. The 2 KB console ring is `.bss` — zeroed by startup on
every reset — so there is no post-IWDG breadcrumb at all (`tools/console_tail.sh` is SWD-only
and pre-reset). After launch, every reboot (including all the P0/P1 scenarios above) is
invisible beyond a heartbeat gap and a state regression. Fix: move the ring (or its last
~256 B) plus a crash record {RCC_CSR, CFSR, HFSR, BFAR, stacked PC, boot counter} into a
`.noinit` section in RAM2; populate from the (new) fault handlers and app_main; expose via SDO —
short-term in the spare 0x4006 floats, properly as a 0x4008 diagnostic record. Cheap,
transformative for anomaly triage.

### RE-5. Identity: 0x1018 all zeros; duplicate-ID failure mode undiagnosable ✅ verified

All four wheels are LSS-identical and serial-less. Combined with the strap problem (P0-1) or a
single strap joint failing at vibration (wheel N collapses to wheel N−1's ID): two nodes share
SDO COB-IDs — **every command addressed to that ID actuates both wheels** — and their
TPDO/heartbeat frames collide as bit errors. The host's symptom is "one wheel missing, another
glitchy," with no SDO-readable serial to tell which physical unit answers. Fix: STM32 UID →
0x1018 serial; strap sanity (refuse CAN start on impossible strap reads after debounce — or a
parity strap); host-side heartbeat census before actuation; publish wheel number + raw strap
averages in an OD object.

### RE-6. Boot-up frames, NMT overrides, heartbeat consumer phantom ✅ verified

Boot-up is emitted on every FSM transition (§3.3) — reboot detection must use
heartbeat-gap + state regression instead (document this in `can-interface.md`). Host-commanded
NMT pre-operational is silently reverted to operational on the next transition. The 0x1016
heartbeat-consumer entry is half-configured (node 0, 1000 ms): inactive, but it registers an RX
slot matching COB-ID 0x000 that only loses to the NMT buffer by array-order luck — zero it in
the EDS, and make the "wheel does not monitor host liveness" policy explicit (ties into P1-6's
comms-loss-while-armed gap).

### RE-7. ~8.7 KB of the control loop runs from CCM SRAM with no parity and no scrub

Optimized builds place the entire fast-loop chain (FOC, SVM, estimator, **the software trips
themselves**) at 0x10000000; flash ECC can't see it after the boot copy, and CCM parity is not
enabled (SRAM_PE option, P2-8). An SEU in CCM-resident code persists until reboot — arbitrary
control behavior with the protections corrupted along with it. The pristine image sits at the
flash LMA, so the mitigation is nearly free: a foreground CRC32 of `_sccmram.._eccmram` against
the flash copy every few seconds; on mismatch, `controller_disarm()` + crash-record + reset
(boot re-copies). Sub-ms per pass, amortizable.

### RE-8. Boot-to-CAN is ~1.5–2 s of serially-blocking init; everything before `canopen_init` is a CAN-invisible failure domain ✅ verified (inventory)

Order: option-byte check (reboot loop on mismatch) → synchronous boot banner @921600 →
DAC/CORDIC self-test → `HAL_Delay(100)` → 4× `ma702_preheat` → `ma732_ensure_filter_window`
(+20 ms NVM on mismatch) → `inverter_setup` → `encoder_init` → `cal_store_restore` (P0-3
window) → `aux_adc_start` → `inverter_amp_offset_correction` (20000 ticks ≈ 625 ms, the P1-8
unbounded spin) → FSM start → strap vote → `canopen_init`. Every wait in the chain except the
offset-cal spin is HAL-timeout-bounded (verified). Bound the one spin, and consider bringing
CAN up earlier so the wheel is observable during the slow steps.

### RE-9. Small hardware-config notes ✅ verified vs. RM

- TIM4/TIM17 tick at **993 Hz**, TIM7 at 992 Hz (prescaler written as 144, divides by 145) —
  every "per-ms" constant is ~0.7 % long. Cosmetic; fix to 143 in the .ioc if exactness ever
  matters.
- FDCAN `NominalSyncJumpWidth = 1` of 16 tq — works crystal-to-crystal; CiA practice is
  SJW = Seg2 = 2 for free margin (matters if any HSI-clocked node or CSS-fallback ever joins).
- Bit timing otherwise verified: PCLK1 144 MHz, NBRP 9 → tq 62.5 ns, 16 tq = exactly
  1.000 Mbit/s, sample point 87.5 %, TX FIFO mode.
- Dead-time verified **10.42 ns** rising and falling (DTPRSC=0b000 → t_DTG = t_HRTIM/8 =
  868 ps @ 144 MHz; DTR=DTF=0xC) — matches the GaN intent; the HAL name/comment inverts
  reality (documented footgun; don't "fix" the constant or it becomes ~667 ns).
- ADC chain verified: injected groups on HRTIM ADC trigger 2 = Timer A CMP4, crest-gated
  (ADROM), ADC clock 36 MHz (legal), 2.5-cycle shunt sampling (69 ns — fine from the INA296
  buffered output), 4× oversample shift-2 (12-bit scale preserved), full sequence 8.3 µs <
  10.4 µs trigger period, current full-scale ±16.5 A, CMP4 runtime writes land in the preloaded
  register (glitchless at REP).

### RE-10. Doc/comment drift worth a sweep

`CLAUDE.md` still says the fast loop is 16 kHz (`PWM_CYCLES_PER_ISR=6`) — it's 3 → 32 kHz.
`inverter.c:21-45` header describes the previous board (INA240, 325 kHz, Timer-B CMP3, old
divider) and the struct is still named `INA240_offset_correction`. `periphmap.h` claims
`ENCODER_TIM_HANDLE htim1` and `SYSTICK_HANDLE TIM15` — neither exists in the .ioc.
`docs/control-laws.md` says VEL_RAMP_RATE is 200 rev/s² — config is 20. `set_tpdo_send_rate`'s
comment says the rate applies "on next NMT reset" — it resets immediately.
`docs/bootloader-integration.md` §4.3 describes the removed `hiwdg.Instance` guard. §7's
bootstrap examples reference `cmake-build-debug` while `flash.sh` defaults to relwithdebinfo.
`foc_capture.h` says "512 × 92 B = 46 KB" — it's 256. `docs/control-laws.md` describes state 6
as "disarms almost immediately" — it HardFaults (P1-2). `state_names[]` indexing is unguarded
(§3.2 row 7). The dead `ma702_fast_read_finish()` inline (`ma702.h:39-45`) masks `& 0x3FFF`
without the `>>2` the live macro uses — a scrambled-angle landmine if ever swapped in; delete
it. `Firmware/Src/bootloader.c:82-84` carries a dead duplicate `System_DeInit`.

---

## 6. What checked out clean

Listed so the review is calibrated — these were hunted explicitly and verified handled:

**Fast loop / FOC core.** Control-law installation is tear-free *by construction*: laws are
swapped only while disarmed with the HRTIM repetition callback unregistered and counters
stopped, init runs in-loop via `control_law_needs_init`, and error paths disarm in the same
tick before that tick's compare write (`control_ll.c:139-143` ordering verified). NaN cannot
pass `svm_output_valid` (must-prove-valid polarity), float→compare conversion is deterministic
on M4F, and final compares clamp to [192, 23808] honoring the RM's up-down forbidden zone.
Overmodulation clamps with integrator bleed (no spurious disarm at the voltage limit — correct
for flight). Loop counters: u32 wrap-safe subtraction in the hot paths; the few absolute
`>=`-form cal-state end-tick compares (37 h wrap exposure on entry) fail safe into a clean
`exit_ok=false`. `foc_capture` is SWD-only, bounds-checked, one load+branch when idle. Shunt-sel
history indexing matches the bench-measured K=2/DELAY=1. h5/h7→6f_e feedforward demod and
runtime application use the same θ convention by construction (verified against the runtime
lerp and the bench A/B residual numbers); coefficient bounds enforced at apply and at restore;
absent blob → exact no-op.

**FSM / laws.** Every completing law is loop-count-bounded (R/L cal, dir/phase cal, validation,
single-point, backemf spin/coast timeouts). Integrators are zeroed on init and clamped in run;
re-arm is torque-bumpless (iq=0). `fsm_exit_state` disarms before calling finish; no law leaves
PWM energized after cancel (backemf's floated bridge self-heals on next arm via
`WaveformOutputStart`). Setpoint stores are single-f32 atomic on M4; no torn-pair consumer
exists.

**CAN / OD.** NMT auto-returns to OPERATIONAL after comm reset (`CO_NMT_STARTUP_TO_OPERATIONAL`
verified) — telemetry resumes without master action. Heartbeat producer configured at 1000 ms.
TPDO/SDO/HB COB-IDs correctly node-derived (stored bases match the `&0xFF80` pattern). OD
telemetry fields are ≤32-bit aligned stores — atomic; only cross-field skew exists (one frame,
self-correcting; host rule: errors≠0 is authoritative). `console_pump` is budget-bounded — a
flooding ISR producer can't wedge the foreground via the ring. Foreground ordering
(`update_tpdo_od_vals` → `canopen_process` → `process_sdo_vals`) acts on SDO writes in the same
pass. Boot edge-trigger baselines are consistent with `init_sdo` (no spurious first-pass
actions). The 0x4007 reboot path disarms and flushes before reset.

**Cal store / encoder cal.** Blob integrity is solid modulo P0-3: magic+version+CRC32 with CRC
programmed last (front-to-back order → torn program = CRC-invalid, cleanly rejected); erased
page rejected; v1 blobs version-rejected. Bank/page math correct for 0x0807F800 (HAL
bank-relative page + BKER); DBANK runtime tripwire; unlock/lock paired on all paths;
erase-before-write and readback-verify always; identical-save skipped. States 18/19/20 install
no law and never arm; any armed state is forced through IDLE + disarm before they run. The
runtime LUT is a RAM copy (never read from flash in the fast path); lerp indexing is wrap-safe;
the full sign round-trip (residual → fit → LUT entry → runtime correction) was verified
analytically and matches the bench A/B (0.77°→0.10°). LUT build gates (coverage, amplitude,
≤4/rev harmonic limit killing the 36/rev trapezoid alias) verified; gate failure verifiably
keeps the previous live LUT. Bidir fit gates (min-per-dir before the divide, Δ/k bounds)
enforced before any apply; one-sided fits are never applied (state 17 is log-only). Coast-law
safety is law-independent (OV/UV/OC run in `adc_update_callback` regardless); bridge float is
true hi-Z via direct ODISR; spin-up with a garbage bootstrap offset can't reach speed → timeout
→ nothing applied. Capture logging is bounds-checked, saturating, u32-wrap-safe, no f32
tick-accumulator freeze. R-cal divides by the constant 2 A setpoint (no div-by-zero; open phase
→ voltage ramp → error); L-cal's ÷0 → inf is rejected by the min/max bounds; measured R/L
results are print-only (documented footgun — FOC uses config hardcodes).

**Boot / infra.** IWDG: 512 ms (LSI/4, reload 4095), started by the bootloader and re-armed in
`main()`, fed via direct `IWDG->KR` (deref-free by design — the "Stop crashing" fix is
root-cause-correct, verified in source and rationale); it genuinely covers every ISR-starvation
case including spun fault handlers; no DBG-freeze bits are set anywhere. Option-byte compare
masks exactly the three intended bits — no reserved-bit/silicon-rev trap; app and bootloader OB
expectations match bit-for-bit; no boot-time fight. Console ring: reserve+copy+commit fully
under PRIMASK (no producer interleave), free-running u32 indices with power-of-two masking
(wrap-correct), drop counting race-safe, 255-char truncation verified. `console_flush` present
at the option-byte reset and 0x4007 paths (missing only at NMT reset-node, P1-5 note). Stack:
~40 KB real headroom vs ~3 KB worst-case nesting (foreground printf 256 B + TIM4 + TIM17 +
HRTIM + FPU frames); largest ISR-path locals enumerated (256 B printf buffer, 128 B in
encoder-cal). Heap: single boot-time `CO_new` (NULL-checked); no runtime malloc outside the
P1-5 path. `HAL_GetTick` usage audited — only wrap-safe elapsed-form remains. CCM copy loop and
`SystemInit`'s VTOR=0x08008000 store verified in the built ELF (the documented regen footgun is
currently correctly applied). SWO/debug leftovers benign when undebugged (ITM stores to
disabled ports are ignored; `swo_setup` never called at boot; DWT CYCCNT feeds the budget
counters).

**Hardware config.** Clock tree verified legal (HSE 16 MHz → PLL ×18/2 → 144 MHz, boost mode,
4 WS; HRTIM ×32 DLL → the "4.608 GHz tick" claim is exact). HRTIM preload + update-on-REP means
duty and CMP4 writes transfer atomically at the ISR boundary — no torn-duty window. Output
polarity/complementary config consistent with active-high gate drivers (assumption noted:
non-inverting drivers — unverifiable without the schematic). NVIC re-ranking in `app_main` is
correct for HRTIM(0)/TIM17(1)/TIM4(2)/TIM7+DMA(3) — only the FDCAN pair is overwritten later
(P2-6). G474 RM facts pinned: flash ECC single-bit auto-corrects (ECCC), double-bit → NMI
(ECCD); parity exists for CCM + first 32 KB SRAM1 behind SRAM_PE (disabled as flown).

**Bootloader.** Jump hygiene verified: SysTick fully dead (CTRL/LOAD/VAL + PENDSTCLR), both
USARTs deinit'd, RCC returned to reset state, MSP loaded from app vector, no DMA running, no
NVIC sources ever enabled (grep-verified), faults pre-VTOR dispatch into IWDG-backed handlers.
IWDG fed during DFU RX waits; mass-erase blocked at the descriptor; AN3155-compatible command
set (stm32flash/CubeProgrammer work); per-byte XOR checking; bootloader fits its 32 K at ~48 %
with a sane stack; soft-float bootloader vs hard-float app is safe (no cross-image calls; app
enables CPACR in its own SystemInit before any FP).

---

## 7. Recommended fix order

Ordered by (consequence × likelihood) / effort. IDs refer to sections above.

| # | What | Findings | Effort |
|---|------|----------|--------|
| 1 | Strap read → CAN_ADDR0/1 (or input-mode vote) + publish strap result. **Blocks any multi-wheel integration.** Needs schematic confirmation | P0-1, RE-5 | hours + HW check |
| 2 | Voltage divider decision + OV/UV thresholds + boot Vbus plausibility gate | P0-2 | config + ~10 lines |
| 3 | NMI/ECCD handler + common fault handler (ODISR-first, noinit crash record, reset-not-spin) | P0-3, P1-9, RE-4 | ~50 lines |
| 4 | Bootloader: app-validity check before jump; flash floor at 0x08008000; IWDG before clock config; OB-write masking | P0-4, P2-19, P2-20 | ~40 lines |
| 5 | FSM: consume request in branch 4; CLEAR_ERRORS exemption (or clear-on-entry); clear `law_done` on entry; NULL-guard finish; bounds-check requested state | P1-1, P1-2, §3.2 rows 6–8 | ~15 lines |
| 6 | `is_currents_updated = false` in `controller_arm()` | P1-3 | 1 line |
| 7 | `isfinite` + range gate in `process_sdo_vals`; vel clamp; `CTRLR_ERR_OVERSPEED`; overtemp trip | P1-4 | ~30 lines |
| 8 | FDCAN: AutoRetransmission ENABLE; bus-off recovery; priorities via .ioc (with P2-6's interlock: move flash ops to foreground first); remove `CO_delete` | P1-5, P1-6, P1-7, P2-6, P2-7 | ~half day |
| 9 | IWDG foreground-liveness gate; bound the offset-cal and UART spins; delete `run_state`/`delay_us` | P1-8 | ~20 lines |
| 10 | Dir-cal snapshot/rollback (and phase-cal); SAVE never-calibrated guard | P1-11, P2-16 | ~20 lines |
| 11 | Host-interface redesign: OD write extensions, truth echo, latched procedure result, EMCY, reset-cause OD object, vel telemetry unconditional | §3.4, RE-1..4, P2-18 | ~a day |
| 12 | Hardware OCP via COMP→FLT (or documented SOA acceptance); shunt-sel leakage pre-check; offset-cal reject restore | P1-10, P2-3, P2-4 | half day + HW question |
| 13 | The long tail: option-byte retry bound + BOR/IWDG_SW, VDDA clamp, MA732 read-verify, vel re-arm seeding, NaN-guard hardening vs -Ofast, cal-pipeline P2s, CCM scrub, doc sweep | P2-*, RE-* | as scheduled |

Items 2, 5, 6, 7 and most of 10 are an afternoon combined. Items 1, 3, 4 are the ones that
should gate flight.

### Punch list

> **Status 2026-06-10:** the overnight fix session (owner responses in
> `docs/flight_review_response.txt`) landed everything marked `[x]` below — **all of it
> build+lint-verified only, zero hardware time**. The bench-verification checklist for the new
> behavior is at the bottom. *(skipped — owner)* = explicitly declined; *(open)* = still
> unimplemented.

**P0**
- [x] P0-1 Node-ID from CAN_ADDR0/1 (internal pull-ups, strap-to-GND, none=wheel1 … both=wheel4; raw counts printed) — **bench-verify on a strapped board; an unstrapped board now enumerates 0x3C, not 0x48**
- [x] P0-2 UV threshold 5.0 V (2.5 V/cell). OV: flight boards build with `IS_CORRECTED_VOLTAGE_DIVIDER` (ceiling 13.3 V > 8.6 V trip); the desk board keeps the old divider where the trip remains unreachable — *(open)* optional boot Vbus-plausibility gate against build/board mismatch
- [x] P0-3 NMI handler clears ECCD + latches ECCR + returns; cal_store brackets every page read and erases on a hit — **bench-test by power-cut mid-save**
- [x] P0-4 Bootloader validates app SP/PC before jump → falls into DFU on invalid; IWDG started before clock config; Error_Handler resets. PB8 confirmed spacecraft-drivable (owner)

**P1**
- [x] P1-1 FSM ladder: requests consumed; non-CLEAR requests dropped (with console note) while errors latched; CLEAR_ERRORS exempt as current state; law_done cleared on NULL-law entry; same-state request = idempotent no-op
- [x] P1-2 NULL-guard `control_law_finish`; POS_CONTROL → NULL law and rejected at SDO ingestion
- [x] P1-3 `is_currents_updated`/`is_encoder_updated` cleared in `controller_arm`
- [x] P1-4 NaN/range gate at SDO ingestion (bit-pattern, -Ofast-proof) — overspeed + overtemp trips *(skipped — owner: validation only)*
- [x] P1-5 `CO_delete` removed from CO_RESET_COMM; reset-node path disarms + flushes
- [x] P1-6 Bus-off recovery (CCCR.INIT cleared after 100 ms back-off) — comms-loss-while-armed policy *(open — needs an ops decision)*
- [x] P1-7 AutoRetransmission = ENABLE (+ SJW 1→2)
- [x] P1-8 IWDG foreground-liveness gate (2 s window, boot exempt); offset-cal wait bounded 2 s; console TX spins bounded with a self-healing wedge latch; run_state/micros/delay_us deleted
- [x] P1-9 Common fault tail: ODISR first, crash record at 0x20017FC0, reset; HardFault shim verified prologue-free in the disassembly
- [ ] P1-10 Hardware OCP *(skipped — owner: protected upstream; software-only ~45 µs documented as accepted)*
- [x] P1-11 Dir/phase-cal snapshot & rollback on failure/cancel (cal-validity flags travel with the values)
- [ ] P1-13 Bootloader flash floor *(skipped — owner: host trusted not to touch the bootloader region)*

**Interface (§3) + P2/RE**
- [x] OD write extensions on 0x4004/0x4005/0x4007 — edge-trigger family dead; validation via SDO abort; deferred reboot with SDO-response egress; docs updated — *(open)* update `can_console/cli.py` (NONE-nudge now unnecessary; bench default node 0x3C)
- [x] TPDO rate change without comm reset (pending value applied via the OD interface from the foreground)
- [x] procedure_result latched (transit states no longer clobber it); EMCY on error edges with the raw bitmask in bytes 4–7; 0x1014 → 0x80 (hand-edit in OD.c — regen retired)
- [x] Reset cause + crash kind + boot counter + fault PC → RAM2 cells (`crash_record.h`) + OD 0x4006 bit-packed — console-ring-to-noinit superseded by the crash record
- [x] 0x1018 serial from the silicon UID; strap raw counts printed at boot
- [x] Velocity telemetry in every state (armed → PLL, any law incl. backemf coast; disarmed → ~200 Hz foreground SPI sampler)
- [x] vel / vel-ramp (re-)arm seeded from measured velocity (bumpless — no more full-authority brake to implicit zero)
- [ ] Shunt-sel leakage check + IBUS constants *(skipped — owner)*; default-ON decision still owner's call at flight freeze
- [x] Offset-cal sane range ±50 mV with early return — offsets not installed, arming refused via INVERTER_CALIBRATION_INVALID
- [x] `controller_disarm` ODISR-first (assert itself remains print-and-continue — *(open)* CAN-visible assert flag if ever wanted)
- [x] FDCAN prio 3 (MspInit direct edit) + cal-store flash ops moved to foreground + CO_LOCK → BASEPRI(2) — **re-measure `isr_cycles_max` armed on a busy bus**
- [x] Option-byte retry bound (3 per power-on, RAM2 cell, fail-open) + BOR_LEV 4 (~2.8 V) enforced — IWDG_SW left software-start *(deliberate: hardware-start fights bench debugging)*
- [x] VDDA raw-count clamp (1300–1750 band, nominal-3.3 V fallback) — **regression, fixed 2026-10-26:** the band was in 12-bit counts but the sample is 16x-oversampled, so every tick fell back to 3.3 V on a 3.0 V reference (Vbus/currents +10% in v1.0.0–v1.0.2). Now a 16384-count floor plus a 2.5–3.4 V result band with a 3.0 V fallback — **bench-verified** (rev 8.1.1: 7.605 V reported vs 7.64 V meter, die 36 °C)
- [ ] MA732 double-read verify *(skipped — owner)*
- [x] h6-FF coefficients zeroed when states 10/15 change the offset; SAVE guard now uses the cal-provenance flags — *(open)* state-21 sub-result reporting, fit R gate, restore-bounds tightening (bench-untested subsystem deliberately left minimally diffed)
- [x] NaN guards hardened via `f32_finite_bits()` (mathutil.h) in blob_valid, the backemf fit gates, and SDO ingestion
- [x] CCM CRC scrub: HW-CRC of CCM vs its flash LMA every 5 s → disarm + crash record + reboot on mismatch
- [ ] Bootloader PB8 majority sampling, DFU inactivity timeout, OB write masking, erase-loop IWDG refresh *(open — lower priority; the validity-check DFU fallback covers the brick class)*. Reflash procedures should mandate read-back verify regardless (erase/program failures are ACKed by stock OpenBootloader)
- [x] Bootloader autobaud ABRE latch (P2-19 family; found 2026-06-10 during DFU bring-up): a garbage first byte (glitch, wrong baud) latched ABRE and `OPENBL_USART_ProtocolDetection` never re-armed the measurement — DFU deaf to every later 0x7F until the ~33 s IWDG cycle. Matters more now that the P0-4 invalid-app fallback idles in DFU unattended. **Fixed:** ABRRQ + RX flush on ABRE (`usart_interface.c`). Idle-DFU ~33 s IWDG reboot accepted as the de-facto inactivity timeout (owner)
- [x] Doc sweep: can-interface.md (new command semantics, strap table, 0x4006 encoding), footguns.md (2 entries rewritten, 4 added), control-laws/control-cycle (32 kHz, extension ingestion, POS row, ramp rate), CLAUDE.md (32 kHz + **CubeMX/CANopenEditor regen RETIRED**), bootloader-integration §4.3/§4.4

### Independent post-fix review (2026-06-10, two adversarial agents over the diff)

Three blockers found and fixed the same night:
- **RAM2 cells were inside the live stack** — `_estack` sat at top-of-RAM2−8, so app_main's
  first frames overlapped the new crash record / boot counter / ECC latch / OB counter.
  Fixed: `_estack = top−0x58` (linker reservation, see the .ld comment).
- **The "bumpless" vel seed read the frozen PLL** (only updates while armed; reads 0 after a
  mid-spin reboot — the case that needs the seed most). Fixed: seeds from the disarmed
  foreground sampler (`get_disarmed_vel()`).
- **A deferred cal-store RESTORE could execute while a law armed in the gap** (host writes
  20 then 5 back-to-back) — live dir/offset/LUT swap under the 32 kHz loop. Fixed: FSM holds
  any new state entry while a cal-store op is pending (also kills the op-slot ABA).

Minors fixed: EMCY re-emits when the error mask *changes* (reset+report pulse); sampler drains
SPI RX residue and the spin bound/comment corrected (~60 µs); TPDO-rate failure printf moved
outside CO_LOCK; ECC heal-by-erase now gated on BK_ECC = bank 2 (an unrelated bank-1 fault in
the bracket no longer costs a healthy cal); HardFault handler made `naked` (prologue-free at
every optimization level, verified in both ELFs); DMA1 ch2–5 re-ranked to priority 3; arming
refused synchronously when the inverter offset cal is invalid (the fast-loop check can't fire
if the HRTIM never ticks).

Accepted-open minor: a cancelled backemf leg-flip can re-enable outputs for ~µs between
disarm's ODISR and the counter stop (end state always off, energy negligible) — fixing it by
clearing `controller_armed` first risks spurious flag-machine errors; deliberately left.

### Bench-verification checklist (everything above is build+lint-tested only)

1. **Strap read:** desk-board boot banner must say `wheel number 1` with plausible raw counts;
   verify a strapped combination on hardware before any multi-wheel test. Update host tooling
   for node 0x3C.
2. **Smoke / commissioning:** 9 → 15 → 21 → 18 → power-cycle → auto-restore → arm VEL_RAMP.
   Exercises the extensions, cal flags + gating, foreground flash ops, and procedure_result
   latching in one pass (poll procedure_result ~50 ms after 18/19/20).
3. **Error recovery over CAN:** force a trip, confirm the EMCY frame + TPDO1, then 13 → poll
   IDLE → re-request → setpoint. Confirm non-13 requests are dropped while errored.
4. **Re-arm on a spinning wheel:** trip at speed, clear, re-enter VEL_CONTROL — the wheel must
   hold speed (bumpless seed), not brake; disarmed TPDO2 velocity must track the coast.
5. **ECC path:** power-cut mid-save (state 18), reboot — expect `cal_store: flash ECC fault …
   erasing page`, boot continues uncalibrated, no boot loop.
6. **Fault tail:** force a HardFault over SWD while armed — bridge drops immediately, clean
   reboot, boot banner shows LAST FAULT, 0x4006 carries kind + PC.
7. **Watchdog gate:** wedge only the foreground (SWD breakpoint in the app_main loop with ISRs
   running) — IWDG reset within ~2.5 s.
8. **Timing:** `isr_cycles_max` armed at speed with 100 ms TPDOs — confirm the BASEPRI/priority
   changes improved (not regressed) the 72 % budget.
9. **Bus-off:** short CANH/CANL > 1 s, release — node rejoins ~100 ms later without reboot.
10. **Bootloader:** erase the app over SWD, reset with PB8 low — must enter DFU (no reset
    loop); reflash over UART; confirm the normal jump afterwards.
