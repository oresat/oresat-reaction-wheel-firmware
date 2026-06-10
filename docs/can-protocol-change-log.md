# CAN protocol changes — 2026-06-10 fix session

Host-facing summary of exactly what changed on the bus in the overnight review-fix session.
The full interface spec is `docs/can-interface.md`; per-change implementation detail is in
`docs/review-fixes-change-log.md`. Audience: anyone writing or updating host software
(`can_console/cli.py`, the C3 flatsat stack).

## Wire-compatible, unchanged

COB-ID scheme, all four TPDO payload layouts and field types, the OD indices/subindices and
their data types, expedited SDO downloads, heartbeat at 1000 ms, 1 Mbit/s classic frames.
A host that only *reads* TPDOs keeps working untouched.

## Node identity

- An **unstrapped board now enumerates as wheel 1 (`0x3C`)**, not `0x48`. Straps are
  CAN_ADDR0/1 to GND: none = wheel 1, ADDR0 = wheel 2, ADDR1 = wheel 3, both = wheel 4.
  (The old DEBUG-pin vote read output-driven pins and always answered "wheel 4".)
- `0x1018:04` (identity serial) is now a nonzero per-chip UID hash — wheels are
  LSS-distinguishable and a duplicate-node-ID fault is diagnosable.

## Command writes (the big change)

Ingestion moved from foreground polling-with-change-detection to OD **write extensions**:

| You write | Old behavior | New behavior |
|---|---|---|
| Same state again (`0x4004`) | Silently eaten (edge trigger) | Applies every time; if already *in* that state it's an explicit idempotent no-op (no re-init blip) — command replay is safe |
| `0x4004` = 0, 2, 3, 6, or > 21 | Forwarded blind (zombie states; state 6 HardFaulted → wheel reboot) | **SDO abort `0x0609 0030`** (invalid value) |
| `0x4004` = 4/5/16/17/21 without valid encoder cal | Armed anyway on default/garbage cal | **SDO abort `0x0800 0022`** (device state) — run or restore cal first |
| Any state except 13 while errors latched | Triggered an IDLE↔ERROR transition storm | Accepted at SDO level, then **dropped by the FSM** (console note). Only CLEAR_ERRORS proceeds |
| Same setpoint again (`0x4005`) | Eaten | Applies every time |
| NaN/Inf or \|x\| > 1e5 setpoint | NaN was laundered into **−5 A full reverse torque**, no error | **SDO abort `0x0609 0030`** |
| `0x4007` = anything ≠ 1 | Silently ignored | Any nonzero reboots |

Setpoint routing is still by-current-state-at-write-time: **confirm `0x4000:01` before writing
the setpoint**. A write that raced a transition can now simply be re-sent (no edge trigger
left to poison).

## Responses and timing

- The reboot SDO **response now arrives before the reset** (~3 ms deferral). Tooling that
  treated the SDO timeout as success should expect a confirmed write instead.
- Cal-store states 18/19/20: the flash op runs in the foreground after the state bounces
  through — `procedure_result` reads 0 for a few tens of ms; **poll ~50 ms after commanding**.
- `procedure_result` (TPDO1 byte 1) now **latches** the last completed procedure's outcome
  instead of being clobbered to 1 by the next state command.
- Entering VEL/VEL_RAMP seeds the internal setpoint with the **measured** velocity (bumpless):
  arming a spinning wheel holds speed until your setpoint arrives, instead of braking hard
  toward an implicit zero.

## Gone from the bus

State transitions no longer reset the CANopen comm layer, so:

- **No boot-up frame per transition** — a boot-up frame means a real reboot again.
- No purged TX queue → no sporadic SDO-response timeouts right after state commands.
- No NMT auto-override: a host-commanded pre-operational survives state changes.
- Host-written TPDO event timers aren't clobbered per transition (firmware still sets
  100 ms in law states / 5000 ms otherwise, applied live with an immediate burst).

## New on the bus

- **EMCY** at `0x80 + nodeID` on every controller-error *change* (raised / mask-changed /
  cleared), with the raw `controller_errors` bitmask in bytes 4–7, logged in `0x1003`.
  Fault notification is immediate even at the 5 s idle TPDO rate.
- `0x4006` is now **read-only boot diagnostics** (bit-packed u32s, not meaningful floats):
  `reserved_1` = RCC reset-cause flags [31:24] | last crash kind [19:16] | boot count [15:0];
  `reserved_2` = last-fault stacked PC (0 = none).
- TPDO2 `motor_vel` is valid in **every** state — a disarmed/coasting wheel reports real speed
  (~200 Hz foreground sampler). `motor_current`/`bus_*` remain control-states-only.
- TPDO4 `0x4003` fields **remapped and recalibrated** (2026-06-10): `temp_1` = average of the
  three phase thermistors (was: aux), `temp_2` = aux/board thermistor (was: phase A only;
  phases B/C were sampled but never reported), `temp_3` = MCU die (unchanged). The external
  thermistor curve was also fixed — the old LUT assumed a 10 k pullup / β≈4100 NTC, but the
  board is a 3.3 k pullup with a β=3380 10 k NTC, so old values read ~20–60 °C low (≈3 °C at a
  true 25 °C). Old telemetry from these fields should be considered garbage. Faulted-sensor
  sentinels: open ≈ −100 °C, shorted ≈ +300 °C.

## Link layer

- Auto-retransmission is ON: frames no longer silently vanish on arbitration loss — host-side
  blind-retry workarounds can be removed. SJW raised to 2.
- Bus-off self-recovers ~100 ms after the fault clears (no reboot needed).
- NMT reset-node (0x81) now disarms + flushes like `0x4007`; NMT reset-communication (0x82)
  no longer corrupts the heap (safe to use).

## can_console/cli.py to-do list

Drop the `FSM_STATE_NONE`-nudge (unnecessary; writing 0 now aborts), default node `0x3C`,
poll `0x4000` before setpoints, wait ~50 ms before reading `procedure_result` after 18/19/20,
expect confirmed reboot responses, optionally decode `0x4006` + EMCY frames, and relabel the
TPDO4 temp fields (phase-avg / aux / die).
