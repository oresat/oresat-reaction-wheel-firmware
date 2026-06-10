# CAN / CANopen host interface

How to drive and monitor a wheel from a host over the bus. (Upgrading host tooling across the
2026-06-10 firmware fixes? The behavioral diff is `docs/can-protocol-change-log.md`.) Firmware side lives in
`Firmware/Src/can_comms.c`; the object dictionary is `OD/OD.{c,h}` (generated — see `CLAUDE.md`).

- **Bus:** FDCAN1, **1 Mbit/s**, classic CAN frames (no BRS/FD payloads).
- **Node ID:** hardware-strapped at boot from the dedicated **CAN_ADDR0/1** inputs (PB7/PB5,
  internal pull-ups, strap-to-GND = bit set; 1000-sample majority vote in `app_main.c`). The old
  DEBUG1/2/3 scheme was never functional — those pins are push-pull outputs (the `gpio_debug`
  scope pins), so every board read "wheel 4" (flight-review P0-1). **An unstrapped bench board
  now enumerates as wheel 1 (`0x3C`)**, not `0x48`.

  | Wheel | Node ID | Straps (to GND) |
  |---|---|---|
  | 1 | `0x3C` (60) | none |
  | 2 | `0x40` (64) | CAN_ADDR0 |
  | 3 | `0x44` (68) | CAN_ADDR1 |
  | 4 | `0x48` (72) | CAN_ADDR0 + CAN_ADDR1 |

  The boot console prints the raw high-counts for both pins so a marginal/floating strap is
  diagnosable at the bench.

All COB-IDs below follow the CANopen predefined connection set: **effective ID = base + node ID**
(the stack adds the node ID; `CANopen.c`: `CO_CAN_ID_TPDO_1 + i*0x100 + nodeId`).

## COB-IDs per wheel

| Function | Base | Wheel 1 (0x3C) | Wheel 2 (0x40) | Wheel 3 (0x44) | Wheel 4 (0x48) |
|---|---|---|---|---|---|
| TPDO1 ctrl/status | 0x180 | 0x1BC | 0x1C0 | 0x1C4 | 0x1C8 |
| TPDO2 motor vel/iq | 0x280 | 0x2BC | 0x2C0 | 0x2C4 | 0x2C8 |
| TPDO3 bus V/I | 0x380 | 0x3BC | 0x3C0 | 0x3C4 | 0x3C8 |
| TPDO4 temps | 0x480 | 0x4BC | 0x4C0 | 0x4C4 | 0x4C8 |
| SDO request (rx, host→node) | 0x600 | 0x63C | 0x640 | 0x644 | 0x648 |
| SDO response (tx, node→host) | 0x580 | 0x5BC | 0x5C0 | 0x5C4 | 0x5C8 |
| Heartbeat | 0x700 | 0x73C | 0x740 | 0x744 | 0x748 |

Heartbeat producer period is 1000 ms (`0x1017`).

## TPDOs (node → host telemetry)

All four are **asynchronous, event-timer driven** (transmission type `0xFE`). The event timer is
set by firmware via `set_tpdo_send_rate()`: **100 ms while a control-law state is active, 5000 ms
otherwise.** The rate is applied live through the OD interface from the foreground — state
transitions **no longer reset the CANopen comm layer** (no spurious boot-up frames, no purged TX
queue, no NMT blips; the running event timer is zeroed on each change, so all four TPDOs still
burst promptly after a transition). Payloads are little-endian, packed in OD sub-index order:

| PDO | Bytes | Fields (type) | Source |
|---|---|---|---|
| TPDO1 `0x4000` | 6 | `current_state` (u8), `procedure_result` (u8), `errors` (u32) | FSM state, last `finish()` bool, `CONTROLLER_ERRORS_t` bitmask |
| TPDO2 `0x4001` | 8 | `motor_vel` (f32), `motor_current` (f32) | encoder vel [rev/s], measured `iq` [A] |
| TPDO3 `0x4002` | 8 | `bus_voltage` (f32), `bus_current` (f32) | Vbus [V], estimated ibus [A] |
| TPDO4 `0x4003` | 6 | `temp_1` (i16), `temp_2` (i16), `temp_3` (i16) | thermistors 1/2 + internal, [centi-°C] |

`current_state` decodes via `fsm_state_t` (see `docs/control-laws.md`); `errors` decodes via
`CONTROLLER_ERRORS_t` (`control_ll.h`) / `STRINGIFY_CONTROLLER_ERRORS()`.

- **`motor_vel` is live in every state**: armed → the encoder PLL (any law, including the
  back-EMF coast with a floated bridge); disarmed → a coarse ~200 Hz foreground SPI sampler. A
  coasting wheel after a trip/cancel is therefore visible to the ADCS (it used to read 0).
- `motor_current`/`bus_voltage`/`bus_current` are only populated in the closed-loop control
  states (torque/vel/vel-ramp) — the FOC consumers that produce them don't run elsewhere.
- `procedure_result` **latches** the outcome of the last completed procedure (law or cal-store
  action) until the next one runs — it is no longer clobbered by transits through IDLE. For the
  cal-store states (18/19/20) the flash op executes in the foreground a few tens of ms after the
  state bounces through; poll `procedure_result` ~50 ms after commanding.
- TPDO4 temps are always live (`int16` hundredths of °C, 2500 = 25.00 °C).
- **EMCY**: controller-error edges also emit a CANopen emergency (COB-ID `0x80 + node ID`) with
  the raw `controller_errors` bitmask in bytes 4–7, and land in the `0x1003` error history —
  faults are announced immediately even at the 5 s idle TPDO rate.

## SDOs (host → node commands)

Write these OD entries via standard SDO download (expedited). Command objects carry **OD write
extensions**: every download invokes its handler exactly once, **regardless of value** — the old
edge-triggered polling (and its whole silently-eaten-rewrite trap family, flight-review §3) is
gone. Invalid writes are answered with an **SDO abort** instead of silence.

| Index:sub | Name | Type | Meaning |
|---|---|---|---|
| `0x4004:01` | `req_state` | u8 | Requested `fsm_state_t`. Every accepted write → `fsm_request_state()`. |
| `0x4005:01` | `setpoint_1` | f32 | Routed by **current** state (see below). Applied on every accepted write. |
| `0x4005:02` | `setpoint_2` | f32 | Reserved: stored + readable, applied nowhere. |
| `0x4006:01/02` | `reserved_1/2` | f32 | **Boot diagnostics, read-only by convention** (see below). |
| `0x4007:01` | `reboot_request` | u8 | Any nonzero → disarm + console flush + reset. The SDO response is sent **before** the reset (≈3 ms delay), so a confirmed write = reboot accepted. |

**`0x4004` validation** (SDO abort `0x0609 0030` invalid value / `0x0800 0022` device state):
- Rejected outright: `0` (request sentinel), `2`/`3` (error states — the FSM enters those, hosts
  don't), `6` (`POS_CONTROL` — unimplemented stub), anything > 21.
- The closed-loop states (4/5/16) and the back-EMF cal states (17/21) abort with `0x0800 0022`
  unless **encoder dir + offset cal are valid** (run/restore cal first — see commissioning below).
- Re-requesting the state the FSM is already in is accepted and treated as an idempotent no-op
  (no cancel/re-init blip) — command replay is safe.
- While errors are latched, any request except `13` (CLEAR_ERRORS) is **dropped by the FSM** with
  a console note: send CLEAR_ERRORS first.

**`setpoint_1` routing** (by the FSM state active *at the time of the write*):

| Active state | `setpoint_1` interpreted as |
|---|---|
| `TORQUE_CONTROL` (4) | torque [N·m] |
| `VEL_CONTROL` (5) | velocity [rev/s] |
| `VEL_RAMP_CONTROL` (16) | target velocity [rev/s], slew-limited |
| any other | accepted (readable back) but applied to nothing |

Setpoint writes reject NaN/Inf and |x| > 1e5 with an SDO abort (a NaN used to be laundered by
the clamp into **full negative current**, flight-review P1-4). Because routing happens at write
time and a state transition takes ≤ ~2 FSM ticks, still **confirm the state via `0x4000:01`
before writing the setpoint** — but a write that raced the transition can now simply be
re-sent (there is no edge-trigger left to poison).

**Bumpless (re-)arm:** entering VEL/VEL_RAMP seeds the internal setpoint with the *measured*
velocity, so arming on a spinning wheel holds speed instead of full-authority braking to an
implicit zero. Write the real setpoint after confirming the state.

**`0x4006` boot diagnostics** (bit-packed u32s — reinterpret the 4 raw bytes, not the float):
- `reserved_1` = `RCC_CSR` reset-cause flags `[31:24]` | last crash kind `[19:16]`
  (1 = HardFault, 2 = unknown NMI, 3 = Error_Handler, 4 = CCM scrub) | boot count since
  power-on `[15:0]`
- `reserved_2` = stacked PC of the last fault (0 = no crash record)

## Typical command sequences

Bring a wheel to 50 rev/s under ramped velocity control (replace `0x63C` with the target wheel's
SDO-rx COB-ID):

1. NMT: start the node (enter Operational) so TPDOs flow (the node also auto-starts).
2. SDO download `0x4004:01` = `16` (`FSM_STATE_VEL_RAMP_CONTROL`). Aborts with `0x0800 0022` if
   encoder cal isn't valid yet.
3. Poll `0x4000:01` (or TPDO1) until `current_state == 16`.
4. SDO download `0x4005:01` = `50.0` (f32).
5. Watch TPDO2 (`0x2BC`) for `motor_vel` climbing; TPDO1 (`0x1BC`) `errors` should stay 0.
6. To stop: `0x4005:01` = `0.0`, then `0x4004:01` = `1` (IDLE), or `0x4007:01` = nonzero to reboot.

Clear a latched fault: `0x4004:01` = `13` (`FSM_STATE_CLEAR_ERRORS`) — errors reset on its exit
and the FSM genuinely auto-returns to IDLE (the old ladder parked in `CONTROLLER_ERROR` with
`errors == 0`; fixed). While errors are latched the FSM drops every other request, so the
recovery sequence is exactly: `13` → poll for `current_state == 1` → re-request the target state
→ confirm → write the setpoint.

**Calibration persistence** (same action-state pattern as CLEAR_ERRORS — request, action fires,
auto-return to IDLE; success/failure lands in TPDO1 `procedure_result`):

- `0x4004:01` = `18` (`SAVE_CALIBRATION`) — snapshot live encoder offset/dir → flash cal page
  (skips the write if unchanged; **refuses unless both dir and offset cal are actually valid** —
  the provenance flags, not just value heuristics).
- `0x4004:01` = `19` (`ERASE_CALIBRATION`) — wipe the cal page (live cal stays applied).
- `0x4004:01` = `20` (`RESTORE_CALIBRATION`) — re-apply the stored cal (also runs automatically
  at every boot, so a typical commissioning is: dir-cal (9) → offset cal (15) → save (18),
  and subsequent boots need nothing).

The flash op runs in the **foreground**, a few tens of ms after the state bounces through —
poll `procedure_result` ~50 ms after commanding 18/19/20 (it reads 0 until the op lands).
A failed/cancelled dir or offset cal now **rolls back** to the previous live values (and their
validity), so a botched in-flight re-cal can't leave a probe value (dir = +1) silently armed.

Commissioning a wheel end-to-end: `0x4004:01` = `9` (dir cal, wait done) → `15` (bootstrap offset
cal, wait done) → `21` (bidirectional back-EMF self-cal; fits offset + LUT **on-board** and applies
them live — full runbook in backemf-coastdown-implementation.md) → `18` (save). Check
`procedure_result` = 1 after each. Subsequent boots auto-restore — no recal needed.
`can_console/cli.py commission` automates the whole sequence.

## Notes for host tooling

- The vendored EDS (`CANopenSTM32/CANopenNode_STM32/DS301_profile.eds` / `.xpd`) does **not**
  contain the manufacturer `0x4000`–`0x4007` block (`[ManufacturerObjects]` is empty) — the
  generated `OD/OD.c` is the source of truth. Host tooling must define the block by hand; see
  `build_object_dictionary()` in `can_console/cli.py`.
- `python-can` + the `canopen` package is a convenient host stack; node ID and COB-IDs as above.
  `can_console/cli.py` wraps all of this (status / live TPDO monitor / commission / spin / error
  decode) — see `can_console/README.md`. **Note:** cli.py's `FSM_STATE_NONE`-nudge workaround
  for the old edge-trigger is now unnecessary (harmless — a write of `0` gets an SDO abort), and
  its default node ID should change to wheel 1 (`0x3C`) for an unstrapped bench board.
- Firmware boots to `IDLE`; no motion happens until a control-law state is requested.
- A wheel reboot is detectable as: heartbeat gap + boot count increment in `0x4006` + state
  regression to IDLE. (Boot-up frames are once again meaningful — they're only emitted at real
  comm init, not on every state transition.)
- Identity (`0x1018`), device type (`0x1000`), vendor/product codes are currently 0 (not set).
