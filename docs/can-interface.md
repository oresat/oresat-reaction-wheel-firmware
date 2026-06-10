# CAN / CANopen host interface

How to drive and monitor a wheel from a host over the bus. Firmware side lives in
`Firmware/Src/can_comms.c`; the object dictionary is `OD/OD.{c,h}` (generated — see `CLAUDE.md`).

- **Bus:** FDCAN1, **1 Mbit/s**, classic CAN frames (no BRS/FD payloads).
- **Node ID:** hardware-strapped at boot from DEBUG1/2/3 (majority vote in `app_main.c`):

  | Wheel | Node ID | Straps (DEBUG1/2/3) |
  |---|---|---|
  | 1 | `0x3C` (60) | none |
  | 2 | `0x40` (64) | DEBUG1 |
  | 3 | `0x44` (68) | DEBUG1+2 |
  | 4 | `0x48` (72) | DEBUG1+2+3 |

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
set by firmware via `set_tpdo_send_rate()`: **100 ms while a control-law state is active, 5000 ms in
IDLE.** Changing the rate calls `canopen_app_resetCommunication()` (takes effect after an NMT reset-
comm). Payloads are little-endian, packed in OD sub-index order:

| PDO | Bytes | Fields (type) | Source |
|---|---|---|---|
| TPDO1 `0x4000` | 6 | `current_state` (u8), `procedure_result` (u8), `errors` (u32) | FSM state, last `finish()` bool, `CONTROLLER_ERRORS_t` bitmask |
| TPDO2 `0x4001` | 8 | `motor_vel` (f32), `motor_current` (f32) | encoder vel [rev/s], measured `iq` [A] |
| TPDO3 `0x4002` | 8 | `bus_voltage` (f32), `bus_current` (f32) | Vbus [V], estimated ibus [A] |
| TPDO4 `0x4003` | 6 | `temp_1` (i16), `temp_2` (i16), `temp_3` (i16) | thermistors 1/2 + internal, [centi-°C] |

`current_state` decodes via `fsm_state_t` (see `docs/control-laws.md`); `errors` decodes via
`CONTROLLER_ERRORS_t` (`control_ll.h`) / `STRINGIFY_CONTROLLER_ERRORS()`. **TPDO2/TPDO3 are only
populated in the four closed-loop control states** (torque/vel/pos/vel-ramp); otherwise they read 0.
TPDO4 temps are always live. (Temps are `int16` hundredths of a degree C, i.e. 2500 = 25.00 °C.)

## SDOs (host → node commands)

Write these OD entries via standard SDO download (expedited). The firmware acts on them in
`process_sdo_vals()`, called from the foreground loop at ~1 kHz.

| Index:sub | Name | Type | Meaning |
|---|---|---|---|
| `0x4004:01` | `req_state` | u8 | Requested `fsm_state_t`. On change → `fsm_request_state()`. |
| `0x4005:01` | `setpoint_1` | f32 | Routed by **current** state (see below). On change only. |
| `0x4005:02` | `setpoint_2` | f32 | Read but currently unused. |
| `0x4006:01/02` | `reserved_1/2` | f32 | Read but currently unused. |
| `0x4007:01` | `reboot_request` | u8 | Write `1` → `controller_disarm()` + `HAL_NVIC_SystemReset()`. |

**`setpoint_1` routing** (by the FSM state active *at the time of the write*):

| Active state | `setpoint_1` interpreted as |
|---|---|
| `TORQUE_CONTROL` (4) | torque [N·m] |
| `VEL_CONTROL` (5) | velocity [rev/s] |
| `POS_CONTROL` (6) | position — but this state is a non-functional stub (see control-laws.md) |
| `VEL_RAMP_CONTROL` (16) | target velocity [rev/s], slew-limited |
| any other | ignored |

The handler is **edge-triggered**: it only re-applies a setpoint when the written value *differs*
from the last one it saw. Two consequences:
1. **Order matters** — request the state (`0x4004`) first, *then* write `setpoint_1`. A setpoint
   written while in the wrong state is dropped (and won't be re-applied later unless it changes).
2. Writing the *same* numeric value again is a no-op. To re-assert, write a different value first.

`req_state` is edge-triggered the same way, and the firmware never rewrites `0x4004` after a
procedure auto-returns to IDLE — so re-requesting the state that just ran is a silent no-op. To
re-run it, first write `0` (`FSM_STATE_NONE`, which the FSM treats as "no request" — guaranteed
side-effect-free), then the target state. `can_console/cli.py` does this automatically.

## Typical command sequences

Bring a wheel to 50 rev/s under ramped velocity control (replace `0x63C` with the target wheel's
SDO-rx COB-ID):

1. NMT: start the node (enter Operational) so TPDOs flow.
2. SDO download `0x4004:01` = `16` (`FSM_STATE_VEL_RAMP_CONTROL`).
3. SDO download `0x4005:01` = `50.0` (f32).
4. Watch TPDO2 (`0x2BC`) for `motor_vel` climbing; TPDO1 (`0x1BC`) `errors` should stay 0.
5. To stop: `0x4005:01` = `0.0`, then `0x4004:01` = `1` (IDLE), or `0x4007:01` = `1` to reboot.

Clear a latched fault: `0x4004:01` = `13` (`FSM_STATE_CLEAR_ERRORS`) — errors reset on exit and the
FSM auto-returns to IDLE. Any nonzero `errors` otherwise pins the node in `CONTROLLER_ERROR` (3).

**Calibration persistence** (same action-state pattern as CLEAR_ERRORS — request, action fires,
auto-return to IDLE; success/failure lands in TPDO1 `procedure_result`):

- `0x4004:01` = `18` (`SAVE_CALIBRATION`) — snapshot live encoder offset/dir → flash cal page
  (skips the write if unchanged; refuses if dir-cal never ran).
- `0x4004:01` = `19` (`ERASE_CALIBRATION`) — wipe the cal page.
- `0x4004:01` = `20` (`RESTORE_CALIBRATION`) — re-apply the stored cal (also runs automatically
  at every boot, so a typical commissioning is: dir-cal (9) → offset cal (10/15/17) → save (18),
  and subsequent boots need nothing).

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
  decode) — see `can_console/README.md`.
- Firmware boots to `IDLE`; no motion happens until a control-law state is requested.
- Identity (`0x1018`), device type (`0x1000`), vendor/product codes are currently 0 (not set).
