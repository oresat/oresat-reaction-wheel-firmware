# can_console

Host-side CAN bench tool for the ACS reaction wheels. `cli.py` needs
`pip install canopen`. The bring-up commands (`bringup.py`) also need
`pip install pyserial`, to read the debug UART. Wire protocol:
`docs/can-interface.md`. Per-state behavior: `docs/control-laws.md`.

```bash
ip link set can0 up type can bitrate 1000000   # once, as root

python cli.py scan                    # who's on the bus?
python cli.py --wheel 4 status       # one-shot state/errors/telemetry
python cli.py --wheel 4 monitor      # live TPDO dashboard (no SDO traffic, safe while armed)
python cli.py --wheel 4 commission   # first-time cal: 9 -> 15 -> 21 -> 18
python cli.py --wheel 4 spin 50      # closed-loop to 50 rev/s (VEL_RAMP_CONTROL)
python cli.py --wheel 4 spintest 50  # spin, sample, mean/stdev report, ramp down
python cli.py --wheel 4 stop
python cli.py --wheel 4 cal bidir    # run any single completing procedure (cal = list them)
python cli.py --wheel 4 errors       # decode the latched error bitmask
python cli.py --wheel 4 clear        # CLEAR_ERRORS
python cli.py --wheel 4 reboot
python cli.py selftest               # board self-test (state 22), per-check table from the debug UART
python cli.py bringup                # guided bring-up of a new board -> bringup_logs/*.log
```

## Bring-up of a new board

`bringup` walks an operator through power-up, the self-test, an encoder magnet check, the
undervoltage trip, a CAN reboot and an optional soak, and writes a log per board. The
operator guide is `docs/board-bringup.md`. `selftest` runs just state 22. Both find the one
wheel on the bus, and find the debug-UART serial port by itself (the port that prints the boot
banner or a `SELFTEST` line). Use `--uart` to pick the port by hand.

Interactive bench session (old workflow, kept):

```bash
python -i cli.py --wheel 4 shell
>>> rxw.status()
>>> rxw.commission()              # or commission(reboot_after=True)
>>> rxw.spin(50); rxw.stop()
>>> rxw.run_procedure(S.BACKEMF_CAL_BIDIR)
```

## Commissioning

`commission` runs the first-time-cal runbook from
`docs/backemf-coastdown-implementation.md`: encoder dir cal (9) → bootstrap
single-point offset (15) → bidirectional back-EMF self-cal with on-board
offset+LUT fit (21, ~15–60 s, spins both directions) → save to flash (18).
Each step waits for the FSM to return to IDLE and checks `procedure_result`.
Cal auto-restores at every boot afterwards; `--reboot` exercises that path
immediately. Acceptance detail (UART-only): re-run state 21 after a power
cycle and check the printed `D` is ≈ 0.

## Firmware behavior this tool relies on

- Every write to `0x4004` / `0x4005` is applied (OD write extensions), so a repeat of the
  same state or setpoint works. The old edge-trigger workarounds are gone: current firmware
  answers a write of `0` to `0x4004` with an SDO abort.
- Bad requests come back as SDO aborts (`canopen.SdoAbortedError`): `0x06090030` = not a
  requestable state, `0x08000022` = closed-loop state without encoder cal.
- `procedure_result` holds the last procedure's outcome until the next one runs.
  For 18/19/20 and 22 it lands after the FSM is back in IDLE, so `run_procedure()`
  waits 0.4 s before reading it.
- TPDO2/3 iq/vbus/ibus read 0 outside the closed-loop states (velocity is live everywhere).

The 0x4000–0x4007 OD block is hand-defined in `cli.py`: no EDS in the repo
carries it (the vendored `DS301_profile.eds` has an empty
`[ManufacturerObjects]`); `OD/OD.c` is the source of truth. If you touch the
block in firmware, update `build_object_dictionary()` and the enums here.

`cli.py.orig` is the pre-2026-06 version, kept for reference — delete at will.
