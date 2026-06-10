# can_console

Host-side CAN bench tool for the ACS reaction wheels. One file (`cli.py`),
one dependency (`pip install canopen`). Wire protocol: `docs/can-interface.md`;
per-state behavior: `docs/control-laws.md`.

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
```

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

## Firmware gotchas this tool works around

- **`0x4004` (req_state) is edge-triggered** and never rewritten by firmware,
  so re-requesting the state a procedure just finished would be a silent no-op.
  `request_state()` first writes `0` (`FSM_STATE_NONE`, ignored by the FSM) to
  reset the edge detector.
- **Setpoints are edge-triggered too**: `set_setpoint()` nudges through an
  epsilon-different value when rewriting the stale one (otherwise
  `spin(50); idle(); spin(50)` would hold zero).
- **`procedure_result` is overwritten by every FSM exit transition** (even
  IDLE→IDLE), so `run_procedure()` reads it before any further `0x4004` traffic.
- TPDO2/3 fields (vel/iq/vbus/ibus) read 0 outside the four closed-loop states.

The 0x4000–0x4007 OD block is hand-defined in `cli.py`: no EDS in the repo
carries it (the vendored `DS301_profile.eds` has an empty
`[ManufacturerObjects]`); `OD/OD.c` is the source of truth. If you touch the
block in firmware, update `build_object_dictionary()` and the enums here.

`cli.py.orig` is the pre-2026-06 version, kept for reference — delete at will.
