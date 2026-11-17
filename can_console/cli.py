#!/usr/bin/env python3
"""OreSat ACS reaction-wheel CAN console.

Host-side bench tool for driving and monitoring a wheel over CANopen.
Wire protocol: docs/can-interface.md. Per-state behavior: docs/control-laws.md.
First-time-cal runbook: docs/backemf-coastdown-implementation.md.

The 0x4000-0x4007 manufacturer block is hand-defined below: the vendored
DS301_profile.eds does NOT contain it ([ManufacturerObjects] is empty) -- the
generated OD/OD.c is the source of truth. Keep the enums here in sync with
Firmware/Inc/fsm.h and Firmware/Inc/control_ll.h.

Usage:
  python cli.py scan                          # who's on the bus?
  python cli.py --wheel 4 status              # one-shot state/errors/telemetry
  python cli.py --wheel 4 monitor             # live TPDO dashboard (Ctrl-C exits)
  python cli.py --wheel 4 commission          # first-time cal: 9 -> 15 -> 21 -> 18
  python cli.py --wheel 4 spin 50             # closed-loop to 50 rev/s (VEL_RAMP)
  python cli.py --wheel 4 stop                # ramp to 0, drop to IDLE
  python cli.py --wheel 4 cal bidir           # run any single procedure
  python cli.py --wheel 4 errors              # decode latched error flags
  python cli.py --wheel 4 clear               # CLEAR_ERRORS
  python cli.py --wheel 4 reboot
  python cli.py selftest                      # board self-test (state 22), details from the debug UART
  python cli.py bringup                       # guided step-by-step bring-up of a new board

Interactive bench session (the old workflow, kept):
  python -i cli.py --wheel 4 shell
  >>> rxw.status()
  >>> rxw.commission()
  >>> rxw.spin(50); rxw.stop()
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from enum import IntEnum, IntFlag
from statistics import mean, stdev

try:
    import can  # python-can, installed with canopen
    import canopen
    from canopen.objectdictionary import (INTEGER16, REAL32, UNSIGNED8,
                                          UNSIGNED32, ODVariable, Record)
except ImportError:
    sys.exit("this tool needs the 'canopen' package:  pip install canopen")

# --------------------------------------------------------------- bus identity

# config.h CANID_WHEEL{1..4}; strapped at boot from DEBUG1/2/3 (can-interface.md)
WHEEL_NODE_IDS = {1: 0x3C, 2: 0x40, 3: 0x44, 4: 0x48}
DEFAULT_WHEEL = 4

# ----------------------------------------------------------- firmware mirrors

class ControllerState(IntEnum):
    """Mirror of fsm_state_t (Firmware/Inc/fsm.h)."""
    NONE = 0                  # FSM "no request" sentinel; 0x4004 rejects it (SDO abort)
    IDLE = 1
    SYSTEM_ERROR = 2
    CONTROLLER_ERROR = 3
    TORQUE_CONTROL = 4
    VEL_CONTROL = 5
    POS_CONTROL = 6           # firmware stub: rejected at 0x4004 (SDO abort)
    MOTOR_RESISTANCE_CAL = 7  # diagnostic only -- FOC uses hardcoded R/L
    MOTOR_INDUCTANCE_CAL = 8  # diagnostic only
    ENCODER_DIR_CAL = 9
    ENCODER_OFFSET_CAL = 10   # 4-rev sweep, coarse + fine offset, applied
    ENCODER_TEST = 11
    OPEN_LOOP_CONTROL = 12    # setpoint not wired to CAN -- holds zero current
    CLEAR_ERRORS = 13
    ENCODER_VALIDATION = 14
    SHITTY_OFFSET_CAL = 15    # single-point offset cal; the practical bootstrap path
    VEL_RAMP_CONTROL = 16
    BACKEMF_CAL = 17          # single-direction coast capture; log-only, never applied
    SAVE_CALIBRATION = 18     # action states: fire on the exit transition,
    ERASE_CALIBRATION = 19    # auto-return to IDLE, result -> procedure_result
    RESTORE_CALIBRATION = 20  # (also auto-runs at every boot)
    BACKEMF_CAL_BIDIR = 21    # fwd+rev coast capture; on-board offset+LUT fit, applied live
    SELF_TEST = 22            # desk bring-up self-test; report on the debug UART (bringup.py)


class ProcedureResult(IntEnum):
    FAIL = 0
    SUCCESS = 1


class ControllerErrors(IntFlag):
    """Mirror of CONTROLLER_ERRORS_t (Firmware/Inc/control_ll.h)."""
    NONE = 0
    CTRLR_ERR_INVERTER_CALIBRATION_INVALID = 1 << 0
    CTRLR_ERR_PHASE_CURRENTS_INVALID = 1 << 1
    CTRLR_ERR_PHASE_CURRENTS_MEASUREMENT_MISSING = 1 << 2
    CTRLR_ERR_PWM_TIMING_INVALID = 1 << 3
    CTRLR_ERR_PWM_TIMING_UPDATE_MISSING = 1 << 4
    CTRLR_ERR_VBUS_OVERVOLTAGE = 1 << 5
    CTRLR_ERR_VBUS_UNDERVOLTAGE = 1 << 6
    CTRLR_ERR_IBUS_OVERCURRENT = 1 << 7
    CTRLR_ERR_MOTOR_OVERCURRENT = 1 << 8
    CTRLR_ERR_MOTOR_PHASE_LEAKAGE = 1 << 9
    CTRLR_ERR_MOTOR_RESISTANCE_OUT_OF_RANGE = 1 << 10
    CTRLR_ERR_MOTOR_INDUCTANCE_OUT_OF_RANGE = 1 << 11
    CTRLR_ERR_ENCODER_READING_MISSING = 1 << 12
    CTRLR_ERR_ENCODER_ESTIMATE_MISSING = 1 << 13
    CTRLR_ERR_ENCODER_READING_INVALID = 1 << 14
    CTRLR_ERR_ENCODER_FAILURE = 1 << 15
    CTRLR_ERR_PHASE_CURRENT_USAGE_MISSING = 1 << 16
    CTRLR_ERR_PWM_TIMING_USAGE_MISSING = 1 << 17
    CTRLR_ERR_PHASE_CURRENT_LEAKAGE = 1 << 18
    CTRLR_ERR_ENCODER_READING_USAGE_MISSING = 1 << 19
    CTRLR_ERR_MOTOR_UNBALANCED_PHASES = 1 << 20
    CTRLR_ERR_MODULATION = 1 << 21
    CTRLR_ERR_NOT_CALIBRATED = 1 << 22


_ALL_KNOWN_ERRORS = ControllerErrors((1 << 23) - 1)


def fmt_errors(errs: ControllerErrors | int) -> str:
    errs = ControllerErrors(int(errs))
    if not errs:
        return "none"
    parts = [f.name for f in ControllerErrors if f and f in errs]
    residual = int(errs) & ~int(_ALL_KNOWN_ERRORS)
    if residual:
        parts.append(f"UNKNOWN({residual:#x})")
    return " | ".join(parts)


# Conservative completion timeouts [s], from config.h timing + docs/control-laws.md.
PROCEDURE_TIMEOUT_S = {
    ControllerState.MOTOR_RESISTANCE_CAL: 15.0,   # RESISTANCE_CAL_TIME = 3 s
    ControllerState.MOTOR_INDUCTANCE_CAL: 15.0,   # also 3 s (firmware uses RESISTANCE_CAL_TIME)
    ControllerState.ENCODER_DIR_CAL: 20.0,        # 1 s lock + 1 rev @ 0.5 rev/s
    ControllerState.ENCODER_OFFSET_CAL: 60.0,     # 4-rev up/down sweep
    ControllerState.ENCODER_VALIDATION: 60.0,     # 12 points x ~2 s + UART table dump
    ControllerState.SHITTY_OFFSET_CAL: 20.0,      # 4 s lock-in
    ControllerState.BACKEMF_CAL: 90.0,            # spinup <=10 s + settle + coast <=30 s
    ControllerState.BACKEMF_CAL_BIDIR: 150.0,     # two legs + drain; typically 15-60 s
    ControllerState.SAVE_CALIBRATION: 5.0,
    ControllerState.ERASE_CALIBRATION: 5.0,
    ControllerState.RESTORE_CALIBRATION: 5.0,
    ControllerState.CLEAR_ERRORS: 5.0,
    ControllerState.SELF_TEST: 5.0,               # ~0.15 s armed + ~50 ms UART report
}

# Procedures whose result lands AFTER the FSM is back in IDLE: the foreground finishes them
# (flash op / self-test report) tens of ms later. procedure_result is false until then.
DEFERRED_RESULT_STATES = {
    ControllerState.SAVE_CALIBRATION,
    ControllerState.ERASE_CALIBRATION,
    ControllerState.RESTORE_CALIBRATION,
    ControllerState.SELF_TEST,
}

# States that run forever until told otherwise -- run_procedure() refuses these.
NON_COMPLETING_STATES = {
    ControllerState.TORQUE_CONTROL,
    ControllerState.VEL_CONTROL,
    ControllerState.POS_CONTROL,
    ControllerState.VEL_RAMP_CONTROL,
    ControllerState.ENCODER_TEST,
    ControllerState.OPEN_LOOP_CONTROL,
}

VEL_RAMP_RATE = 20.0  # rev/s^2, config.h VEL_RAMP_RATE -- used to size spin() wait timeouts

STATE_ALIASES = {
    "idle": ControllerState.IDLE,
    "torque": ControllerState.TORQUE_CONTROL,
    "vel": ControllerState.VEL_CONTROL,
    "ramp": ControllerState.VEL_RAMP_CONTROL,
    "vel_ramp": ControllerState.VEL_RAMP_CONTROL,
    "resistance": ControllerState.MOTOR_RESISTANCE_CAL,
    "inductance": ControllerState.MOTOR_INDUCTANCE_CAL,
    "dir": ControllerState.ENCODER_DIR_CAL,
    "offset": ControllerState.SHITTY_OFFSET_CAL,      # the cal that actually applies
    "shitty": ControllerState.SHITTY_OFFSET_CAL,
    "offset_sweep": ControllerState.ENCODER_OFFSET_CAL,
    "enc_test": ControllerState.ENCODER_TEST,
    "validation": ControllerState.ENCODER_VALIDATION,
    "backemf": ControllerState.BACKEMF_CAL,
    "bidir": ControllerState.BACKEMF_CAL_BIDIR,
    "backemf_bidir": ControllerState.BACKEMF_CAL_BIDIR,
    "save": ControllerState.SAVE_CALIBRATION,
    "erase": ControllerState.ERASE_CALIBRATION,
    "restore": ControllerState.RESTORE_CALIBRATION,
    "clear": ControllerState.CLEAR_ERRORS,
    "selftest": ControllerState.SELF_TEST,
    "self_test": ControllerState.SELF_TEST,
}


def parse_state(text: str) -> ControllerState:
    """Accept a number, an fsm_state_t name (with or without FSM_STATE_), or an alias."""
    try:
        return ControllerState(int(text, 0))
    except ValueError:
        pass
    key = text.strip().lower()
    if key in STATE_ALIASES:
        return STATE_ALIASES[key]
    name = text.strip().upper().removeprefix("FSM_STATE_")
    try:
        return ControllerState[name]
    except KeyError:
        raise argparse.ArgumentTypeError(
            f"unknown state {text!r}; use a number, an FSM_STATE_* name, or one of: "
            + ", ".join(sorted(STATE_ALIASES)))


# ------------------------------------------------------------------ OD (hand-built)

def build_object_dictionary() -> canopen.ObjectDictionary:
    """0x4000-0x4007 block per docs/can-interface.md (no EDS carries it; OD.c is truth)."""
    od = canopen.ObjectDictionary()

    def var(name, index, sub, dtype, access="ro", unit="", factor=1.0):
        v = ODVariable(name, index, subindex=sub)
        v.data_type = dtype
        v.access_type = access
        v.unit = unit
        v.factor = factor
        return v

    rec = Record("rxw_ctrl_stat", 0x4000)
    rec.add_member(var("current_state", 0x4000, 1, UNSIGNED8))
    rec.add_member(var("procedure_result", 0x4000, 2, UNSIGNED8))
    rec.add_member(var("errors", 0x4000, 3, UNSIGNED32))
    od.add_object(rec)

    rec = Record("rxw_motor_vel_current", 0x4001)
    rec.add_member(var("motor_vel", 0x4001, 1, REAL32, unit="rev/s"))
    rec.add_member(var("motor_current", 0x4001, 2, REAL32, unit="A"))
    od.add_object(rec)

    rec = Record("rxw_bus_volt_current", 0x4002)
    rec.add_member(var("bus_voltage", 0x4002, 1, REAL32, unit="V"))
    rec.add_member(var("bus_current", 0x4002, 2, REAL32, unit="A"))
    od.add_object(rec)

    rec = Record("rxw_temp", 0x4003)
    for i in (1, 2, 3):
        rec.add_member(var(f"temp_{i}", 0x4003, i, INTEGER16, unit="C", factor=0.01))
    od.add_object(rec)

    rec = Record("rxw_requested_state", 0x4004)
    rec.add_member(var("req_state", 0x4004, 1, UNSIGNED8, access="rw"))
    od.add_object(rec)

    rec = Record("rxw_setpoint", 0x4005)
    rec.add_member(var("setpoint_1", 0x4005, 1, REAL32, access="rw"))
    rec.add_member(var("setpoint_2", 0x4005, 2, REAL32, access="rw"))
    od.add_object(rec)

    rec = Record("rxw_reserved", 0x4006)
    rec.add_member(var("reserved_1", 0x4006, 1, REAL32, access="rw"))
    rec.add_member(var("reserved_2", 0x4006, 2, REAL32, access="rw"))
    od.add_object(rec)

    rec = Record("rxw_reboot", 0x4007)
    rec.add_member(var("reboot_request", 0x4007, 1, UNSIGNED8, access="rw"))
    od.add_object(rec)

    return od


# ---------------------------------------------------------------- wheel object

class ReactionWheel:
    def __init__(self, network: canopen.Network, node_id: int):
        self.network = network
        self.node = network.add_node(node_id, build_object_dictionary())
        self.node.sdo.RESPONSE_TIMEOUT = 1.0

    # --- telemetry (SDO reads; TPDO2/3 fields only live in closed-loop states) ---

    @property
    def current_state(self) -> ControllerState:
        return ControllerState(self.node.sdo["rxw_ctrl_stat"]["current_state"].raw)

    @property
    def procedure_result(self) -> ProcedureResult:
        return ProcedureResult(self.node.sdo["rxw_ctrl_stat"]["procedure_result"].raw)

    @property
    def errors(self) -> ControllerErrors:
        return ControllerErrors(self.node.sdo["rxw_ctrl_stat"]["errors"].raw)

    @property
    def motor_vel(self) -> float:
        return float(self.node.sdo["rxw_motor_vel_current"]["motor_vel"].raw)

    @property
    def motor_current(self) -> float:
        """Measured iq [A]."""
        return float(self.node.sdo["rxw_motor_vel_current"]["motor_current"].raw)

    @property
    def bus_voltage(self) -> float:
        return float(self.node.sdo["rxw_bus_volt_current"]["bus_voltage"].raw)

    @property
    def bus_current(self) -> float:
        return float(self.node.sdo["rxw_bus_volt_current"]["bus_current"].raw)

    @property
    def temperature(self) -> tuple[float, float, float]:
        """(therm1, therm2, internal) [degC]."""
        rec = self.node.sdo["rxw_temp"]
        return tuple(float(rec[f"temp_{i}"].phys) for i in (1, 2, 3))

    # --- state requests ---

    @property
    def requested_state(self) -> ControllerState:
        """What 0x4004 currently holds. NB: firmware never rewrites it, so after a
        procedure auto-returns to IDLE this still reads the old request."""
        return ControllerState(self.node.sdo["rxw_requested_state"]["req_state"].raw)

    @requested_state.setter
    def requested_state(self, state):
        self.request_state(state)

    def request_state(self, state: ControllerState | int):
        """Request an FSM state. Every 0x4004 write is applied (OD write extension), so
        re-requesting the same state works. Bad requests come back as an SDO abort
        (canopen.SdoAbortedError): 0x06090030 = not a requestable state,
        0x08000022 = closed-loop state without encoder cal. (Older firmware edge-triggered
        on 0x4004 and needed a write of 0 first; current firmware aborts a write of 0.)"""
        state = ControllerState(int(state))
        self.node.sdo["rxw_requested_state"]["req_state"].raw = int(state)

    def idle(self):
        self.request_state(ControllerState.IDLE)

    def set_setpoint(self, value: float, channel: int = 1):
        """Write setpoint_1 (or _2). Routed by the FSM state active *at write time* --
        request the state first. Every write is applied, including a repeat of the
        same value."""
        self.node.sdo["rxw_setpoint"][f"setpoint_{channel}"].raw = float(value)

    @property
    def setpoint(self) -> float:
        return float(self.node.sdo["rxw_setpoint"]["setpoint_1"].raw)

    @setpoint.setter
    def setpoint(self, value: float):
        self.set_setpoint(value)

    # --- procedures (states that complete and auto-return to IDLE) ---

    def run_procedure(self, state: ControllerState | int, timeout: float | None = None,
                      poll_s: float = 0.1, quiet: bool = False) -> ProcedureResult:
        """Run a completing state and wait for the result.

        Requests the state, waits for the FSM to come back to IDLE (or fall into
        CONTROLLER_ERROR), then reads procedure_result before any further 0x4004
        traffic (the next procedure overwrites it). For DEFERRED_RESULT_STATES the
        result lands a little after IDLE, so it waits for that first."""
        state = ControllerState(int(state))
        if state in NON_COMPLETING_STATES:
            raise ValueError(f"{state.name} never completes; use request_state()/spin()")
        timeout = timeout if timeout is not None else PROCEDURE_TIMEOUT_S.get(state, 30.0)

        if state != ControllerState.CLEAR_ERRORS:
            errs = self.errors
            if errs:
                if not quiet:
                    print(f"  refusing {state.name}: errors latched: {fmt_errors(errs)}\n"
                          f"  (run 'clear' / clear_errors() first)")
                return ProcedureResult.FAIL

        if not quiet:
            print(f"  -> {state.name} (timeout {timeout:.0f} s)")
        self.request_state(state)
        time.sleep(0.15)  # let process_sdo_vals + the 1 kHz FSM tick actually enter it

        t0 = time.monotonic()
        last_note = t0
        while True:
            cur = self.current_state
            elapsed = time.monotonic() - t0
            if cur == ControllerState.IDLE:
                break
            if cur in (ControllerState.CONTROLLER_ERROR, ControllerState.SYSTEM_ERROR):
                if not quiet:
                    print(f"  {state.name} faulted after {elapsed:.1f} s: {fmt_errors(self.errors)}")
                return ProcedureResult.FAIL
            if elapsed > timeout:
                if not quiet:
                    print(f"  {state.name} timed out after {elapsed:.1f} s (still in {cur.name}); cancelling")
                self.request_state(ControllerState.IDLE)
                return ProcedureResult.FAIL
            if not quiet and time.monotonic() - last_note > 5.0:
                print(f"     ... still in {cur.name} ({elapsed:.0f} s)")
                last_note = time.monotonic()
            time.sleep(poll_s)

        if state in DEFERRED_RESULT_STATES:
            time.sleep(0.4)  # foreground flash op / self-test report finishes after IDLE
        result = self.procedure_result  # read BEFORE any further request traffic
        if not quiet:
            print(f"  <- {state.name} done in {time.monotonic() - t0:.1f} s: {result.name}")
        return result

    def clear_errors(self, quiet: bool = False) -> bool:
        self.run_procedure(ControllerState.CLEAR_ERRORS, quiet=quiet)
        ok = not self.errors
        if not quiet and not ok:
            print(f"  errors still latched after CLEAR_ERRORS: {fmt_errors(self.errors)}")
        return ok

    def save_calibration(self) -> ProcedureResult:
        return self.run_procedure(ControllerState.SAVE_CALIBRATION)

    def erase_calibration(self) -> ProcedureResult:
        return self.run_procedure(ControllerState.ERASE_CALIBRATION)

    def restore_calibration(self) -> ProcedureResult:
        return self.run_procedure(ControllerState.RESTORE_CALIBRATION)

    # --- first-time calibration (docs/backemf-coastdown-implementation.md runbook) ---

    COMMISSION_SEQUENCE = (
        (ControllerState.ENCODER_DIR_CAL,
         "encoder direction cal (1 s lock-in + 1 open-loop rev)"),
        (ControllerState.SHITTY_OFFSET_CAL,
         "bootstrap single-point offset cal (4 s lock-in)"),
        (ControllerState.BACKEMF_CAL_BIDIR,
         "bidirectional back-EMF self-cal (closed-loop spin both ways + coast; "
         "fits offset + LUT on-board; ~15-60 s)"),
        (ControllerState.SAVE_CALIBRATION,
         "save offset/dir/LUT to the flash cal page"),
    )

    def commission(self, reboot_after: bool = False, pause_s: float = 0.5) -> bool:
        """First-time calibration: 9 -> 15 -> 21 -> 18. Wheel must be free to spin.

        After this, every boot auto-restores the cal (boot UART prints
        'cal_store: restored offset ... dir ... +LUT') -- no recal needed.
        Acceptance check (once per wheel, optional): power-cycle, re-run state 21
        and confirm the UART-printed D is ~0 (+-~0.3 deg elec)."""
        errs = self.errors
        if errs:
            print(f"errors latched: {fmt_errors(errs)}\nclear them first (clear_errors())")
            return False

        n = len(self.COMMISSION_SEQUENCE)
        for i, (state, why) in enumerate(self.COMMISSION_SEQUENCE, 1):
            print(f"[{i}/{n}] {why}")
            if self.run_procedure(state) != ProcedureResult.SUCCESS:
                print(f"commissioning FAILED at {state.name}")
                return False
            time.sleep(pause_s)

        print("commissioning complete -- cal saved to flash, auto-restores at every boot")
        if reboot_after:
            print("rebooting to verify the boot-time restore path...")
            self.reboot()
            if not self.wait_until_responsive(timeout=15.0):
                print("node did not come back after reboot!")
                return False
            state, errs = self.current_state, self.errors
            print(f"node back: state={state.name}, errors={fmt_errors(errs)}")
            print("(restore result is on the boot UART: 'cal_store: restored ...')")
            return state == ControllerState.IDLE and not errs
        return True

    # --- motion helpers ---

    def spin(self, rev_s: float, wait: bool = True, quiet: bool = False) -> bool:
        """Closed-loop to rev_s using VEL_RAMP_CONTROL (firmware slews at VEL_RAMP_RATE)."""
        errs = self.errors
        if errs:
            if not quiet:
                print(f"refusing to spin: errors latched: {fmt_errors(errs)}")
            return False
        if self.current_state != ControllerState.VEL_RAMP_CONTROL:
            self.request_state(ControllerState.VEL_RAMP_CONTROL)
            t0 = time.monotonic()
            while self.current_state != ControllerState.VEL_RAMP_CONTROL:
                if time.monotonic() - t0 > 2.0:
                    if not quiet:
                        print(f"never entered VEL_RAMP_CONTROL (in {self.current_state.name})")
                    return False
                time.sleep(0.05)
        self.set_setpoint(float(rev_s))
        if not wait:
            return True
        tol = max(0.5, 0.02 * abs(rev_s))
        timeout = abs(rev_s) / VEL_RAMP_RATE + 5.0
        t0 = time.monotonic()
        while abs(self.motor_vel - rev_s) > tol:
            if self.current_state != ControllerState.VEL_RAMP_CONTROL:
                if not quiet:
                    print(f"dropped out of VEL_RAMP_CONTROL: {self.current_state.name}, "
                          f"errors: {fmt_errors(self.errors)}")
                return False
            if time.monotonic() - t0 > timeout:
                if not quiet:
                    print(f"vel {self.motor_vel:.2f} rev/s never reached {rev_s} rev/s")
                return False
            time.sleep(0.1)
        if not quiet:
            print(f"at {self.motor_vel:.2f} rev/s")
        return True

    def stop(self, to_idle: bool = True, quiet: bool = False):
        """Ramp velocity to zero (if in a velocity state) and optionally drop to IDLE."""
        cur = self.current_state
        if cur in (ControllerState.VEL_CONTROL, ControllerState.VEL_RAMP_CONTROL,
                   ControllerState.TORQUE_CONTROL):
            self.set_setpoint(0.0)
            timeout = abs(self.motor_vel) / VEL_RAMP_RATE + 5.0
            t0 = time.monotonic()
            while abs(self.motor_vel) > 0.5 and time.monotonic() - t0 < timeout:
                time.sleep(0.1)
        if to_idle:
            self.idle()
        if not quiet:
            print(f"stopped (state {self.current_state.name})")

    # --- misc ---

    def reboot(self):
        try:
            self.node.sdo["rxw_reboot"]["reboot_request"].raw = 1
        except (canopen.SdoCommunicationError, can.CanError):
            pass  # node can reset before the SDO response makes it out
        time.sleep(0.5)

    def wait_until_responsive(self, timeout: float = 10.0) -> bool:
        saved = self.node.sdo.RESPONSE_TIMEOUT
        self.node.sdo.RESPONSE_TIMEOUT = 0.3
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                try:
                    self.node.sdo["rxw_ctrl_stat"]["current_state"].raw
                    return True
                except (canopen.SdoCommunicationError, can.CanError):
                    # CanError: with nobody ACKing, the adapter's TX queue fills (ENOBUFS)
                    time.sleep(0.3)
            return False
        finally:
            self.node.sdo.RESPONSE_TIMEOUT = saved

    def status(self):
        state = self.current_state
        closed_loop = state in (ControllerState.TORQUE_CONTROL, ControllerState.VEL_CONTROL,
                                ControllerState.POS_CONTROL, ControllerState.VEL_RAMP_CONTROL)
        t1, t2, t3 = self.temperature
        print(f"node 0x{self.node.id:02X}")
        print(f"  state:            {state.name} ({state.value})")
        print(f"  procedure_result: {self.procedure_result.name}")
        print(f"  errors:           {fmt_errors(self.errors)}")
        note = "" if closed_loop else "   (only live in closed-loop states)"
        print(f"  vel / iq:         {self.motor_vel:8.3f} rev/s / {self.motor_current:6.3f} A{note}")
        print(f"  vbus / ibus:      {self.bus_voltage:8.3f} V     / {self.bus_current:6.3f} A{note}")
        print(f"  temps:            {t1:.2f} / {t2:.2f} / {t3:.2f} C  (therm1/therm2/internal)")


# ------------------------------------------------------------------ spin test

def spin_test(rxw: ReactionWheel, rev_s: float = 50.0, sample_s: float = 10.0,
              poll_s: float = 0.05):
    """Spin to rev_s, hold, sample telemetry, report mean/stdev, ramp back down.
    (Replaces the old host-side ramp -- firmware VEL_RAMP_CONTROL slews now.)"""
    print(f"spinning to {rev_s} rev/s...")
    if not rxw.spin(rev_s):
        return
    time.sleep(1.0)
    print(f"sampling for {sample_s:.0f} s...")
    bufs = {"vel [rev/s]": [], "iq [A]": [], "ibus [A]": [], "vbus [V]": []}
    t0 = time.monotonic()
    while time.monotonic() - t0 < sample_s:
        bufs["vel [rev/s]"].append(rxw.motor_vel)
        bufs["iq [A]"].append(rxw.motor_current)
        bufs["ibus [A]"].append(rxw.bus_current)
        bufs["vbus [V]"].append(rxw.bus_voltage)
        time.sleep(poll_s)
    for name, buf in bufs.items():
        sd = stdev(buf) if len(buf) > 1 else 0.0
        print(f"  {name:12s} mean {mean(buf):9.4f}   stdev {sd:8.4f}   n={len(buf)}")
    print("ramping down...")
    rxw.stop()


# -------------------------------------------------------------------- monitor

_NMT_STATES = {0: "BOOT-UP", 4: "STOPPED", 5: "OPERATIONAL", 127: "PRE-OPERATIONAL"}


def monitor(network: canopen.Network, node_id: int, channel: str, refresh_s: float = 0.5):
    """Live dashboard from raw TPDO frames (no SDO traffic, safe while armed).
    TPDOs are event-timer driven: 100 ms in active states, 5 s in IDLE."""
    rows: dict[str, tuple[bytes, float]] = {}

    def make_cb(key):
        def cb(can_id, data, ts):
            rows[key] = (bytes(data), time.monotonic())
        return cb

    cobs = {0x180 + node_id: "tpdo1", 0x280 + node_id: "tpdo2",
            0x380 + node_id: "tpdo3", 0x480 + node_id: "tpdo4",
            0x700 + node_id: "hb"}
    cbs = {cob: make_cb(key) for cob, key in cobs.items()}
    for cob, cb in cbs.items():
        network.subscribe(cob, cb)

    def field(key, fmt, parse):
        if key not in rows:
            return "(waiting)", ""
        data, ts = rows[key]
        age = f"{time.monotonic() - ts:5.1f} s ago"
        try:
            return fmt(*parse(data)), age
        except (struct.error, ValueError):
            return f"unparseable: {data.hex(' ')}", age

    try:
        while True:
            l1, a1 = field("tpdo1", lambda s, r, e:
                           f"state {ControllerState(s).name:<22s} result {ProcedureResult(r).name:<7s} "
                           f"errors {fmt_errors(e)}",
                           lambda d: struct.unpack("<BBI", d))
            l2, a2 = field("tpdo2", lambda v, i: f"vel {v:9.3f} rev/s        iq   {i:7.3f} A",
                           lambda d: struct.unpack("<ff", d))
            l3, a3 = field("tpdo3", lambda v, i: f"vbus {v:8.3f} V            ibus {i:7.3f} A",
                           lambda d: struct.unpack("<ff", d))
            l4, a4 = field("tpdo4", lambda a, b, c: f"temps {a/100:.2f} / {b/100:.2f} / {c/100:.2f} C",
                           lambda d: struct.unpack("<hhh", d))
            lhb, ahb = field("hb", lambda s: f"NMT {_NMT_STATES.get(s, f'?{s}')}",
                             lambda d: (d[0],))
            out = [
                f"ACS wheel monitor -- node 0x{node_id:02X} on {channel}   (Ctrl-C to exit)",
                "TPDO rate: 100 ms in active states, 5 s in IDLE; vel/iq/vbus/ibus only",
                "live in closed-loop states.",
                "",
                f"  {lhb:<70s} {ahb}",
                f"  {l1:<70s} {a1}",
                f"  {l2:<70s} {a2}",
                f"  {l3:<70s} {a3}",
                f"  {l4:<70s} {a4}",
            ]
            print("\x1b[H\x1b[2J" + "\n".join(out), flush=True)
            time.sleep(refresh_s)
    except KeyboardInterrupt:
        print()
    finally:
        for cob, cb in cbs.items():
            try:
                network.unsubscribe(cob, cb)
            except Exception:
                pass


# ----------------------------------------------------------------------- scan

def scan(network: canopen.Network):
    """Probe all four wheel node IDs with a short-timeout SDO read."""
    print("wheel  node  state")
    for wheel, nid in WHEEL_NODE_IDS.items():
        node = network.add_node(nid, build_object_dictionary())
        node.sdo.RESPONSE_TIMEOUT = 0.3
        try:
            state = ControllerState(node.sdo["rxw_ctrl_stat"]["current_state"].raw)
            errs = ControllerErrors(node.sdo["rxw_ctrl_stat"]["errors"].raw)
            extra = "" if not errs else f"   errors: {fmt_errors(errs)}"
            print(f"  {wheel}    0x{nid:02X}  {state.name}{extra}")
        except canopen.SdoCommunicationError:
            print(f"  {wheel}    0x{nid:02X}  (no response)")
        finally:
            try:
                del network[nid]
            except Exception:
                pass


# ----------------------------------------------------------------------- main

def main(argv=None):
    p = argparse.ArgumentParser(
        description="OreSat ACS reaction-wheel CAN console",
        epilog=__doc__.split("Usage:")[1],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--channel", default="can0", help="CAN channel (default: can0)")
    p.add_argument("--interface", default="socketcan", help="python-can interface (default: socketcan)")
    p.add_argument("--bitrate", type=int, default=1_000_000)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--wheel", type=int, choices=sorted(WHEEL_NODE_IDS),
                   help=f"wheel number (default: {DEFAULT_WHEEL})")
    g.add_argument("--node-id", type=lambda s: int(s, 0), help="raw CANopen node ID")

    sub = p.add_subparsers(dest="cmd")
    sub.add_parser("status", help="one-shot state/errors/telemetry (default)")
    sub.add_parser("monitor", help="live TPDO dashboard")
    sub.add_parser("scan", help="probe all four wheel node IDs")
    pc = sub.add_parser("commission", help="first-time cal: dir(9) -> offset(15) -> backemf-bidir(21) -> save(18)")
    pc.add_argument("--reboot", action="store_true", help="reboot afterwards to exercise the boot restore path")
    pc.add_argument("--yes", "-y", action="store_true", help="skip the are-you-sure prompt")
    pcal = sub.add_parser("cal", help="run a single completing procedure")
    pcal.add_argument("procedure", nargs="?", type=parse_state,
                      help="state number, FSM_STATE_* name, or alias (omit to list)")
    pst = sub.add_parser("state", help="request a raw FSM state (no wait)")
    pst.add_argument("state", type=parse_state)
    psp = sub.add_parser("setpoint", help="write setpoint_1 (request the state first!)")
    psp.add_argument("value", type=float)
    pspin = sub.add_parser("spin", help="closed-loop to a velocity via VEL_RAMP_CONTROL")
    pspin.add_argument("rev_s", type=float)
    pspin.add_argument("--no-wait", action="store_true")
    ptest = sub.add_parser("spintest", help="spin, sample telemetry, report stats, ramp down")
    ptest.add_argument("rev_s", type=float, nargs="?", default=50.0)
    ptest.add_argument("--seconds", type=float, default=10.0)
    sub.add_parser("stop", help="ramp to zero and drop to IDLE")
    sub.add_parser("idle", help="request IDLE")
    sub.add_parser("errors", help="read + decode latched error flags")
    sub.add_parser("clear", help="run CLEAR_ERRORS")
    sub.add_parser("reboot", help="write-1-to-reboot (0x4007)")
    sub.add_parser("shell", help="connect and stop -- for `python -i cli.py shell`")
    for name, helptext in (("selftest", "board self-test (state 22); details from the debug UART"),
                           ("bringup", "guided step-by-step bring-up of a new board (docs/board-bringup.md)")):
        pb = sub.add_parser(name, help=helptext)
        pb.add_argument("--uart", help="debug UART serial port (default: find it automatically)")
        pb.add_argument("--no-uart", action="store_true", help="don't open any serial port")
        if name == "bringup":
            pb.add_argument("--skip", help="comma-separated steps to skip: power,selftest,magnet,"
                                           "undervoltage,reboot,soak")

    args = p.parse_args(argv)
    cmd = args.cmd or "status"
    if args.node_id is not None:
        node_id = args.node_id
    elif args.wheel is not None:
        node_id = WHEEL_NODE_IDS[args.wheel]
    elif cmd in ("selftest", "bringup"):
        node_id = None  # find the one board on the bus
    else:
        node_id = WHEEL_NODE_IDS[DEFAULT_WHEEL]

    if cmd in ("selftest", "bringup"):
        try:
            import bringup  # noqa: F401  (pyserial is optional; bringup.py copes without it)
        except ImportError as e:
            sys.exit(f"could not load bringup.py next to cli.py: {e}")

    network = canopen.Network()
    try:
        network.connect(channel=args.channel, interface=args.interface, bitrate=args.bitrate)
    except Exception as e:  # python-can raises OSError / CanError subclasses
        sys.exit(f"could not open CAN interface {args.interface}:{args.channel}: {e}\n"
                 f"If it's a socketcan adapter, bring it up first (once per boot):\n"
                 f"    sudo ip link set {args.channel} up type can bitrate {args.bitrate}")

    try:
        if cmd == "scan":
            scan(network)
            return
        if cmd == "selftest":
            sys.exit(0 if bringup.cmd_selftest(sys.modules[__name__], network, args, node_id) else 1)
        if cmd == "bringup":
            sys.exit(0 if bringup.cmd_bringup(sys.modules[__name__], network, args, node_id) else 1)

        rxw = ReactionWheel(network, node_id)
        # leave handles around for `python -i cli.py ... shell`
        globals().update(network=network, node=rxw.node, rxw=rxw,
                         S=ControllerState, E=ControllerErrors)

        if cmd == "status":
            rxw.status()
        elif cmd == "monitor":
            monitor(network, node_id, args.channel)
        elif cmd == "commission":
            if not args.yes:
                input(f"About to commission node 0x{node_id:02X} (states 9 -> 15 -> 21 -> 18).\n"
                      "The wheel WILL spin both directions; it must be free. "
                      "Enter to continue, Ctrl-C to abort: ")
            ok = rxw.commission(reboot_after=args.reboot)
            sys.exit(0 if ok else 1)
        elif cmd == "cal":
            if args.procedure is None:
                completing = [s for s in ControllerState
                              if s in PROCEDURE_TIMEOUT_S]
                print("completing procedures:")
                for s in completing:
                    aliases = [k for k, v in STATE_ALIASES.items() if v == s]
                    print(f"  {s.value:3d}  {s.name:<24s} {' / '.join(aliases)}")
                return
            result = rxw.run_procedure(args.procedure)
            sys.exit(0 if result == ProcedureResult.SUCCESS else 1)
        elif cmd == "state":
            rxw.request_state(args.state)
            time.sleep(0.3)
            print(f"requested {args.state.name}; now in {rxw.current_state.name}")
        elif cmd == "setpoint":
            st = rxw.current_state
            rxw.set_setpoint(args.value)
            print(f"setpoint_1 = {args.value} (routed by current state: {st.name})")
        elif cmd == "spin":
            ok = rxw.spin(args.rev_s, wait=not args.no_wait)
            sys.exit(0 if ok else 1)
        elif cmd == "spintest":
            spin_test(rxw, args.rev_s, sample_s=args.seconds)
        elif cmd == "stop":
            rxw.stop()
        elif cmd == "idle":
            rxw.idle()
            print(f"now in {rxw.current_state.name}")
        elif cmd == "errors":
            print(fmt_errors(rxw.errors))
        elif cmd == "clear":
            sys.exit(0 if rxw.clear_errors() else 1)
        elif cmd == "reboot":
            rxw.reboot()
            if rxw.wait_until_responsive(timeout=15.0):
                print(f"node back: state={rxw.current_state.name}, errors={fmt_errors(rxw.errors)}")
            else:
                print("node did not come back within 15 s")
        elif cmd == "shell":
            rxw.status()
            print("\nhandles: rxw, node, network, S=ControllerState, E=ControllerErrors")
            print("e.g.  rxw.commission()   rxw.spin(50)   rxw.stop()   spin_test(rxw, 50)")
            return  # keep the network up for python -i
    except canopen.SdoAbortedError as e:
        sys.exit(f"the wheel refused the request: {e}")
    except (canopen.SdoCommunicationError, can.CanError) as e:
        where = f"node 0x{node_id:02X}" if node_id is not None else "the wheel"
        sys.exit(f"no answer from {where}: {e} "
                 f"(is the node powered and {args.channel} up?)")
    finally:
        if cmd != "shell":
            try:
                network.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
