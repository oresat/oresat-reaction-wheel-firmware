"""Guided desk bring-up for one reaction-wheel board, and the state-22 self-test runner.

Run it through cli.py (it needs cli.py's CAN plumbing):

    python3 cli.py bringup              # step-by-step, writes a log file
    python3 cli.py selftest             # just the self-test, prints the table

Operator guide: docs/board-bringup.md. Firmware side: Firmware/Src/self_test.c,
docs/control-laws.md (state 22).

Needs `pip install canopen pyserial`. The debug UART (USART2, 921600 8N1) carries the
self-test details; this module finds the right serial port by itself (the one that prints the
boot banner or a SELFTEST line).
"""
from __future__ import annotations

import datetime as dt
import logging
import re
import select
import struct
import sys
import threading
import time
import traceback
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # only `selftest`/`bringup` need it; cli.py checks before calling in
    serial = None
    list_ports = None

import can
import canopen
from canopen.sdo.exceptions import SdoError

# Anything that means "the wheel didn't answer" or "the bus refused the frame". can.CanError
# covers the socketcan TX queue filling up (ENOBUFS) when nobody ACKs: a powered-down board.
COMM_ERRORS = (SdoError, can.CanError)

# canopen logs every SDO timeout as a warning; probing empty node IDs would spam the operator.
logging.getLogger("canopen").setLevel(logging.ERROR)

DEBUG_BAUD = 921600
LOG_DIR = Path(__file__).resolve().parent / "bringup_logs"

# What the firmware's check ids mean, in plain words (Firmware/Src/self_test.c).
CHECK_NAMES = {
    "clock": "Clock runs from the crystal",
    "option_bytes": "Option bytes",
    "node_id": "Node-ID straps",
    "vref": "ADC reference (3.0 V)",
    "temps": "Temperature sensors",
    "encoder_spi": "Encoder chip answers",
    "magnet": "Magnet field at the encoder",
    "can": "CAN error counters",
    "adc_mid": "Current-sense zero points",
    "vbus": "Bus voltage",
    "bridge_off": "Bridge off: nothing moving/leaking",
    "encoder_angle": "Encoder angle while idle",
    "bridge": "Bridge drive test",
    "bridge_A": "Half-bridge A switches",
    "bridge_B": "Half-bridge B switches",
    "bridge_C": "Half-bridge C switches",
    "bridge_current": "No current while switching",
    "bridge_vbus": "Bus holds while switching",
    "errors": "Controller errors",
    "boot": "Last reset",
    "boot0": "BOOT0 pin",
    "encoder_cal": "Encoder calibration",
    "fast_loop": "Fast-loop timing",
}

# What to check when a line FAILs. These are likely causes, not a diagnosis.
CHECK_HINTS = {
    "clock": "The crystal isn't running. Check the crystal and its two small capacitors.",
    "option_bytes": "Reflash with ./flash-release.sh, which sets the option bytes.",
    "node_id": "Check the CAN_ADDR0/CAN_ADDR1 strap resistors or jumpers.",
    "vref": "Check the 3.0 V reference chip (VREF+) and its capacitor.",
    "temps": "The named thermistor reads open or shorted. Check it and its resistor.",
    "encoder_spi": "The encoder chip doesn't answer. Check it is fitted the right way round "
                   "and soldered (SPI pins, power, decoupling cap).",
    "can": "Check CANH/CANL wiring and that the bus has 120-ohm termination at both ends.",
    "adc_mid": "That phase's current-sense amplifier output is off-center. Check the INA296 "
               "on that phase and its reference pin.",
    "vbus": "Check the supply is set to about 8 V, and the bus-voltage divider resistors.",
    "bridge_off": "If a motor is attached, make sure it is not turning. If nothing is turning, "
                  "the named phase leaks to the bus: suspect a shorted high-side transistor.",
    "bridge": "See the other bridge lines. A sag or current means a possible short: "
              "do not raise the supply current limit, send the log to Solomon.",
    "bridge_A": "Half-bridge A doesn't switch. Check the LMG2100 on phase A (soldering, "
                "bootstrap capacitor, the 2.7-ohm resistors).",
    "bridge_B": "Half-bridge B doesn't switch. Check the LMG2100 on phase B (soldering, "
                "bootstrap capacitor, the 2.7-ohm resistors).",
    "bridge_C": "Half-bridge C doesn't switch. Check the LMG2100 on phase C (soldering, "
                "bootstrap capacitor, the 2.7-ohm resistors).",
    "bridge_current": "Disconnect any motor and rerun. If it stays, look for a short between phases.",
    "bridge_vbus": "Possible shoot-through (shorted transistor), or the supply current limit "
                   "is too low. Don't raise the limit; send the log to Solomon.",
    "errors": "Send the log to Solomon.",
}

SELFTEST_LINE = re.compile(r"SELFTEST (PASS|FAIL|WARN|SKIP|INFO) (\w+): (.*)")
SELFTEST_END = r"SELFTEST END (PASS|FAIL)"
BOOT_BANNER = r"THIS FIRMWARE IS DEDICATED|Init complete!"
_ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


class FirmwareTooOld(Exception):
    pass


# ------------------------------------------------------------------- logging + UI

class Log:
    def __init__(self, path: Path | None):
        self.path = path
        self._lock = threading.Lock()
        self._f = None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._f = open(path, "w", buffering=1, encoding="utf-8")

    def write(self, text: str):
        if self._f is None:
            return
        stamp = dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self._lock:
            for line in text.splitlines() or [""]:
                self._f.write(f"{stamp} {line}\n")

    def uart(self, port: str, text: str):
        self.write(f"uart {port} | {text}")

    def rename(self, new_path: Path):
        if self._f is None or self.path is None:
            return
        with self._lock:
            self._f.close()
            self.path.rename(new_path)
            self.path = new_path
            self._f = open(new_path, "a", buffering=1, encoding="utf-8")

    def close(self):
        if self._f is not None:
            self._f.close()
            self._f = None


class Ui:
    COLORS = {"PASS": "32", "FAIL": "31;1", "WARN": "33", "SKIP": "2", "INFO": "2"}

    def __init__(self, log: Log):
        self.log = log
        self.color = sys.stdout.isatty()

    def _c(self, text: str, code: str) -> str:
        return f"\x1b[{code}m{text}\x1b[0m" if self.color else text

    def say(self, text: str = ""):
        print(text, flush=True)
        self.log.write("    " + text)

    def heading(self, text: str):
        print()
        print(self._c(text, "1"), flush=True)
        self.log.write("")
        self.log.write("=== " + text)

    def verdict_word(self, v: str) -> str:
        return self._c(f"{v:4s}", self.COLORS.get(v, "0"))

    def result(self, v: str, name: str, detail: str = ""):
        print(f"  {self.verdict_word(v)}  {name}" + (f"  -- {detail}" if detail else ""), flush=True)
        self.log.write(f"RESULT {v} {name}" + (f" -- {detail}" if detail else ""))

    def ask(self, prompt: str) -> str:
        self.log.write(f"ASK {prompt}")
        try:
            ans = input(self._c("> ", "36") + prompt + " ").strip()
        except EOFError:
            ans = ""
        self.log.write(f"ANSWER {ans!r}")
        return ans

    def yes(self, prompt: str, default: bool = True) -> bool:
        ans = self.ask(prompt + (" [Y/n]" if default else " [y/N]")).lower()
        return default if not ans else ans.startswith("y")

    def enter(self, prompt: str):
        self.ask(prompt + " (press Enter)")


def flush_stdin():
    """Drop type-ahead (a double Enter at the previous prompt) before polling enter_pressed()."""
    if not sys.stdin.isatty():
        return
    try:
        import termios
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except (ImportError, OSError):
        pass


def enter_pressed() -> bool:
    """Non-blocking: True if the operator pressed Enter (POSIX terminals only)."""
    if not sys.stdin.isatty():
        return False  # piped/closed stdin reads as "ready" forever
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
    except (ValueError, OSError):
        return False
    if r:
        sys.stdin.readline()
        return True
    return False


# --------------------------------------------------------------------- debug UART

def uart_candidates(exclude=()) -> list[str]:
    """Serial ports that might be the wheel's debug UART (USB serial adapters, ST-Link VCP)."""
    if list_ports is None:
        return []
    out = []
    for p in list_ports.comports():
        dev = p.device
        if dev in exclude:
            continue
        if sys.platform.startswith("linux") and not re.search(r"tty(ACM|USB)\d+$", dev):
            continue
        if "bluetooth" in dev.lower():
            continue
        out.append(dev)
    return sorted(out)


class UartBank:
    """Listens on every candidate port at once. The port that prints the boot banner or a
    SELFTEST line is the debug UART; lock_to() then closes the others."""

    def __init__(self, ports: list[str], log: Log):
        self._lock = threading.Lock()
        self._lines: list[tuple[float, str, str]] = []  # (monotonic time, port, text)
        self._ports: dict[str, "serial.Serial"] = {}
        self._stop = threading.Event()
        self.failed: dict[str, str] = {}
        self.log = log
        self.active: str | None = None
        for p in ports:
            try:
                s = serial.Serial(p, DEBUG_BAUD, timeout=0.2)
                s.reset_input_buffer()
            except (serial.SerialException, OSError) as e:
                self.failed[p] = str(e)
                continue
            self._ports[p] = s
            threading.Thread(target=self._reader, args=(p, s), daemon=True).start()

    @property
    def opened(self) -> list[str]:
        return list(self._ports)

    def _reader(self, port: str, ser):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = ser.read(512)
            except (serial.SerialException, OSError, TypeError, AttributeError):
                return  # port closed by lock_to()/close(), or unplugged
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = _ANSI.sub("", raw.decode("utf-8", "replace")).rstrip("\r")
                with self._lock:
                    self._lines.append((time.monotonic(), port, text))
                self.log.uart(port, text)

    def mark(self) -> int:
        with self._lock:
            return len(self._lines)

    def _items(self, since: int):
        with self._lock:
            items = self._lines[since:]
        return [it for it in items if self.active is None or it[1] == self.active]

    def find(self, pattern: str, since: int = 0):
        rx = re.compile(pattern)
        for it in self._items(since):
            if rx.search(it[2]):
                return it
        return None

    def wait_for(self, pattern: str, since: int, timeout: float):
        deadline = time.monotonic() + timeout
        while True:
            hit = self.find(pattern, since)
            if hit is not None or time.monotonic() > deadline:
                return hit
            time.sleep(0.05)

    def lines_since(self, since: int) -> list[str]:
        return [it[2] for it in self._items(since)]

    def lock_to(self, port: str):
        self.active = port
        for p, s in list(self._ports.items()):
            if p != port:
                try:
                    s.close()
                except Exception:
                    pass
                del self._ports[p]

    def close(self):
        self._stop.set()
        for s in self._ports.values():
            try:
                s.close()
            except Exception:
                pass


def open_uart(args, log: Log, ui: Ui | None) -> UartBank | None:
    if getattr(args, "no_uart", False):
        return None
    if serial is None:
        msg = ("pyserial is not installed, so the self-test details can't be read.\n"
               "Install it with:  pip install pyserial")
        if ui:
            ui.say(msg)
        else:
            print(msg)
        return None
    exclude = (args.channel,) if args.interface == "slcan" else ()
    ports = [args.uart] if args.uart else uart_candidates(exclude)
    bank = UartBank(ports, log)
    if args.uart and bank.opened:
        bank.lock_to(args.uart)
    return bank


# ----------------------------------------------------------------- CAN helpers

def read_u32(node, index: int, sub: int) -> int:
    return struct.unpack("<I", node.sdo.upload(index, sub)[:4])[0]


def boot_diagnostics(node) -> dict:
    """0x4006:01 = RCC_CSR flags [31:24] | crash kind [19:16] | boot count [15:0]."""
    w = read_u32(node, 0x4006, 1)
    flags = w >> 24
    names = {0x80: "LPWR", 0x40: "WWDG", 0x20: "IWDG", 0x10: "SOFT", 0x08: "BOR",
             0x04: "PIN", 0x02: "OBL"}
    return {
        "boot_count": w & 0xFFFF,
        "crash_kind": (w >> 16) & 0xF,
        "reset": [n for bit, n in names.items() if flags & bit],
    }


class Heartbeats:
    """Records NMT heartbeats/boot-ups from the four wheel node IDs (0x700 + id)."""

    def __init__(self, network, node_ids):
        self.network = network
        self.events: list[tuple[float, int, int]] = []  # (monotonic, node id, NMT state)
        self._lock = threading.Lock()
        self._subs = []
        for nid in node_ids:
            cb = self._make_cb(nid)
            network.subscribe(0x700 + nid, cb)
            self._subs.append((0x700 + nid, cb))

    def _make_cb(self, nid):
        def cb(can_id, data, ts):
            with self._lock:
                self.events.append((time.monotonic(), nid, data[0] if data else -1))
        return cb

    def mark(self) -> int:
        with self._lock:
            return len(self.events)

    def since(self, mark: int):
        with self._lock:
            return list(self.events[mark:])

    def close(self):
        for cob, cb in self._subs:
            try:
                self.network.unsubscribe(cob, cb)
            except Exception:
                pass


class Emcy:
    def __init__(self, network, node_id):
        self.network = network
        self.cob = 0x80 + node_id
        self.frames: list[bytes] = []
        self._cb = lambda can_id, data, ts: self.frames.append(bytes(data))
        network.subscribe(self.cob, self._cb)

    def close(self):
        try:
            self.network.unsubscribe(self.cob, self._cb)
        except Exception:
            pass


def find_single_node(cli, network, ui_say=print) -> int | None:
    """Probe the four wheel IDs; return the one node that answers, else None."""
    found = []
    for wheel, nid in cli.WHEEL_NODE_IDS.items():
        node = network.add_node(nid, cli.build_object_dictionary())
        node.sdo.RESPONSE_TIMEOUT = 0.3
        try:
            node.sdo["rxw_ctrl_stat"]["current_state"].raw
            found.append(nid)
        except COMM_ERRORS:
            pass
        finally:
            try:
                del network[nid]
            except Exception:
                pass
    if len(found) == 1:
        return found[0]
    if not found:
        ui_say("No wheel answered on the CAN bus (tried node IDs 0x3C, 0x40, 0x44, 0x48).")
    else:
        ui_say("More than one wheel answered: " + ", ".join(f"0x{n:02X}" for n in found)
               + ". Leave only the board under test on the bus, or pass --wheel N.")
    return None


# ------------------------------------------------------- encoder motion vs noise

def steady(prev: float | None, v: float) -> bool:
    """One step of a 'steady turning' run. With no magnet over it the MA732 outputs noise, so the
    disarmed velocity (0x4001, a 5 ms difference) jumps around with random sign and size. A hand
    turn, or a coasting rotor, gives a velocity that keeps its sign and changes smoothly between
    20 Hz polls. Noise passes this test at a few-percent rate per step, so a run of 10 is ~1e-9."""
    if not 0.2 < abs(v) < 30.0:
        return False
    if prev is None:
        return True
    return (v > 0) == (prev > 0) and abs(v - prev) < max(0.5, 0.5 * abs(prev))


def looks_turning(rxw, samples: int = 12, period: float = 0.05) -> bool:
    """True if the encoder shows a steady rotation of >= 1 rev/s (noise doesn't count)."""
    run, prev = 0, None
    for _ in range(samples):
        try:
            v = rxw.motor_vel
        except COMM_ERRORS:
            v = 0.0
        run = run + 1 if (steady(prev, v) and abs(v) >= 1.0) else 0
        prev = v
        if run >= 8:
            return True
        time.sleep(period)
    return False


# ----------------------------------------------------------------- self-test

def run_selftest(cli, rxw, uart: UartBank | None, ui: Ui | None):
    """Request state 22, collect its report. Returns (overall 'PASS'/'FAIL', lines, note).
    lines = [(verdict, check id, text)]; empty if no debug UART output was seen."""
    S = cli.ControllerState
    say = ui.say if ui else print

    errs = rxw.errors
    if errs:
        say(f"Errors were latched before the test ({cli.fmt_errors(errs)}); clearing them first.")
        rxw.clear_errors(quiet=True)

    mark = uart.mark() if uart else 0
    try:
        rxw.request_state(S.SELF_TEST)
    except canopen.SdoAbortedError as e:
        if e.code == 0x06090030:
            raise FirmwareTooOld() from e
        raise

    end = None
    if uart is not None and uart.opened:
        end = uart.wait_for(SELFTEST_END, mark, timeout=6.0)
        if end is not None and uart.active is None:
            uart.lock_to(end[1])

    # The FSM is back in IDLE (or CONTROLLER_ERROR) well before the report ends; the result
    # latches when the report finishes, so read it after that.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        if rxw.current_state in (S.IDLE, S.CONTROLLER_ERROR):
            break
        time.sleep(0.1)
    time.sleep(0.4)
    result = rxw.procedure_result

    lines = []
    if end is not None:
        for text in uart.lines_since(mark):
            m = SELFTEST_LINE.search(text)
            if m:
                lines.append((m.group(1), m.group(2), m.group(3)))
        overall = re.search(SELFTEST_END, end[2]).group(1)
        note = ""
        if (overall == "PASS") != (result == cli.ProcedureResult.SUCCESS):
            note = (f"UART says {overall} but procedure_result over CAN says {result.name}; "
                    "trusting the UART report")
    else:
        overall = "PASS" if result == cli.ProcedureResult.SUCCESS else "FAIL"
        note = "no self-test report on any serial port; result from CAN procedure_result only"
    return overall, lines, note


def print_selftest(lines, ui: Ui | None):
    out = ui.result if ui else (lambda v, n, d="": print(f"  {v:4s}  {n}" + (f"  -- {d}" if d else "")))
    for v, cid, text in lines:
        out(v, CHECK_NAMES.get(cid, cid), text)
    fails = [(cid, text) for v, cid, text in lines if v == "FAIL"]
    if fails:
        say = ui.say if ui else print
        say("")
        say("What to check:")
        for cid, _ in fails:
            hint = CHECK_HINTS.get(cid, "Send the log to Solomon.")
            say(f"  - {CHECK_NAMES.get(cid, cid)}: {hint}")


def cmd_selftest(cli, network, args, node_id: int | None) -> bool:
    if node_id is None:
        node_id = find_single_node(cli, network)
        if node_id is None:
            return False
    log = Log(None)
    uart = open_uart(args, log, None)
    try:
        rxw = cli.ReactionWheel(network, node_id)
        print(f"Running the self-test on node 0x{node_id:02X} (takes about a second)...")
        try:
            overall, lines, note = run_selftest(cli, rxw, uart, None)
        except FirmwareTooOld:
            print("This board's firmware has no self-test (state 22). Flash a newer build first.")
            return False
        if lines:
            print_selftest(lines, None)
        if note:
            print(f"note: {note}")
        print(f"\nSELF-TEST {overall}")
        return overall == "PASS"
    finally:
        if uart:
            uart.close()


# ------------------------------------------------------------------- bring-up

class Bringup:
    STEPS = ("power", "selftest", "magnet", "undervoltage", "reboot", "soak")

    def __init__(self, cli, network, args, node_id: int | None):
        self.cli = cli
        self.network = network
        self.args = args
        self.node_id = node_id
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.log = Log(LOG_DIR / f"bringup_{stamp}.log")
        self.ui = Ui(self.log)
        self.results: list[tuple[str, str, str]] = []  # (verdict, step, detail)
        self.uart: UartBank | None = None
        self.rxw = None
        self.serial_no: int | None = None
        self.skip = set(s.strip() for s in (args.skip or "").split(",") if s.strip())
        self.selftest_overall: str | None = None
        self.selftest_fails: set[str] = set()

    def record(self, v: str, name: str, detail: str = ""):
        self.results.append((v, name, detail))
        self.ui.result(v, name, detail)

    # --- steps ---

    def step_power(self) -> bool:
        ui = self.ui
        ui.heading("Step 1: power up and find the board")
        ui.say("Set the bench supply to 8.0 V with the current limit at 0.5 A.")
        ui.enter("Turn the supply OFF")
        hb = Heartbeats(self.network, self.cli.WHEEL_NODE_IDS.values())
        hb_mark = hb.mark()
        u_mark = self.uart.mark() if self.uart else 0
        ui.say("Now turn the supply ON. Waiting up to 15 s for the board to start...")
        t0 = time.monotonic()
        node = None
        booted = False  # saw the CANopen boot-up frame (NMT state 0): a real power-on
        banner = None
        while time.monotonic() - t0 < 15.0:
            for _, nid, state in hb.since(hb_mark):
                if state == 0:
                    node, booted = nid, True
                elif node is None:
                    node = nid
            if self.uart and banner is None:
                banner = self.uart.find(r"Init complete!", u_mark)
            if booted and (banner is not None or not self.uart or not self.uart.opened):
                break
            if booted and time.monotonic() - t0 > 8.0:
                break  # CAN is up; don't hold the operator hostage for the UART
            time.sleep(0.1)
        hb.close()

        if self.uart is not None:
            hit = banner or self.uart.find(BOOT_BANNER, u_mark)
            if hit is not None:
                if self.uart.active is None:
                    self.uart.lock_to(hit[1])
                ui.say(f"Debug UART found on {hit[1]}.")
            else:
                ui.say("No boot messages on any serial port "
                       f"(listened on: {', '.join(self.uart.opened) or 'none'}).")

        if node is None:
            if self.uart and self.uart.find(BOOT_BANNER, u_mark):
                self.record("FAIL", "Board talks on CAN",
                            "it boots (UART shows the banner) but sends no CAN heartbeat: "
                            "check the CAN cable, adapter and termination")
            else:
                self.record("FAIL", "Board starts",
                            "no CAN heartbeat and no boot messages: check power, the supply "
                            "current, and that the board is flashed")
            return False

        self.node_id = node
        wheel = next(w for w, n in self.cli.WHEEL_NODE_IDS.items() if n == node)
        if booted:
            self.record("PASS", "Board starts and talks on CAN", f"wheel {wheel}, node 0x{node:02X}")
        else:
            self.record("WARN", "Board starts and talks on CAN",
                        f"wheel {wheel}, node 0x{node:02X} answers, but no power-on was seen (it was "
                        "already running), so the boot log wasn't captured")

        # Boot banner problems the self-test can't see as clearly
        if self.uart and self.uart.active:
            text = "\n".join(self.uart.lines_since(u_mark))
            for pat, what in ((r"Amp offset cal (REJECTED|TIMED OUT)[^\n]*", "current-sense zero"),
                              (r"CORDIC FAIL", "CORDIC math unit"),
                              (r"Option bytes still mismatched[^\n]*", "option bytes"),
                              (r"MA732 FW write FAILED[^\n]*", "encoder setup"),
                              (r"LAST FAULT[^\n]*", "a crash record from before")):
                m = re.search(pat, text)
                if m:
                    self.record("WARN", f"Boot log: {what}", m.group(0).strip())

        ans = self.ui.ask("Which wheel number should this board be (1-4)? Press Enter if you don't know.")
        if ans in ("1", "2", "3", "4"):
            want = int(ans)
            if want == wheel:
                self.record("PASS", "Node ID matches the straps", f"wheel {wheel}")
            else:
                self.record("FAIL", "Node ID matches the straps",
                            f"you expected wheel {want}, the straps say wheel {wheel}: "
                            "check the CAN_ADDR0/1 straps")
        ma = self.ui.ask("What current does the supply show now, in mA? Press Enter to skip.")
        if ma:
            self.record("INFO", "Idle supply current", f"{ma} mA at 8 V (operator reading)")
        return True

    def ensure_node(self) -> bool:
        if self.node_id is None:
            self.node_id = find_single_node(self.cli, self.network, self.ui.say)
        if self.node_id is None:
            return False
        if self.rxw is None:
            self.rxw = self.cli.ReactionWheel(self.network, self.node_id)
            try:
                self.serial_no = read_u32(self.rxw.node, 0x1018, 4)
                self.ui.say(f"Board serial (from the chip ID): 0x{self.serial_no:08X}")
            except COMM_ERRORS:
                pass
        return True

    def step_selftest(self):
        ui = self.ui
        ui.heading("Step 2: self-test")
        ui.say("The board checks itself: clock, sensors, encoder chip, current sense, and each")
        ui.say("half-bridge switching low and high. About one second. Nothing should be connected")
        ui.say("to the motor phases.")
        try:
            overall, lines, note = run_selftest(self.cli, self.rxw, self.uart, ui)
        except FirmwareTooOld:
            self.record("FAIL", "Self-test", "this firmware has no self-test (state 22): flash a newer build")
            return
        self.selftest_overall = overall
        self.selftest_fails = {cid for v, cid, _ in lines if v == "FAIL"}
        if lines:
            print_selftest(lines, ui)
        if note:
            ui.say(f"note: {note}")
        self.record(overall, "Self-test", f"{sum(1 for v, _, _ in lines if v == 'FAIL')} failed checks"
                    if lines else "")

    def step_magnet(self):
        ui = self.ui
        S = self.cli.ControllerState
        ui.heading("Step 3: encoder with a magnet")
        if not ui.yes("Do you have a magnet to hold over the encoder chip (MA732)?", default=True):
            self.record("SKIP", "Encoder sees a turning magnet", "no magnet")
            return
        if self.rxw.current_state != S.IDLE:
            self.rxw.idle()
        ui.say("Hold the magnet flat over the encoder chip, a few mm above it.")
        ui.say("Turn it slowly one way, then the other way. You have 30 seconds.")
        seen = {+1: False, -1: False}
        run_len, prev = 0, None
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < 30.0 and not all(seen.values()):
            try:
                v = self.rxw.motor_vel
            except COMM_ERRORS:
                time.sleep(0.05)
                continue
            # Half a second of smooth, same-sign turning counts; noise can't fake it (steady()).
            run_len = run_len + 1 if steady(prev, v) else 0
            prev = v
            if run_len >= 10:
                seen[1 if v > 0 else -1] = True
            if time.monotonic() - last_print > 1.0:
                last_print = time.monotonic()
                print(f"\r    now {v:+7.2f} rev/s   one way: {'yes' if seen[+1] else 'no '}   "
                      f"other way: {'yes' if seen[-1] else 'no '}", end="", flush=True)
            time.sleep(0.05)
        print()
        if all(seen.values()):
            self.record("PASS", "Encoder sees a turning magnet", "both directions")
        else:
            self.record("FAIL", "Encoder sees a turning magnet",
                        "never saw steady turning both ways. Use a diametrically magnetized "
                        "magnet, centered over the chip, a few mm away")

    def step_undervoltage(self):
        ui = self.ui
        cli = self.cli
        S = cli.ControllerState
        E = cli.ControllerErrors
        rxw = self.rxw
        ui.heading("Step 4: undervoltage shut-off")
        ui.say("This checks the board turns its outputs off when the supply drops below 5 V.")
        ui.say("The trip only runs while the outputs are on, so the board switches them on with")
        ui.say("no voltage across the phases (encoder-test mode). No current flows.")
        if self.selftest_overall == "FAIL":
            bad = {c for c in self.selftest_fails
                   if c.startswith("bridge") or c in ("errors", "adc_mid", "vbus")}
            if bad or not self.selftest_fails:  # no details (no UART): don't guess
                self.record("SKIP", "Undervoltage shut-off",
                            "the self-test failed on the bridge or current sense, so the outputs "
                            "stay off. Fix that first")
                return
        if not ui.yes("Ready?", default=True):
            self.record("SKIP", "Undervoltage shut-off", "skipped by operator")
            return
        if looks_turning(rxw):
            self.record("SKIP", "Undervoltage shut-off", "something is turning; stop it first")
            return
        if rxw.errors:
            rxw.clear_errors(quiet=True)
        hb = Heartbeats(self.network, [self.node_id])
        emcy = Emcy(self.network, self.node_id)
        try:
            before = boot_diagnostics(rxw.node)["boot_count"]
            rxw.request_state(S.ENCODER_TEST)
            time.sleep(0.3)
            if rxw.current_state != S.ENCODER_TEST:
                self.record("FAIL", "Undervoltage shut-off",
                            f"could not switch the outputs on (state {rxw.current_state.name})")
                return
            hb_mark = hb.mark()
            ui.say("Now turn the supply voltage DOWN slowly (about 0.1 V per second) toward 4.5 V.")
            ui.say("It should trip just below 5.0 V. If you reach 4.5 V and nothing happens, press Enter.")
            tripped, rebooted, gave_up = False, False, False
            flush_stdin()
            t0 = time.monotonic()
            last_answer = t0
            while time.monotonic() - t0 < 120.0:
                if any(state == 0 for _, _, state in hb.since(hb_mark)):
                    rebooted = True
                    break
                try:
                    st, errs = rxw.current_state, rxw.errors
                    last_answer = time.monotonic()
                except COMM_ERRORS:
                    if time.monotonic() - last_answer > 3.0:
                        rebooted = True  # silent for 3 s: its own supply dropped out
                        break
                    time.sleep(0.2)
                    continue
                if st == S.CONTROLLER_ERROR and errs & E.CTRLR_ERR_VBUS_UNDERVOLTAGE:
                    tripped = True
                    break
                if st != S.ENCODER_TEST:
                    break
                if enter_pressed():
                    gave_up = True
                    break
                time.sleep(0.1)
            if tripped:
                self.record("PASS", "Undervoltage shut-off",
                            "tripped; " + ("EMCY frame seen" if emcy.frames else "no EMCY frame seen"))
            elif rebooted:
                self.record("WARN", "Undervoltage shut-off",
                            "the board reset or went silent (its own supply dropped out) before "
                            "the 5 V trip")
            elif gave_up:
                self.record("FAIL", "Undervoltage shut-off", "no trip by 4.5 V")
            else:
                self.record("FAIL", "Undervoltage shut-off",
                            f"left encoder-test mode without the trip (state {rxw.current_state.name}, "
                            f"errors {cli.fmt_errors(rxw.errors)})")
            ui.enter("Turn the supply back UP to 8.0 V")
            if not rxw.wait_until_responsive(timeout=15.0):
                self.record("FAIL", "Recovers after undervoltage", "no CAN answer after the supply came back")
                return
            if rxw.current_state not in (S.IDLE,) or rxw.errors:
                rxw.clear_errors(quiet=True)
            if rxw.current_state != S.IDLE:
                rxw.idle()
                time.sleep(0.3)
            ok = rxw.current_state == S.IDLE and not rxw.errors
            after = boot_diagnostics(rxw.node)["boot_count"]
            self.record("PASS" if ok else "FAIL", "Recovers after undervoltage",
                        ("back in IDLE, errors cleared" if ok else
                         f"state {rxw.current_state.name}, errors {cli.fmt_errors(rxw.errors)}")
                        + ("" if after == before else f" (board rebooted: boot count {before} -> {after})"))
        finally:
            hb.close()
            emcy.close()
            try:
                if rxw.current_state == S.ENCODER_TEST:
                    rxw.idle()
            except COMM_ERRORS:
                pass

    def step_reboot(self):
        ui = self.ui
        S = self.cli.ControllerState
        rxw = self.rxw
        ui.heading("Step 5: reboot over CAN")
        before = boot_diagnostics(rxw.node)
        u_mark = self.uart.mark() if self.uart else 0
        rxw.reboot()
        if not rxw.wait_until_responsive(timeout=15.0):
            self.record("FAIL", "Reboot over CAN", "the board did not come back within 15 s")
            return
        time.sleep(0.5)
        after = boot_diagnostics(rxw.node)
        detail = (f"boot count {before['boot_count']} -> {after['boot_count']}, "
                  f"reset cause {' '.join(after['reset']) or '?'}")
        ok = after["boot_count"] == before["boot_count"] + 1 and "SOFT" in after["reset"]
        if self.uart and self.uart.active:
            seen = self.uart.wait_for(r"Init complete!", u_mark, timeout=3.0) is not None
            detail += ", boot messages " + ("seen" if seen else "NOT seen on the UART")
        ok = ok and rxw.current_state == S.IDLE
        self.record("PASS" if ok else "FAIL", "Reboot over CAN", detail)

    def step_soak(self):
        ui = self.ui
        cli = self.cli
        rxw = self.rxw
        ui.heading("Step 6: soak (optional)")
        ans = ui.ask("Leave it running to catch random resets? Enter minutes (e.g. 30), or press Enter to skip.")
        try:
            minutes = float(ans) if ans else 0.0
        except ValueError:
            minutes = 0.0
        if minutes <= 0:
            self.record("SKIP", "Soak", "skipped")
            return
        hb = Heartbeats(self.network, [self.node_id])
        mark = hb.mark()
        start_boot = boot_diagnostics(rxw.node)["boot_count"]
        ui.say(f"Soaking for {minutes:g} min. Ctrl-C stops early and keeps the results so far.")
        t0 = time.monotonic()
        problems, max_temp, max_gap = [], -1e9, 0.0
        last_hb = time.monotonic()
        next_poll = t0
        try:
            while time.monotonic() - t0 < minutes * 60.0:
                time.sleep(1.0)
                evs = hb.since(mark)
                mark += len(evs)
                for t, _, state in evs:
                    max_gap = max(max_gap, t - last_hb)
                    last_hb = t
                    if state == 0:
                        problems.append(f"reboot at {(t - t0) / 60:.1f} min")
                max_gap = max(max_gap, time.monotonic() - last_hb)
                if time.monotonic() >= next_poll:
                    next_poll += 10.0
                    try:
                        errs = rxw.errors
                        temps = rxw.temperature
                        max_temp = max(max_temp, *temps)
                        if errs:
                            problems.append(f"errors {cli.fmt_errors(errs)}")
                            rxw.clear_errors(quiet=True)
                    except COMM_ERRORS:
                        problems.append(f"no SDO answer at {(time.monotonic() - t0) / 60:.1f} min")
                    print(f"\r    {(time.monotonic() - t0) / 60:5.1f} / {minutes:g} min   "
                          f"longest heartbeat gap {max_gap:4.1f} s   max temp {max_temp:5.1f} C   "
                          f"problems {len(problems)}", end="", flush=True)
        except KeyboardInterrupt:
            ui.say("")
            ui.say("Soak stopped early.")
        finally:
            hb.close()
        print()
        end_boot = boot_diagnostics(rxw.node)["boot_count"]
        if end_boot != start_boot:
            problems.append(f"boot count {start_boot} -> {end_boot}")
        if max_gap > 3.0:
            problems.append(f"heartbeat gap {max_gap:.1f} s")
        elapsed = (time.monotonic() - t0) / 60
        self.record("FAIL" if problems else "PASS", "Soak",
                    f"{elapsed:.1f} min, max temp {max_temp:.1f} C"
                    + ("; " + "; ".join(problems[:5]) if problems else ", no resets or gaps"))

    # --- driver ---

    def run(self) -> bool:
        ui = self.ui
        ui.heading("Reaction-wheel board bring-up")
        ui.say(f"Log file: {self.log.path}")
        ui.say("You need: the bench supply, the ST-Link with the adapter board, the CAN adapter,")
        ui.say("and a magnet (optional). Nothing connected to the motor phases. About 5 minutes.")
        ui.say("Ctrl-C stops at any point; the log keeps what happened.")

        self.uart = open_uart(self.args, self.log, ui)
        if self.uart is not None:
            if self.uart.opened:
                ui.say(f"Listening for the debug UART on: {', '.join(self.uart.opened)}")
            else:
                ui.say("No serial ports found: the self-test will show PASS/FAIL only, "
                       "without details. Plug in the ST-Link / UART cable for the details.")
            for p, err in self.uart.failed.items():
                ui.say(f"(could not open {p}: {err})")

        try:
            if "power" not in self.skip:
                if not self.step_power():
                    return self.finish()
            if not self.ensure_node():
                self.record("FAIL", "Find the board", "no single wheel on the bus")
                return self.finish()
            for name, fn in (("selftest", self.step_selftest), ("magnet", self.step_magnet),
                             ("undervoltage", self.step_undervoltage), ("reboot", self.step_reboot),
                             ("soak", self.step_soak)):
                if name in self.skip:
                    continue
                try:
                    fn()
                except COMM_ERRORS as e:
                    self.record("FAIL", f"{name} step", f"lost CAN contact: {e}")
                    if not self.rxw.wait_until_responsive(timeout=10.0):
                        ui.say("The board stopped answering on CAN; stopping here.")
                        break
        except KeyboardInterrupt:
            print()
            ui.say("Stopped by Ctrl-C.")
            try:
                if self.rxw is not None:
                    self.rxw.idle()
            except Exception:
                pass
        except Exception as e:  # never lose the summary + log to a tool bug
            self.log.write(traceback.format_exc())
            self.record("FAIL", "Bring-up tool error", f"{type(e).__name__}: {e} (traceback in the log)")
        return self.finish()

    def finish(self) -> bool:
        ui = self.ui
        if self.uart is not None:
            self.uart.close()
        ui.heading("Summary")
        for v, name, detail in self.results:
            ui.result(v, name, detail)
        failed = [r for r in self.results if r[0] == "FAIL"]
        overall = "FAIL" if failed or not self.results else "PASS"
        ui.say("")
        ui.say(f"BRING-UP {overall}")
        if self.serial_no is not None:
            new = self.log.path.with_name(f"board_{self.serial_no:08X}_{self.log.path.stem.split('_', 1)[1]}.log")
            self.log.rename(new)
        ui.say(f"Log saved to {self.log.path}")
        if failed:
            ui.say("Send that log file to Solomon.")
        self.log.close()
        return overall == "PASS"


def cmd_bringup(cli, network, args, node_id: int | None) -> bool:
    return Bringup(cli, network, args, node_id).run()
