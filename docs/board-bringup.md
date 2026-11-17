# Board bring-up: desk test before a board goes into a wheel

One command walks you through testing one bare power-stage board. It takes about 5 minutes and
saves a log file. You do not need to know the firmware: the tool tells you what to do at each
step and prints PASS or FAIL.

> **Status (2026-11-17):** new and not yet run on a real board. The self-test thresholds are
> first guesses from the design. Expect the first boards to need a threshold tweak or two.

## What you need

- A bench supply, set to **8.0 V** with the current limit at **0.5 A**.
- The board on the debug adapter board, with the ST-Link connected.
- The debug UART (USART2, 921600 baud) reaching the computer as a serial port. The ST-Link's
  virtual COM port and a separate USB-to-UART cable both work, and so do both at once. The tool
  listens on every serial port and uses the one the board talks on. Without the UART the
  self-test still gives PASS/FAIL, but not the details.
- A CAN adapter on the board's CAN bus, with termination.
- A magnet (optional) for the encoder step. It must be diametrically magnetized, like the rotor
  magnet.
- **Nothing connected to the motor phases.**
- Firmware v1.1.0 or newer: older releases have no self-test. Flash it with:

  ```bash
  ./flash-release.sh v1.1.0
  ```

## One-time setup on the computer

```bash
pip install canopen pyserial
```

If pip refuses with "externally-managed-environment" (Debian, Raspberry Pi OS), use a virtual
environment instead:

```bash
python3 -m venv ~/rw-venv
```

```bash
~/rw-venv/bin/pip install canopen pyserial
```

Then run the tool with `~/rw-venv/bin/python3` in place of `python3`.

Bring the CAN adapter up once after each computer boot:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

## Run it

```bash
cd can_console
python3 cli.py bringup
```

Then follow the prompts. Ctrl-C stops at any time, and the log keeps what happened up to then.

## The steps

| Step | What you do | What it checks |
|---|---|---|
| 1. Power up | Turn the supply off, then on. Type the wheel number and the supply current. | The board boots and talks on CAN. The node ID matches the straps. The idle current is logged. |
| 2. Self-test | Nothing. | The board checks itself (list below). |
| 3. Magnet | Turn a magnet over the encoder chip, one way and then the other. | The encoder sees real rotation. |
| 4. Undervoltage | Turn the supply down slowly to 4.5 V, then back up to 8.0 V. | The board turns its outputs off below 5 V, then recovers. |
| 5. Reboot | Nothing. | The reboot command works. The board comes back on the same node. |
| 6. Soak (optional) | Type a number of minutes, then leave it. | No random resets, heartbeat gaps or errors. |

The self-test (step 2) checks:

- the clock runs from the crystal, and the option bytes are right;
- the node-ID strap pins read cleanly;
- the 3.0 V ADC reference;
- all four thermistors and the chip temperature read something sane;
- the encoder chip answers on SPI;
- the CAN error counters;
- the current-sense amplifiers sit at their zero point;
- the bus voltage;
- with the bridge off, no phase shows a voltage (nothing turning, nothing leaking);
- each half-bridge switches low and then high, with no current and no bus sag.

## Reading the results

- **PASS**: good.
- **FAIL**: something is wrong. The tool says what to check. Send the log file to Solomon.
- **WARN**: worth a look, but it doesn't fail the board. CAN error counters often WARN right
  after the board boots. That is normal.
- **SKIP**: not run (you skipped it, or something earlier made it unsafe or pointless).
- **INFO**: a reading for the log only.

## The log

The log is saved as `can_console/bringup_logs/board_<serial>_<date>.log`. The serial is the
chip's unique ID, the same number as CANopen object `0x1018:04`. The log has every step, every
answer you typed, and every line the board printed on its debug UART.

## Options

Skip steps:

```bash
python3 cli.py bringup --skip magnet,soak
```

Name the debug UART port yourself, if the tool picks the wrong one:

```bash
python3 cli.py bringup --uart /dev/ttyACM0
```

Run only the self-test:

```bash
python3 cli.py selftest
```

The tool expects one board on the bus. If more are connected, add `--wheel N` to name one.

## Safety notes

- Keep the current limit at 0.5 A. If the self-test reports a bus sag or a current, don't raise
  the limit. Send the log to Solomon.
- The self-test drives the bridge only when nothing is turning. All three legs always get the
  same duty, so a motor at rest would see 0 V across it.
- The undervoltage step switches the outputs on (encoder-test mode) with all three legs at
  50 %, so no current flows.

## Firmware details (for Solomon)

- State 22 reference and per-check thresholds: `docs/control-laws.md` and
  `Firmware/Src/self_test.c`.
- Host side: `can_console/bringup.py`. It is driven by `cli.py bringup` and `cli.py selftest`.
- The undervoltage and overvoltage trips only run while the controller is armed: the fast loop
  and its ADC triggers stop when it disarms. That is why step 4 arms encoder-test mode first.
