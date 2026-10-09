# FOC capture — per-tick scope of the inner loop

A per-tick ring-buffer snapshot of FOC inner-loop state. Use it for current-PI tuning, encoder
phase verification, modulation-saturation hunting, deadtime-distortion analysis — anything where
SWO bandwidth is too tight to stream all the variables you want.

**Mode:** continuous ring while running, frozen on stop. Newest sample overwrites oldest. When you
stop the capture (explicitly or via FSM exit), the last ~8 ms of FOC state is preserved for dump.

Code lives in `Firmware/Inc/foc_capture.h` + `Firmware/Src/foc_capture.c`; the write site is at the
end of `foc_gen_pwm_cb()` in `Firmware/Src/foc.c`; auto-stop is in `fsm.c:fsm_exit_state`. Bench
tooling is in `tools/foc_capture.sh` (arm / stop / status / dump / decode subcommands; uses
OpenOCD SWD memory access — no firmware mailbox needed) and `tools/decode_foc_capture.py`
(the binary → CSV decoder it wraps).

## What gets captured

Each fast-loop tick (32 kHz, when state == `FOC_CAPTURE_RUNNING`) writes one `foc_capture_t` to
`foc_capture_buf[idx % FOC_CAPTURE_LEN]`, then increments `idx` (monotonic u32). Layout — **must
stay in sync with `tools/decode_foc_capture.py`'s `DTYPE`**:

| Field | Type | Unit | Source |
|---|---|---|---|
| `loop_count` | u32 | — | `controller_get_loop_count()` at capture time; monotonic, ISR-incremented. Use to align across runs. |
| `Ia`, `Ib`, `Ic` | f32 ×3 | A | Phase currents *after* zero-sum common-mode subtraction — or best-2-of-3 reconstruction when `shunt_selection_enabled` — in `adc_update_callback` (`control_ll.c`). Stored from `foc_meas_current_cb()`. |
| `Ialpha`, `Ibeta` | f32 ×2 | A | Clarke output. |
| `Id_meas`, `Iq_meas` | f32 ×2 | A | Park output (this tick's measurement, not filtered). |
| `Id_set`, `Iq_set` | f32 ×2 | A | Current setpoint (d-axis is always 0 in normal FOC; q-axis from torque controller). |
| `motor_phase` | f32 | rad | Electrical angle used for *measurement* Park this tick. |
| `motor_vel` | f32 | rad/s | Electrical angular velocity from encoder estimator. |
| `Vd_int`, `Vq_int` | f32 ×2 | V | PI integrator state at end of this tick. |
| `mod_d`, `mod_q` | f32 ×2 | (0..1) | PI output normalized by `mod_to_V = 2/3·Vbus`. |
| `mod_alpha`, `mod_beta` | f32 ×2 | (0..1) | Inverse Park output (modulation in α/β frame). |
| `svm_A`, `svm_B`, `svm_C` | f32 ×3 | (0..1) | SVM output → fed straight to `hrtim_pwm_set_dcycle_f32()`. |
| `Vbus` | f32 | V | Bus voltage at sample time. |
| `shunt_drop` | i32 | — | Best-2-of-3 selection: which phase was reconstructed this tick (−1 = none/disabled, 0/1/2 = A/B/C). Set in `adc_update_callback`; always −1 while `shunt_selection_enabled` is false. |

Total **92 bytes per sample**. Ring depth is `FOC_CAPTURE_LEN = 256` samples = **23 KB**, **8 ms
at 32 kHz**.

## State machine

```
            arm()              stop() or FSM exit
   IDLE ─────────────► RUNNING ────────────────────► STOPPED
    ▲                    │                              │
    │                    │ arm() resets idx=0           │ arm()
    │                    │                              │ resets
    └────────────────────┴──────────────────────────────┘ idx=0
```

| State | Value | Meaning |
|---|---|---|
| `FOC_CAPTURE_IDLE` | 0 | Power-on default. ISR hook is gated off. |
| `FOC_CAPTURE_RUNNING` | 1 | Ring writes active. `idx` increments each tick, oldest sample is overwritten. |
| `FOC_CAPTURE_STOPPED` | 2 | Ring frozen. `idx` retained — tells the decoder how many samples were captured and where the wrap is. |

Transitions:

- **`foc_capture_arm()`** (`tools/foc_capture.sh arm` via SWD `mww`+`mwb`):
  `idx = 0`, state → `RUNNING`.
- **`foc_capture_stop()`** (`tools/foc_capture.sh stop` via SWD `mwb`, or FSM exiting a control
  state): state → `STOPPED`. `idx` and buffer contents preserved.

The FSM auto-stop fires from `fsm_exit_state()` whenever leaving any of the control / cal / open-
loop / vel-ramp states. This includes both clean exits (state finished its run) and forced exits
(error trip, host-requested transition mid-run).

## How it's driven: SWD only

The state machine is controlled by direct SWD memory writes to `foc_capture_state` and
`foc_capture_idx` via OpenOCD. There is no CAN/SDO trigger — the bench probe is the only control
surface. (Earlier revisions used OD `0x4006:01` as an SDO trigger; that was removed because the
probe path is faster, halts the CPU for zero ticks, and there's no in-flight use case where you'd
want a 8 ms FOC capture but couldn't also attach a probe.)

`tools/foc_capture.sh` is the one-stop wrapper: `arm`, `stop`, `status`, `dump`, `decode`.

## How to use it

### 1. Arm

```bash
$ tools/foc_capture.sh arm
arm: idx=0, state=RUNNING (1)  @  0x20001f70 / 0x20001f6c
```

Resets `idx` to 0 (via SWD `mww`) and sets `state = RUNNING` (via SWD `mwb`). Neither halts the
target — `mww`/`mwb` go through the MEM-AP in the background while the CPU keeps executing. The
next fast-loop tick starts writing to `buf[0]`.

### 2. Run your experiment

The ring continuously overwrites itself. The buffer always holds *the last ~8 ms* of FOC state.

Check on it any time with:
```bash
$ tools/foc_capture.sh status
state=RUNNING idx=4128 (samples_in_buf=256)
```

(`samples_in_buf` saturates at `FOC_CAPTURE_LEN = 256`; once the ring has wrapped, the buffer is
always full.)

### 3. Stop (manually or automatically)

Two ways the capture stops:

- **Explicit stop:** `tools/foc_capture.sh stop` (single SWD `mwb`, no halt).
- **FSM auto-stop:** any transition out of a control state (`TORQUE_CONTROL`, `VEL_CONTROL`,
  `OPEN_LOOP_CONTROL`, calibrations, etc.) auto-stops the capture. This is the workflow for
  "capture the moment a fault happened" — the buffer preserves the ~8 ms leading up to the
  controller_disarm. No host action required.

Stopping mid-tick won't corrupt the partial sample; the state check is at the top of the write
block in `foc_gen_pwm_cb`.

**Always stop before dumping** if the controller is armed. See "Dump halt safety" below.

### 4. Dump

```bash
$ tools/foc_capture.sh dump
dump: foc_capture.bin (+.head) <- buf 0x20001f74/23552 + idx 0x20001f70
wrote foc_capture.bin (23552 B), foc_capture.bin.head (idx=128043)
```

The script looks up addresses + size from the ELF symbol table and reads three things via OpenOCD:
the current state byte (for the post-read consistency check), the buffer (23 KB), and the idx.
**No halt.** Reads are background MEM-AP transactions — the CPU keeps running, PWM keeps updating,
the FOC loop keeps controlling. See "Dump halt safety" below for why this matters.

Defaults: ELF = `cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf`, output = `foc_capture.bin`,
OpenOCD config = `openocd.cfg`. Override via env vars (`ELF=...`, `OPENOCD_CFG=...`) or the
positional arg (`tools/foc_capture.sh dump my_run.bin`).

The script warns if it found `state == RUNNING` — meaning the ISR was actively writing the buffer
while you were reading it, and the resulting dump may have torn samples (one half-old, one
half-new). Re-run after `stop` for a clean snapshot.

### 5. Decode + plot

```bash
$ tools/foc_capture.sh decode foc_capture.bin > foc.csv
head=128043, emitting 256 samples
```

`decode` is a thin wrapper around `python3 tools/decode_foc_capture.py`. The decoder reads
`foc_capture.bin.head` to know how to unwrap the ring:

| Head value | Decoder behavior |
|---|---|
| 0 | Empty CSV (no samples captured). |
| `0 < head ≤ LEN` (256) | Capture stopped before wrap; emits `buf[0:head]` in chronological order. |
| `head > LEN` | Ring wrapped; oldest sample is `buf[head % LEN]`, newest is `buf[(head-1) % LEN]`. Emits all 256 samples chronological. |

Output is one CSV row per sample, all 23 named columns.

**Plotting.** `tools/plot_foc_capture.py` renders a dump into a stacked, time-aligned figure
(phase currents, dq meas-vs-setpoint, SVM duties, ω, Vbus — plus the dropped-shunt phase if
best-2-of-3 selection was active). The shell wrapper exposes it as a `plot` subcommand:

```bash
$ tools/foc_capture.sh plot foc_capture.bin --show   # window ($DISPLAY set); else writes a PNG
$ tools/plot_foc_capture.py foc.bin --xy             # + the Iα–Iβ current-vector locus
$ tools/plot_foc_capture.py foc.csv --cols Ia,Ib,Ic,Iq_meas   # ad-hoc columns
$ tools/foc_capture.sh decode foc.bin | tools/plot_foc_capture.py -   # stdin CSV
```

Takes a `.bin` (reuses the decoder's `load()` for ring-unwrap), a `.csv`, or `-` (stdin). Flags:
`--xy` (current-vector locus — a clean spin is a circle; a distorted/polygonal locus is the
high-speed lumpiness signature), `--cols`/`--groups` (pick variables), `--rate` (time-axis rate,
default 32 kHz), `--list-cols`, `--save`, `--show`; `--help` covers the rest. For the lumpiness
hunt the default stack answers the key question: are `Id`/`Iq` themselves lumpy (control /
measurement), or clean while the phase currents are lumpy (voltage-vector quantization)?

## Dump halt safety

### TL;DR

`tools/foc_capture.sh dump` **does not halt the target**. The bulk buffer read goes through the
MEM-AP background memory channel on the Cortex-M; the CPU keeps executing instructions, the
HRTIM repetition ISR keeps firing, and the FOC loop keeps controlling the motor for the full
duration of the read. Safe to run while spinning.

### Why it matters

If we *did* halt for the dump (an earlier draft of the script had explicit `halt`/`resume`), here's
what would happen:

- The CPU stops on the next debug breakpoint; the fast-loop ISR stops firing.
- HRTIM and the ADC keep running by default (DBGMCU freeze is off for those peripherals). PWM
  outputs stay frozen at whatever compare values were last loaded — the motor sees a *fixed* stator
  voltage vector for the entire halt window.
- The rotor, with its mechanical inertia and the energy already in the windings, keeps spinning.
  Once the rotor moves ~90° electrical away from the frozen-field d-axis, the back-EMF and the
  applied voltage stop being aligned; the current is then ≈ `V_applied / R_phase` (minus a small
  BEMF term at low speed).
- For this board: `R_phase ≈ 0.1 Ω` (post-cal estimate), Vbus ≈ 8 V, max modulation ≈ 0.5 →
  worst-case phase current ≈ **40 A**. Way past the firmware's 5 A trip, but the firmware can't
  trip because the CPU is halted. HRTIM has hardware fault inputs that *could* catch this, but
  they're disabled (`pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE` — `hrtim.c:172`).

The buffer is 23 KB. At a ST-Link V3's MEM-AP rate of ~200-400 KB/s, that's **roughly 50-150 ms**
of halt; at ST-Link V2 rates (~50-100 KB/s) it's **250-500 ms**. Either way: long enough at 40 A
to thermally stress the shunts, the FETs, and the motor windings. So we don't halt.

### Why no halt is correct, not just expedient

Cortex-M debug architecture exposes both a halt-mode debug interface (DHCSR.C_HALT) and a
run-mode memory access port (MEM-AP). For a SRAM read, MEM-AP works fine while the CPU runs —
it just arbitrates with the CPU's own bus accesses through the BusMatrix. Background reads cost
a small amount of extra CPU stall (a few cycles when MEM-AP wins arbitration on SRAM1/SRAM2),
which on our fast loop is ~negligible (<0.2% of the 31.25 µs budget).

The risk that *does* remain: **data tearing**. The ISR is writing to the buffer (when RUNNING),
and we're reading the same buffer concurrently. The MEM-AP reads 32-bit words atomically, so we
never see a half-written word; but a single 92-byte sample is 23 word-writes, and our 23 KB
read happens word-at-a-time over hundreds of ms. So a sample we're reading at the moment the ISR
overwrites it can show up as half-old, half-new fields. The dump script warns when this is a risk
(reads `state` first, warns if it's RUNNING).

To get a fully consistent dump, **stop first**:
```bash
tools/foc_capture.sh stop && tools/foc_capture.sh dump
```

The FSM auto-stop handles this for the post-mortem-after-disarm case automatically.

## Cost in the fast loop

When `foc_capture_state != FOC_CAPTURE_RUNNING` the entire hook is one branch (~2 cycles ≈ 14 ns).
While running: 92-byte struct copy + `idx % LEN` (single AND because `LEN = 256 = 2^8`) + index
update ≈ **~180 ns** per tick, < 0.6 % of the 31.25 µs ISR budget.

The compiler turns `idx % FOC_CAPTURE_LEN` into a single bitwise-AND with `0xFF` because
`FOC_CAPTURE_LEN` is a power of two. Verified with `-O2` disassembly.

## When you add or remove a captured field

Three places must move together — the compiler catches the C side, but the Python side is on the
honor system:

1. `Firmware/Inc/foc_capture.h` — add the f32 to `foc_capture_t` (keep all fields 4-byte for no
   padding). New field count → new total size (88 → 92 → ...).
2. `Firmware/Src/foc.c` — set the field in the designated-init at the bottom of `foc_gen_pwm_cb`.
3. `tools/decode_foc_capture.py` — add the field to `DTYPE` *in the same order*, then update the
   `assert DTYPE.itemsize == 92` to the new size. This assert is the trip wire: if the dtype gets
   out of sync, the Python loud-fails on import. If you leave it in sync but in the wrong order,
   you get silently-misaligned values — **always append at the end of both the struct and the
   dtype, never in the middle**.

If you change `FOC_CAPTURE_LEN`, also update the comment in `foc_capture.h` (RAM cost, ms at
32 kHz). The decoder doesn't hard-code `LEN` — it reads it from the file size — so no Python
change needed.

## Bench workflow checklists

### Step-response tuning of the current PI (SWD bench workflow)

1. Build + flash.
2. From your host, SDO-request `TORQUE_CONTROL` (`0x4004 = 4`) and a small `Iq` setpoint via
   `0x4005:01`; let it settle.
3. `tools/foc_capture.sh arm`.
4. Host writes a new `setpoint_1` (the step).
5. Wait ≥ 8 ms for the ring to overwrite the pre-step state with post-step.
6. `tools/foc_capture.sh stop && tools/foc_capture.sh dump && tools/foc_capture.sh decode > step.csv`.
7. Plot `Iq_meas` vs `Iq_set`, `Vq_int` for integrator wind-up, `mod_d/q` vs `mod_alpha/beta` for
   modulation saturation.

### Post-mortem after an unexpected disarm

1. Arm capture *before* the run that might fail (`tools/foc_capture.sh arm`).
2. Let the system run normally. If it errors out, the FSM auto-stops the capture as part of the
   exit sequence — no host action required.
3. After the disarm: `tools/foc_capture.sh dump && tools/foc_capture.sh decode > postmortem.csv`.
4. The CSV holds the last 8 ms before the trip. Look at the trip flag in the OD
   (`0x4000.errors`) to know what to look for in the trace.

### Steady-state spin analysis (deadtime / harmonic distortion)

1. Host SDO-requests a control state, sets steady-state velocity, lets it spin steady.
2. `tools/foc_capture.sh arm && sleep 0.02 && tools/foc_capture.sh stop` — captures exactly one
   ~8 ms window of steady-state, no transient.
3. `tools/foc_capture.sh dump && tools/foc_capture.sh decode > spin.csv`.
4. Plot phase currents; 6th-harmonic ripple is the textbook deadtime-distortion signature for a
   3-phase BLDC.

## Limitations / known footguns

- **`FOC_CAPTURE_LEN` is a power of two**, by design (the `% LEN` becomes a free bitwise-AND).
  Don't change it to a non-power-of-two without expecting the modulo to become a real divide
  in the hot path.
- **No timestamping beyond `loop_count`.** Wall-clock alignment to host events requires the host
  to know the wheel's tick rate (32 kHz) and the offset of `loop_count` at arm time. For tighter
  sync you'd need an external trigger or a clock-sync mechanism that doesn't exist here.
- **Capture is at the end of `foc_gen_pwm_cb`**, after the integrator updates and modulation clamp.
  The PI integrator state captured is *post-update for this tick*. To compare against the
  pre-update value, look at the previous sample's `Vd_int`/`Vq_int`.
- **Auto-stop fires on every control-state exit**, including error trips. If you don't want the
  trip to freeze the buffer, re-arm with `tools/foc_capture.sh arm` after re-entering the state.
- **SWD writes race against the ISR.** Writing `state = RUNNING` while the ISR is in flight can
  produce one extra sample at the previous `idx` position before the host's `idx = 0` write lands.
  The dump unwrap tolerates this — the worst-case visible artifact is a single sample written to
  the wrong slot — but if you need bit-perfect arm timing, halt the target first
  (`openocd -c "init; halt; mww ...; mwb ...; resume"`).
- **Dump while RUNNING produces torn samples.** See "Dump halt safety" above for details. Always
  `stop` before `dump` for a clean snapshot; the script warns when it spots `state == RUNNING`
  during the dump.
- **`idx` is u32 monotonic**, so it overflows after `2^32 / 32000` ≈ 37 hours of continuous
  capture. After overflow the decoder's "head > LEN → wrap" branch still works correctly
  (modular arithmetic), but `loop_count` aliasing makes the timeline ambiguous. In practice you
  stop the capture long before this.

## See also

- `docs/control-cycle.md` — where in the tick the capture write happens.
- `docs/can-interface.md` — SDO mechanics generally; FOC capture is *not* exposed on the bus
  (probe-only).
- `Firmware/Inc/foc_capture.h` — the source of truth for struct layout.
