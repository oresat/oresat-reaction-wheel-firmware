#!/usr/bin/env bash
# FOC capture control + readout via OpenOCD (SWD background memory access).
#
# Looks up firmware symbol addresses from the ELF; talks to the target via
# `openocd` and SWD without halting (except for `dump`, which briefly halts
# during the bulk `dump_image`). Pairs with the SDO trigger on OD 0x4006:01
# (same state machine, either path works) — SWD is more convenient at the
# bench; SDO is what you have available over the bus in flight.
#
# Usage:
#   tools/foc_capture.sh arm                 # arm/start ring capture
#   tools/foc_capture.sh stop                # freeze ring for readout
#   tools/foc_capture.sh status              # print state + idx
#   tools/foc_capture.sh dump [OUT_BIN]      # buffer + idx -> OUT_BIN(+.head)
#   tools/foc_capture.sh decode [BIN]        # OUT_BIN -> CSV on stdout
#   tools/foc_capture.sh plot [BIN] [opts]   # decode + plot (tools/plot_foc_capture.py)
#   tools/foc_capture.sh grab [BIN] [opts]   # arm→stop→dump→plot in one shot (GRAB_SECONDS dwell)
#
# Env overrides:
#   ELF=...          path to oresat-acs-fw-v8.elf (default: cmake-build-debug/...)
#   OPENOCD_CFG=...  OpenOCD config file (default: st_nucleo_g4.cfg)
set -euo pipefail
cd "$(dirname "$0")/.."  # repo root

ELF="${ELF:-cmake-build-debug/oresat-acs-fw-v8.elf}"
CFG="${OPENOCD_CFG:-st_nucleo_g4.cfg}"

usage() {
  sed -n '3,/^set -e/p' "$0" | sed -n 's/^# \?//p'
  exit "${1:-2}"
}

# Look up an ELF symbol's address (printed as 0xHEX) — empty if not found.
# NB: no early `exit` in awk — that closes the pipe mid-stream and objdump dies
# with SIGPIPE (141), which `set -o pipefail` + `set -e` turn into a fatal script
# abort inside the $(...) capture. The `!f` guard keeps first-match-only semantics
# while letting objdump finish writing.
sym_addr() {
  arm-none-eabi-objdump -t "$ELF" \
    | awk -v sym="$1" '$NF == sym && !f {print "0x"$1; f=1}'
}
# Look up an ELF symbol's size (printed as DECIMAL).
sym_size() {
  arm-none-eabi-objdump -t "$ELF" \
    | awk -v sym="$1" '$NF == sym && !f {printf "%d\n", strtonum("0x"$(NF-1)); f=1}'
}

require_elf() {
  if [[ ! -f "$ELF" ]]; then
    echo "error: ELF not found: $ELF" >&2
    echo "  (set ELF=... or build with: cmake --build cmake-build-debug -j)" >&2
    exit 1
  fi
}

# Run OpenOCD with a sequence of -c commands, hiding its banner spam unless
# OPENOCD_VERBOSE is set. Stdout of OpenOCD goes to stderr so script callers
# can capture our own structured output cleanly.
ocd() {
  if [[ "${OPENOCD_VERBOSE:-}" == "1" ]]; then
    openocd -f "$CFG" "$@" -c "exit"
  else
    openocd -f "$CFG" "$@" -c "exit" 2>&1 | grep -vE \
      '^(Open On-Chip|Licensed under|For bug reports|Info :|Warn : |srst_|none|adapter speed:)' || true
  fi >&2
}

cmd_arm() {
  require_elf
  local state_addr idx_addr
  state_addr="$(sym_addr foc_capture_state)"
  idx_addr="$(sym_addr foc_capture_idx)"
  [[ -n "$state_addr" && -n "$idx_addr" ]] || { echo "error: symbols not found" >&2; exit 1; }
  echo "arm: idx=0, state=RUNNING (1)  @  $idx_addr / $state_addr"
  # Order matters: clear idx first so the first ISR write after RUNNING lands at buf[0].
  ocd -c "init" \
      -c "mww $idx_addr 0" \
      -c "mwb $state_addr 1"
}

cmd_stop() {
  require_elf
  local state_addr
  state_addr="$(sym_addr foc_capture_state)"
  [[ -n "$state_addr" ]] || { echo "error: foc_capture_state not found" >&2; exit 1; }
  echo "stop: state=STOPPED (2)  @  $state_addr"
  ocd -c "init" -c "mwb $state_addr 2"
}

cmd_status() {
  require_elf
  local state_addr idx_addr
  state_addr="$(sym_addr foc_capture_state)"
  idx_addr="$(sym_addr foc_capture_idx)"
  [[ -n "$state_addr" && -n "$idx_addr" ]] || { echo "error: symbols not found" >&2; exit 1; }

  local tmp; tmp="$(mktemp -d)"
  trap "rm -rf '$tmp'" RETURN
  ocd -c "init" \
      -c "dump_image $tmp/s $state_addr 1" \
      -c "dump_image $tmp/i $idx_addr 4"

  local state_byte idx_val state_name
  state_byte=$(od -An -tu1 "$tmp/s" | tr -d ' \n')
  idx_val=$(python3 -c "import struct,sys;print(struct.unpack('<I', open('$tmp/i','rb').read())[0])")
  case "$state_byte" in
    0) state_name="IDLE" ;;
    1) state_name="RUNNING" ;;
    2) state_name="STOPPED" ;;
    *) state_name="?($state_byte)" ;;
  esac
  local samples
  if (( idx_val == 0 )); then samples=0
  elif (( idx_val < 256 )); then samples=$idx_val
  else samples=256
  fi
  echo "state=$state_name idx=$idx_val (samples_in_buf=$samples)"
}

cmd_dump() {
  require_elf
  local out="${1:-foc_capture.bin}"
  local buf_addr buf_size idx_addr state_addr
  buf_addr="$(sym_addr foc_capture_buf)"
  buf_size="$(sym_size foc_capture_buf)"
  idx_addr="$(sym_addr foc_capture_idx)"
  state_addr="$(sym_addr foc_capture_state)"
  [[ -n "$buf_addr" && -n "$idx_addr" && -n "$state_addr" ]] \
    || { echo "error: symbols not found" >&2; exit 1; }

  local tmp; tmp="$(mktemp -d)"
  trap "rm -rf '$tmp'" RETURN

  # NO `halt` / `resume`: dump_image on Cortex-M uses MEM-AP background memory
  # access — the CPU keeps executing while OpenOCD reads. This is the safe
  # default for an armed motor; halting would freeze PWM at the last loaded
  # compare values while the rotor keeps spinning, potentially driving large
  # currents through the inverter for the duration of the read (~100-500 ms).
  echo "dump: $out (+.head) <- buf $buf_addr/$buf_size + idx $idx_addr"
  ocd -c "init" \
      -c "dump_image $tmp/state $state_addr 1" \
      -c "dump_image $out $buf_addr $buf_size" \
      -c "dump_image $tmp/idx $idx_addr 4"

  local state_byte head
  state_byte=$(od -An -tu1 "$tmp/state" | tr -d ' \n')
  head=$(python3 -c "import struct,sys;print(struct.unpack('<I', open(sys.argv[1],'rb').read())[0])" "$tmp/idx")
  echo "$head" > "$out.head"
  echo "wrote $out ($buf_size B), $out.head (idx=$head)"

  if [[ "$state_byte" == "1" ]]; then
    echo "warning: state=RUNNING during dump — ISR was writing the buffer" >&2
    echo "         concurrently. Samples may be torn or inconsistent. Run" >&2
    echo "         'tools/foc_capture.sh stop' before dumping for a clean snapshot." >&2
  fi
}

cmd_decode() {
  local bin="${1:-foc_capture.bin}"
  python3 tools/decode_foc_capture.py "$bin"
}

cmd_plot() {
  # First non-option arg is the bin (default foc_capture.bin); rest pass through.
  local bin="foc_capture.bin"
  if [[ $# -gt 0 && "$1" != -* ]]; then bin="$1"; shift; fi
  python3 tools/plot_foc_capture.py "$bin" "$@"
}

cmd_grab() {
  # All-in-one: arm → (optional GRAB_SECONDS dwell) → stop → dump → plot. The ring is
  # only ~8 ms deep, so this freezes the most-recent ~8 ms; set GRAB_SECONDS=N to land
  # that window N seconds after arming (e.g. just after a host setpoint step). The first
  # non-option arg is the bin; remaining args (--show, --xy, --groups, ...) pass to plot.
  local bin="foc_capture.bin"
  if [[ $# -gt 0 && "$1" != -* ]]; then bin="$1"; shift; fi
  cmd_arm
  if [[ -n "${GRAB_SECONDS:-}" && "${GRAB_SECONDS}" != "0" ]]; then
    echo "grab: dwell ${GRAB_SECONDS}s before stop"
    sleep "${GRAB_SECONDS}"
  fi
  cmd_stop
  cmd_dump "$bin"
  python3 tools/plot_foc_capture.py "$bin" "$@"
}

case "${1:-}" in
  arm)    shift; cmd_arm    "$@" ;;
  stop)   shift; cmd_stop   "$@" ;;
  status) shift; cmd_status "$@" ;;
  dump)   shift; cmd_dump   "$@" ;;
  decode) shift; cmd_decode "$@" ;;
  plot)   shift; cmd_plot   "$@" ;;
  grab)   shift; cmd_grab   "$@" ;;
  -h|--help|"") usage 0 ;;
  *) echo "unknown subcommand: $1" >&2; usage ;;
esac
