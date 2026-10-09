#!/usr/bin/env bash
# SHUNT_SEL_DELAY_TICKS measurement — duty->sample pipeline lag via a torque step (SWD, no halt).
#
# Answers worklog open question Q2-A directly, with the shunt-selection feature OFF: while
# foc_capture records every tick, step the q-axis current setpoint and count the ticks K between
# the command appearing in the capture (Iq_set / svm duties move) and the measured current first
# responding (Iq_meas moves). The sample read at tick M reflects the duty written at tick M-K, and
# the history ring is pushed at the end of each tick, so:
#
#     SHUNT_SEL_DELAY_TICKS = K - 1        (predicted K=2 -> DELAY=1; see config.h comment)
#
# REQUIRES: target flashed with a matching RelWithDebInfo ELF (Debug IWDG-reboots when armed),
# encoder calibration valid (TORQUE_CONTROL runs closed-loop FOC), wheel free to spin (it gets
# ~50 ms of small torque), OpenOCD >= 0.12. Non-halting throughout (background MEM-AP only).
#
# Usage:
#   tools/shuntsel_delay.sh run [OUT_BIN]    # default OUT_BIN=shuntsel_step.bin
#
# Env overrides:
#   ELF=... OPENOCD_CFG=...     as usual (defaults: relwithdebinfo ELF, openocd.cfg)
#   IQ_BASE / IQ_STEP           baseline / stepped q-current [A] (default 0.05 / 0.30)
set -euo pipefail
cd "$(dirname "$0")/.."  # repo root

ELF="${ELF:-cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf}"
CFG="${OPENOCD_CFG:-openocd.cfg}"
IQ_BASE="${IQ_BASE:-0.05}"
IQ_STEP="${IQ_STEP:-0.30}"

usage() {
  sed -n '3,/^set -e/p' "$0" | sed -n 's/^# \?//p'
  exit "${1:-2}"
}

# NB: no early `exit` in awk — that closes the pipe mid-stream and objdump dies with SIGPIPE,
# which silently kills the script under `set -euo pipefail`.
sym_addr() {
  arm-none-eabi-objdump -t "$ELF" \
    | awk -v sym="$1" '$NF == sym && !f {print "0x"$1; f=1}'
}
sym_size() {
  arm-none-eabi-objdump -t "$ELF" \
    | awk -v sym="$1" '$NF == sym && !f {printf "%d\n", strtonum("0x"$(NF-1)); f=1}'
}

require_elf() {
  if [[ ! -f "$ELF" ]]; then
    echo "error: ELF not found: $ELF" >&2
    echo "  (set ELF=... or build with: cmake --build cmake-build-relwithdebinfo -j)" >&2
    exit 1
  fi
}

f32_hex() {
  python3 -c "import struct;print('0x%08x'%struct.unpack('<I',struct.pack('<f',float('$1')))[0])"
}

cmd_run() {
  require_elf
  local out="${1:-shuntsel_step.bin}"

  local a_fsmreq a_fsmstate a_armed a_errors a_idq a_capstate a_capidx a_capbuf capbuf_size a_shunt
  a_fsmreq="$(sym_addr fsm_requested_state)"
  a_fsmstate="$(sym_addr fsm_state)"
  a_armed="$(sym_addr controller_armed)"
  a_errors="$(sym_addr controller_errors)"
  a_idq="$(sym_addr idq_setpoint)"
  a_capstate="$(sym_addr foc_capture_state)"
  a_capidx="$(sym_addr foc_capture_idx)"
  a_capbuf="$(sym_addr foc_capture_buf)"
  capbuf_size="$(sym_size foc_capture_buf)"
  a_shunt="$(sym_addr shunt_selection_enabled)"
  for v in "$a_fsmreq" "$a_fsmstate" "$a_armed" "$a_errors" "$a_idq" \
           "$a_capstate" "$a_capidx" "$a_capbuf" "$capbuf_size" "$a_shunt"; do
    [[ -n "$v" ]] || { echo "error: symbol lookup failed (stale ELF?)" >&2; exit 1; }
  done

  # idq_setpoint is dq_vec_t {f32 d; f32 q;} — q lives at +4.
  local a_iq
  a_iq=$(printf '0x%x' $(( a_idq + 4 )))

  local base_hex step_hex zero_hex
  base_hex="$(f32_hex "$IQ_BASE")"
  step_hex="$(f32_hex "$IQ_STEP")"
  zero_hex="$(f32_hex 0)"

  local tcl
  tcl="$(mktemp --suffix=.tcl)"
  # shellcheck disable=SC2064
  trap "rm -f '$tcl'" EXIT

  cat > "$tcl" <<EOF
init

# fsm_state / fsm_requested_state are 1-BYTE objects (short-enum ABI) — byte access only.
set fsm [lindex [read_memory $a_fsmstate 8 1] 0]
if {\$fsm != 1} {
    echo "shuntsel_delay: FSM not IDLE (state=\$fsm) - aborting, nothing touched"
    echo "shuntsel_delay: (nonsense value => flashed image doesn't match the ELF)"
    shutdown error
}

# Feature OFF for ground-truth measurement (restored after).
set shunt_saved [lindex [read_memory $a_shunt 8 1] 0]
mwb $a_shunt 0

echo "shuntsel_delay: requesting TORQUE_CONTROL, baseline iq=$IQ_BASE A"
mwb $a_fsmreq 4
sleep 300
set armed [lindex [read_memory $a_armed 8 1] 0]
if {!\$armed} {
    echo "shuntsel_delay: controller failed to arm - restoring and aborting"
    echo "shuntsel_delay: (encoder cal valid? controller_errors set?)"
    mwb $a_fsmreq 1
    sleep 300
    mwb $a_shunt \$shunt_saved
    shutdown error
}

mww $a_iq $base_hex
sleep 100

# Arm capture (idx=0 first so the ring starts at buf[0]), then FIVE alternating setpoint edges
# ~1 ms (~32 ticks) apart. The analyzer aligns and averages all edges — single-edge SNR is
# marginal (the first affected sample carries only ~2% of the step against mA-level noise,
# since the closed loop's tau ~= 8 ticks), and averaging is what makes the onset fit unambiguous.
mww $a_capidx 0
mwb $a_capstate 1
sleep 1
echo "shuntsel_delay: 5 alternating steps $IQ_BASE <-> $IQ_STEP A"
mww $a_iq $step_hex
sleep 1
mww $a_iq $base_hex
sleep 1
mww $a_iq $step_hex
sleep 1
mww $a_iq $base_hex
sleep 1
mww $a_iq $step_hex
sleep 1
mwb $a_capstate 2

# Teardown: zero setpoint, back to IDLE, restore the shunt-selection flag.
mww $a_iq $zero_hex
mwb $a_fsmreq 1
sleep 300
mwb $a_shunt \$shunt_saved

dump_image $out $a_capbuf $capbuf_size
dump_image $out.idx $a_capidx 4

set errs [lindex [read_memory $a_errors 32 1] 0]
echo "shuntsel_delay: done, controller_errors=\$errs"
if {\$errs != 0} {
    echo "shuntsel_delay: WARNING controller_errors nonzero"
}
shutdown
EOF

  echo "shuntsel_delay: step $IQ_BASE -> $IQ_STEP A, capture -> $out"
  openocd -f "$CFG" -f "$tcl"

  # Convert the raw idx dump to the .head sidecar the decoder expects.
  python3 - "$out" <<'PY'
import struct, sys
out = sys.argv[1]
head = struct.unpack("<I", open(out + ".idx", "rb").read())[0]
open(out + ".head", "w").write(f"{head}\n")
print(f"wrote {out}.head (idx={head})")
PY
  rm -f "$out.idx"

  echo "shuntsel_delay: analyze with: python3 tools/shuntsel_delay_analyze.py $out"
}

case "${1:-}" in
  run) shift; cmd_run "$@" ;;
  -h|--help|"") usage 0 ;;
  *) echo "unknown subcommand: $1" >&2; usage ;;
esac
