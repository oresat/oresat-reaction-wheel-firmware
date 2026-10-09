#!/usr/bin/env bash
# Bench "CAN over SWD": write the host-facing OD variables (requested FSM state,
# setpoints, reboot) and read back the TPDO status block via OpenOCD background
# memory access — no CAN adapter needed for bringup.
#
# Writes land in the SAME OD_RAM fields a CAN SDO write would, so the firmware's
# foreground process_sdo_vals() does all the routing exactly as it does for CAN
# (edge-detected: only a CHANGED value triggers). Member addresses are resolved
# from the ELF's debug info, so the ELF must match the flashed image.
#
# Usage:
#   tools/swd_sdo.sh state <name|num>     # request FSM state (0x4004); auto-nudges
#                                         #   through NONE so re-requesting the same
#                                         #   state still triggers
#   tools/swd_sdo.sh setpoint <sp1> [sp2] # write setpoint floats (0x4005)
#   tools/swd_sdo.sh status               # read 0x4000-0x4002 + echoed setpoints
#   tools/swd_sdo.sh states               # list FSM state names/numbers
#   tools/swd_sdo.sh reboot               # write 0x4007 reboot_request=1
#
# Order matters for control modes: enter the state FIRST, then write the
# setpoint — process_sdo_vals() routes setpoint_1 by the CURRENT state, so a
# value written while IDLE is edge-consumed and dropped.
#
# All accesses are MEM-AP background reads/writes (no halt) — safe while armed.
#
# Env overrides:
#   ELF=...          path to oresat-acs-fw-v8.elf (default: cmake-build-relwithdebinfo/...)
#   OPENOCD_CFG=...  OpenOCD config file (default: openocd.cfg)
set -euo pipefail
cd "$(dirname "$0")/.."  # repo root

ELF="${ELF:-cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf}"
CFG="${OPENOCD_CFG:-openocd.cfg}"

usage() {
  sed -n '2,/^set -e/p' "$0" | sed -n 's/^# \?//p'
  exit "${1:-2}"
}

# Single source for FSM state numbering (mirrors Firmware/Inc/fsm.h).
STATES="0:NONE 1:IDLE 2:SYSTEM_ERROR 3:CONTROLLER_ERROR 4:TORQUE_CONTROL
5:VEL_CONTROL 6:POS_CONTROL 7:MOTOR_RESISTANCE_CAL 8:MOTOR_INDUCTANCE_CAL
9:ENCODER_DIR_CAL 10:ENCODER_OFFSET_CAL 11:ENCODER_TEST 12:OPEN_LOOP_CONTROL
13:CLEAR_ERRORS 14:ENCODER_VALIDATION 15:SHITTY_OFFSET_CAL 16:VEL_RAMP_CONTROL
17:BACKEMF_CAL 18:SAVE_CALIBRATION 19:ERASE_CALIBRATION 20:RESTORE_CALIBRATION
21:BACKEMF_CAL_BIDIR"

require_elf() {
  if [[ ! -f "$ELF" ]]; then
    echo "error: ELF not found: $ELF" >&2
    echo "  (set ELF=... or build with: cmake --build cmake-build-relwithdebinfo -j)" >&2
    exit 1
  fi
}

# Resolve struct-member addresses from ELF debug info, one hex addr per line,
# in argument order. Needs a -g build (RelWithDebInfo/Debug both qualify).
member_addrs() {
  local args=()
  local m
  for m in "$@"; do
    args+=(-ex "printf \"0x%x\n\", (unsigned int)&($m)")
  done
  local out
  out="$(arm-none-eabi-gdb -batch -nx -q "${args[@]}" "$ELF" 2>/dev/null | grep '^0x' || true)"
  if [[ "$(wc -l <<<"$out")" -ne $# || -z "$out" ]]; then
    echo "error: failed to resolve OD member addresses from $ELF (debug info present? ELF matches source?)" >&2
    exit 1
  fi
  echo "$out"
}

# Resolve a state name (case-insensitive, unique-prefix ok: vel_r, torque, clear)
# or bare number to its enum value.
state_num() {
  local in="${1^^}" tok num name matches=()
  if [[ "$in" =~ ^[0-9]+$ ]]; then
    (( in <= 21 )) || { echo "error: state number $in out of range (0-21)" >&2; exit 1; }
    echo "$in"; return
  fi
  in="${in#FSM_STATE_}"
  for tok in $STATES; do
    num="${tok%%:*}"; name="${tok#*:}"
    [[ "$name" == "$in" ]] && { echo "$num"; return; }
    [[ "$name" == "$in"* ]] && matches+=("$tok")
  done
  if [[ ${#matches[@]} -eq 1 ]]; then
    echo "${matches[0]%%:*}"; return
  elif [[ ${#matches[@]} -gt 1 ]]; then
    echo "error: ambiguous state '$1' — matches: ${matches[*]}" >&2; exit 1
  fi
  echo "error: unknown state '$1' (try 'tools/swd_sdo.sh states')" >&2; exit 1
}

state_name() {
  local tok
  for tok in $STATES; do
    [[ "${tok%%:*}" == "$1" ]] && { echo "${tok#*:}"; return; }
  done
  echo "?($1)"
}

# float literal -> 0xHEX of its IEEE-754 bits (for mww)
f2hex() {
  python3 -c "import struct,sys;print('0x%08x'%struct.unpack('<I',struct.pack('<f',float(sys.argv[1])))[0])" "$1"
}

# Run OpenOCD with a sequence of -c commands, hiding its banner spam unless
# OPENOCD_VERBOSE is set. Stdout of OpenOCD goes to stderr so script callers
# can capture our own structured output cleanly.
ocd() {
  if [[ "${OPENOCD_VERBOSE:-}" == "1" ]]; then
    openocd -f "$CFG" "$@" -c "exit"
  else
    openocd -f "$CFG" "$@" -c "exit" 2>&1 | grep -vE \
      '^(Open On-Chip|Licensed under|For bug reports|Info :|Warn : |srst_|none|adapter speed:|[[:space:]]|DEPRECATED|force hard|dumped [0-9]+ bytes)' || true
  fi >&2
}

cmd_state() {
  [[ $# -eq 1 ]] || usage
  require_elf
  local num addr
  num="$(state_num "$1")"
  addr="$(member_addrs OD_RAM.x4004_rxw_requested_state.req_state)"
  echo "state: request $(state_name "$num") ($num)  @  $addr  (nudge via NONE)"
  # process_sdo_vals() only acts on a CHANGED value; writing NONE first (a no-op
  # request) guarantees an edge even when re-requesting the current value. The
  # foreground loop polls at ~1 kHz, so 150 ms is ample for it to see the 0.
  ocd -c "init" \
      -c "mwb $addr 0" \
      -c "sleep 150" \
      -c "mwb $addr $num"
}

cmd_setpoint() {
  [[ $# -ge 1 && $# -le 2 ]] || usage
  require_elf
  local addrs a1 a2 h1
  addrs="$(member_addrs OD_RAM.x4005_rxw_setpoint.setpoint_1 \
                        OD_RAM.x4005_rxw_setpoint.setpoint_2)"
  a1="$(sed -n 1p <<<"$addrs")"; a2="$(sed -n 2p <<<"$addrs")"
  h1="$(f2hex "$1")"
  if [[ $# -eq 2 ]]; then
    echo "setpoint: sp1=$1 @ $a1, sp2=$2 @ $a2"
    ocd -c "init" -c "mww $a1 $h1" -c "mww $a2 $(f2hex "$2")"
  else
    echo "setpoint: sp1=$1 @ $a1"
    ocd -c "init" -c "mww $a1 $h1"
  fi
}

cmd_status() {
  require_elf
  local members=(
    OD_RAM.x4000_rxw_ctrl_stat.current_state
    OD_RAM.x4000_rxw_ctrl_stat.procedure_result
    OD_RAM.x4000_rxw_ctrl_stat.errors
    OD_RAM.x4001_rxw_motor_vel_current.motor_vel
    OD_RAM.x4001_rxw_motor_vel_current.motor_current
    OD_RAM.x4002_rxw_bus_volt_current.bus_voltage
    OD_RAM.x4002_rxw_bus_volt_current.bus_current
    OD_RAM.x4004_rxw_requested_state.req_state
    OD_RAM.x4005_rxw_setpoint.setpoint_1
    OD_RAM.x4005_rxw_setpoint.setpoint_2
  )
  local sizes=(1 1 4 4 4 4 4 1 4 4)
  local addrs tmp i a
  addrs="$(member_addrs "${members[@]}")"
  tmp="$(mktemp -d)"
  trap "rm -rf '$tmp'" RETURN

  local dump_cmds=(-c "init")
  i=0
  while IFS= read -r a; do
    dump_cmds+=(-c "dump_image $tmp/$i $a ${sizes[$i]}")
    i=$((i+1))
  done <<<"$addrs"
  ocd "${dump_cmds[@]}"

  STATES="$STATES" python3 - "$tmp" <<'EOF'
import os, struct, sys
tmp = sys.argv[1]
def u8(i):  return open(f"{tmp}/{i}","rb").read()[0]
def u32(i): return struct.unpack("<I", open(f"{tmp}/{i}","rb").read())[0]
def f32(i): return struct.unpack("<f", open(f"{tmp}/{i}","rb").read())[0]
names = dict(t.split(":") for t in os.environ["STATES"].split())
def st(n): return names.get(str(n), f"?({n})")
ERR_BITS = [  # CONTROLLER_ERRORS_t, Firmware/Inc/control_ll.h
    "INVERTER_CALIBRATION_INVALID","PHASE_CURRENTS_INVALID",
    "PHASE_CURRENTS_MEASUREMENT_MISSING","PWM_TIMING_INVALID",
    "PWM_TIMING_UPDATE_MISSING","VBUS_OVERVOLTAGE","VBUS_UNDERVOLTAGE",
    "IBUS_OVERCURRENT","MOTOR_OVERCURRENT","MOTOR_PHASE_LEAKAGE",
    "MOTOR_RESISTANCE_OUT_OF_RANGE","MOTOR_INDUCTANCE_OUT_OF_RANGE",
    "ENCODER_READING_MISSING","ENCODER_ESTIMATE_MISSING",
    "ENCODER_READING_INVALID","ENCODER_FAILURE",
    "PHASE_CURRENT_USAGE_MISSING","PWM_TIMING_USAGE_MISSING",
    "PHASE_CURRENT_LEAKAGE","ENCODER_READING_USAGE_MISSING",
    "MOTOR_UNBALANCED_PHASES","MODULATION",
]
errs = u32(2)
err_names = [n for b, n in enumerate(ERR_BITS) if errs & (1 << b)]
if errs >> len(ERR_BITS):
    err_names.append(f"UNKNOWN(0x{errs >> len(ERR_BITS):x}<<{len(ERR_BITS)})")
print(f"state      = {st(u8(0))} ({u8(0)})   requested = {st(u8(7))} ({u8(7)})")
print(f"proc_result= {u8(1)}")
print(f"errors     = 0x{errs:08x}" + (f"  [{' | '.join(err_names)}]" if err_names else "  [none]"))
print(f"motor      = vel {f32(3):+.3f} rad/s   iq {f32(4):+.3f} A")
print(f"bus        = {f32(5):.3f} V   {f32(6):+.3f} A")
print(f"setpoints  = sp1 {f32(8):g}   sp2 {f32(9):g}")
EOF
}

cmd_states() {
  local tok
  for tok in $STATES; do
    printf "%3d  %s\n" "${tok%%:*}" "${tok#*:}"
  done
}

cmd_reboot() {
  require_elf
  local addr
  addr="$(member_addrs OD_RAM.x4007_rxw_reboot.reboot_request)"
  echo "reboot: writing 1 @ $addr (firmware disarms, flushes console, resets)"
  ocd -c "init" -c "mwb $addr 1"
}

case "${1:-}" in
  state)    shift; cmd_state    "$@" ;;
  setpoint) shift; cmd_setpoint "$@" ;;
  status)   shift; cmd_status   "$@" ;;
  states)   shift; cmd_states   "$@" ;;
  reboot)   shift; cmd_reboot   "$@" ;;
  -h|--help|"") usage 0 ;;
  *) echo "unknown subcommand: $1" >&2; usage ;;
esac
