#!/usr/bin/env bash
# Bench "CAN over SWD": request FSM states, write setpoints, reboot, and read back the TPDO
# status block via OpenOCD background memory access — no CAN adapter needed for bringup.
#
# SWD writes can't trigger the OD write extensions in can_comms.c, so this writes the variables
# those handlers set and repeats their checks — keep it in sync with od_write_*(). Addresses and
# constants come from the ELF, which must match the flashed image.
#
# Usage:
#   tools/swd_sdo.sh state <name|num>     # request an FSM state, then report what the FSM did
#   tools/swd_sdo.sh setpoint <sp1> [sp2] # setpoint_1 routed by current state; sp2 stored only
#   tools/swd_sdo.sh status               # read 0x4000-0x4002, pending request, encoder cal
#   tools/swd_sdo.sh states               # list FSM state names/numbers
#   tools/swd_sdo.sh reboot               # firmware reboot path (disarm, flush console, reset)
#
# Enter the state FIRST, then write the setpoint: setpoint_1 is routed by the current state.
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

REQUESTABLE=" 1 4 5 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 " # fsm_state_requestable()
NEEDS_CAL=" 4 5 16 21 " # fsm_state_requires_encoder_cal(); +17 unless BACKEMF_CAL_SPIN_TARGET == 0

# CONTROLLER_ERRORS_t bit order (Firmware/Inc/control_ll.h).
ERR_BITS="INVERTER_CALIBRATION_INVALID PHASE_CURRENTS_INVALID
PHASE_CURRENTS_MEASUREMENT_MISSING PWM_TIMING_INVALID PWM_TIMING_UPDATE_MISSING
VBUS_OVERVOLTAGE VBUS_UNDERVOLTAGE IBUS_OVERCURRENT MOTOR_OVERCURRENT MOTOR_PHASE_LEAKAGE
MOTOR_RESISTANCE_OUT_OF_RANGE MOTOR_INDUCTANCE_OUT_OF_RANGE ENCODER_READING_MISSING
ENCODER_ESTIMATE_MISSING ENCODER_READING_INVALID ENCODER_FAILURE PHASE_CURRENT_USAGE_MISSING
PWM_TIMING_USAGE_MISSING PHASE_CURRENT_LEAKAGE ENCODER_READING_USAGE_MISSING
MOTOR_UNBALANCED_PHASES MODULATION NOT_CALIBRATED"

die() { echo "$*" >&2; exit 1; }

require_elf() {
  if [[ ! -f "$ELF" ]]; then
    echo "error: ELF not found: $ELF" >&2
    echo "  (set ELF=... or build with: cmake --build cmake-build-relwithdebinfo -j)" >&2
    exit 1
  fi
}

# gdb_eval FMT EXPR [FMT EXPR ...]: one value per line, from the ELF's debug info.
gdb_eval() {
  local args=() n=0 out
  while [[ $# -ge 2 ]]; do
    args+=(-ex "printf \"@@ $1\n\", $2")
    shift 2; n=$((n + 1))
  done
  out="$(arm-none-eabi-gdb -batch -nx -q "${args[@]}" "$ELF" 2>/dev/null | sed -n 's/^@@ //p' || true)"
  if [[ -z "$out" || "$(wc -l <<<"$out")" -ne $n ]]; then
    echo "error: failed to resolve symbols from $ELF (debug info present? ELF matches source?)" >&2
    exit 1
  fi
  echo "$out"
}

# Addresses, one per line. File-statics are written 'file.c'::name.
member_addrs() {
  local args=() m
  for m in "$@"; do args+=("0x%x" "(unsigned int)&($m)"); done
  gdb_eval "${args[@]}"
}

# ocd_vals N CMD...: run OpenOCD commands, print the N values echoed as "@@ ..."; fewer is an error.
ocd_vals() {
  local want="$1" cmds=(-c "init") c out
  shift
  for c in "$@"; do cmds+=(-c "$c"); done
  out="$(openocd -f "$CFG" "${cmds[@]}" -c "exit" 2>&1 | sed -n 's/^@@ //p' || true)"
  if [[ "$(grep -c . <<<"$out")" -ne "$want" ]]; then
    echo "error: OpenOCD returned no/partial data (probe connected? target powered?)" >&2
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

# CONTROLLER_ERRORS_t bitmask -> "A | B" (or "none").
decode_errs() {
  local e=$(( $1 )) i=0 n out=()
  for n in $ERR_BITS; do
    (( (e >> i) & 1 )) && out+=("$n")
    i=$((i + 1))
  done
  (( e >> i )) && out+=("UNKNOWN(0x$(printf %x $((e >> i)))<<$i)")
  if [[ ${#out[@]} -eq 0 ]]; then echo "none"; else local IFS='|'; echo "${out[*]}" | sed 's/|/ | /g'; fi
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
  local num name out a vals
  num="$(state_num "$1")"
  name="$(state_name "$num")"
  [[ "$REQUESTABLE" == *" $num "* ]] ||
    die "refused: $name ($num) is not requestable (over CAN: SDO abort 0x06090030)"

  out="$(member_addrs "'fsm.c'::fsm_requested_state" "'fsm.c'::fsm_state" controller_errors \
                      "'encoder.c'::dir_cal_valid" "'encoder.c'::offset_cal_valid" \
                      OD_RAM.x4004_rxw_requested_state.req_state)"
  mapfile -t a <<<"$out" # 0 request, 1 state, 2 errors, 3 dir-cal, 4 offset-cal, 5 OD 0x4004

  local needs="$NEEDS_CAL"
  [[ "$(gdb_eval "%.9g" BACKEMF_CAL_SPIN_TARGET)" == "0" ]] || needs+="17 "
  if [[ "$needs" == *" $num "* ]]; then
    out="$(ocd_vals 2 "echo \"@@ [mrb ${a[3]}]\"" "echo \"@@ [mrb ${a[4]}]\"")"
    mapfile -t vals <<<"$out"
    if (( !vals[0] || !vals[1] )); then
      die "refused: $name needs a valid encoder cal (dir $((vals[0])), offset $((vals[1])); over CAN:" \
          "SDO abort 0x08000022) — run 9 -> 15 -> 21 -> 18, or 20 to restore a saved one"
    fi
  fi

  # As od_write_requested_state(): OD copy, then the FSM request; read back after ~100 ticks.
  echo "state: request $name ($num)"
  out="$(ocd_vals 3 "mwb ${a[5]} $num" "mwb ${a[0]} $num" "sleep 100" \
                    "echo \"@@ [mrb ${a[1]}]\"" "echo \"@@ [mrb ${a[0]}]\"" "echo \"@@ [mrw ${a[2]}]\"")"
  mapfile -t vals <<<"$out"
  local st=$((vals[0])) req=$((vals[1])) errs=$((vals[2]))
  if (( st == num )); then
    echo "  now in $name"
  elif (( req == num )); then
    echo "  still pending — the FSM tick hasn't taken it (state $(state_name "$st"))"
  else
    echo "  FSM is in $(state_name "$st") ($st), errors 0x$(printf %08x "$errs") [$(decode_errs "$errs")]"
    if (( errs != 0 )); then
      echo "  (latched errors make the FSM drop every request but CLEAR_ERRORS (13))"
    else
      echo "  (one-shot states like 13 and 18-20 finish within a tick — 'status' shows proc_result)"
    fi
  fi
}

cmd_setpoint() {
  [[ $# -ge 1 && $# -le 2 ]] || usage
  require_elf
  # od_write_setpoint()'s check: a NaN torque would become full negative current (review P1-4)
  python3 -c 'import math,sys; v=float(sys.argv[1]); sys.exit(0 if math.isfinite(v) and abs(v) <= 1e5 else 1)' "$1" \
    2>/dev/null || die "refused: setpoint_1 '$1' must be a finite number within ±1e5 (over CAN: SDO abort 0x06090030)"

  local out a vals st
  out="$(member_addrs "'fsm.c'::fsm_state" "'encoder.c'::encoder_dir" \
                      OD_RAM.x4005_rxw_setpoint.setpoint_1 OD_RAM.x4005_rxw_setpoint.setpoint_2 \
                      "'closed_loop_controller.c'::torque_setpoint" idq_setpoint.d idq_setpoint.q \
                      "'closed_loop_controller.c'::vel_setpoint" \
                      "'closed_loop_controller.c'::ramped_vel_setpoint")"
  mapfile -t a <<<"$out" # 0 state, 1 encoder_dir, 2-3 OD 0x4005, 4 torque, 5-6 idq, 7 vel, 8 ramp
  out="$(ocd_vals 2 "echo \"@@ [mrb ${a[0]}]\"" "echo \"@@ [mrw ${a[1]}]\"")"
  mapfile -t vals <<<"$out"
  st=$((vals[0]))

  local writes=("mww ${a[2]} $(f2hex "$1")")
  [[ $# -eq 2 ]] && writes+=("mww ${a[3]} $(f2hex "$2")")
  case "$st" in
    4) # as set_torque_setpoint(): torque * dir / Kt, clamped, into idq_setpoint
       local consts kt lim calc tq iq dir
       consts="$(gdb_eval "%.9g" MOTOR_TORQUE_CONSTANT "%.9g" MOTOR_CURRENT_CMD_LIMIT)"
       kt="$(sed -n 1p <<<"$consts")"; lim="$(sed -n 2p <<<"$consts")"
       calc="$(python3 -c '
import struct, sys
v, dirbits, kt, lim = float(sys.argv[1]), int(sys.argv[2], 0), float(sys.argv[3]), float(sys.argv[4])
d = struct.unpack("<f", struct.pack("<I", dirbits))[0]
tq = struct.unpack("<f", struct.pack("<f", v * d))[0]
iq = max(-lim, min(tq / kt, lim))
print(repr(tq), repr(iq), repr(d))' "$1" "${vals[1]}" "$kt" "$lim")"
       read -r tq iq dir <<<"$calc"
       echo "setpoint: torque $1 N*m -> iq $iq A (encoder_dir $dir, Kt $kt, clamp ±$lim A)"
       writes+=("mww ${a[4]} $(f2hex "$tq")" "mww ${a[5]} 0x00000000" "mww ${a[6]} $(f2hex "$iq")") ;;
    5) echo "setpoint: velocity $1"
       writes+=("mww ${a[7]} $(f2hex "$1")") ;;
    16) echo "setpoint: ramped velocity target $1"
       writes+=("mww ${a[8]} $(f2hex "$1")") ;;
    *) echo "setpoint: stored at 0x4005, applied to nothing — $(state_name "$st") doesn't consume" \
            "setpoints (same as CAN); enter 4/5/16 first" ;;
  esac
  [[ $# -eq 2 ]] && echo "  setpoint_2 = $2 stored (reserved: applied nowhere)"
  ocd_vals 1 "${writes[@]}" 'echo "@@ ok"' >/dev/null
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
    "'fsm.c'::fsm_requested_state"
    OD_RAM.x4005_rxw_setpoint.setpoint_1
    OD_RAM.x4005_rxw_setpoint.setpoint_2
    "'encoder.c'::dir_cal_valid"
    "'encoder.c'::offset_cal_valid"
  )
  local sizes=(1 1 4 4 4 4 4 1 4 4 1 1)
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

  STATES="$STATES" ERR_BITS="$ERR_BITS" python3 - "$tmp" <<'EOF'
import os, struct, sys
tmp = sys.argv[1]
def u8(i):  return open(f"{tmp}/{i}","rb").read()[0]
def u32(i): return struct.unpack("<I", open(f"{tmp}/{i}","rb").read())[0]
def f32(i): return struct.unpack("<f", open(f"{tmp}/{i}","rb").read())[0]
names = dict(t.split(":") for t in os.environ["STATES"].split())
def st(n): return names.get(str(n), f"?({n})")
ERR_BITS = os.environ["ERR_BITS"].split()  # CONTROLLER_ERRORS_t, Firmware/Inc/control_ll.h
errs = u32(2)
err_names = [n for b, n in enumerate(ERR_BITS) if errs & (1 << b)]
if errs >> len(ERR_BITS):
    err_names.append(f"UNKNOWN(0x{errs >> len(ERR_BITS):x}<<{len(ERR_BITS)})")
req = u8(7)
print(f"state      = {st(u8(0))} ({u8(0)})   pending request = " + ("none" if req == 0 else f"{st(req)} ({req})"))
print(f"proc_result= {u8(1)}")
print(f"errors     = 0x{errs:08x}" + (f"  [{' | '.join(err_names)}]" if err_names else "  [none]"))
print(f"motor      = vel {f32(3):+.3f} rad/s   iq {f32(4):+.3f} A")
print(f"bus        = {f32(5):.3f} V   {f32(6):+.3f} A")
print(f"setpoints  = sp1 {f32(8):g}   sp2 {f32(9):g}   (0x4005 copy)")
ok = lambda b: "valid" if b else "MISSING"
print(f"encoder cal= dir {ok(u8(10))}, offset {ok(u8(11))}")
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
  # od_write_reboot()'s flag: the foreground disarms, flushes the console, resets
  addr="$(member_addrs "'can_comms.c'::reboot_pending")"
  echo "reboot: setting reboot_pending (firmware disarms, flushes console, resets)"
  ocd_vals 1 "mwb $addr 1" 'echo "@@ ok"' >/dev/null
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
