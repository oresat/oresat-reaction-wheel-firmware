#!/usr/bin/env bash
# CMP4 sweep — scope-free ADC sample-point calibration via OpenOCD (SWD, no halt).
#
# Sweeps the HRTIM Timer A CMP4 ADC-trigger position across the PWM period while the motor
# holds a FIXED duty vector (locked rotor), accumulating mean/variance of the pre-CM-removal
# phase currents at each trigger position. The result is an equivalent-time-sampled
# reconstruction of the intra-period current waveform: the known PWM switching edges act as
# time fiducials, and tools/cmp4_sweep_analyze.py extracts the trigger->sample delay D and
# recommends CMP4 = 24000 - D. See docs/adc-sampling-tuning.md for the full procedure.
#
# REQUIRES: target flashed with the matching ELF, controller in IDLE, rotor free to twitch
# (it locks to the commanded vector at sweep start), OpenOCD >= 0.12 (read_memory).
# The whole run is non-halting (background MEM-AP only) — safe with the motor armed.
#
# Usage:
#   tools/cmp4_sweep.sh run [OUTDIR]      # full orchestration; default OUTDIR=cmp4_sweep_out
#
# Env overrides:
#   ELF=...           firmware ELF (default: cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf —
#                     the sweep arms the fast loop, and Debug builds can't run it at 32 kHz:
#                     the ISR overruns, SysTick/IWDG starves, and the board reboots mid-sweep)
#   OPENOCD_CFG=...   OpenOCD config (default: st_nucleo_g4.cfg)
#   START/STOP/STEP   sweep range in HRTIM ticks (default: 300..23700 step 200)
#   DWELL             ticks accumulated per step (default: 1024 = 32 ms @ 32 kHz)
#   MODA/MODB         fixed modulation vector (default: 0.0375 / 0.012 -> ~2 A locked rotor,
#                     beta != 0 so B/C duties differ and all three edges are distinct)
set -euo pipefail
cd "$(dirname "$0")/.."  # repo root

ELF="${ELF:-cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf}"
CFG="${OPENOCD_CFG:-st_nucleo_g4.cfg}"
START="${START:-300}"
STOP="${STOP:-23700}"
STEP="${STEP:-200}"
DWELL="${DWELL:-1024}"
MODA="${MODA:-0.0375}"
MODB="${MODB:-0.012}"

usage() {
  sed -n '3,/^set -e/p' "$0" | sed -n 's/^# \?//p'
  exit "${1:-2}"
}

# NOTE: no awk early-`exit` here — quitting mid-pipe SIGPIPEs objdump (exit 141), which under
# `set -euo pipefail` kills the whole script SILENTLY at the first lookup. First-match flag
# instead; objdump always runs to completion. (foc_capture.sh has the same latent bug —
# whether it bites depends on where the symbol sits in the table vs. the 64K pipe buffer.)
sym_addr() {
  arm-none-eabi-objdump -t "$ELF" \
    | awk -v sym="$1" '$NF == sym && !f {print "0x"$1; f=1}'
}

require_elf() {
  if [[ ! -f "$ELF" ]]; then
    echo "error: ELF not found: $ELF" >&2
    echo "  (set ELF=... or build with: cmake --build cmake-build-relwithdebinfo -j)" >&2
    exit 1
  fi
}

f32_hex() { # float literal -> 0xHEX of its IEEE-754 LE bit pattern
  python3 -c "import struct;print('0x%08x'%struct.unpack('<I',struct.pack('<f',float('$1')))[0])"
}

cmd_run() {
  require_elf
  local out="${1:-cmp4_sweep_out}"
  mkdir -p "$out"

  local a_cmp4 a_sweep a_moden a_moda a_modb a_shunt a_fsmreq a_fsmstate a_armed a_errors a_duties
  a_cmp4="$(sym_addr adc_trig_cmp4_ticks)"
  a_sweep="$(sym_addr adcsweep)"
  a_moden="$(sym_addr open_loop_fixed_mod_enabled)"
  a_moda="$(sym_addr open_loop_fixed_mod_alpha)"
  a_modb="$(sym_addr open_loop_fixed_mod_beta)"
  a_shunt="$(sym_addr shunt_selection_enabled)"
  a_fsmreq="$(sym_addr fsm_requested_state)"
  a_fsmstate="$(sym_addr fsm_state)"
  a_armed="$(sym_addr controller_armed)"
  a_errors="$(sym_addr controller_errors)"
  a_duties="$(sym_addr new_pwm_timings)"
  for v in "$a_cmp4" "$a_sweep" "$a_moden" "$a_moda" "$a_modb" "$a_shunt" \
           "$a_fsmreq" "$a_fsmstate" "$a_armed" "$a_errors" "$a_duties"; do
    [[ -n "$v" ]] || { echo "error: symbol lookup failed (stale ELF?)" >&2; exit 1; }
  done

  local moda_hex modb_hex
  moda_hex="$(f32_hex "$MODA")"
  modb_hex="$(f32_hex "$MODB")"

  # Record run metadata for the analyzer.
  {
    echo "moda=$MODA"
    echo "modb=$MODB"
    echo "dwell=$DWELL"
    echo "start=$START"
    echo "stop=$STOP"
    echo "step=$STEP"
    echo "elf=$ELF"
    echo "date=$(date -Is)"
  } > "$out/meta.txt"

  local tcl
  tcl="$(mktemp --suffix=.tcl)"
  # Bake the path in now: the trap fires at script exit, after `local tcl` is out of scope
  # (set -u would otherwise kill the trap itself with "unbound variable").
  # shellcheck disable=SC2064
  trap "rm -f '$tcl'" EXIT

  cat > "$tcl" <<EOF
init

# ---- preconditions ----
# NOTE fsm_state / fsm_requested_state are 1-BYTE objects (arm-none-eabi short-enum ABI packs
# fsm_state_t; they sit 2 bytes apart at odd addresses). Byte access only — a 32-bit write to
# fsm_requested_state would clobber fsm_state and a neighboring static.
set fsm [lindex [read_memory $a_fsmstate 8 1] 0]
if {\$fsm != 1} {
    echo "cmp4_sweep: FSM not IDLE (state=\$fsm) - aborting, nothing touched"
    echo "cmp4_sweep: (a nonsense state value usually means the FLASHED image doesn't match"
    echo "cmp4_sweep:  the ELF - flash this branch's relwithdebinfo build first)"
    shutdown error
}

# ---- setup (all while disarmed) ----
set shunt_saved [lindex [read_memory $a_shunt 8 1] 0]
set cmp4_saved  [lindex [read_memory $a_cmp4 32 1] 0]
mwb $a_shunt 0
mww $a_moda $moda_hex
mww $a_modb $modb_hex
mwb $a_moden 1
echo "cmp4_sweep: fixed mod ($MODA, $MODB) set; requesting OPEN_LOOP_CONTROL"
mwb $a_fsmreq 12
sleep 300

set armed [lindex [read_memory $a_armed 8 1] 0]
if {!\$armed} {
    echo "cmp4_sweep: controller failed to arm - restoring and aborting"
    mwb $a_fsmreq 1
    sleep 300
    mwb $a_moden 0
    mwb $a_shunt \$shunt_saved
    shutdown error
}
sleep 200
dump_image $out/duties.bin $a_duties 12

# ---- sweep ----
set npts 0
for {set v $START} {\$v <= $STOP} {incr v $STEP} {
    mww $a_cmp4 \$v
    sleep 3
    mww $a_sweep 0 9
    mww $a_sweep $DWELL
    while {[lindex [read_memory [expr {$a_sweep + 4}] 32 1] 0] < $DWELL} { sleep 5 }
    dump_image $out/cmp4_\$v.bin $a_sweep 36
    incr npts
    if {\$npts % 20 == 0} { echo "cmp4_sweep: \$npts points done (cmp4=\$v)" }
}

# ---- teardown: stop accumulating, back to IDLE, restore everything ----
mww $a_sweep 0
mwb $a_fsmreq 1
sleep 300
mwb $a_moden 0
mwb $a_shunt \$shunt_saved
mww $a_cmp4 \$cmp4_saved

set errs [lindex [read_memory $a_errors 32 1] 0]
set fsm  [lindex [read_memory $a_fsmstate 8 1] 0]
echo "cmp4_sweep: done - \$npts points, fsm_state=\$fsm, controller_errors=\$errs"
if {\$errs != 0} {
    echo "cmp4_sweep: WARNING controller_errors nonzero - decode with STRINGIFY_CONTROLLER_ERRORS bits"
}
shutdown
EOF

  echo "cmp4_sweep: $START..$STOP step $STEP, dwell $DWELL ticks, mod=($MODA,$MODB) -> $out/"
  openocd -f "$CFG" -f "$tcl"
  echo "cmp4_sweep: analyze with: python3 tools/cmp4_sweep_analyze.py $out"
}

case "${1:-}" in
  run) shift; cmd_run "$@" ;;
  -h|--help|"") usage 0 ;;
  *) echo "unknown subcommand: $1" >&2; usage ;;
esac
