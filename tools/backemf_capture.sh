#!/usr/bin/env bash
# Back-EMF coast-down capture readout via OpenOCD (SWD background memory access).
#
# Unlike foc_capture there is no arm/stop: the capture is driven by the FSM state
# (FSM_STATE_BACKEMF_CAL = 17). Request the state, let the run finish (or flick the
# wheel in hand-spin mode), then dump. The buffer is linear (no wrap) and stays
# frozen after the state exits.
#
# Usage:
#   tools/backemf_capture.sh status            # phase + sample count + run metadata
#   tools/backemf_capture.sh dump [OUT_BIN]    # buffer -> OUT_BIN, metadata -> OUT_BIN.meta.json
#   tools/backemf_capture.sh fit [BIN...] [opts]  # offline fit (tools/backemf_cal_fit.py)
#   tools/backemf_capture.sh grab [BIN] [opts] # dump + fit in one shot
#   tools/backemf_capture.sh show-offset       # read live encoder phase_offset (turns) via SWD
#   tools/backemf_capture.sh poke-offset VAL   # write encoder phase_offset (turns) via SWD —
#                                              # for the method-doc §7 sign check (+5 deg elec
#                                              # = +0.002315 turns at 6 pp). Poke while IDLE.
#
# Env overrides:
#   ELF=...          path to oresat-acs-fw-v8.elf (default: cmake-build-relwithdebinfo/...)
#   OPENOCD_CFG=...  OpenOCD config file (default: st_nucleo_g4.cfg)
set -euo pipefail
cd "$(dirname "$0")/.."  # repo root

ELF="${ELF:-cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf}"
CFG="${OPENOCD_CFG:-st_nucleo_g4.cfg}"

usage() {
  sed -n '3,/^set -e/p' "$0" | sed -n 's/^# \?//p'
  exit "${1:-2}"
}

# Symbol address (0xHEX) / size (decimal) from the ELF. Same no-early-exit awk as
# foc_capture.sh (early exit -> SIGPIPE -> pipefail abort).
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

ocd() {
  if [[ "${OPENOCD_VERBOSE:-}" == "1" ]]; then
    openocd -f "$CFG" "$@" -c "exit"
  else
    openocd -f "$CFG" "$@" -c "exit" 2>&1 | grep -vE \
      '^(Open On-Chip|Licensed under|For bug reports|Info :|Warn : |srst_|none|adapter speed:)' || true
  fi >&2
}

# Read the capture metadata words (idx, phase enum, offsets...) into a temp dir.
# Layout knowledge lives here + backemf_cal.h + backemf_cal_fit.py — keep in sync.
read_meta() {  # $1 = tmpdir
  local tmp="$1"
  local idx_a phase_a off_a dir_a omc_a cutl_a
  idx_a="$(sym_addr backemf_capture_idx)"
  phase_a="$(sym_addr bemf_phase)"
  off_a="$(sym_addr backemf_meta_phase_offset)"
  dir_a="$(sym_addr backemf_meta_enc_dir)"
  omc_a="$(sym_addr backemf_meta_omega_cut)"
  cutl_a="$(sym_addr backemf_meta_cut_loop)"
  [[ -n "$idx_a" && -n "$off_a" && -n "$dir_a" && -n "$omc_a" && -n "$cutl_a" ]] \
    || { echo "error: backemf symbols not found in $ELF (old firmware?)" >&2; exit 1; }
  local phase_cmd=()
  [[ -n "$phase_a" ]] && phase_cmd=(-c "dump_image $tmp/phase $phase_a 1")
  ocd -c "init" \
      -c "dump_image $tmp/idx $idx_a 4" \
      "${phase_cmd[@]}" \
      -c "dump_image $tmp/off $off_a 4" \
      -c "dump_image $tmp/dir $dir_a 4" \
      -c "dump_image $tmp/omc $omc_a 4" \
      -c "dump_image $tmp/cutl $cutl_a 4"
}

meta_json() {  # $1 = tmpdir -> json on stdout
  python3 - "$1" <<'EOF'
import json, struct, sys, os
t = sys.argv[1]
def u32(n): return struct.unpack('<I', open(os.path.join(t,n),'rb').read())[0]
def f32(n): return struct.unpack('<f', open(os.path.join(t,n),'rb').read())[0]
m = {
    "idx": u32("idx"),
    "phase_offset_turns": f32("off"),
    "encoder_dir": f32("dir"),
    "omega_cut_revs": f32("omc"),
    "cut_loop": u32("cutl"),
}
if os.path.exists(os.path.join(t, "phase")):
    m["fw_phase"] = open(os.path.join(t,"phase"),"rb").read()[0]
print(json.dumps(m, indent=2))
EOF
}

cmd_status() {
  require_elf
  local tmp; tmp="$(mktemp -d)"
  trap "rm -rf '$tmp'" RETURN
  read_meta "$tmp"
  meta_json "$tmp"
}

cmd_dump() {
  require_elf
  local out="${1:-backemf_capture.bin}"
  local buf_addr buf_size
  buf_addr="$(sym_addr backemf_capture_buf)"
  buf_size="$(sym_size backemf_capture_buf)"
  [[ -n "$buf_addr" && -n "$buf_size" ]] || { echo "error: backemf_capture_buf not found" >&2; exit 1; }

  local tmp; tmp="$(mktemp -d)"
  trap "rm -rf '$tmp'" RETURN

  # No halt — MEM-AP background read; CPU keeps running (see foc_capture.md "Dump halt safety").
  # The buffer is only written during the COAST phase; dump after the run for a clean snapshot.
  echo "dump: $out <- buf $buf_addr/$buf_size"
  ocd -c "init" -c "dump_image $out $buf_addr $buf_size"
  read_meta "$tmp"
  meta_json "$tmp" > "$out.meta.json"
  echo "wrote $out ($buf_size B) + $out.meta.json:"
  cat "$out.meta.json"
}

cmd_fit() {
  # Pass everything through — the fit script takes one or MORE bins (fwd + rev pooling).
  python3 tools/backemf_cal_fit.py "$@"
}

cmd_grab() {
  local bin="backemf_capture.bin"
  if [[ $# -gt 0 && "$1" != -* ]]; then bin="$1"; shift; fi
  cmd_dump "$bin"
  python3 tools/backemf_cal_fit.py "$bin" "$@"
}

# Read/write encoder.c's live phase_offset (f32, mech turns) over SWD. Used for the method-doc
# §7 empirical sign check: poke a known +5 deg elec perturbation after the bootstrap cal, re-run
# state 17, and the fitted Delta must move by -5 deg. 32-bit MEM-AP writes are atomic so the
# fast loop can't see a torn float, but poke while IDLE anyway — a closed-loop law would see the
# offset step as an instant torque step.
cmd_show_offset() {
  require_elf
  local addr; addr="$(sym_addr phase_offset)"
  [[ -n "$addr" ]] || { echo "error: phase_offset symbol not found" >&2; exit 1; }
  local tmp; tmp="$(mktemp -d)"
  trap "rm -rf '$tmp'" RETURN
  ocd -c "init" -c "dump_image $tmp/off $addr 4"
  python3 -c "import struct;print('phase_offset = %.6f turns' % struct.unpack('<f', open('$tmp/off','rb').read())[0])"
}

cmd_poke_offset() {
  require_elf
  local turns="${1:?usage: poke-offset <turns>}"
  local addr; addr="$(sym_addr phase_offset)"
  [[ -n "$addr" ]] || { echo "error: phase_offset symbol not found" >&2; exit 1; }
  local hex
  hex=$(python3 -c "import struct;print('0x%08x' % struct.unpack('<I', struct.pack('<f', float('$turns')))[0])")
  echo "poke: phase_offset = $turns turns @ $addr ($hex)"
  ocd -c "init" -c "mww $addr $hex"
}

# A/B-toggle the runtime LUT correction (encoder.c enc_lut_enabled) without touching the table
# or the flash blob. For ripple verification: spin at constant speed, foc_capture grab, flip,
# grab again, compare Iq spectra (1/rev line = eccentricity correction working; 6*f_e line
# rising with LUT on = contaminated table). Flip while IDLE, then re-enter the control state.
cmd_lut_enable() {
  require_elf
  local val="${1:?usage: lut-enable 0|1}"
  local addr; addr="$(sym_addr enc_lut_enabled)"
  [[ -n "$addr" ]] || { echo "error: enc_lut_enabled symbol not found (old firmware?)" >&2; exit 1; }
  echo "lut-enable: $val @ $addr"
  ocd -c "init" -c "mwb $addr $val"
}

case "${1:-}" in
  status)      shift; cmd_status      "$@" ;;
  dump)        shift; cmd_dump        "$@" ;;
  fit)         shift; cmd_fit         "$@" ;;
  grab)        shift; cmd_grab        "$@" ;;
  show-offset) shift; cmd_show_offset "$@" ;;
  poke-offset) shift; cmd_poke_offset "$@" ;;
  lut-enable)  shift; cmd_lut_enable  "$@" ;;
  -h|--help|"") usage 0 ;;
  *) echo "unknown subcommand: $1" >&2; usage ;;
esac
