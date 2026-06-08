#!/usr/bin/env bash
# Dump the FOC capture buffer from a running target via OpenOCD.
#
# Usage: tools/foc_capture_dump.sh [ELF] [OUT_BIN]
#   ELF     - path to oresat-acs-fw-v8.elf (default: cmake-build-debug/...)
#   OUT_BIN - path to output binary       (default: foc_capture.bin)
#
# Looks up the runtime address + size of `foc_capture_buf` from the ELF symbol
# table, halts the target, dumps the bytes, resumes. Run while target is
# running and the host has already written reserved_1 = 1 via SDO to fill the
# buffer (foc_capture_state should be FOC_CAPTURE_DONE = 2 by then).
set -euo pipefail

ELF="${1:-cmake-build-debug/oresat-acs-fw-v8.elf}"
OUT="${2:-foc_capture.bin}"
CFG="${OPENOCD_CFG:-st_nucleo_g4.cfg}"

if [[ ! -f "$ELF" ]]; then
  echo "error: ELF not found: $ELF" >&2
  exit 1
fi

# Symbol address + size from the ELF
read ADDR_HEX SIZE_HEX < <(
  arm-none-eabi-objdump -t "$ELF" \
    | awk '$NF == "foc_capture_buf" {printf "0x%s 0x%s\n", $1, $(NF-1); exit}'
)
if [[ -z "${ADDR_HEX:-}" ]]; then
  echo "error: symbol foc_capture_buf not found in $ELF" >&2
  exit 1
fi
SIZE=$((SIZE_HEX))

echo "foc_capture_buf @ $ADDR_HEX, size $SIZE bytes -> $OUT"

openocd -f "$CFG" \
  -c "init" \
  -c "halt" \
  -c "dump_image $OUT $ADDR_HEX $SIZE" \
  -c "resume" \
  -c "exit"

echo "wrote $OUT"
echo "decode: python3 tools/decode_foc_capture.py $OUT"
