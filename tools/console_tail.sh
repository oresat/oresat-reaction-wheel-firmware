#!/usr/bin/env bash
# Read the deferred-console ring out of a live (or wedged) target over SWD.
#
# The console (Firmware/Src/console.c) keeps the last ~2 KB of printf output in a RAM ring;
# drained bytes are not erased, so this doubles as a postmortem black box: after a hang or
# hardfault the most recent console output is recoverable without a UART terminal attached.
# The dump is a background MEM-AP read (no halt) — the CPU keeps running.
#
# Usage:
#   tools/console_tail.sh        # ring history (oldest -> newest) + undrained-bytes marker
#
# Env overrides:
#   ELF=...          path to oresat-acs-fw-v8.elf (default: cmake-build-relwithdebinfo/...)
#   OPENOCD_CFG=...  OpenOCD config file (default: openocd.cfg)
set -euo pipefail
cd "$(dirname "$0")/.."  # repo root

ELF="${ELF:-cmake-build-relwithdebinfo/oresat-acs-fw-v8.elf}"
CFG="${OPENOCD_CFG:-openocd.cfg}"

sym_addr() {
  arm-none-eabi-objdump -t "$ELF" \
    | awk -v sym="$1" '$NF == sym && !f {print "0x"$1; f=1}'
}
sym_size() {
  arm-none-eabi-objdump -t "$ELF" \
    | awk -v sym="$1" '$NF == sym && !f {printf "%d\n", strtonum("0x"$(NF-1)); f=1}'
}

ocd() {
  if [[ "${OPENOCD_VERBOSE:-}" == "1" ]]; then
    openocd -f "$CFG" "$@" -c "exit"
  else
    openocd -f "$CFG" "$@" -c "exit" 2>&1 | grep -vE \
      '^(Open On-Chip|Licensed under|For bug reports|Info :|Warn : |srst_|none|adapter speed:|DEPRECATED|force hard)' || true
  fi >&2
}

[[ -f "$ELF" ]] || { echo "error: ELF not found: $ELF (build first)" >&2; exit 1; }

ring_a="$(sym_addr console_ring)";    ring_n="$(sym_size console_ring)"
head_a="$(sym_addr console_head)";    tail_a="$(sym_addr console_tail)"
drop_a="$(sym_addr console_dropped)"
[[ -n "$ring_a" && -n "$ring_n" && -n "$head_a" && -n "$tail_a" && -n "$drop_a" ]] \
  || { echo "error: console symbols not found in $ELF (firmware without console.c?)" >&2; exit 1; }

tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
ocd -c "init" \
    -c "dump_image $tmp/ring $ring_a $ring_n" \
    -c "dump_image $tmp/head $head_a 4" \
    -c "dump_image $tmp/tail $tail_a 4" \
    -c "dump_image $tmp/drop $drop_a 4"

python3 - "$tmp" "$ring_n" <<'EOF'
import struct, sys, os
t, n = sys.argv[1], int(sys.argv[2])
ring = open(os.path.join(t, "ring"), "rb").read()
u32 = lambda f: struct.unpack("<I", open(os.path.join(t, f), "rb").read())[0]
head, tail, dropped = u32("head"), u32("tail"), u32("drop")
# Indices are free-running; the ring holds bytes [head-n, head). Before the first wrap the
# unwritten region is NULs — strip them.
start = head - n if head >= n else 0
hist = bytes(ring[i % n] for i in range(start, head)).lstrip(b"\x00")
print(f"== console ring: head={head} tail={tail} undrained={head - tail} dropped={dropped} ==")
sys.stdout.write(hist.decode("ascii", "replace"))
if head != tail:
    print(f"\n== trailing {head - tail} byte(s) were appended but not yet pumped to the UART ==")
EOF
