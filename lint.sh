#!/usr/bin/env bash
# lint.sh — warning pass over application code (Firmware/Src) only.
#
# The CMake build deliberately runs without -Wall/-Wextra (and the vendored HAL / CMSIS-DSP /
# CANopenNode would drown out app warnings if it did). This script compiles each Firmware/Src
# translation unit with -Wall -Wextra using the EXACT include paths and defines from the build's
# compile_commands.json, so you get real static-analysis coverage on the code you actually own.
#
# Baseline (2026-06): clean — Firmware/Src has zero -Wall -Wextra warnings. Keep it that way;
# this script is the gate that stops new warnings sneaking in. Deliberately-unused debug locals
# are marked __maybe_unused (typedefs.h); printf-family calls are format-checked via printf.h.
#
# Usage:
#   ./lint.sh                 # uses cmake-build-debug
#   BUILD_DIR=cmake-build-release ./lint.sh
#
# Exit code is non-zero if any warning/error is found (suitable for a pre-commit / CI gate).

set -euo pipefail
cd "$(dirname "$0")"

BUILD_DIR="${BUILD_DIR:-cmake-build-debug}"
DB="$BUILD_DIR/compile_commands.json"

if [[ ! -f "$DB" ]]; then
  echo "error: $DB not found. Configure the build first:" >&2
  echo "  cmake -S . -B $BUILD_DIR -DCMAKE_BUILD_TYPE=Debug" >&2
  exit 2
fi

# Extra warnings beyond the build. -Wno-unused-parameter: the control_law_t callback contract
# (fixed signatures) means several callbacks legitimately ignore args. Tune to taste.
EXTRA_WARNINGS=(-Wall -Wextra -Wno-unused-parameter)

python3 - "$DB" "${EXTRA_WARNINGS[@]}" <<'PY'
import json, shlex, subprocess, sys, re

db_path, *extra = sys.argv[1:]
db = {e['file']: e for e in json.load(open(db_path))}
fw = sorted(f for f in db if '/Firmware/Src/' in f)
if not fw:
    print("error: no Firmware/Src entries in compile_commands.json", file=sys.stderr)
    sys.exit(2)

total = 0
by_cat = {}
for f in fw:
    toks = shlex.split(db[f]['command'])
    out, skip = [], False
    for t in toks:
        if skip:           # drop the arg after -o
            skip = False; continue
        if t == '-o':
            skip = True; continue
        if t == '-c':
            continue
        out.append(t)
    # -fno-diagnostics-color: the compile DB sets -fdiagnostics-color=always; ANSI codes would
    # otherwise corrupt the warning/category counting below. Last color flag wins in GCC.
    out += extra + ['-fsyntax-only', '-fno-diagnostics-color']
    p = subprocess.run(out, cwd=db[f]['directory'], capture_output=True, text=True)
    err = re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', p.stderr)  # strip ANSI defensively
    if err.strip():
        print(err, end='')
    total += len(re.findall(r'\bwarning:|\berror:', err))
    for cat in re.findall(r'\[-W([a-z0-9-]+)\]', err):
        by_cat[cat] = by_cat.get(cat, 0) + 1

rel = lambda p: p.split('/Firmware/Src/')[-1]
print(f"\nlint: {len(fw)} Firmware/Src files checked with {' '.join(extra)}")
if total == 0:
    print("lint: clean ✓")
    sys.exit(0)
print(f"lint: {total} warning(s)/error(s):")
for cat, n in sorted(by_cat.items(), key=lambda x: -x[1]):
    print(f"  {n:4d}  -W{cat}")
sys.exit(1)
PY
