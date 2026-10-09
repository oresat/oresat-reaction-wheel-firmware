#!/usr/bin/env bash
# Live "top" for the MCU: hardware PC sampling over SWO, decoded by orbtop.
# See docs/swo-pc-sampling.md for the full story.
#
# Zero firmware involvement: the DWT sampler, ITM, TPIU and the PB3 pin mux are
# all configured from the debugger side, so this attaches to whatever is already
# running — including an armed fast loop. Nothing is halted and the DWT samples
# in hardware, so the 16/32 kHz tick budget is untouched.
#
#   ./tools/orbtop.sh                      # profile cmake-build-relwithdebinfo ELF
#   BUILD_DIR=cmake-build-release ./tools/orbtop.sh
#   SWO_HZ=6000000 ./tools/orbtop.sh       # STLINK-V3 handles >2 MHz fine
#   SAMPLE_DIV=8 ./tools/orbtop.sh         # 2x sample rate (~17.6 kHz)
#   EXC=1 ./tools/orbtop.sh                # + exception entry/exit stats (needs fast SWO)
#   ./tools/orbtop.sh -l                   # extra args pass through to orbtop (-l = per-line)
#
# Gotchas:
#  - Holds the ST-Link for the whole session: flash.sh / gdb can't run in parallel.
#  - Any target reset re-runs MX_GPIO_Init (PB3 back to GPIO-out) and clears the
#    DWT/ITM config — restart this script after a reflash or reboot.
#  - The ELF must match what's on the target or the symbol table is fiction.

set -euo pipefail
cd "$(dirname "$0")/.."

BUILD_DIR=${BUILD_DIR:-cmake-build-relwithdebinfo}
ELF=${ELF:-$BUILD_DIR/oresat-acs-fw-v8.elf}
CPU_HZ=${CPU_HZ:-144000000} # HCLK feeding the TPIU; wrong value = wrong SWO baud
SWO_HZ=${SWO_HZ:-2000000}   # 2 MHz works on every ST-Link; V3 goes much higher
SAMPLE_DIV=${SAMPLE_DIV:-16} # 1..16; PC sample rate = CPU_HZ / 1024 / SAMPLE_DIV
EXC=${EXC:-0}                # 1 = also trace exception entry/exit (orbtop -E)
ORB=tools/orbuculum/build
SWO_PORT=3443 # OpenOCD serves raw SWO here; orbuculum mux serves clients on 3402

if [[ ! -x $ORB/orbtop || ! -x $ORB/orbuculum ]]; then
    echo "orbuculum tools not built. One-time setup:"
    echo "  git submodule update --init tools/orbuculum"
    echo "  meson setup $ORB tools/orbuculum -Dlibdwarf:werror=false"
    echo "  ninja -C $ORB"
    exit 1
fi
if [[ ! -f $ELF ]]; then
    echo "ELF not found: $ELF (set BUILD_DIR= or ELF=)"
    exit 1
fi
if ss -ltn 2>/dev/null | grep -qE ":(3402|$SWO_PORT) "; then
    echo "Port 3402/$SWO_PORT already in use — stale openocd/orbuculum running?"
    exit 1
fi

# DWT_CTRL: CYCCNTENA | POSTPRESET=div-1 | CYCTAP=1 (POSTCNT clocked at HCLK/1024)
#           | SYNCTAP=1 (sync packet ~every 2^24 cycles) | PCSAMPLENA
dwt_ctrl=$((1 | (SAMPLE_DIV - 1) << 1 | 1 << 9 | 1 << 10 | 1 << 12))
orbtop_extra=()
if [[ $EXC == 1 ]]; then
    dwt_ctrl=$((dwt_ctrl | 1 << 16)) # EXCTRCENA — ~3 packets per IRQ, needs SWO headroom
    orbtop_extra+=(-E)
fi

echo "PC sampling @ $((CPU_HZ / 1024 / SAMPLE_DIV)) Hz, SWO NRZ @ $SWO_HZ baud, ELF: $ELF"

ocd_log=/tmp/orbtop-openocd.log
mux_log=/tmp/orbtop-mux.log
pids=()
cleanup() {
    [[ ${#pids[@]} -gt 0 ]] && kill "${pids[@]}" 2>/dev/null
    wait 2>/dev/null || true
}
trap cleanup EXIT

# TPIU: NRZ (UART) framing-off; OpenOCD's stm32g4x.cfg pre-enable hook sets
# DBGMCU_CR.TRACE_IOEN. The mmw/mww pokes (target keeps running):
#   GPIOB MODER3 01->10, AFRL3=0  — PB3 from firmware's GPIO-out back to AF0 TRACESWO
#   ITM LAR unlock; TCR = TraceBusID 1 | TXENA (forward DWT) | SYNCENA | ITMENA
#   DWT_CTRL — the PC sampler itself
openocd -f openocd.cfg \
    -c "stm32g4x.tpiu configure -protocol uart -traceclk $CPU_HZ -pin-freq $SWO_HZ -formatter 0 -output :$SWO_PORT" \
    -c init \
    -c "stm32g4x.tpiu enable" \
    -c "mmw 0x48000400 0x00000080 0x00000040" \
    -c "mmw 0x48000420 0x00000000 0x0000F000" \
    -c "mww 0xE0000FB0 0xC5ACCE55" \
    -c "mww 0xE0000E80 0x0001000D" \
    -c "mww 0xE0001000 $(printf '0x%08X' "$dwt_ctrl")" \
    >"$ocd_log" 2>&1 &
pids+=($!)

for _ in $(seq 1 50); do
    ss -ltn 2>/dev/null | grep -q ":$SWO_PORT " && break
    if ! kill -0 "${pids[0]}" 2>/dev/null; then
        echo "openocd died — $ocd_log says:"
        tail -15 "$ocd_log"
        exit 1
    fi
    sleep 0.1
done

"$ORB/orbuculum" -s "localhost:$SWO_PORT" >"$mux_log" 2>&1 &
pids+=($!)

command -v arm-none-eabi-objdump >/dev/null && export OBJDUMP=arm-none-eabi-objdump
# logs live in $ocd_log / $mux_log if anything misbehaves
"$ORB/orbtop" -e "$ELF" "${orbtop_extra[@]}" "$@"
