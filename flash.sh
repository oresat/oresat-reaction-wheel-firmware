#!/bin/sh
# Flash BOTH images for the OreSat ACS rev8 board over SWD in one OpenOCD session:
#   bootloader -> 0x08000000   (bootloader/ subproject)   runs first, checks BOOT0
#   app        -> 0x08008000   (this repo)                reaction-wheel firmware
#
# Each is programmed at the load address baked into its ELF, so they never
# overlap. The app must be built relinked to 0x08008000 (it is, on master).
#
# Usage:
#   ./flash.sh [APP_BUILD_DIR] [BL_BUILD_DIR]
#       defaults: cmake-build-debug   bootloader/cmake-build-debug
#   OPENOCD=~/openocd/bin/openocd ./flash.sh      # override the openocd binary
#
# Build first with ./build.sh (or your usual CLion build of each).
set -e
cd "$(dirname "$0")"

APP_BUILD="${1:-cmake-build-debug}"
BL_BUILD="${2:-bootloader/cmake-build-debug}"
OPENOCD="${OPENOCD:-$(command -v openocd || echo "$HOME/openocd/bin/openocd")}"
CFG="$PWD/st_nucleo_g4.cfg"

APP_ELF="$APP_BUILD/oresat-acs-fw-v8.elf"
BL_ELF="$BL_BUILD/oresat-acs-bootloader-v8.elf"

for f in "$BL_ELF" "$APP_ELF"; do
    [ -f "$f" ] || { echo "ERROR: missing $f -- build first (./build.sh)"; exit 1; }
done

echo "openocd : $OPENOCD"
echo "flashing: bootloader -> 0x08000000  ($BL_ELF)"
echo "          app        -> 0x08008000  ($APP_ELF)"
echo

exec "$OPENOCD" -f "$CFG" \
    -c "init" \
    -c "reset halt" \
    -c "program $BL_ELF verify" \
    -c "program $APP_ELF verify" \
    -c "reset run" \
    -c "shutdown"
