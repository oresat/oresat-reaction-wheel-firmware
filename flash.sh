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
#       defaults: cmake-build-relwithdebinfo   bootloader/cmake-build-relwithdebinfo
#   OPENOCD=~/openocd/bin/openocd ./flash.sh      # override the openocd binary
#
# Build first with ./build.sh (or your usual CLion build of each).
set -e
cd "$(dirname "$0")"

APP_BUILD="${1:-cmake-build-relwithdebinfo}"
BL_BUILD="${2:-bootloader/cmake-build-relwithdebinfo}"
OPENOCD="${OPENOCD:-$(command -v openocd || echo "$HOME/openocd/bin/openocd")}"
CFG="$PWD/st_nucleo_g4.cfg"

APP_ELF="$APP_BUILD/oresat-acs-fw-v8.elf"
BL_ELF="$BL_BUILD/oresat-acs-bootloader-v8.elf"

for f in "$BL_ELF" "$APP_ELF"; do
    [ -f "$f" ] || { echo "ERROR: missing $f -- build first (./build.sh)"; exit 1; }
done

# Refuse an app ELF with a LOAD segment below its base: `program` would write it over the
# bootloader (what a link without --nmagic gives on 64K-page toolchains; see CMakeLists.txt).
APP_BASE=0x08008000
READELF="$(command -v arm-none-eabi-readelf || command -v readelf || true)"
if [ -n "$READELF" ]; then
    for pa in $("$READELF" -lW "$APP_ELF" | awk '$1 == "LOAD" && $5 !~ /^0x0+$/ { print $4 }'); do
        if [ $((pa)) -lt $((APP_BASE)) ]; then
            echo "ERROR: $APP_ELF has a LOAD segment at $pa, below the app base $APP_BASE --"
            echo "       flashing it would overwrite the bootloader. Rebuild with ./build.sh."
            exit 1
        fi
    done
else
    echo "WARNING: no readelf found -- skipping the app-ELF load-address check"
fi

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
