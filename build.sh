#!/bin/sh
# Build BOTH images (app at repo root + bootloader/) with one command.
# Usage: ./build.sh [BUILD_TYPE]      (default RelWithDebInfo; Debug/Release/MinSizeRel)
#
# Default is RelWithDebInfo, NOT Debug: at the 32 kHz control rate the armed fast loop
# does not fit its 31.25 us budget in a -Og flash-resident build -- the pri-0 ISR
# saturates, SysTick (the sole IWDG feeder) starves, and the board watchdog-reboots
# ~512 ms after arming (measured 2026-06-09: optimized+CCM tick = 72% of budget; Debug
# VEL_CONTROL barely fits, VEL_RAMP does not). Debug at 16 kHz is fine if you need -Og.
#
# The two are independent standalone CMake projects (different FPU ABIs, linker
# scripts and vector-table bases); this just runs both. Build dirs follow the
# repo's CLion convention: cmake-build-<type>. Then flash with ./flash.sh.
set -e
cd "$(dirname "$0")"

TOOLCHAIN_DIR="" # Path to ARM toolchain. Leave blank for system default

BUILD_TYPE="${1:-RelWithDebInfo}"
SUFFIX=$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')
APP_BUILD="cmake-build-$SUFFIX"
BL_BUILD="bootloader/cmake-build-$SUFFIX"

echo "==== app ($BUILD_TYPE) -> $APP_BUILD ===="
cmake -S . -B "$APP_BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" ${TOOLCHAIN_DIR:+"-DTOOLCHAIN_DIR=$TOOLCHAIN_DIR"}
cmake --build "$APP_BUILD" -j

echo "==== bootloader ($BUILD_TYPE) -> $BL_BUILD ===="
cmake -S bootloader -B "$BL_BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" ${TOOLCHAIN_DIR:+"-DTOOLCHAIN_DIR=$TOOLCHAIN_DIR"}
cmake --build "$BL_BUILD" -j

echo
echo "Built:"
ls -1 "$APP_BUILD/oresat-acs-fw-v8.elf" "$BL_BUILD/oresat-acs-bootloader-v8.elf" 2>/dev/null
