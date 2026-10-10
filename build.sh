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

# Toolchain bin dir: $TOOLCHAIN_DIR if set in the environment (empty = PATH), else the pinned
# one if ./toolchain.sh has installed it, else PATH.
if [ -z "${TOOLCHAIN_DIR+set}" ]; then
    TOOLCHAIN_DIR="$(./toolchain.sh --path)"
fi
GCC="${TOOLCHAIN_DIR:+$TOOLCHAIN_DIR/}arm-none-eabi-gcc"
command -v "$GCC" >/dev/null 2>&1 \
    || { echo "ERROR: $GCC not found -- run ./toolchain.sh to fetch the pinned toolchain."; exit 1; }
echo "toolchain: $("$GCC" --version | head -n 1)"

# Bounded: plain `-j` is unlimited under make and takes a 4 GB machine down.
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"

BUILD_TYPE="${1:-RelWithDebInfo}"
case "$BUILD_TYPE" in
    Debug|Release|RelWithDebInfo|MinSizeRel) ;;
    *) echo "ERROR: build type must be Debug, Release, RelWithDebInfo or MinSizeRel"; exit 2 ;;
esac
SUFFIX=$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')
APP_BUILD="cmake-build-$SUFFIX"
BL_BUILD="bootloader/cmake-build-$SUFFIX"

# CMake bakes the compiler into a build dir at first configure and silently ignores a later
# change, so a dir first configured with another toolchain would keep using that one. Start
# such a dir over (its objects are from the other compiler anyway).
GCC_REAL="$(readlink -f "$(command -v "$GCC")")"
fresh_if_other_compiler() { # <build dir>
    was="$(sed -n 's/^set(CMAKE_C_COMPILER "\(.*\)")$/\1/p' "$1"/CMakeFiles/*/CMakeCCompiler.cmake 2>/dev/null | head -n 1)"
    if [ -n "$was" ] && [ "$(readlink -f "$was")" != "$GCC_REAL" ]; then
        echo "$1 was configured with $was -- starting it over with this toolchain"
        rm -rf "$1"
    fi
}

echo "==== app ($BUILD_TYPE) -> $APP_BUILD ===="
fresh_if_other_compiler "$APP_BUILD"
cmake -S . -B "$APP_BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" "-DTOOLCHAIN_DIR=$TOOLCHAIN_DIR"
cmake --build "$APP_BUILD" -j "$JOBS"

echo "==== bootloader ($BUILD_TYPE) -> $BL_BUILD ===="
fresh_if_other_compiler "$BL_BUILD"
cmake -S bootloader -B "$BL_BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" "-DTOOLCHAIN_DIR=$TOOLCHAIN_DIR"
cmake --build "$BL_BUILD" -j "$JOBS"

echo
echo "Built:"
ls -1 "$APP_BUILD/oresat-acs-fw-v8.elf" "$BL_BUILD/oresat-acs-bootloader-v8.elf" 2>/dev/null
