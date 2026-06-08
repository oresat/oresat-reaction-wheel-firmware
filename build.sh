#!/bin/sh
# Build BOTH images (app at repo root + bootloader/) with one command.
# Usage: ./build.sh [BUILD_TYPE]      (default Debug; Release/RelWithDebInfo/MinSizeRel)
#
# The two are independent standalone CMake projects (different FPU ABIs, linker
# scripts and vector-table bases); this just runs both. Build dirs follow the
# repo's CLion convention: cmake-build-<type>. Then flash with ./flash.sh.
set -e
cd "$(dirname "$0")"

BUILD_TYPE="${1:-Debug}"
SUFFIX=$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')
APP_BUILD="cmake-build-$SUFFIX"
BL_BUILD="bootloader/cmake-build-$SUFFIX"

echo "==== app ($BUILD_TYPE) -> $APP_BUILD ===="
cmake -S . -B "$APP_BUILD" -G Ninja -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$APP_BUILD" -j

echo "==== bootloader ($BUILD_TYPE) -> $BL_BUILD ===="
cmake -S bootloader -B "$BL_BUILD" -G Ninja -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$BL_BUILD" -j

echo
echo "Built:"
ls -1 "$APP_BUILD/oresat-acs-fw-v8.elf" "$BL_BUILD/oresat-acs-bootloader-v8.elf" 2>/dev/null
