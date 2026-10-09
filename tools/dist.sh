#!/bin/sh
# Collect what a release ships into dist/: both images as .elf/.hex/.bin, the combined
# cold-bootstrap image, SHA256SUMS and BUILDINFO. CI runs this on a tag and uploads the result;
# ./flash-release.sh fetches exactly these names. Build first (./build.sh).
#
# Usage: tools/dist.sh [APP_BUILD_DIR] [BL_BUILD_DIR]
#        defaults: cmake-build-relwithdebinfo  bootloader/cmake-build-relwithdebinfo
set -e
cd "$(dirname "$0")/.."

APP_BUILD="${1:-cmake-build-relwithdebinfo}"
BL_BUILD="${2:-bootloader/cmake-build-relwithdebinfo}"
APP="$APP_BUILD/oresat-acs-fw-v8"
BL="$BL_BUILD/oresat-acs-bootloader-v8"

for f in "$APP.elf" "$APP.hex" "$APP.bin" "$BL.elf" "$BL.hex" "$BL.bin"; do
    [ -f "$f" ] || { echo "ERROR: missing $f -- run ./build.sh first"; exit 1; }
done

rm -rf dist
mkdir dist
cp "$APP.elf" "$APP.hex" "$APP.bin" "$BL.elf" "$BL.hex" "$BL.bin" dist/

# The toolchain this build dir was actually configured with, not whatever is on PATH.
GCC="$(sed -n 's/^set(CMAKE_C_COMPILER "\(.*\)")$/\1/p' "$APP_BUILD"/CMakeFiles/*/CMakeCCompiler.cmake | head -n 1)"
[ -x "$GCC" ] || { echo "ERROR: cannot tell which compiler built $APP_BUILD"; exit 1; }

OBJCOPY="${GCC%gcc}objcopy" ./bootstrap-bin.sh "$APP_BUILD" "$BL_BUILD" >/dev/null
mv combined-bootstrap.bin dist/

{
    echo "version:    ${CI_COMMIT_TAG:-$(git describe --tags --always --dirty 2>/dev/null || echo unknown)}"
    echo "commit:     ${CI_COMMIT_SHA:-$(git rev-parse HEAD 2>/dev/null || echo unknown)}"
    echo "build type: $(sed -n 's/^CMAKE_BUILD_TYPE:[A-Z]*=//p' "$APP_BUILD/CMakeCache.txt")"
    echo "compiler:   $("$GCC" --version | head -n 1)"
    echo "built:      $(date -u +%Y-%m-%dT%H:%M:%SZ)${CI_PIPELINE_URL:+  $CI_PIPELINE_URL}"
} > dist/BUILDINFO.txt

(cd dist && sha256sum -- * > SHA256SUMS)
echo "dist/:"
ls -l dist | sed 1d
