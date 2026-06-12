#!/bin/sh
# Delete the bench-default build dirs (app + bootloader). Rebuild with ./build.sh.
set -e
cd "$(dirname "$0")"

rm -rf cmake-build-relwithdebinfo bootloader/cmake-build-relwithdebinfo
echo "Removed cmake-build-relwithdebinfo and bootloader/cmake-build-relwithdebinfo"
