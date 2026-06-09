#!/bin/sh
# Build a SINGLE combined image for cold-bootstrapping a board through the STM32
# ROM (system-memory) bootloader -- i.e. when the custom bootloader is NOT on the
# chip yet and you can only flash over UART / USB DFU (stm32flash / dfu-util).
#
# Layout (flash the whole file at 0x08000000):
#   0x08000000  bootloader            (bootloader/)
#   ......       0xFF padding          (reads as erased flash)
#   0x08008000  app                   (this repo)
#
# Because the two images live at fixed, non-overlapping addresses, one flat .bin
# written at 0x08000000 places both correctly. After flashing, set BOOT0 LOW and
# power-cycle: chip -> custom bootloader -> app, and the app rewrites the option
# bytes (nBOOT0=1, nSWBOOT0=0) on first boot so BOOT0 becomes a plain GPIO.
# Full procedure: docs/bootloader-integration.md (Cold bootstrap).
#
# Usage: ./bootstrap-bin.sh [APP_BUILD_DIR] [BL_BUILD_DIR]
#        defaults: cmake-build-relwithdebinfo  bootloader/cmake-build-relwithdebinfo
# Build both first:  ./build.sh
set -e
cd "$(dirname "$0")"

APP_BUILD="${1:-cmake-build-relwithdebinfo}"
BL_BUILD="${2:-bootloader/cmake-build-relwithdebinfo}"
OBJCOPY="${OBJCOPY:-arm-none-eabi-objcopy}"
APP_BASE=0x08008000          # MUST match the app linker ORIGIN (STM32G474VEHX_FLASH.ld)
OUT=combined-bootstrap.bin

BL_ELF="$BL_BUILD/oresat-acs-bootloader-v8.elf"
APP_ELF="$APP_BUILD/oresat-acs-fw-v8.elf"
for f in "$BL_ELF" "$APP_ELF"; do
    [ -f "$f" ] || { echo "ERROR: missing $f -- run ./build.sh first"; exit 1; }
done

tmp_bl=".bootstrap.$$.bl.bin"
tmp_app=".bootstrap.$$.app.bin"
trap 'rm -f "$tmp_bl" "$tmp_app"' EXIT

# Bootloader, 0xFF-padded up to the app base so the gap reads as erased flash.
"$OBJCOPY" -O binary --gap-fill 0xFF --pad-to "$APP_BASE" "$BL_ELF" "$tmp_bl"
# App, flat (its byte 0 == APP_BASE). objcopy -O binary spans only used sections,
# so this is ~app size, not the full 480K region.
"$OBJCOPY" -O binary "$APP_ELF" "$tmp_app"
cat "$tmp_bl" "$tmp_app" > "$OUT"

SZ=$(wc -c < "$OUT")
echo "wrote $OUT  ($SZ bytes)  -- flash the WHOLE file at 0x08000000"
echo
echo "UART  (ROM bootloader; connect to USART1 PA9/PA10 or USART2 PA2/PA3 -- NOT PE0/PE1):"
echo "  stm32flash -b 115200 -w $OUT -S 0x08000000 -v /dev/ttyUSB0"
echo "USB DFU (ROM bootloader on PA11/PA12), same file:"
echo "  dfu-util -a 0 -s 0x08000000 -D $OUT"
echo
echo "Then set BOOT0 LOW and power-cycle -> the app rewrites the option bytes on first boot."
