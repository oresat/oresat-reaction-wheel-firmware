#!/usr/bin/env bash
# Flash a tagged release onto a wheel over SWD. Nothing is built locally: the bootloader and app
# that CI built for the tag are downloaded, checksum-verified and programmed, and the board is
# checked to have booted. Needs openocd (>= 0.11), curl and sha256sum -- no ARM toolchain.
#
#   ./flash-release.sh v1.0.2             flash that release
#   ./flash-release.sh                    list the releases there are
#   ./flash-release.sh v1.0.2 --erase-cal also wipe the encoder calibration (default: kept)
#   ./flash-release.sh v1.0.2 --serial SN pick an ST-Link when several are plugged in
#   ./flash-release.sh v1.0.2 -y          don't ask before erasing
#
# In order: download + verify into releases/<tag>/ (kept; the ELFs there are what tools/*.sh
# need) -> back the whole flash up to backups/ -> remove write/read protection -> put the option
# bytes where the firmware wants them -> erase everything except the calibration page ->
# program + verify -> reset and confirm the app is running.
#
# Env: OPENOCD=<binary>   RELEASE_URL=<base holding <tag>/<file>; file:// works>
set -euo pipefail
cd "$(dirname "$0")"

PROJECT="solomondg/oresat-acs-fw-v8"
API="https://gitlab.com/api/v4/projects/${PROJECT//\//%2F}"
RELEASE_URL="${RELEASE_URL:-$API/packages/generic/oresat-acs-fw}"
OPENOCD="${OPENOCD:-openocd}"
CFG="$PWD/openocd.cfg"

BL=oresat-acs-bootloader-v8
APP=oresat-acs-fw-v8
FILES="$BL.hex $APP.hex $BL.elf $APP.elf BUILDINFO.txt"

FLASH_BASE=0x08000000
FLASH_SIZE=0x80000
APP_BASE=0x08008000
CAL_ADDR=0x0807F800 # cal_store page: last 2K page of bank 2 (Firmware/Inc/cal_store.h)
PAGE=0x800

# FLASH_OPTR as the firmware enforces it at boot (nSWBOOT0=0, nBOOT0=1, nBOOT1=1, BOR_LEV=4),
# everything else at the factory value (dual bank, software watchdogs, NRST bidirectional).
# The mask leaves out the reserved bits and RDP[7:0]: this script never writes the RDP byte,
# so it cannot set read protection.
OPTR_GOLDEN=0x7BCF7400
OPTR_MASK=0x7FDF7700

die()  { echo "ERROR: $*" >&2; exit 1; }
step() { echo; echo "==> $*"; }

# ---------------------------------------------------------------- arguments
TAG="" ERASE_CAL=0 YES=0 SERIAL=""
while [ $# -gt 0 ]; do
    case "$1" in
        --erase-cal) ERASE_CAL=1 ;;
        --serial)    shift; SERIAL="${1:-}"; [ -n "$SERIAL" ] || die "--serial needs a value" ;;
        -y|--yes)    YES=1 ;;
        -h|--help)   sed -n '2,/^set -euo/p' "$0" | sed -n 's/^# \{0,1\}//p'; exit 0 ;;
        -*)          die "unknown option $1 (see --help)" ;;
        *)           [ -z "$TAG" ] || die "one release at a time (got '$TAG' and '$1')"; TAG="$1" ;;
    esac
    shift
done

list_releases() {
    curl --fail --silent --show-error --location "$API/releases?per_page=30" 2>/dev/null \
        | grep -o '"tag_name":"[^"]*"' | sed 's/.*:"\(.*\)"/  \1/' || true
}

if [ -z "$TAG" ]; then
    echo "usage: $0 <release-tag> [--erase-cal] [--serial SN] [-y]"
    echo "releases:"
    r="$(list_releases)"; echo "${r:-  (could not list them -- no network, or none published yet)}"
    exit 2
fi

for t in curl sha256sum; do
    command -v "$t" >/dev/null 2>&1 || die "$t not found -- sudo apt install curl coreutils"
done
command -v "$OPENOCD" >/dev/null 2>&1 || die "openocd not found -- sudo apt install openocd"
ocd_ver="$("$OPENOCD" --version 2>&1 | sed -n '1s/.*Debugger \([0-9]*\)\.\([0-9]*\).*/\1 \2/p')"
read -r ocd_major ocd_minor <<<"${ocd_ver:-0 0}"
[ "$ocd_major" -gt 0 ] || [ "$ocd_minor" -ge 11 ] \
    || die "OpenOCD $ocd_major.$ocd_minor is too old for the STM32G4 (need 0.11 or newer)"
[ -f "$CFG" ] || die "$CFG is missing -- run this from a checkout of the repository"

# ---------------------------------------------------------------- 1. fetch + verify
DIR="releases/$TAG"
sum_ok() { # <file>: listed in SHA256SUMS and matching
    local line
    line="$(awk -v f="$1" '$2 == f' "$DIR/SHA256SUMS" 2>/dev/null | head -n 1)"
    [ -n "$line" ] && (cd "$DIR" && echo "$line" | sha256sum --status -c -)
}
all_ok() { local f; for f in $FILES; do sum_ok "$f" || return 1; done; }
fetch() { # <file>
    curl --fail --silent --show-error --location --retry 2 \
        --output "$DIR/$1.part" "$RELEASE_URL/$TAG/$1" && mv "$DIR/$1.part" "$DIR/$1"
}

step "Release $TAG"
if all_ok; then
    echo "already downloaded: $DIR/"
else
    mkdir -p "$DIR"
    if ! fetch SHA256SUMS; then
        rm -f "$DIR/SHA256SUMS.part"
        rmdir "$DIR" 2>/dev/null || true
        echo "No release '$TAG' at $RELEASE_URL" >&2
        r="$(list_releases)"; [ -z "$r" ] || { echo "releases:" >&2; echo "$r" >&2; }
        exit 1
    fi
    for f in $FILES; do
        echo "fetching $f"
        fetch "$f" || die "download of $f failed"
        sum_ok "$f" || die "$f does not match SHA256SUMS -- delete $DIR/ and try again"
    done
fi
sed 's/^/  /' "$DIR/BUILDINFO.txt"

# ---------------------------------------------------------------- probe plumbing
STLINK_PIDS="3744 3748 374b 374d 374e 374f 3752 3753 3754 3755 3757"
stlinks() { # one line per attached ST-Link: "<devnode> <serial>"
    local d pid
    for d in /sys/bus/usb/devices/*; do
        [ -r "$d/idVendor" ] && [ "$(cat "$d/idVendor")" = 0483 ] || continue
        pid="$(cat "$d/idProduct")"
        case " $STLINK_PIDS " in *" $pid "*) ;; *) continue ;; esac
        printf '/dev/bus/usb/%03d/%03d %s\n' "$(cat "$d/busnum")" "$(cat "$d/devnum")" \
            "$(cat "$d/serial" 2>/dev/null || echo '?')"
    done
}

LOG="$(mktemp)"
trap 'rm -f "$LOG" "$LOG.cal"' EXIT
ocd() { # <tcl command>...: one OpenOCD run, each argument a command; all output to $LOG
    local args=(-f "$CFG") c
    # 'adapter serial' is 0.12+; 0.11 only has hla_serial.
    [ -z "$SERIAL" ] || args+=(-c "if {[catch {adapter serial $SERIAL}]} {hla_serial $SERIAL}")
    for c in "$@"; do args+=(-c "$c"); done
    "$OPENOCD" "${args[@]}" >"$LOG" 2>&1
}
val() { sed -n "s/^$1=//p" "$LOG" | tail -n 1; }

explain_failure() { # after a failed ocd(): say what is wrong in words, then show the log
    local found
    found="$(stlinks)"
    echo >&2
    if [ -z "$found" ]; then
        echo "No ST-Link on USB. Plug it in; inside a VM or WSL it has to be passed through to the guest." >&2
    elif grep -qiE "open failed|LIBUSB_ERROR|unable to open|couldn't open" "$LOG"; then
        if [ ! -w "$(echo "$found" | head -n 1 | cut -d' ' -f1)" ]; then
            cat >&2 <<'EOF'
The ST-Link is plugged in but this user may not open it. Fix it once, then replug the ST-Link:
  echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", MODE="0660", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/49-stlink.rules
  sudo udevadm control --reload-rules
EOF
        else
            echo "The ST-Link is there but OpenOCD could not open it -- is another program using it" >&2
            echo "(another openocd, st-util, STM32CubeIDE/CubeProgrammer)?" >&2
        fi
    elif grep -qiE "init mode failed|unable to connect to the target|target voltage may be too low|Error connecting DP" "$LOG"; then
        echo "The ST-Link answers but the MCU does not: check the SWD cable and that the board is powered" >&2
        echo "($(grep -o 'Target voltage: [0-9.]*' "$LOG" | tail -n 1))." >&2
    fi
    echo "--- OpenOCD said:" >&2
    grep -vE "^(Open On-Chip|Licensed under|For bug reports|[[:space:]]+http)" "$LOG" | tail -n 15 >&2
}

n_links="$(stlinks | grep -c . || true)"
if [ "$n_links" -gt 1 ] && [ -z "$SERIAL" ]; then
    echo "Several ST-Links are plugged in -- say which wheel with --serial:" >&2
    stlinks | sed 's/^[^ ]* /  /' >&2
    exit 1
fi

identify() { # no reset, no halt: who is on the other end, and how is it protected
    ocd "init" \
        'echo "IDCODE=[format 0x%08x [mrw 0xE0042000]]"' \
        'echo "OPTR=[format 0x%08x [mrw 0x40022020]]"' \
        'echo "WRP1A=[format 0x%08x [mrw 0x4002202C]]"' \
        'echo "WRP1B=[format 0x%08x [mrw 0x40022030]]"' \
        'echo "WRP2A=[format 0x%08x [mrw 0x4002204C]]"' \
        'echo "WRP2B=[format 0x%08x [mrw 0x40022050]]"' \
        'echo "PCROP1S=[format 0x%08x [mrw 0x40022024]]"' \
        'echo "PCROP1E=[format 0x%08x [mrw 0x40022028]]"' \
        'echo "PCROP2S=[format 0x%08x [mrw 0x40022044]]"' \
        'echo "PCROP2E=[format 0x%08x [mrw 0x40022048]]"' \
        'catch {echo "UID=[format %08x%08x%08x [mrw 0x1FFF7598] [mrw 0x1FFF7594] [mrw 0x1FFF7590]]"}' \
        "shutdown" || return 1
    local r v
    for r in IDCODE OPTR WRP1A WRP1B WRP2A WRP2B PCROP1S PCROP1E PCROP2S PCROP2E; do
        [ -n "$(val $r)" ] || return 1
    done
    IDCODE="$(val IDCODE)" OPTR="$(val OPTR)" UID_HEX="$(val UID)"
    RDP=$((OPTR & 0xFF))
    WRP_ON=0
    for r in WRP1A WRP1B WRP2A WRP2B; do # an area is active when start <= end
        v="$(val $r)"
        [ $((v & 0x7F)) -gt $(((v >> 16) & 0x7F)) ] || WRP_ON=1
    done
    PCROP_ON=0
    [ $(($(val PCROP1S) & 0x7FFF)) -gt $(($(val PCROP1E) & 0x7FFF)) ] || PCROP_ON=1
    [ $(($(val PCROP2S) & 0x7FFF)) -gt $(($(val PCROP2E) & 0x7FFF)) ] || PCROP_ON=1
}

# ---------------------------------------------------------------- 2. who are we talking to
step "Board"
identify || { explain_failure; die "cannot talk to the board"; }
[ $((IDCODE & 0xFFF)) -eq $((0x469)) ] \
    || die "that is not an STM32G474 (device id $(printf 0x%03x $((IDCODE & 0xFFF)))) -- wrong board on the ST-Link?"
UID_HEX="${UID_HEX:-unknown}"
echo "STM32G474, UID $UID_HEX, option bytes $OPTR"
[ "$RDP" -ne $((0xCC)) ] || die "read protection level 2 is set: this chip can never be reflashed or debugged again"
[ "$PCROP_ON" -eq 0 ] || die "a PCROP (execute-only) area is set on this chip; this script does not remove those"

if [ "$ERASE_CAL" -eq 1 ]; then
    plan="erase the WHOLE chip, encoder calibration included (the wheel must be re-commissioned)"
elif [ "$RDP" -ne $((0xAA)) ]; then
    plan="remove read protection, which erases the WHOLE chip, encoder calibration included"
else
    plan="erase everything except the encoder calibration"
fi
echo "About to $plan, then flash $TAG."
if [ "$YES" -eq 0 ]; then
    read -r -p "Go ahead? [y/N] " answer </dev/tty || answer=""
    case "$answer" in y|Y|yes) ;; *) echo "Nothing touched."; exit 1 ;; esac
fi

# ---------------------------------------------------------------- 3. backup
BACKUP="" CAL_BACKUP="" RESTORE_CAL="" CAL_LOST="$ERASE_CAL"
if [ "$RDP" -eq $((0xAA)) ]; then
    step "Backup"
    mkdir -p "backups/$UID_HEX"
    BACKUP="backups/$UID_HEX/$(date +%Y%m%d-%H%M%S)-flash.bin"
    CAL_BACKUP="${BACKUP%-flash.bin}-cal.bin"
    ocd "init" "reset halt" "dump_image $BACKUP $FLASH_BASE $FLASH_SIZE" "shutdown" \
        || { explain_failure; die "could not read the flash back -- nothing erased"; }
    [ "$(wc -c <"$BACKUP")" -eq $((FLASH_SIZE)) ] || die "short backup ($BACKUP) -- nothing erased"
    tail -c $((PAGE)) "$BACKUP" >"$CAL_BACKUP"
    echo "$BACKUP (whole flash), $CAL_BACKUP (calibration page)"
    RESTORE_CAL="$OPENOCD -f openocd.cfg -c 'init; reset halt; flash write_image erase $CAL_BACKUP $CAL_ADDR bin; reset run; shutdown'"
else
    echo "note: read protection is on, so the flash cannot be backed up first"
fi

# ---------------------------------------------------------------- 4. protection + option bytes
if [ "$RDP" -ne $((0xAA)) ]; then
    step "Removing read protection (mass erase)"
    CAL_LOST=1
    ocd "init" "reset halt" "stm32l4x unlock 0" "stm32l4x option_load 0" "shutdown" || true
    sleep 1
    identify || { explain_failure; die "lost the board after the unlock -- power-cycle it and run this again"; }
    [ "$RDP" -eq $((0xAA)) ] || die "read protection is still on -- power-cycle the board and run this again"
fi

if [ "$WRP_ON" -eq 1 ] || [ $((OPTR & OPTR_MASK)) -ne $((OPTR_GOLDEN)) ]; then
    step "Option bytes"
    cmds=("init" "reset halt")
    if [ "$WRP_ON" -eq 1 ]; then
        echo "clearing write protection"
        cmds+=("flash protect 0 0 last off")
    fi
    if [ $((OPTR & OPTR_MASK)) -ne $((OPTR_GOLDEN)) ]; then
        printf 'user option bits 0x%08x -> 0x%08x\n' $((OPTR & OPTR_MASK)) $((OPTR_GOLDEN))
        cmds+=("stm32l4x option_write 0 0x20 $OPTR_GOLDEN $OPTR_MASK")
    fi
    # option_load resets the chip under OpenOCD; judge by reading back, not by its exit status.
    ocd "${cmds[@]}" "stm32l4x option_load 0" "shutdown" || true
    sleep 1
    identify || { explain_failure; die "lost the board after writing the option bytes"; }
    [ "$WRP_ON" -eq 0 ] || { explain_failure; die "write protection is still set"; }
    [ $((OPTR & OPTR_MASK)) -eq $((OPTR_GOLDEN)) ] \
        || { explain_failure; die "option bytes read back as $OPTR, not as written"; }
    echo "option bytes now $OPTR"
fi

# ---------------------------------------------------------------- 5. erase, program, verify
step "Erase + program"
if [ "$ERASE_CAL" -eq 1 ]; then
    erase="stm32l4x mass_erase 0"
else
    erase="flash erase_sector 0 0 $(((CAL_ADDR - FLASH_BASE) / PAGE - 1))"
fi
ocd "init" "reset halt" "$erase" \
    "flash write_image $DIR/$BL.hex" "verify_image $DIR/$BL.hex" \
    "flash write_image $DIR/$APP.hex" "verify_image $DIR/$APP.hex" \
    "dump_image $LOG.cal $CAL_ADDR $PAGE" \
    "reset run" "shutdown" \
    || { explain_failure; die "erase/program/verify failed -- the board is NOT in a good state; run this again"; }
[ "$(grep -c '^verified ' "$LOG")" -eq 2 ] || { explain_failure; die "verification did not report both images"; }
echo "bootloader @ $FLASH_BASE and app @ $APP_BASE written and verified"

if [ "$ERASE_CAL" -eq 0 ] && [ -n "$CAL_BACKUP" ]; then
    cmp -s "$LOG.cal" "$CAL_BACKUP" \
        || die "the calibration page changed during flashing -- restore it with:
  $RESTORE_CAL"
    echo "calibration page untouched"
fi

# ---------------------------------------------------------------- 6. did it boot
step "Boot check"
sleep 3
ocd "init" \
    'echo "DHCSR=[format 0x%08x [mrw 0xE000EDF0]]"' \
    'echo "VTOR=[format 0x%08x [mrw 0xE000ED08]]"' \
    'echo "ICSR=[format 0x%08x [mrw 0xE000ED04]]"' \
    'echo "CFSR=[format 0x%08x [mrw 0xE000ED28]]"' \
    "shutdown" || { explain_failure; die "flashed, but could not read the board back afterwards"; }
DHCSR="$(val DHCSR)" VTOR="$(val VTOR)" ICSR="$(val ICSR)" CFSR="$(val CFSR)"
if [ $((DHCSR & 0x20000)) -ne 0 ]; then
    die "flashed, but the CPU is sitting halted -- reset or power-cycle the board"
elif [ $((ICSR & 0x1FF)) -eq 3 ]; then
    die "flashed, but the CPU is in a HardFault (VTOR $VTOR, CFSR $CFSR)"
elif [ $((VTOR)) -ne $((APP_BASE)) ]; then
    die "flashed, but the app did not start (VTOR $VTOR): the bootloader is holding in DFU mode -- is the BOOT0 strap (PB8) high?"
fi

echo
echo "OK: $TAG is running on wheel $UID_HEX."
if [ "$CAL_LOST" -eq 1 ]; then
    echo "    Calibration was erased: commission the wheel again (can_console: commission)."
    [ -z "$RESTORE_CAL" ] || echo "    Or put the old calibration back:  $RESTORE_CAL"
fi
[ -z "$BACKUP" ] || echo "    Backup of what was there: $BACKUP"
echo "    Symbols for tools/*.sh:   ELF=$DIR/$APP.elf"
