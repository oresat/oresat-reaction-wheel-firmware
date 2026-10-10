#!/bin/sh
# Fetch the pinned ARM toolchain into .toolchain/ -- no root, no Docker, nothing installed
# system-wide. build.sh uses it once it is there, and CI builds releases with the same one, so
# a build here is the compiler a release was built with regardless of what the distro ships.
#
#   ./toolchain.sh            download (~150 MB), verify, unpack (~1 GB); then check cmake/make
#   ./toolchain.sh --path     print its bin dir; nothing if there is no .toolchain/ at all;
#                             an error if .toolchain/ exists without the pinned version in it
#   ./toolchain.sh --remove   delete .toolchain/
set -e
cd "$(dirname "$0")"

VERSION=14.3.rel1 # Arm GNU Toolchain (GCC 14.3.1). Bump the checksums with it.
case "$(uname -s)-$(uname -m)" in
    Linux-x86_64)  HOST=x86_64;  SHA256=8f6903f8ceb084d9227b9ef991490413014d991874a1e34074443c2a72b14dbd ;;
    Linux-aarch64) HOST=aarch64; SHA256=2d465847eb1d05f876270494f51034de9ace9abe87a4222d079f3360240184d3 ;;
    *) HOST="" ;;
esac
NAME="arm-gnu-toolchain-$VERSION-$HOST-arm-none-eabi"
URL="https://developer.arm.com/-/media/Files/downloads/gnu/$VERSION/binrel/$NAME.tar.xz"
DEST=".toolchain/$NAME"
GCC="$DEST/bin/arm-none-eabi-gcc"

case "${1:-}" in
    --path)
        if [ -n "$HOST" ] && [ -x "$GCC" ]; then
            echo "$PWD/$DEST/bin"
        elif [ -e .toolchain ]; then
            # Falling back to PATH here would be a silent switch of compiler. Make the user say.
            cat >&2 <<EOF

ERROR: .toolchain/ exists, but the pinned toolchain ($VERSION) is not in it -- an older
       version, or a download that never finished. Not guessing which compiler you meant.
       Do one of:
           ./toolchain.sh              install the pinned toolchain
           ./toolchain.sh --remove     delete .toolchain/ and build with the compiler on PATH
           TOOLCHAIN_DIR= ./build.sh   build with the compiler on PATH, this once

EOF
            exit 1
        fi
        exit 0 ;;
    --remove)
        rm -rf .toolchain
        echo "Removed .toolchain/"
        exit 0 ;;
    "") ;;
    *) echo "usage: $0 [--path | --remove]"; exit 2 ;;
esac

[ -n "$HOST" ] || { echo "ERROR: no pinned toolchain for $(uname -s) $(uname -m) (Linux x86_64/aarch64 only)."; exit 1; }

missing=""
for t in curl tar xz sha256sum; do
    command -v "$t" >/dev/null 2>&1 || missing="$missing $t"
done
[ -z "$missing" ] || { echo "ERROR: missing:$missing -- sudo apt install curl tar xz-utils coreutils"; exit 1; }

if [ -x "$GCC" ] && "$GCC" --version >/dev/null 2>&1; then
    echo "toolchain: already installed"
else
    mkdir -p .toolchain
    TARBALL=".toolchain/$NAME.tar.xz"
    # An existing file is resumed, or kept if complete; the checksum decides either way.
    if ! echo "$SHA256  $TARBALL" | sha256sum -c - >/dev/null 2>&1; then
        echo "toolchain: downloading $NAME.tar.xz"
        if [ -t 2 ]; then progress=--progress-bar; else progress="--silent --show-error"; fi
        # shellcheck disable=SC2086
        curl --fail --location --retry 3 $progress --continue-at - --output "$TARBALL" "$URL"
        echo "$SHA256  $TARBALL" | sha256sum -c - >/dev/null \
            || { echo "ERROR: checksum mismatch on $TARBALL -- delete it and re-run."; exit 1; }
    fi
    echo "toolchain: unpacking"
    rm -rf "$DEST" "$DEST.part"
    mkdir "$DEST.part"
    tar -xJf "$TARBALL" -C "$DEST.part" --strip-components=1
    mv "$DEST.part" "$DEST"
    rm -f "$TARBALL"
    "$GCC" --version >/dev/null || { echo "ERROR: $GCC does not run on this machine."; exit 1; }
fi
# .toolchain/ holds the pinned version and nothing else: drop older versions and stale downloads.
for f in .toolchain/* .toolchain/.[!.]*; do
    if [ -e "$f" ] && [ "$f" != "$DEST" ]; then
        echo "toolchain: removing leftover $f"
        rm -rf "$f"
    fi
done
echo "toolchain: $("$GCC" --version | head -n 1)"
echo "           $PWD/$DEST/bin"

# The rest of what build.sh needs. Only report; installing packages is the user's call.
need=""
command -v make >/dev/null 2>&1 || command -v ninja >/dev/null 2>&1 || need="$need make"
if command -v cmake >/dev/null 2>&1; then
    v=$(cmake --version | sed -n '1s/[^0-9]*\([0-9]*\)\.\([0-9]*\).*/\1 \2/p')
    # shellcheck disable=SC2086
    set -- $v
    if [ "${1:-0}" -lt 3 ] || { [ "${1:-0}" -eq 3 ] && [ "${2:-0}" -lt 22 ]; }; then
        echo "WARNING: cmake $1.$2 is older than 3.22 -- 'pip install --user cmake' gets a current one."
    fi
else
    need="$need cmake"
fi
if [ -n "$need" ]; then
    echo "Still needed: sudo apt install$need"
    exit 1
fi
echo "Ready: ./build.sh"
