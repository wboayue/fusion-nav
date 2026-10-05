#!/usr/bin/env bash
# Build the firmware #41 measures with, under one named configuration, to
# target/onboard/<build>.bin for `dfu-util`; the published figures name the build they came from.
#
#   onboard/build.sh primary   the profile data/footprint.txt's frames are measured in: the pinned
#                              nightly, opt-level 3, no LTO, so a painted stack and the host walk
#                              describe the same code
#   onboard/build.sh shipped   opt-level "s" under fat LTO, as a firmware ships
#   onboard/build.sh fp64      primary with `-C target-cpu=cortex-m7`: the M7's double-precision
#                              FPU, where `f64` (the window's sums, the geodetic conversion) is
#                              hardware rather than library calls
#
# Flash: put the board in DFU (`r` over the port, or BOOT0 and RESET), then
#   dfu-util -a 0 -s 0x08000000:leave -D target/onboard/<build>.bin
# and tap NRST: the H743's ROM bootloader does not start the application cleanly on `:leave`
# (ark-fpv-discovery's CLAUDE.md, "Build & flash").

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
die() { echo "onboard: $*" >&2; exit 1; }

# shellcheck source=tools/nightly.sh
. "$root/tools/nightly.sh"
target=thumbv7em-none-eabihf

build=${1:-}
rustflags=
case "$build" in
    primary) opt=3; lto=false ;;
    shipped) opt=s; lto=fat ;;
    fp64) opt=3; lto=false; rustflags="-C target-cpu=cortex-m7" ;;
    *) die "usage: onboard/build.sh primary|shipped|fp64" ;;
esac

nightly_tools

# One directory per build, so switching between them rebuilds nothing twice.
export CARGO_TARGET_DIR="$root/target/onboard/$build"
export CARGO_PROFILE_RELEASE_OPT_LEVEL=$opt
export CARGO_PROFILE_RELEASE_LTO=$lto
# The frame table goes in every build: it moves no code (the flashed image is byte-identical
# without it), and `data/onboard.sh` reads `Machine::execute`'s frame off it, which every
# painted stack includes.
export RUSTFLAGS="$rustflags -Zemit-stack-sizes"
export ONBOARD_BUILD=$build
export ONBOARD_LTO=$lto
ONBOARD_COMMIT=$(git describe --always --dirty --abbrev=10)
export ONBOARD_COMMIT
cargo "+$TOOLCHAIN" build --quiet --release -p onboard --bin onboard --features firmware \
    --target "$target"
elf="$CARGO_TARGET_DIR/$target/release/onboard"

# Every frame between the board's paint and a call is a `Machine` function's, read off this ELF
# beside it, so the figures and the frames subtracted from them come from one build.
frames="$root/target/onboard/$build.frames"
"$tools/llvm-readobj" --stack-sizes --demangle "$elf" > "$frames.readobj"
{
    echo "# $build $ONBOARD_COMMIT"
    python3 tools/footprint.py --frames "<onboard::machine::Machine>::" "$frames.readobj"
} > "$frames"
rm "$frames.readobj"

# `execute` must call every entry point, never jump to one: a tail call runs beneath where its
# frame was, and the frame subtracted from it would publish that call low.
jumps=$("$tools/llvm-objdump" -d --demangle --no-show-raw-insn "$elf" | awk '
    /^[0-9a-f]+ <<onboard::machine::Machine>::execute>:$/ { inside = 1; next }
    inside && /^$/ { inside = 0 }
    inside && $2 ~ /^b(\.w)?$/ && /</ && !/<<onboard::machine::Machine>::execute/ { print }')
[ -z "$jumps" ] || die "Machine::execute tail-calls, so its frame is not above the call:
$jumps"

"$tools/llvm-objcopy" -O binary "$elf" "$root/target/onboard/$build.bin"
echo "target/onboard/$build.bin ($(wc -c < "$root/target/onboard/$build.bin" | tr -d ' ') bytes) from $elf"
