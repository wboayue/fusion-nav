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

toolchain=$(sed -n 's/^TOOLCHAIN=//p' tools/footprint.sh)
[ -n "$toolchain" ] || die "tools/footprint.sh pins no TOOLCHAIN"
target=thumbv7em-none-eabihf

build=${1:-}
rustflags=
case "$build" in
    primary) opt=3; lto=false ;;
    shipped) opt=s; lto=fat ;;
    fp64) opt=3; lto=false; rustflags="-C target-cpu=cortex-m7" ;;
    *) die "usage: onboard/build.sh primary|shipped|fp64" ;;
esac

host=$(rustc "+$toolchain" -vV 2>/dev/null | sed -n 's/^host: //p' || true)
[ -n "$host" ] || die "no $toolchain: run tools/footprint.sh --install"
objcopy="$(rustc "+$toolchain" --print sysroot)/lib/rustlib/$host/bin/llvm-objcopy"
[ -x "$objcopy" ] || die "no llvm-objcopy in $toolchain: run tools/footprint.sh --install"

# One directory per build, so switching between them rebuilds nothing twice.
export CARGO_TARGET_DIR="$root/target/onboard/$build"
export CARGO_PROFILE_RELEASE_OPT_LEVEL=$opt
export CARGO_PROFILE_RELEASE_LTO=$lto
export RUSTFLAGS=$rustflags
export ONBOARD_BUILD=$build
export ONBOARD_LTO=$lto
cargo "+$toolchain" build --quiet --release -p onboard --bin onboard --features firmware \
    --target "$target"
elf="$CARGO_TARGET_DIR/$target/release/onboard"
"$objcopy" -O binary "$elf" "$root/target/onboard/$build.bin"
echo "target/onboard/$build.bin ($(wc -c < "$root/target/onboard/$build.bin" | tr -d ' ') bytes) from $elf"
