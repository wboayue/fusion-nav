# The nightly every bare-metal measurement is taken on, sourced by `tools/footprint.sh`, which
# pins frames, sizes and flash, and by `onboard/build.sh`, which builds the firmware #41 times
# with, so a painted stack and a pinned frame describe one compiler's code. `tools/validation.py`
# reads `TOOLCHAIN=` from here for the cost page. Moving it is a re-pin of both.

# shellcheck disable=SC2034 # read by the scripts that source this file
TOOLCHAIN=nightly-2026-08-06

# nightly_tools: set `host`, `sysroot` and `tools`, the nightly's LLVM binaries, or fail naming
# the install command. `die` is the caller's.
nightly_tools() {
    host=$(rustc "+$TOOLCHAIN" -vV 2>/dev/null | sed -n 's/^host: //p' || true)
    [ -n "$host" ] || die "no $TOOLCHAIN: run tools/footprint.sh --install"
    sysroot=$(rustc "+$TOOLCHAIN" --print sysroot)
    tools="$sysroot/lib/rustlib/$host/bin"
    local tool
    for tool in llvm-readobj llvm-objdump llvm-nm llvm-size llvm-objcopy; do
        [ -x "$tools/$tool" ] || die "$tool not in $tools: run tools/footprint.sh --install"
    done
}
