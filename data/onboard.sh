#!/usr/bin/env bash
# Cost on the board (#41): every trace it is timed on, and the timing.
#
#   data/onboard.sh traces [LOGDIR]          traces into target/onboard/traces: each log
#                                            data/manifest.txt names, converted in LOGDIR
#                                            (default data/logs), the hover_outage scenario,
#                                            and onboard/examples/paths.rs
#   data/onboard.sh time BUILD [--cold] [TRACE ...]
#                                            every trace (or those named) timed on the board,
#                                            which must be running BUILD (onboard/build.sh),
#                                            into target/onboard/BUILD-warm or -cold
#   data/onboard.sh pin                      data/onboard.txt's lines, from every result
#                                            directory under target/onboard
#
# Local: it needs the board, and the corpus fetched (data/fetch.sh). Every trace is checked on the
# host first (`verify`), so a trace the board disagrees with is the board's finding, not the
# recorder's.

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
traces="$root/target/onboard/traces"
die() { echo "onboard: $*" >&2; exit 1; }

case "${1:-}" in
    traces)
        logs=${2:-$root/data/logs}
        mkdir -p "$traces"
        cargo build --quiet --release --example replay --example simulate
        cargo build --quiet --release -p onboard --example paths --example verify
        replay="$root/target/release/examples/replay"
        for name in $(awk '!/^#/ && NF { sub(/\.ulg$/, "", $2); print $2 }' data/manifest.txt); do
            input="$logs/$name.csv"
            [ -f "$input" ] || die "no $input: convert the corpus first (data/README.md)"
            "$replay" --trace "$traces/${name:0:8}.trace" "$input" \
                "$root/target/onboard/replay/${name:0:8}.csv" > /dev/null
        done
        "$root/target/release/examples/simulate" > /dev/null
        "$replay" --trace "$traces/hover_outage.trace" "$root/target/sim/hover_outage.csv" \
            "$root/target/onboard/replay/hover_outage.csv" > /dev/null
        "$root/target/release/examples/paths" "$traces/paths.trace"
        "$root/target/release/examples/verify" "$traces"/*.trace
        ;;
    time)
        build=${2:-}
        [ -n "$build" ] || die "time wants the build the board runs"
        shift 2
        mode=warm cold=()
        if [ "${1:-}" = --cold ]; then
            mode=cold cold=(--cold)
            shift
        fi
        out="$root/target/onboard/$build-$mode"
        running=$(python3 tools/onboard.py info)
        case "$running" in
            *" build=$build "*) ;;
            *) die "the board runs: $running" ;;
        esac
        if [ $# -eq 0 ]; then
            set -- "$traces"/*.trace
        fi
        # The dispatcher's frame, off the ELF the board runs: a painted stack is measured from
        # the call into `Machine::execute`, and the published figure starts beneath it.
        toolchain=$(sed -n 's/^TOOLCHAIN=//p' tools/footprint.sh)
        host=$(rustc "+$toolchain" -vV | sed -n 's/^host: //p')
        readobj="$(rustc "+$toolchain" --print sysroot)/lib/rustlib/$host/bin/llvm-readobj"
        elf="$root/target/onboard/$build/thumbv7em-none-eabihf/release/onboard"
        mkdir -p "$out"
        frame=$("$readobj" --stack-sizes --demangle "$elf" | awk '
            /Functions: \[<onboard::machine::Machine>::execute\]/ { found = 1; next }
            found && /Size:/ { print $2; exit }')
        [ -n "$frame" ] || die "no frame for Machine::execute in $elf"
        printf '%d\n' "$frame" > "$out/dispatch.txt"
        python3 tools/onboard.py run --out "$out" "${cold[@]}" "$@"
        python3 tools/onboard.py fmodf --out "$out" "${cold[@]}"
        ;;
    pin)
        python3 tools/onboard.py pin "$root"/target/onboard/*-warm "$root"/target/onboard/*-cold
        ;;
    *) die "usage: data/onboard.sh traces [LOGDIR] | time BUILD [--cold] [TRACE ...] | pin" ;;
esac
