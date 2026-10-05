#!/usr/bin/env bash
# Cost on the board (#41): every trace it is timed on, and the timing.
#
#   data/onboard.sh traces [LOGDIR]          traces into target/onboard/traces: each log
#                                            data/manifest.txt names, converted in LOGDIR
#                                            (default data/logs), as <first 8 of its name>,
#                                            the `scenarios` below, and
#                                            onboard/examples/paths.rs
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
# The simulated scenarios traced beside the corpus: what no log covers, the hold over a long
# outage in a hover.
scenarios=(hover_outage)
die() { echo "onboard: $*" >&2; exit 1; }

case "${1:-}" in
    traces)
        logs=${2:-$root/data/logs}
        mkdir -p "$traces"
        cargo build --quiet --release --example simulate
        cargo build --quiet --release -p onboard --example paths --example verify
        # The harness that writes traces is its own build (`examples/replay/filter.rs`), in a
        # directory of its own so the flag rebuilds nothing in target/.
        CARGO_TARGET_DIR="$root/target/onboard/host" RUSTFLAGS="--cfg fusion_nav_onboard" \
            cargo build --quiet --release --example replay
        replay="$root/target/onboard/host/release/examples/replay"
        for name in $(awk '!/^#/ && NF { sub(/\.ulg$/, "", $2); print $2 }' data/manifest.txt); do
            input="$logs/$name.csv"
            [ -f "$input" ] || die "no $input: convert the corpus first (data/README.md)"
            "$replay" --trace "$traces/${name:0:8}.trace" "$input" \
                "$root/target/onboard/replay/${name:0:8}.csv" > /dev/null
        done
        "$root/target/release/examples/simulate" > /dev/null
        for scenario in "${scenarios[@]}"; do
            "$replay" --trace "$traces/$scenario.trace" "$root/target/sim/$scenario.csv" \
                "$root/target/onboard/replay/$scenario.csv" > /dev/null
        done
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
        # The frames above every painted call, off the ELF `onboard/build.sh` built, and only if
        # it built the one the board runs: frames from a later build would be subtracted from
        # this one's figures.
        frames="$root/target/onboard/$build.frames"
        [ -f "$frames" ] || die "no $frames: build with onboard/build.sh $build"
        built=$(head -1 "$frames" | awk '{ print $3 }')
        case "$running" in
            *" commit=$built "*) ;;
            *) die "$frames is of $built; the board runs: $running" ;;
        esac
        mkdir -p "$out"
        cp "$frames" "$out/frames.txt"
        python3 tools/onboard.py run --out "$out" ${cold[@]+"${cold[@]}"} "$@"
        python3 tools/onboard.py fmodf --out "$out" ${cold[@]+"${cold[@]}"}
        ;;
    pin)
        python3 tools/onboard.py pin "$root"/target/onboard/*-warm "$root"/target/onboard/*-cold
        ;;
    *) die "usage: data/onboard.sh traces [LOGDIR] | time BUILD [--cold] [TRACE ...] | pin" ;;
esac
