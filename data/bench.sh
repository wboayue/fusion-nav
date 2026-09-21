#!/usr/bin/env bash
#
# Score every simulated scenario against the ceilings in data/scenarios.txt.
#
#   data/bench.sh              generate, replay and compare every scenario
#   data/bench.sh mission      only these
#
# `examples/simulate.rs` is the only source with truth to score against, so this is the only
# place the crate says how *accurate* the filter is rather than how self-consistent. It runs
# in CI: no network, no PX4 tooling, no hardware (GOALS.md, "Harness constraint"), which is
# what separates it from `data/fetch.sh --check`.
#
# Debug, where fetch.sh --check uses --release. The corpus includes a two-hour log at 1.4M
# epochs and pays for the optimised build many times over; the nine scenarios here are 4 s to
# generate and about 1 s each to replay, against roughly a minute to build the crate again
# under a second profile. The `score` lines are identical either way -- Rust contracts no
# FMAs and opt-level does not change float semantics, which is the same property the
# determinism job asserts across architectures.
#
# Generation is unconditional rather than reusing whatever is in target/sim, so there is no
# question of a ceiling being met by a scenario generated before the change under test.

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
scenarios="$root/data/scenarios.txt"
out="$root/target/sim"

# shellcheck source=data/expect.sh
. "$root/data/expect.sh"

die() { echo "bench: $*" >&2; exit 1; }

[ -f "$scenarios" ] || die "no ceilings at $scenarios"

# The seed is in examples/simulate.rs; the column in data/scenarios.txt is a pin on the one
# that produced these numbers, checked here against the header the generator writes. Without
# it a seed could be changed in the table and the ceilings would quietly become a claim about
# a flight nobody flew -- the same reason examples/replay.rs refuses a truth file whose header
# names another scenario.
check_seed() {
    local name=$1 want=$2 got
    got=$(sed -n '1s/.*seed \([0-9]*\).*/\1/p' "$out/$name.csv")
    [ -n "$got" ] || die "$name.csv has no seed in its header"
    if [ "$got" != "$want" ]; then
        echo "  WRONG SEED $name: ceilings were measured on seed $want, generated with $got" >&2
        return 1
    fi
}

# A string rather than an array: bash 3.2 is what macOS ships, and an empty array under
# `set -u` is an error there.
wanted=" $* "
selected() {
    [ "$wanted" = "  " ] && return 0
    case "$wanted" in *" $1 "*) return 0 ;; esac
    return 1
}

echo "generating scenarios"
# Kept rather than discarded: the generator names every log it wrote, and that list is what
# the coverage check below is about. Reading target/sim instead would be reading a directory
# that anyone can write a replay output into.
simulated=$(cd "$root" && cargo run --quiet --example simulate) || die "simulate failed"
generated=$(printf '%s\n' "$simulated" | sed -n 's|^  log   .*/\([^/]*\)\.csv$|\1|p')
[ -n "$generated" ] || die "the simulator reported no logs; its output format moved"

failed=0
found=0
gated=" "
# `|| [ -n "$name" ]` so a file that lost its trailing newline does not lose its last
# scenario: `read` returns non-zero on an unterminated line after setting the variables, and
# the loop would otherwise drop `flight` and report success over eight scenarios.
while read -r name seed expect || [ -n "$name" ]; do
    case "$name" in ''|\#*) continue ;; esac
    [ -n "$seed" ] || die "malformed line in data/scenarios.txt: $name"
    # A line with a name and a seed and no ceilings compares nothing and passes. That is a
    # disarmed gate that reads as a green one, so it is the file being wrong rather than the
    # filter — `data/fetch.sh` says "no expectations" and carries on because the corpus can
    # honestly have a log nothing is pinned about yet; every scenario here is generated to be
    # scored.
    [ -n "$expect" ] || die "$name has no ceilings; a line with none gates nothing"
    gated="$gated$name "
    selected "$name" || continue
    found=$((found + 1))

    [ -f "$out/$name.csv" ] || die "$name is in data/scenarios.txt but the simulator wrote no log for it"
    # Every failure in one run rather than the first: a wrong seed says these ceilings belong
    # to another flight, which is worth knowing alongside whatever else moved, not instead.
    if ! check_seed "$name" "$seed"; then
        failed=1
        continue
    fi

    score=$(cd "$root" && cargo run --quiet --example replay -- \
        "$out/$name.csv" "$out/$name.replay.csv" "$out/$name.truth.csv" | grep '^score ') ||
        die "$name: replay printed no score line"

    if compare_pairs "$score" "$expect" "$name"; then
        echo "  ok       $name"
    else
        # The whole line, so re-measuring a ceiling that moved for a good reason is a copy
        # rather than a second run.
        echo "    got ${score#score }" >&2
        failed=1
    fi
done < "$scenarios"

[ "$found" -gt 0 ] || die "no scenario matched $*"

# The other direction, and the one that matters: the loop above reads data/scenarios.txt, so
# a scenario added to examples/simulate.rs and not listed here is simply never scored, and
# the run says "9 scenarios within their ceilings" while the tenth has no accuracy bound at
# all. This is the repository's only accuracy gate; silence is not coverage. Skipped for a
# selective run, which is deliberately partial.
#
# Against what the generator said it wrote, not against target/sim: that directory is an
# output path like any other, and `cargo run --example replay -- ... target/sim/out.csv`
# leaves a file there that is nobody's scenario. Globbing it reported the leftovers of an
# unrelated command as ungated scenarios, which is a local-only failure -- a fresh CI
# checkout has none -- and a gate that cries wolf on a developer's machine is one they learn
# to run with a shrug.
if [ "$wanted" = "  " ]; then
    for base in $generated; do
        case "$gated" in
            *" $base "*) ;;
            *)
                echo "  UNGATED    $base: generated by examples/simulate.rs, absent from data/scenarios.txt" >&2
                failed=1
                ;;
        esac
    done
fi

[ "$failed" = 0 ] || die "one or more scenarios breached their ceilings"
echo "bench: $found scenarios within their ceilings"
