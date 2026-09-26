#!/usr/bin/env bash
#
# Fly every scenario on N seeds and gate the ensemble's NEES against the chi-square bound (#89).
#
#   data/anees.sh              every scenario in data/anees.txt
#   data/anees.sh mission      only these
#
# data/bench.sh asks whether one pinned flight got less accurate. This asks whether the
# covariance tells the truth about the error, which no ceiling can: a ceiling passes a filter
# that grew more accurate and more overconfident at once. The statistic is tools/anees.py's;
# this script only flies the seeds, and compares what it prints with data/expect.sh, the same
# comparator bench.sh and fetch.sh --check read.
#
# Release, where bench.sh builds debug: N runs per scenario is 500 replays over the ten, and
# at ~1 s each in release the build pays for itself many times over. The figures are
# identical either way, for the reason bench.sh gives.
#
# Each run keeps only its .nees.csv. A log, its truth and its replay are ~20 MB together, so
# keeping all 500 would be ~10 GB of target/ for files nothing reads again.

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
bounds="$root/data/anees.txt"
out="$root/target/anees"

# shellcheck source=data/expect.sh
. "$root/data/expect.sh"

die() { echo "anees: $*" >&2; exit 1; }

[ -f "$bounds" ] || die "no bounds at $bounds"
command -v python3 >/dev/null || die "python3 is required (standard library only)"

jobs=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)

wanted=" $* "
selected() {
    [ "$wanted" = "  " ] && return 0
    case "$wanted" in *" $1 "*) return 0 ;; esac
    return 1
}

echo "building"
(cd "$root" && cargo build --quiet --release --example simulate --example replay) ||
    die "build failed"
simulate="$root/target/release/examples/simulate"
replay="$root/target/release/examples/replay"

# One seed of one scenario: generate, replay, keep the .nees.csv. Exported for xargs, which
# runs each in its own shell.
fly() {
    local name=$1 seed=$2 dir="$3/$2"
    mkdir -p "$dir"
    "$simulate" "$name" "$dir" --seed "$seed" >/dev/null ||
        { echo "anees: simulate $name seed $seed failed" >&2; return 1; }
    "$replay" "$dir/$name.csv" "$dir/replay.csv" "$dir/$name.truth.csv" >/dev/null ||
        { echo "anees: replay $name seed $seed failed" >&2; return 1; }
    mv "$dir/replay.nees.csv" "$3/$2.nees.csv"
    rm -rf "$dir"
}
export -f fly
export simulate replay

failed=0
found=0
gated=" "
while read -r name runs expect || [ -n "$name" ]; do
    case "$name" in ''|\#*) continue ;; esac
    [ -n "$runs" ] || die "malformed line in data/anees.txt: $name"
    # The same refusal bench.sh makes: a line with nothing to compare is a disarmed gate that
    # reads as a green one.
    [ -n "$expect" ] || die "$name has no bounds; a line with none gates nothing"
    gated="$gated$name "
    selected "$name" || continue
    found=$((found + 1))

    dir="$out/$name"
    rm -rf "$dir"
    mkdir -p "$dir"
    # Seeds 1..N. The departures from `mission` fly the same seeds it does, so at every seed
    # they stay paired with it exactly as they are at the pinned one.
    seq 1 "$runs" | xargs -P "$jobs" -I{} bash -c 'fly "$0" {} "$1"' "$name" "$dir" ||
        die "$name: a run failed"

    line=$(python3 "$root/tools/anees.py" "$dir"/*.nees.csv) || die "$name: aggregation failed"
    rm -rf "$dir"
    if compare_pairs "$line" "$expect" "$name"; then
        echo "  ok       $name"
    else
        # The whole line, so re-measuring a bound that moved for a good reason is a copy.
        echo "    got ${line#anees "$name" }" >&2
        failed=1
    fi
done < "$bounds"

[ "$found" -gt 0 ] || die "no scenario matched $*"

# Every scenario the simulator can fly, gated here as well as in bench.sh. The list comes from
# the scenarios bench.sh gates, which its own coverage check ties to the generator.
if [ "$wanted" = "  " ]; then
    while read -r name _ || [ -n "$name" ]; do
        case "$name" in ''|\#*) continue ;; esac
        case "$gated" in
            *" $name "*) ;;
            *)
                echo "  UNGATED    $name: in data/scenarios.txt, absent from data/anees.txt" >&2
                failed=1
                ;;
        esac
    done < "$root/data/scenarios.txt"
fi

[ "$failed" = 0 ] || die "one or more scenarios breached their bounds"
echo "anees: $found scenarios within their bounds"
