#!/usr/bin/env bash
#
# Replay UrbanNav-HK-Medium-Urban-1 against its truth and assert data/urbannav-pins.txt.
#
#   data/urbannav.sh          convert, replay each receiver under raw and px4, compare
#   data/urbannav.sh --pin    print the lines to commit instead
#
# The gate benchmark of #60: the one source with hostile GNSS *and* truth, so the one place a
# rejection is scored as right or wrong (`bad_`, `rejected_bad_`, `rejected_good_` and the
# recovery split on the `score` line; examples/replay/main.rs owns what each means). Two receivers
# on one drive: the M8T, which claims a few metres while hundreds out, and the F9P, honest
# to its own accuracy, which is the test of rejecting good fixes at road speed.
#
# Local, like `data/fetch.sh --check`, and never CI: UrbanNav states no licence, so its files
# are fetched (`data/fetch.sh --manifest data/urbannav.txt`) and nothing drawn from them is
# committed but the scalars below (data/urbannav.txt carries the terms).

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
data="$root/data/urbannav"
pins="$root/data/urbannav-pins.txt"

die() { echo "urbannav: $*" >&2; exit 1; }

# What a pin line carries. The `summary` keys a hostile receiver moves, and every truth-scored
# key on the `score` line that says what the gate did and how far off the receiver was
# (`rms_`); the per-key meanings are in
# examples/replay/main.rs. The rest of either line is on the terminal when this runs, and pinning
# all of it would re-pin on every change that touched a figure nobody reads for this.
keys="recovered rejected_gnss_pos rejected_gnss_hgt rejected_gnss_vel rejected_course
aligned_at degraded_s dead_reckoning_s transitions status
pos_h pos_v pos_h_max yaw nees_pos"
for half in gnss_pos gnss_hgt; do
    for count in offered bad rejected_bad rejected_good accepted_far adopted_bad \
        recovered_after_bad recovered_after_lockout unjudged rms; do
        keys="$keys ${count}_$half"
    done
done

pin=0
case "${1:-}" in
    --pin) pin=1 ;;
    '') ;;
    *) die "unknown option $1 (usage: data/urbannav.sh [--pin])" ;;
esac
# shellcheck source=data/truth-runs.sh
. "$root/data/truth-runs.sh"

"$root/data/fetch.sh" --manifest "$root/data/urbannav.txt" --verify >/dev/null ||
    die "the segment is missing or changed; data/fetch.sh --manifest data/urbannav.txt"
command -v uv >/dev/null || die "uv not found; tools/urbannav2replay.py runs through it"

build_replay
out="$target/urbannav"
mkdir -p "$out"

# Each receiver under both R policies, and the M8T once more with `Recovery::OFF`: the filter
# that only reports, which is what recovery's worth on a hostile receiver is measured against.
runs="m8t:raw m8t:px4 m8t:raw-norecovery f9p:raw f9p:px4"

for receiver in m8t f9p; do
    uv run --quiet "$root/tools/urbannav2replay.py" "$data" --receiver "$receiver" \
        -o "$out/$receiver.csv" --truth "$out/$receiver.truth.csv" 2>/dev/null ||
        die "converting $receiver failed"
done
for entry in $runs; do
    receiver=${entry%%:*} policy=${entry#*:}
    options="--r-policy ${policy%-norecovery}"
    [ "$policy" = "${policy%-norecovery}" ] || options="$options --recovery off"
    # shellcheck disable=SC2086 # two flags, split on purpose
    pinned_run "$receiver" "$policy" "$out/$receiver.$policy" "$out/$receiver.csv" \
        "$out/$receiver.truth.csv" $options
done
[ "$failed" = 0 ] || die "one or more runs did not match data/urbannav-pins.txt"
