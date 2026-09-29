#!/usr/bin/env bash
#
# Replay INSANE's three sequences against their truth and assert data/insane-pins.txt.
#
#   data/insane.sh          convert, replay each sequence, compare
#   data/insane.sh --pin    print the lines to commit instead
#
# The accuracy benchmark of #9: a UAV's own PX4 sensors against RTK truth, the one source
# with a real barometer and magnetometer *and* truth. What its truth can score is position,
# height and velocity, and whether the covariance covers them (nees_pos, nees_vel); its
# attitude cannot be, and no attitude key is pinned (tools/insane2replay.py says why).
#
# One run per sequence, raw `R`, under `--declination model` (the converter writes no
# declination). `--r-policy px4` is not run: every fix claims more than PX4's floors, so it
# replays byte for byte what raw does.
#
# Local, like `data/fetch.sh --check`, and never CI: INSANE's terms forbid selling what
# derives from it, so its files are fetched (`data/fetch.sh --manifest data/insane.txt`) and
# nothing drawn from them is committed but the scalars below (data/insane.txt carries them).

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
data="$root/data/insane"
pins="$root/data/insane-pins.txt"

die() { echo "insane: $*" >&2; exit 1; }

# What a pin line carries: the `summary` keys that say what the filter did with each
# source, and the truth-scored keys a position, height and velocity truth can support.
keys="rejected_gnss_pos rejected_gnss_hgt rejected_baro rejected_mag recovered alpha0
floored degraded_s dead_reckoning_s transitions status
pos_h pos_v vel pos_h_max nees_pos nees_vel"

pin=0
case "${1:-}" in
    --pin) pin=1 ;;
    '') ;;
    *) die "unknown option $1 (usage: data/insane.sh [--pin])" ;;
esac
# shellcheck source=data/truth-runs.sh
. "$root/data/truth-runs.sh"

"$root/data/fetch.sh" --manifest "$root/data/insane.txt" --verify >/dev/null ||
    die "a sequence is missing or changed; data/fetch.sh --manifest data/insane.txt"
command -v uv >/dev/null || die "uv not found; tools/insane2replay.py runs through it"

build_replay
out="$target/insane"
mkdir -p "$out"

# The manifest names the sequences, so a new one is converted here or refused by the
# converter, and never silently skipped.
sequences=$(awk '$1 !~ /^#/ && $2 ~ /_sensors\.zip$/ { sub(/_sensors\.zip$/, "", $2); print $2 }' \
    "$root/data/insane.txt")
for sequence in $sequences; do
    uv run --quiet "$root/tools/insane2replay.py" "$data" --sequence "$sequence" \
        -o "$out/$sequence.csv" --truth "$out/$sequence.truth.csv" 2>/dev/null ||
        die "converting $sequence failed"
    pinned_run "$sequence" raw "$out/$sequence.raw" "$out/$sequence.csv" \
        "$out/$sequence.truth.csv" --declination model
done
[ "$failed" = 0 ] || die "one or more runs did not match data/insane-pins.txt"
