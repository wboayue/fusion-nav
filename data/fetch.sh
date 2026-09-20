#!/usr/bin/env bash
#
# Fetch and verify the replay corpus.
#
#   data/fetch.sh                    fetch everything in the manifest, verify checksums
#   data/fetch.sh --verify           verify what is already on disk, download nothing
#   data/fetch.sh --add URL [NAME]   download once, record its checksum, append to manifest
#   data/fetch.sh --check            convert and replay each log, assert its expectations
#   data/fetch.sh --list             show the manifest
#
# Logs are large and their redistribution terms are usually unstated, so they are
# fetched rather than committed. The manifest pins a sha256 per file so a corpus is
# reproducible without the repo carrying it: someone runs --add once and commits the
# manifest line, everyone else gets a verified copy.
#
# data/flight.csv is not managed here. It is synthetic, small, and checked in so that
# `cargo run --example replay` works with no network.
#
# --check is a local tool, not a CI job. It needs pyulog, and putting the converter in
# the test path is exactly what GOALS.md's harness constraint rules out: CI replays the
# synthetic CSV only, so it needs no network, no PX4 tooling and no hardware. Run
# --check before a release, or after touching the converter or anything it asserts.

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
manifest="$root/data/manifest.txt"
dest="$root/data/logs"
# The converter needs pyulog, which CI never installs (GOALS.md, "Harness constraint").
# `uv venv && uv pip install pyulog` puts it in .venv, which is gitignored and picked up
# here without an activated shell; PYTHON= overrides for any other interpreter that has it.
if [ -n "${PYTHON:-}" ]; then
    python=$PYTHON
elif [ -x "$root/.venv/bin/python" ]; then
    python=$root/.venv/bin/python
else
    python=python3
fi

sha256() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | cut -d' ' -f1
    else
        shasum -a 256 "$1" | cut -d' ' -f1
    fi
}

die() { echo "fetch: $*" >&2; exit 1; }

# Manifest lines are: <sha256>  <name>  <url>  [key=value ...]. Blank lines and #
# comments skipped. Callbacks that do not care about the expectations ignore $4.
each_entry() {
    [ -f "$manifest" ] || return 0
    local sum name url expect rc=0
    while read -r sum name url expect; do
        case "$sum" in ''|\#*) continue ;; esac
        [ -n "$name" ] && [ -n "$url" ] || die "malformed manifest line: $sum $name $url"
        "$1" "$sum" "$name" "$url" "$expect" || rc=1
    done < "$manifest"
    return $rc
}

# Convert one log and replay it, asserting the manifest's expectations against the
# `summary` line the example prints.
check_one() {
    local name=$2 expect=$4
    local ulg="$dest/$name" csv="$dest/${name%.ulg}.csv"
    if [ ! -f "$ulg" ]; then
        echo "  missing  $name — run data/fetch.sh first" >&2
        return 1
    fi
    if ! "$python" "$root/tools/ulog2replay.py" "$ulg" -o "$csv" >/dev/null 2>&1; then
        echo "  CONVERT FAILED  $name" >&2
        return 1
    fi
    local summary
    # --release: the corpus includes a two-hour log, 1.4M epochs.
    summary=$(cd "$root" && cargo run --quiet --release --example replay -- "$csv" \
        "$csv.replay.csv" 2>/dev/null | grep '^summary ') || true
    if [ -z "$summary" ]; then
        echo "  REPLAY FAILED   $name" >&2
        return 1
    fi
    if [ -z "$expect" ]; then
        echo "  no expectations  $name — ${summary#summary }"
        return 0
    fi
    local rc=0 pair
    for pair in $expect; do
        case " $summary " in
            *" $pair "*) ;;
            *) echo "  MISMATCH $name: wanted $pair" >&2; rc=1 ;;
        esac
    done
    if [ "$rc" != 0 ]; then
        echo "    got ${summary#summary }" >&2
    else
        echo "  ok       $name"
    fi
    return $rc
}

verify_one() {
    local want=$1 name=$2 file="$dest/$2"
    if [ ! -f "$file" ]; then
        echo "  missing  $name"
        return 1
    fi
    local got
    got=$(sha256 "$file")
    if [ "$got" = "$want" ]; then
        echo "  ok       $name"
        return 0
    fi
    echo "  MISMATCH $name" >&2
    echo "    expected $want" >&2
    echo "    actual   $got" >&2
    return 1
}

fetch_one() {
    local want=$1 name=$2 url=$3 file="$dest/$2"
    if [ -f "$file" ] && [ "$(sha256 "$file")" = "$want" ]; then
        echo "  ok       $name (cached)"
        return 0
    fi
    echo "  fetching $name"
    mkdir -p "$dest"
    # Download beside the target so an interrupted transfer cannot masquerade as a
    # complete file on the next run.
    curl -fL --progress-bar -o "$file.part" "$url" || die "download failed: $url"
    local got
    got=$(sha256 "$file.part")
    if [ "$got" != "$want" ]; then
        rm -f "$file.part"
        echo "  MISMATCH $name" >&2
        echo "    expected $want" >&2
        echo "    actual   $got" >&2
        return 1
    fi
    mv "$file.part" "$file"
    echo "  ok       $name"
}

show_one() { printf '  %s  %s\n            %s\n' "${1:0:12}…" "$2" "$3"; }

cmd=${1:---fetch}
case "$cmd" in
--fetch)
    [ -f "$manifest" ] || die "no manifest at $manifest"
    echo "fetching into data/logs"
    failed=0
    each_entry fetch_one || failed=1
    [ "$failed" = 0 ] || die "one or more entries failed"
    ;;

--check)
    [ -f "$manifest" ] || die "no manifest at $manifest"
    command -v "$python" >/dev/null || die "$python not found; set PYTHON="
    "$python" -c 'import pyulog' 2>/dev/null ||
        die "pyulog not available to $python. \`uv venv && uv pip install pyulog\` from the repository root, or run the converter directly with \`uv run tools/ulog2replay.py\`, or set PYTHON= to an interpreter that has it"
    echo "checking the corpus end to end"
    failed=0
    each_entry check_one || failed=1
    [ "$failed" = 0 ] || die "one or more entries did not match their expectations"
    ;;

--verify)
    [ -f "$manifest" ] || die "no manifest at $manifest"
    failed=0
    each_entry verify_one || failed=1
    [ "$failed" = 0 ] || die "verification failed"
    ;;

--list)
    [ -f "$manifest" ] || die "no manifest at $manifest"
    each_entry show_one
    ;;

--add)
    url=${2:-}
    [ -n "$url" ] || die "usage: data/fetch.sh --add URL [NAME]"
    name=${3:-$(basename "${url%%\?*}")}
    # A PX4 Flight Review download URL carries the log id in a query string, so the
    # basename is useless; name it after the id.
    case "$name" in
        download|'') name="$(echo "$url" | sed -n 's/.*[?&]log=\([0-9a-f-]*\).*/\1/p').ulg" ;;
    esac
    [ "$name" != ".ulg" ] || die "could not infer a name; pass one explicitly"

    mkdir -p "$dest"
    echo "fetching $name"
    curl -fL --progress-bar -o "$dest/$name" "$url" || die "download failed: $url"
    sum=$(sha256 "$dest/$name")

    if [ -f "$manifest" ] && grep -qF "  $name  " "$manifest"; then
        die "$name is already in the manifest; remove that line first"
    fi
    [ -f "$manifest" ] || printf '# sha256  name  url\n' > "$manifest"
    printf '%s  %s  %s\n' "$sum" "$name" "$url" >> "$manifest"
    echo "added to data/manifest.txt:"
    echo "  $sum  $name"
    echo
    echo "Commit the manifest line so the corpus is reproducible. Check the source's"
    echo "terms before redistributing the file itself."
    ;;

-h|--help)
    sed -n '2,/^set -euo/p' "$0" | sed -e 's/^#//' -e 's/^ //' -e '$d'
    ;;

*)
    die "unknown option $cmd (try --help)"
    ;;
esac
