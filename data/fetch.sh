#!/usr/bin/env bash
#
# Fetch and verify the replay corpus.
#
#   data/fetch.sh                    fetch everything in the manifest, verify checksums
#   data/fetch.sh --verify           verify what is already on disk, download nothing
#   data/fetch.sh --add URL [NAME]   download once, record its checksum, append to manifest
#   data/fetch.sh --check            convert and replay each log, assert its expectations
#   data/fetch.sh --pin NAME         convert and replay one log, print expectations to commit
#   data/fetch.sh --compare [--pin]  replay every log beside EKF2 under both R policies,
#                                    assert data/ekf2.txt (or print its lines to commit)
#   data/fetch.sh --list             show the manifest
#   data/fetch.sh --venv             create .venv with the pyulog version the converter pins
#   data/fetch.sh --manifest FILE [--fetch|--verify|--list]
#                                    the same for another manifest, into data/<FILE's stem>
#
# Logs are large and their redistribution terms are usually unstated, so they are
# fetched rather than committed. The manifest pins a sha256 per file so a corpus is
# reproducible without the repo carrying it: someone runs --add once and commits the
# manifest line, everyone else gets a verified copy.
#
# data/flight.csv is not managed here. It is small, checked in so that
# `cargo run --example replay` works with no network, and generated rather than downloaded:
# `cargo run --example simulate -- flight data` rewrites it and its truth file.
#
# A second manifest is a second licence (AGENTS.md, "two corpora, two licences, two
# manifests"): data/urbannav.txt is fetched only when named, into its own directory, and its
# `# terms:` lines are printed before anything is downloaded. Only fetching, verifying and
# listing take it; replaying one is its own script's (data/urbannav.sh), since what is
# converted from which files is particular to the dataset.
#
# --check is a local tool, not a CI job. It needs pyulog, and putting the converter in
# the test path is exactly what GOALS.md's harness constraint rules out: CI replays the
# synthetic CSV only, so it needs no network, no PX4 tooling and no hardware. Run
# --check before a release, or after touching the converter or anything it asserts.

set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
manifest="$root/data/manifest.txt"
dest="$root/data/logs"
if [ "${1:-}" = "--manifest" ]; then
    [ -n "${2:-}" ] || { echo "fetch: usage: data/fetch.sh --manifest FILE [COMMAND]" >&2; exit 1; }
    manifest=$(cd "$(dirname "$2")" && pwd)/$(basename "$2")
    name=$(basename "$manifest" .txt)
    [ "$name" = manifest ] || dest="$root/data/$name"
    shift 2
fi
# The converter needs pyulog, which CI never installs (GOALS.md, "Harness constraint").
# `--venv` puts it in .venv, which is gitignored and picked up here without an activated
# shell; PYTHON= overrides for any other interpreter that has it.
# Which version matters, not just that it is installed: this path and `uv run` have to agree
# or the two stop producing the same CSV. --venv installs the pin and --check asserts it,
# both reading it from tools/ulog2replay.py so that no second copy exists to drift.
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

# shellcheck source=data/expect.sh
. "$root/data/expect.sh"

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

# Convert one log and replay it, printing the `summary` line the example prints.
replay_summary() {
    local name=$1
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
    echo "$summary"
}

# Convert one log with EKF2's reference and replay it under each R policy, into the layout
# `tools/replay_report.py --corpus` reads: `<out>/<log>/{input,reference}.csv` and, per policy,
# `<policy>.csv`, its `.fusion.csv` and the captured `<policy>.summary`. A failure leaves a
# FAILED file beside them rather than an exit status, because these run in the background.
compare_one() {
    local name=$2 dir="$out/${2%.ulg}"
    local ulg="$dest/$name"
    mkdir -p "$dir"
    if [ ! -f "$ulg" ]; then
        echo "missing — run data/fetch.sh first" > "$dir/FAILED"
        return 0
    fi
    if ! "$python" "$root/tools/ulog2replay.py" "$ulg" -o "$dir/input.csv" \
        --reference "$dir/reference.csv" >/dev/null 2>&1; then
        echo "convert failed" > "$dir/FAILED"
        return 0
    fi
    local policy
    for policy in raw px4; do
        "$replay" --r-policy "$policy" "$dir/input.csv" "$dir/$policy.csv" \
            > "$dir/$policy.summary" 2>/dev/null ||
            { echo "replay --r-policy $policy failed" > "$dir/FAILED"; return 0; }
    done
}
spawn_compare() { compare_one "$@" & }

# The keys a log's manifest entry pins, one per line: a raw run's agreement line repeats them,
# and `--compare --pin` leaves them to the manifest rather than pinning them twice.
manifest_keys() {
    local name=$1 sum entry url expect pair
    [ -f "$manifest" ] || return 0
    while read -r sum entry url expect; do
        [ "$entry" = "$name" ] || continue
        for pair in $expect; do
            pair=${pair%%<=*} pair=${pair%%>=*}
            echo "${pair%%=*}"
        done
    done < "$manifest"
}

# `line` without the pairs whose key is one of `keys` (newline-separated).
without_keys() {
    local line=$1 keys=$2 word out=""
    local restore_glob=0
    case $- in *f*) ;; *) restore_glob=1; set -f ;; esac
    for word in $line; do
        if ! printf '%s\n' "$keys" | grep -qxF "${word%%=*}"; then
            out="${out:+$out }$word"
        fi
    done
    [ "$restore_glob" = 1 ] && set +f
    echo "$out"
}

# The expectations data/ekf2.txt holds for one `agreement` line: `<log> <policy> pairs...`.
ekf2_expectations() {
    local log=$1 policy=$2 name want_policy expect
    [ -f "$ekf2" ] || return 0
    while read -r name want_policy expect; do
        case "$name" in ''|\#*) continue ;; esac
        if [ "$name" = "$log" ] && [ "$want_policy" = "$policy" ]; then
            echo "$expect"
            return 0
        fi
    done < "$ekf2"
}

# Replay one log, asserting the manifest's expectations against its `summary` line.
check_one() {
    local name=$2 expect=$4 summary
    summary=$(replay_summary "$name") || return 1
    if [ -z "$expect" ]; then
        echo "  no expectations  $name — ${summary#summary }"
        return 0
    fi
    if compare_pairs "$summary" "$expect" "$name"; then
        echo "  ok       $name"
        return 0
    fi
    echo "    got ${summary#summary }" >&2
    return 1
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

# The pyulog version `tools/ulog2replay.py` declares in its PEP 723 header. That header is
# the pin -- `uv run` resolves it -- so reading it here keeps one copy rather than two that
# drift, and keeps the version out of every set of instructions that would name it.
pyulog_pin() {
    local pin
    pin=$(sed -n 's/^# dependencies = \["pyulog==\([^"]*\)"\].*/\1/p' "$root/tools/ulog2replay.py")
    [ -n "$pin" ] || die "no pyulog pin in tools/ulog2replay.py; expected a PEP 723 \`dependencies = [\"pyulog==X.Y.Z\"]\` line"
    echo "$pin"
}

# Refuse to convert with anything but the pyulog the converter pins, so --check and --pin
# produce the CSV `uv run` would.
require_pinned_pyulog() {
    local pin have
    command -v "$python" >/dev/null || die "$python not found; set PYTHON="
    # Suggesting `uv run` as the remedy would be no remedy at all: replay_summary converts with
    # $python, so uv never enters this path.
    pin=$(pyulog_pin)
    # Imports as well as reports a version. Metadata alone answers a different question:
    # a present dist-info over a package that cannot import -- a missing numpy, a
    # half-removed install, an ABI mismatch after a Python upgrade -- would pass here and
    # then fail inside replay_summary, which discards converter stderr and would print only
    # `CONVERT FAILED` once per log with no cause.
    have=$("$python" -c 'import pyulog, importlib.metadata as m; print(m.version("pyulog"))' 2>/dev/null) ||
        die "pyulog not importable by $python. \`data/fetch.sh --venv\` from the repository root, or set PYTHON= to an interpreter that has it"
    [ "$have" = "$pin" ] ||
        die "$python has pyulog $have, but tools/ulog2replay.py pins $pin. \`uv run\` would use $pin, so the two paths would not produce the same CSV. \`uv pip install pyulog==$pin\`, or move the pin if $have is the version you mean to adopt"
}

cmd=${1:---fetch}
if [ "$dest" != "$root/data/logs" ]; then
    case "$cmd" in
        --fetch|--verify|--list) ;;
        *) die "$cmd reads data/manifest.txt only; $(basename "$manifest") is fetched, verified and listed here" ;;
    esac
fi
case "$cmd" in
--fetch)
    [ -f "$manifest" ] || die "no manifest at $manifest"
    # Stated at the point of download, before anything is fetched under them.
    sed -n 's/^# terms: \{0,1\}//p' "$manifest"
    echo "fetching into ${dest#"$root"/}"
    failed=0
    each_entry fetch_one || failed=1
    [ "$failed" = 0 ] || die "one or more entries failed"
    ;;

--check)
    [ -f "$manifest" ] || die "no manifest at $manifest"
    require_pinned_pyulog
    echo "checking the corpus end to end"
    failed=0
    each_entry check_one || failed=1
    [ "$failed" = 0 ] || die "one or more entries did not match their expectations"
    ;;

--compare)
    [ -f "$manifest" ] || die "no manifest at $manifest"
    require_pinned_pyulog
    command -v uv >/dev/null || die "uv not found; tools/replay_report.py runs through it"
    pin=0
    [ "${2:-}" = "--pin" ] && pin=1
    ekf2="$root/data/ekf2.txt"
    echo "building"
    (cd "$root" && cargo build --quiet --release --example replay) || die "build failed"
    # Where cargo put it, which CARGO_TARGET_DIR or build.target-dir can move out of target/.
    target=$(cd "$root" && cargo metadata --format-version 1 --no-deps |
        python3 -c 'import json, sys; print(json.load(sys.stdin)["target_directory"])') ||
        die "cargo metadata failed"
    replay="$target/release/examples/replay"
    out="$target/compare"
    rm -rf "$out"
    mkdir -p "$out"
    # Stamped so the table names the build that produced it; `-dirty` says when that is not
    # a commit anyone can check out.
    git -C "$root" describe --always --dirty > "$out/commit"
    echo "replaying the corpus beside EKF2, raw and px4"
    each_entry spawn_compare
    wait
    failed=0
    for marker in "$out"/*/FAILED; do
        [ -e "$marker" ] || continue
        echo "  FAILED   $(basename "$(dirname "$marker")"): $(cat "$marker")" >&2
        failed=1
    done
    [ "$failed" = 0 ] || die "one or more logs did not replay"
    lines=$(cd "$root" && uv run --quiet tools/replay_report.py --corpus "$out" \
        -o "$out/agreement.html") || die "tools/replay_report.py --corpus failed"
    # Kept beside the table for tools/validation.sh, which publishes these lines rather than
    # recomputing them.
    printf '%s\n' "$lines" > "$out/agreement.txt"
    while read -r _ line; do
        # By key, not by position, so the line's order is the writer's to choose.
        log=$(pair_value "$line" log) || die "an agreement line with no log=: $line"
        policy=$(pair_value "$line" r_policy) || die "an agreement line with no r_policy=: $line"
        rest=$(without_keys "$line" "log")
        if [ "$pin" = 1 ]; then
            # The manifest pins the raw policy's summary keys already.
            [ "$policy" = raw ] && rest=$(without_keys "$rest" "$(manifest_keys "$log.ulg")")
            echo "$log $policy $(pin_pairs "agreement $rest" --decimal)"
            continue
        fi
        expect=$(ekf2_expectations "$log" "$policy")
        if [ -z "$expect" ]; then
            echo "  no expectations  $log $policy"
            failed=1
        elif compare_pairs "$line" "$expect" "$log $policy"; then
            echo "  ok       $log $policy"
        else
            echo "    got $rest" >&2
            failed=1
        fi
    done <<< "$lines"
    echo "table: $out/agreement.html"
    [ "$pin" = 1 ] || [ "$failed" = 0 ] || die "one or more runs did not match data/ekf2.txt"
    ;;

--pin)
    name=${2:-}
    [ -n "$name" ] || die "usage: data/fetch.sh --pin NAME"
    require_pinned_pyulog
    summary=$(replay_summary "$name") || die "no summary for $name"
    pin_pairs "$summary"
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

--venv)
    command -v uv >/dev/null || die "uv not found; see https://docs.astral.sh/uv/"
    pin=$(pyulog_pin)
    echo "installing pyulog==$pin into .venv"
    (cd "$root" && uv venv && uv pip install "pyulog==$pin")
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
