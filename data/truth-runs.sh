# shellcheck shell=bash
#
# Replaying a dataset against its truth and asserting a pins file, for data/urbannav.sh and
# data/insane.sh: what each converts and which keys it pins is the caller's, the build,
# the replay, the picking and the comparison are here. Sourced, never run.
#
# A pins file holds `<name> <run> key=value...`, the pair language of data/manifest.txt
# (data/expect.sh), decimals with 1 % either side.
#
# The caller sets `root`, `pins`, `keys` (whitespace-separated) and `pin` (1 to print lines
# to commit), defines `die`, and reads `failed` once every run is done.
# shellcheck disable=SC2154,SC2034 # those variables are the caller's

# shellcheck source=data/expect.sh
. "$root/data/expect.sh"

failed=0

# Build the replay example and set `replay` to it, and `target` to cargo's target directory.
build_replay() {
    echo "building"
    (cd "$root" && cargo build --quiet --release --example replay) || die "build failed"
    target=$(cd "$root" && cargo metadata --format-version 1 --no-deps |
        python3 -c 'import json, sys; print(json.load(sys.stdin)["target_directory"])') ||
        die "cargo metadata failed"
    replay="$target/release/examples/replay"
}

# pinned_run NAME RUN OUT INPUT TRUTH [OPTIONS...]: replay INPUT against TRUTH into OUT.csv,
# its console in OUT.out, then print the pin line (--pin) or compare the picked keys with
# the pins file's line.
pinned_run() {
    local name=$1 run=$2 out=$3 input=$4 truth=$5
    shift 5
    "$replay" "$@" "$input" "$out.csv" "$truth" > "$out.out" 2>&1 ||
        die "replaying $name $run failed"
    local line key value picked="" expect
    line="$(grep '^summary ' "$out.out") $(grep '^score ' "$out.out")"
    for key in $keys; do
        value=$(pair_value "$line" "$key") || die "$name $run: no $key= on the lines"
        picked="${picked:+$picked }$key=$value"
    done
    if [ "$pin" = 1 ]; then
        echo "$name $run $(pin_pairs "run $picked" --decimal)"
        return
    fi
    expect=$(awk -v n="$name" -v r="$run" '$1 == n && $2 == r { $1 = $2 = ""; print; exit }' "$pins")
    if [ -z "$expect" ]; then
        echo "  no expectations  $name $run"
        failed=1
    elif compare_pairs "$picked" "$expect" "$name $run"; then
        echo "  ok       $name $run"
    else
        echo "    got $picked" >&2
        failed=1
    fi
}
