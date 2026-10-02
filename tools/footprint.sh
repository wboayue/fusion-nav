#!/usr/bin/env bash
#
# Measure what the filter costs a bare-metal target, and compare it with data/footprint.txt.
#
#   tools/footprint.sh                 measure both thumb targets, compare; a CI gate
#   tools/footprint.sh --pin           print data/footprint.txt's keys at their measured values
#   tools/footprint.sh --all           print every key measured, to choose what to pin
#   tools/footprint.sh --install       install the pinned nightly, its llvm-tools and targets
#
# Three figures per target, each one an integrator plans around (validation/cost.md): a type's
# size, from `-Zprint-type-sizes`; a function's stack frame, from `-Zemit-stack-sizes` at
# `opt-level = 3` in the library's own release profile; and the flash of `panic-check`'s ELF,
# which links the whole public API under fat LTO, at each level `panic-check/profile.sh` names.
# `tools/footprint.py` reads them into keys, and its docstring says what each key means and what it
# leaves out.
#
# Sizes are the target's: on a 64-bit host `usize` is wider, and `Eskf` reads 64 bytes larger
# than any integrator's.
#
# The line printed carries every key; data/footprint.txt pins the ones worth a sentence, and
# `data/expect.sh` compares them, so a pinned function renamed or inlined away is a missing key
# rather than a silent pass.
#
# Pins are exact, which is why the nightly is pinned: a frame moves by tens of bytes between
# nightlies with no line of the crate changed. Moving `TOOLCHAIN` is a re-pin, and its diff
# is what the new compiler did. A figure that shrinks fails too, so validation/cost.md, which
# renders from the pins, cannot fall behind the code it measures.

set -euo pipefail

TOOLCHAIN=nightly-2026-08-06

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
expectations="$root/data/footprint.txt"

die() { echo "footprint: $*" >&2; exit 1; }

# shellcheck source=data/expect.sh
. "$root/data/expect.sh"
# shellcheck source=panic-check/profile.sh
. "$root/panic-check/profile.sh"

mode=check
case "${1:-}" in
    '') ;;
    --pin) mode=pin ;;
    --all) mode=all ;;
    --install)
        targets="${THUMB_TARGETS[*]}"
        exec rustup toolchain install "$TOOLCHAIN" --profile minimal \
            --component llvm-tools --target "${targets// /,}" ;;
    *) die "unknown argument $1" ;;
esac

host=$(rustc "+$TOOLCHAIN" -vV 2>/dev/null | sed -n 's/^host: //p' || true)
[ -n "$host" ] || die "no $TOOLCHAIN: run tools/footprint.sh --install"
tools="$(rustc "+$TOOLCHAIN" --print sysroot)/lib/rustlib/$host/bin"
for tool in llvm-readobj llvm-nm llvm-size; do
    [ -x "$tools/$tool" ] || die "$tool not in $tools: run tools/footprint.sh --install"
done

# Its own directory, so a measurement never reads an artifact some other build left behind,
# and nothing here invalidates the caches `cargo test` keeps in target/.
export CARGO_TARGET_DIR="$root/target/footprint"
# CI's `-D warnings` changes no codegen, but a flag a caller exports could; measure the
# profile and nothing else.
export RUSTFLAGS=

# measure <target>: print one line of `key=value` pairs.
measure() {
    local target=$1 out="$CARGO_TARGET_DIR/$target" lib elf level
    # Called in a command substitution, where bash clears `-e`. Each step below says how it
    # stops rather than leaning on it.

    # `cargo rustc` hands the flags to this crate alone, so dependencies build as an integrator
    # builds them and print no layouts of their own. Cleaned first: a crate cargo considers
    # fresh is not compiled, and prints nothing to read.
    cargo "+$TOOLCHAIN" clean --quiet --release --target "$target" -p fusion-nav || exit 1
    mkdir -p "$out" || exit 1
    cargo "+$TOOLCHAIN" rustc --quiet --lib --release --target "$target" \
        -- -Zprint-type-sizes -Zemit-stack-sizes >"$out/types.txt" || exit 1
    lib="$out/release/libfusion_nav.rlib"
    [ -f "$lib" ] || die "cargo built no rlib at $lib"
    # Exits 1 on the rlib's metadata members, which carry no code, after printing the rest;
    # `footprint.py` refuses a report holding no frame at all.
    "$tools/llvm-readobj" --stack-sizes --demangle "$lib" >"$out/frames.txt" 2>/dev/null || true

    # The ELF under `panic-check`'s profile, in a subshell so its fat LTO never reaches the
    # library build above.
    (
        link_profile
        elf="$out/release/panic-check"
        for level in "${LEVELS[@]}"; do
            # Removed first, so a stale binary cannot be measured in place of this level's.
            rm -f "$elf"
            CARGO_PROFILE_RELEASE_OPT_LEVEL=$level cargo "+$TOOLCHAIN" build --quiet \
                --release --package panic-check --target "$target" || exit 1
            [ -f "$elf" ] || die "cargo built no ELF at $elf"
            echo "level $level"
            "$tools/llvm-size" -A "$elf" || exit 1
            "$tools/llvm-nm" --demangle --print-size --size-sort "$elf" || exit 1
        done
    ) >"$out/flash.txt" || exit 1

    # Through files rather than the environment: the symbol table alone is past the 128 KB
    # Linux allows one environment string.
    python3 "$root/tools/footprint.py" "$out/types.txt" "$out/frames.txt" "$out/flash.txt"
}

[ -f "$expectations" ] || die "no expectations at $expectations"

# pin <target> <line>: the file's lines for <target>, each key at its measured value. What
# `--pin` prints, and what a failure prints, so re-pinning is a copy rather than a second run.
pin() {
    local target=$1 line=$2 label pairs pair key value out
    awk -v t="$target" '$1 == t' "$expectations" | while read -r label pairs; do
        out=$label
        set -f
        for pair in $pairs; do
            key=${pair%%[<>]=*}
            key=${key%%=*}
            value=$(pair_value "$line" "$key") || value=MISSING
            out+=" $key=$value"
        done
        set +f
        echo "$out"
    done
}

failed=0
for target in "${THUMB_TARGETS[@]}"; do
    line=$(measure "$target")
    case $mode in
        all) echo "$target $line"; continue ;;
        pin) pin "$target" "$line"; continue ;;
    esac
    # Every line of the file labeled with this target, joined: one line per kind of figure
    # keeps the file readable and the comparison whole.
    expect=$(awk -v t="$target" '$1 == t { $1 = ""; print }' "$expectations" | tr '\n' ' ')
    [ -n "${expect// /}" ] || die "nothing pinned for $target in $expectations"
    if compare_pairs "$line" "$expect" "$target"; then
        echo "ok   $target"
    else
        echo "FAIL $target, measured:" >&2
        pin "$target" "$line" >&2
        failed=1
    fi
done
# A re-pin moves validation/cost.md, which renders from these pins.
if [ "$failed" = 1 ]; then
    echo "after re-pinning, render validation/cost.md: validation/README.md, \"The cost page\"" >&2
fi
exit $failed
