#!/usr/bin/env bash
#
# Measure what the filter costs a bare-metal target, and compare it with data/footprint.txt.
#
#   tools/footprint.sh                 measure both thumb targets, compare; a CI gate
#   tools/footprint.sh --pin           print data/footprint.txt's keys at their measured values
#   tools/footprint.sh --all           print every key measured, to choose what to pin
#   tools/footprint.sh --toolchain     the pinned nightly, for CI to install
#
# Three figures per target, each the one an integrator plans around (DESIGN.md, "Measured cost,
# by function"):
#
#   size.<path>=   bytes of a type, from `-Zprint-type-sizes`. The filter is allocation-free,
#                  so `size.eskf.Eskf` is its RAM, and it is the pin that notices a new state.
#                  Taken on the target rather than the host, where `usize` is wider and
#                  `Eskf` reads 64 bytes larger than any integrator's.
#   frame.<path>=  bytes of one function's stack frame, from `-Zemit-stack-sizes`, at
#                  `opt-level = 3` in the library's own release profile. A path is the
#                  function's, demangled: `eskf.Eskf.predict`, `update.update.3` for
#                  `update::<3>`. A function generic over a closure is one key, its largest
#                  instance. Trait impls and closures are left out.
#   text.<level>=, rodata.<level>=, text_<crate>.<level>=
#                  flash of `panic-check`'s ELF, which links the whole public API under fat
#                  LTO, at each opt-level `panic-check/profile.sh` names. `text_<crate>` sums
#                  the symbols each crate kept out of line: `libm`, `compiler_builtins` (soft
#                  float on `thumbv6m`) and `nalgebra`. Inlined code is its caller's, so what
#                  `nalgebra` costs mostly reads as the filter's.
#
# The line printed carries every key; data/footprint.txt pins the ones worth a sentence, and
# `data/expect.sh` compares them, so a pinned function that is renamed or inlined away is a
# missing key rather than a silent pass.
#
# Pins are exact, which is why the nightly is pinned: a frame moves by tens of bytes between
# nightlies with no line of the crate changed. Moving `TOOLCHAIN` is a re-pin, and its diff
# is what the new compiler did. A figure that shrinks fails too, so DESIGN.md's table cannot
# fall behind the code it measures.

set -euo pipefail

TOOLCHAIN=nightly-2026-08-06
TARGETS="thumbv6m-none-eabi thumbv7em-none-eabihf"

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
expectations="$root/data/footprint.txt"

die() { echo "footprint: $*" >&2; exit 1; }

mode=check
case "${1:-}" in
    '') ;;
    --pin) mode=pin ;;
    --all) mode=all ;;
    --toolchain) echo "$TOOLCHAIN"; exit 0 ;;
    *) die "unknown argument $1" ;;
esac

# shellcheck source=data/expect.sh
. "$root/data/expect.sh"

host=$(rustc "+$TOOLCHAIN" -vV 2>/dev/null | sed -n 's/^host: //p') ||
    die "no $TOOLCHAIN: rustup toolchain install $TOOLCHAIN --profile minimal -c llvm-tools -t ${TARGETS// /,}"
[ -n "$host" ] || die "no $TOOLCHAIN: rustup toolchain install $TOOLCHAIN --profile minimal -c llvm-tools -t ${TARGETS// /,}"
tools="$(rustc "+$TOOLCHAIN" --print sysroot)/lib/rustlib/$host/bin"
for tool in llvm-readobj llvm-nm llvm-size; do
    [ -x "$tools/$tool" ] || die "$tool not in $tools: rustup component add --toolchain $TOOLCHAIN llvm-tools"
done

# Its own directory, so a measurement never reads an artifact some other build left behind,
# and nothing here invalidates the caches `cargo test` keeps in target/.
export CARGO_TARGET_DIR="$root/target/footprint"
# CI's `-D warnings` changes no codegen, but a flag a caller exports could; measure the
# profile and nothing else.
export RUSTFLAGS=

# measure <target>: print one line of `key=value` pairs.
measure() {
    local target=$1 lib out="$CARGO_TARGET_DIR/$target"
    # Called in a command substitution, where bash clears `-e`; a failed build stops here.
    set -e

    # `cargo rustc` hands the flags to this crate alone, so dependencies build as an integrator
    # builds them and print no layouts of their own. Cleaned first: a crate cargo considers
    # fresh is not compiled, and prints nothing to read.
    cargo "+$TOOLCHAIN" clean --quiet --release --target "$target" -p fusion-nav
    mkdir -p "$out"
    cargo "+$TOOLCHAIN" rustc --quiet --lib --release --target "$target" \
        -- -Zprint-type-sizes -Zemit-stack-sizes >"$out/types.txt"
    lib="$CARGO_TARGET_DIR/$target/release/libfusion_nav.rlib"
    [ -f "$lib" ] || die "cargo built no rlib at $lib"
    # Exits 1 on the rlib's metadata members, which carry no code, after printing the rest.
    "$tools/llvm-readobj" --stack-sizes --demangle "$lib" >"$out/frames.txt" 2>/dev/null || true

    # The ELF under `panic-check`'s profile, in a subshell so its fat LTO never reaches the
    # library build above: an LTO rlib holds bitcode, and bitcode has no stack sizes.
    (
        # shellcheck source=panic-check/profile.sh
        . "$root/panic-check/profile.sh"
        elf="$CARGO_TARGET_DIR/$target/release/panic-check"
        for level in "${LEVELS[@]}"; do
            # Removed first, so a stale binary cannot be measured in place of this level's.
            rm -f "$elf"
            CARGO_PROFILE_RELEASE_OPT_LEVEL=$level \
                cargo "+$TOOLCHAIN" build --quiet --release --package panic-check --target "$target"
            [ -f "$elf" ] || die "cargo built no ELF at $elf"
            echo "level $level"
            "$tools/llvm-size" -A "$elf"
            "$tools/llvm-nm" --demangle --print-size --size-sort "$elf"
        done
    ) >"$out/flash.txt" || exit 1

    # Through files rather than the environment: the symbol table alone is past the 128 KB
    # Linux allows one environment string.
    python3 - "$out/types.txt" "$out/frames.txt" "$out/flash.txt" <<'PY'
import re, sys

types, frames, flash = (open(name).read() for name in sys.argv[1:])

pairs = {}

def put(key, value):
    # Two monomorphizations or codegen units can carry one name; the larger is what a caller
    # can meet.
    pairs[key] = max(value, pairs.get(key, 0))

def generics(name):
    """Drop each `::<...>`, keeping a leading constant: `observe::<3, {closure}>` -> `observe::3`.

    A function generic over a closure is one key, holding its largest instance.
    """
    out, i = "", 0
    while i < len(name):
        if name.startswith("::<", i):
            depth, j = 0, i + 2
            while j < len(name):
                depth += {"<": 1, ">": -1}.get(name[j], 0)
                if depth == 0:
                    break
                j += 1
            constant = re.match(r"\d+", name[i + 3 : j])
            out += "::" + constant.group(0) if constant else ""
            i = j + 1
        else:
            out += name[i]
            i += 1
    return out

def path(name):
    """`<fusion_nav::eskf::Eskf>::predict` -> `eskf.Eskf.predict`; None for any other crate."""
    name = generics(re.sub(r" \(\.llvm\.\d+\)$", "", name))
    m = re.fullmatch(r"<fusion_nav::([A-Za-z0-9_:]+)>::([A-Za-z0-9_:]+)", name)
    if m:
        name = m.group(1) + "::" + m.group(2)
    elif name.startswith("fusion_nav::"):
        name = name[len("fusion_nav::"):]
    else:
        return None
    if not re.fullmatch(r"[A-Za-z0-9_:]+", name):
        return None
    return name.replace("::", ".")

for line in types.splitlines():
    m = re.match(r"print-type-size type: `([A-Za-z0-9_:]+)`: (\d+) bytes", line)
    if m:
        put("size." + m.group(1).replace("::", "."), int(m.group(2)))

name = None
for line in frames.splitlines():
    m = re.search(r"Functions: \[(.*)\]", line)
    if m:
        # A folded entry names every function sharing the code; each has the frame.
        name = m.group(1)
        continue
    m = re.search(r"Size: 0x([0-9A-Fa-f]+)", line)
    if m and name is not None:
        for function in name.split(", "):
            key = path(function)
            if key is not None:
                put("frame." + key, int(m.group(1), 16))
        name = None

def crate(symbol):
    symbol = symbol.lstrip("<&")
    if "::" not in symbol:
        # Unmangled: the soft-float and memory routines, and the C math names
        # `compiler_builtins` exports. `_start` is the driver.
        return None if symbol == "_start" else "compiler_builtins"
    return symbol.split("::", 1)[0]

level = None
seen = set()
for line in flash.splitlines():
    if line.startswith("level "):
        level = line.split()[1]
        seen = set()
        continue
    m = re.match(r"(\.text|\.rodata)\s+(\d+)\s", line)
    if m:
        pairs[f"{m.group(1)[1:]}.{level}"] = int(m.group(2))
        continue
    m = re.match(r"([0-9a-f]{8}) ([0-9a-f]{8}) [tT] (.*)$", line)
    if m:
        address = m.group(1)
        if address in seen:  # an alias of a symbol already counted
            continue
        seen.add(address)
        owner = crate(m.group(3))
        if owner in ("libm", "compiler_builtins", "nalgebra"):
            key = f"text_{owner}.{level}"
            pairs[key] = pairs.get(key, 0) + int(m.group(2), 16)

print(" ".join(f"{k}={v}" for k, v in sorted(pairs.items())))
PY
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
for target in $TARGETS; do
    line=$(measure "$target")
    case $mode in
        all) echo "$target $line"; continue ;;
        pin) pin "$target" "$line"; continue ;;
    esac
    # Every line of the file labelled with this target, joined: one line per kind of figure
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
exit $failed
