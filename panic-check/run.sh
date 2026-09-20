#!/usr/bin/env bash
#
# Fail if the linked library can reach `core::panicking`.
#
# `src/main.rs` calls every entry point the filter publishes; linking it with fat LTO
# leaves exactly the code an application could reach. A surviving reference to
# `core::panicking` is therefore a reachable `unwrap`, `expect`, `panic!` or slice index
# — on `thumbv6m` a `udf` instruction, and the vehicle is a brick.
#
# Usage: panic-check/run.sh [target ...]

set -euo pipefail

cd "$(dirname "$0")"

# Set here rather than in `Cargo.toml`, where a workspace member's `[profile]` is ignored.
# The link has to see the whole program at once: a panic left in `nalgebra` or `libm` is as
# fatal as one left in `src/`, and only fat LTO puts it in the same module as the caller that
# would have to prove it dead.
export CARGO_PROFILE_RELEASE_LTO=fat
export CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1
export CARGO_PROFILE_RELEASE_PANIC=abort

TARGETS=("${@:-}")
[[ -z "${TARGETS[0]}" ]] && TARGETS=(thumbv7em-none-eabihf thumbv6m-none-eabi)

# Both release optimization levels a firmware build plausibly ships. `z` and `1` are
# deliberately not gated: there LLVM stops proving that `nalgebra`'s statically sized
# `Matrix3 * Vector3` is in bounds and leaves the check in as dead code, which is a
# codegen artifact of the optimization level, not a path this crate can take. See
# README.md, "The library cannot panic".
LEVELS=(3 s)

HOST=$(rustc -vV | sed -n 's/^host: //p')
TOOLS="$(rustc --print sysroot)/lib/rustlib/$HOST/bin"
if [[ ! -x "$TOOLS/llvm-nm" ]]; then
  echo "error: llvm-nm not found in $TOOLS -- run 'rustup component add llvm-tools'" >&2
  exit 1
fi

# The gate proves nothing about an entry point nobody calls, so the driver has to keep up
# with the API. These two modules are the ones that do arithmetic; the rest of the public
# surface reaches the linker through them.
missing=()
for name in $(grep -hoE '^    pub (const )?fn [a-z_0-9]+' ../src/eskf.rs ../src/geodetic.rs |
  awk '{ print $NF }' | sort -u); do
  grep -qF "$name(" src/main.rs || missing+=("$name")
done
if ((${#missing[@]})); then
  echo "error: src/main.rs links none of these public entry points:" >&2
  printf '       %s\n' "${missing[@]}" >&2
  echo "       the scan proves nothing about a function the linker never saw" >&2
  exit 1
fi

status=0
for target in "${TARGETS[@]}"; do
  for level in "${LEVELS[@]}"; do
    CARGO_PROFILE_RELEASE_OPT_LEVEL=$level \
      cargo build --quiet --release --package panic-check --target "$target"
    elf="../target/$target/release/panic-check"

    # `rust_begin_unwind` is the panic handler's export name, checked as well in case LTO
    # ever folds `panic_fmt` into its caller and leaves no `core::panicking` symbol.
    if ! "$TOOLS/llvm-nm" --demangle "$elf" | grep -qE 'core::panicking|rust_begin_unwind'; then
      echo "ok   $target opt-level=$level: no reachable panic"
      continue
    fi

    # Name the callers rather than the panic symbol: the symbol says the property broke,
    # the caller says where to look.
    echo "FAIL $target opt-level=$level: these functions can panic" >&2
    "$TOOLS/llvm-objdump" --disassemble --demangle --no-show-raw-insn "$elf" |
      awk '/^[0-9a-f]+ <.*>:$/ { fn = $2 } /<core::panicking|<rust_begin_unwind/ { print fn }' |
      grep -vE 'core::panicking|rust_begin_unwind' | sort -u |
      sed -e 's/^</       /' -e 's/>:$//' -e 's/::h[0-9a-f]*$//' >&2
    status=1
  done
done

exit $status
