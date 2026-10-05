# How `panic-check`'s ELF is built, sourced by `panic-check/run.sh`, which scans it for panics,
# and by `tools/footprint.sh`, which measures its flash. One file, so both read a binary built
# under one profile, for the same targets, at the same levels. Each builds its own (the gate on
# stable, the footprint on its pinned nightly), so they share a recipe rather than a binary.

# shellcheck disable=SC2034 # read by the scripts that source this file
THUMB_TARGETS=(thumbv6m-none-eabi thumbv7em-none-eabihf)

# Both release optimization levels a firmware build plausibly ships. `z` and `1` are
# deliberately not gated: there LLVM stops proving that `nalgebra`'s statically sized
# `Matrix3 * Vector3` is in bounds and leaves the check in as dead code, which is a
# codegen artifact of the optimization level, not a path this crate can take. See
# DESIGN.md, "The library cannot panic".
# shellcheck disable=SC2034
LEVELS=(3 s)

# The release profile, exported. A function rather than top-level exports, so a script can
# build something else first: an rlib under fat LTO holds bitcode, and bitcode has no stack
# sizes for `tools/footprint.sh` to read.
#
# Set here rather than in `Cargo.toml`, where a workspace member's `[profile]` is ignored.
# The link has to see the whole program at once: a panic left in `nalgebra` or `libm` is as
# fatal as one left in `src/`, and only fat LTO puts it in the same module as the caller that
# would have to prove it dead.
link_profile() {
    export CARGO_PROFILE_RELEASE_LTO=fat
    export CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1
    export CARGO_PROFILE_RELEASE_PANIC=abort
}
