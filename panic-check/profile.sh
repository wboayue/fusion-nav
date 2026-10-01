# The release profile `panic-check`'s ELF is linked under, sourced by `panic-check/run.sh`,
# which scans it for panics, and by `tools/footprint.sh`, which measures its flash. One file,
# so the binary the flash figures describe is the one the panic gate cleared.
#
# Set here rather than in `Cargo.toml`, where a workspace member's `[profile]` is ignored.
# The link has to see the whole program at once: a panic left in `nalgebra` or `libm` is as
# fatal as one left in `src/`, and only fat LTO puts it in the same module as the caller that
# would have to prove it dead.
export CARGO_PROFILE_RELEASE_LTO=fat
export CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1
export CARGO_PROFILE_RELEASE_PANIC=abort

# Both release optimization levels a firmware build plausibly ships. `z` and `1` are
# deliberately not gated: there LLVM stops proving that `nalgebra`'s statically sized
# `Matrix3 * Vector3` is in bounds and leaves the check in as dead code, which is a
# codegen artifact of the optimization level, not a path this crate can take. See
# README.md, "The library cannot panic".
# shellcheck disable=SC2034 # read by the scripts that source this file
LEVELS=(3 s)
