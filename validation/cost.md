<!-- Generated from validation/src/cost.md by tools/validation.sh; edit that, not this. -->
# Cost on a target

**What does the filter cost a microcontroller?** The whole filter, `Eskf`, is
3776 bytes on a Cortex-M0 and allocates nothing
else, so that is the RAM to plan for beyond the stack. The deepest stack is a GNSS velocity
update, and linking every entry point takes 107894 bytes of
flash at `opt-level = "s"`. Execution time on hardware is not measured yet (#41).

Every figure on this page is pinned exactly in `data/footprint.txt`, which CI measures on
`nightly-2026-08-06` with `tools/footprint.sh` and fails on any move, growth or shrinkage.
CI also checks that this page shows the pinned values.

## What is measured, and what is not

The two targets are `thumbv6m-none-eabi` (Cortex-M0 and M0+, no FPU, so every float operation
is a library call) and `thumbv7em-none-eabihf` (Cortex-M4 and M7 with a single-precision FPU).

- **Sizes** are `size_of` each type on the target, at any `opt-level`.
- **Stack** is each function's own frame at `opt-level = 3`, read from the compiler
  (`-Zemit-stack-sizes`). A frame is one function's, so a path's stack is the sum of the frames
  along it, down to the last call made out of line. For a generic function the figure is its
  largest instance.
- **Flash** is a bare-metal binary (`panic-check/`) that calls the whole public API, linked
  with fat LTO. An application reaching fewer entry points links less.
- **Not measured:** cycle counts and a painted stack high-water mark on a real board. Both are
  #41, and land on this page.

Why each function has the form it has, and what the forms not taken would have cost, is in
[DESIGN.md, "Measured cost, by function"](../DESIGN.md#measured-cost-by-function), together with
host timings, which are taken on one machine and pinned nowhere.

## Memory

| type | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `Eskf`, the filter | 3776 | 3776 |
| of which the state history of (23′) | 1544 | 1544 |
| of which the covariance `P` | 900 | 900 |
| of which `Diagnostics` | 768 | 768 |
| of which the barometric offset of (30′) | 64 | 64 |
| `State`, the estimate `state()` returns | 72 | 72 |
| `Config` | 244 | 244 |
| `StaticWindow`, at any rate and length | 944 | 944 |
| `StaticSample` | 80 | 80 |
| `Startup`, a start worked out and checked before it commits, on a start's stack | 1088 | 1088 |

Sizes in bytes.

## Stack

Each entry point's own frame, in bytes, and the frames beneath it that set its depth.

| function | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `predict` | 2176 | 2160 |
| `fuse_gnss_position` | 1376 | 1344 |
| `fuse_gnss_geodetic` | 568 | 560 |
| `fuse_gnss_velocity` | 1416 | 1408 |
| `fuse_baro_altitude` | 1272 | 1256 |
| `fuse_mag_heading` | 112 | 104 |
| `fuse_gnss_heading` | 248 | 256 |
| `fuse_course` | 256 | 264 |
| `predicted_validity` | 1856 | 1832 |
| `initialize` | 2224 | 2216 |
| `initialize_coarse` | 3576 | 3552 |
| `initialize_from` | 1928 | 1856 |
| beneath the three heading entry points: the heading update they share | 1232 | 1240 |
| beneath `fuse_gnss_velocity`: the update of (23)–(27), three measurements | 8088 | 7960 |
| beneath `fuse_gnss_position` (and so a geodetic fix): two, the horizontal pair | 7400 | 7320 |
| beneath `fuse_gnss_position`'s height, `fuse_baro_altitude` and the heading update: one | 6368 | 6216 |
| beside the update: the observation formed at the measurement's time, (23′) | 1464 | 1440 |
| beside the update: committing its result, or handing it to an adoption | 1120 | 1064 |
| beside the update: adopting a position, from `fuse_gnss_position` | 1984 | 1992 |
| beside the update: adopting a velocity, from `fuse_gnss_velocity` | 1904 | 1896 |
| beneath the update: the attitude reset of (41) | 456 | 448 |
| beneath the update: the injection of (39)–(40) | 168 | 88 |
| beneath `predict`: one sample, (9)–(22) | 1088 | 1096 |
| beneath that: the covariance step of (22) | 2832 | 2760 |
| beneath that and the reset of (41): symmetry, (42) | 128 | 8 |
| beneath `predict`, across a gap: the coast of (22′) | 2152 | 2144 |
| beneath `predicted_validity`: the covariance carried over the horizon | 1952 | 1936 |
| beneath `initialize` and `initialize_coarse`: the initial covariance | 1120 | 1080 |

The deepest path is `fuse_gnss_velocity` calling the three-measurement update, and below the
update the calls it makes out of line, the deepest of them `nalgebra`'s 15 × 15 matrix product.
The compiler measures `nalgebra`'s frames, but `tools/footprint.sh` keys only this crate's, so
the rows above leave the deepest frame on that path out of the stack an integrator plans for.
One pinned figure for the whole path, walked call by call, is #202; until then DESIGN.md quotes
the walk, and why no other path is deeper.

## Flash

| bytes | `thumbv6m` `3` | `thumbv6m` `s` | `thumbv7em` `3` | `thumbv7em` `s` |
| --- | --- | --- | --- | --- |
| `.text` | 167798 | 107894 | 184012 | 114300 |
| of which `libm` | 19532 | 10432 | 20752 | 12064 |
| of which `compiler_builtins` | 10288 | 10334 | 7558 | 7674 |
| of which `nalgebra`, out of line | 15348 | 2056 | 4450 | 1454 |
| `.rodata` | 4383 | 4455 | 4527 | 4599 |

The column heads are target and `opt-level`. `compiler_builtins` is software floating point on
`thumbv6m`; on `thumbv7em` it is the `f64` arithmetic a single-precision FPU lacks, beside
`memcpy`, 64-bit division and `fmodf` on both. Most of what `nalgebra` costs is not in its row:
its generics are inlined into the filter's functions and counted there.
