<!-- Generated from validation/src/cost.md by tools/validation.sh; edit that, not this. -->
# Cost on a target

**What does the filter cost a microcontroller?** The whole filter, `Eskf`, is
4112 bytes on a Cortex-M0 and allocates nothing
else, so that is the RAM to plan for beyond the stack. The deepest stack is a GNSS velocity
update, 11540 bytes, and linking every entry point takes
112082 bytes of flash at `opt-level = "s"`. Execution time on
hardware is not measured yet (#41).

Every figure on this page is pinned exactly in `data/footprint.txt`, which CI measures on
`nightly-2026-08-06` with `tools/footprint.sh` and fails on any move, growth or shrinkage.
CI also checks that this page shows the pinned values.

## What is measured, and what is not

The two targets are `thumbv6m-none-eabi` (Cortex-M0 and M0+, no FPU, so every float operation
is a library call) and `thumbv7em-none-eabihf` (Cortex-M4 and M7 with a single-precision FPU).

- **Sizes** are `size_of` each type on the target, at any `opt-level`.
- **Stack** is each function's own frame at `opt-level = 3`, read from the compiler
  (`-Zemit-stack-sizes`), and the deepest path beneath each entry point: its frame plus its
  deepest callee's, walked call by call through the library's relocations, down into `nalgebra`,
  `libm`, `core` and `compiler_builtins`. Every frame on a path is the compiler's, except
  `compiler_builtins`' hand-written division routines, which are read from their pushes. A
  call through a function pointer counts as deep as the deepest function whose address the
  entry point's code takes, directly or through the data it reads. For a generic function the
  figure is its largest instance.
- **Not in the stack figures:** the caller's own frames, and the 32 bytes a Cortex-M stacks on
  an exception (104 with the M4F's floating-point context). The frames are the library build's,
  and an application built with LTO can inline them differently.
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
| `Eskf`, the filter | 4112 | 4112 |
| of which the state history of (23′) | 1544 | 1544 |
| of which the covariance `P` | 900 | 900 |
| of which `Diagnostics` | 1048 | 1048 |
| of which the barometric offset of (30′) | 64 | 64 |
| `State`, the estimate `state()` returns | 72 | 72 |
| `Config` | 260 | 260 |
| `StaticWindow`, at any rate and length | 912 | 912 |
| `StaticSample` | 80 | 80 |
| `Startup`, a start worked out and checked before it commits, on a start's stack | 1088 | 1088 |

Sizes in bytes.

## Stack

The deepest stack a call into each entry point takes, in bytes, before the caller's frames.

| entry point | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `predict` | 10748 | 10584 |
| `fuse_gnss_position` | 10804 | 10600 |
| `fuse_gnss_geodetic` | 11140 | 10928 |
| `fuse_gnss_velocity` | 11540 | 11328 |
| `fuse_baro_altitude` | 9652 | 9416 |
| `fuse_mag_heading` | 9732 | 9496 |
| `fuse_gnss_heading` | 9852 | 9640 |
| `fuse_course` | 9868 | 9648 |
| `fuse_stationary` | 11484 | 11264 |
| `predicted_validity` | 8652 | 8448 |
| `initialize` | 5484 | 5312 |
| `initialize_coarse` | 6220 | 6048 |
| `initialize_from` | 3016 | 2896 |
| **the deepest of them** | **11540** | **11328** |

The deepest path is `fuse_gnss_velocity` calling the three-measurement update, which calls
`nalgebra`'s 15 × 15 matrix product, which calls the soft-float multiply on `thumbv6m` and
`memcpy` on `thumbv7em`. `tools/footprint.py --path` prints any entry point's path, frame by frame.

Each entry point's own frame, and the frames beneath it that set its depth:

| function | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `predict` | 16 | 16 |
| `propagate_or_coast`, the step it calls | 2176 | 2160 |
| `hold_if_unaided`, the hold it calls beside the step | 1304 | 1304 |
| `fuse_gnss_position` | 1376 | 1336 |
| `fuse_gnss_geodetic` | 336 | 328 |
| `fuse_gnss_velocity` | 1416 | 1408 |
| `fuse_baro_altitude` | 1264 | 1256 |
| `fuse_mag_heading` | 112 | 104 |
| `fuse_gnss_heading` | 248 | 256 |
| `fuse_course` | 256 | 264 |
| `fuse_stationary` | 1360 | 1344 |
| `predicted_validity` | 1864 | 1832 |
| `initialize` | 2224 | 2216 |
| `initialize_coarse` | 3504 | 3488 |
| `initialize_from` | 1928 | 1856 |
| beneath the three heading entry points: the heading update they share | 1232 | 1232 |
| beneath `fuse_gnss_velocity`: the update of (23)–(27), three measurements | 8120 | 8000 |
| beneath `fuse_gnss_position` (and so a geodetic fix): two, the horizontal pair | 7424 | 7344 |
| beneath `fuse_gnss_position`'s height, `fuse_baro_altitude` and the heading update: one | 6384 | 6240 |
| beside the update: the observation formed at the measurement's time, (23′) | 1464 | 1440 |
| beside the update: committing its result, or handing it to an adoption | 1120 | 1072 |
| beside the update: adopting a position, from `fuse_gnss_position` | 1984 | 1992 |
| beside the update: adopting a velocity, from `fuse_gnss_velocity` | 1904 | 1904 |
| beneath the update: the attitude reset of (41) | 456 | 448 |
| beneath the update: the injection of (39)–(40) | 168 | 88 |
| beneath `predict`: one sample, (9)–(22) | 1088 | 1096 |
| beneath that: the covariance step of (22) | 2832 | 2760 |
| beneath that and the reset of (41): symmetry, (42) | 128 | 8 |
| beneath `predict`, across a gap: the coast of (22′) | 2152 | 2144 |
| beneath `predicted_validity`: the covariance carried over the horizon | 1952 | 1936 |
| beneath `initialize` and `initialize_coarse`: the initial covariance | 1120 | 1080 |

## Flash

| bytes | `thumbv6m` `3` | `thumbv6m` `s` | `thumbv7em` `3` | `thumbv7em` `s` |
| --- | --- | --- | --- | --- |
| `.text` | 173070 | 112082 | 194732 | 118716 |
| of which `libm` | 19532 | 10432 | 20752 | 12620 |
| of which `compiler_builtins` | 10334 | 10334 | 7674 | 7674 |
| of which `nalgebra`, out of line | 15256 | 1900 | 3956 | 924 |
| `.rodata` | 4415 | 4487 | 4559 | 4631 |

The column heads are target and `opt-level`. `compiler_builtins` is software floating point on
`thumbv6m`; on `thumbv7em` it is the `f64` arithmetic a single-precision FPU lacks, beside
`memcpy`, 64-bit division and `fmodf` on both. Most of what `nalgebra` costs is not in its row:
its generics are inlined into the filter's functions and counted there.
