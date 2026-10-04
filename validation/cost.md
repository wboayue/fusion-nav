<!-- Generated from validation/src/cost.md by tools/validation.sh; edit that, not this. -->
# Cost on a target

**What does the filter cost a microcontroller?** The whole filter, `Eskf`, is
4688 bytes on a Cortex-M0 and allocates nothing
else, so that is the RAM to plan for beyond the stack. The deepest stack is a GNSS velocity
update, 11580 bytes, and linking every entry point takes
127526 bytes of flash at `opt-level = "s"`. On a 400 MHz
Cortex-M7, an IMU step takes at most 278.7 µs and a GNSS velocity update
352.0 µs, against the 2500 µs of a 400 Hz loop ([Time on a target](#time-on-a-target)).

Every figure on this page but the board's is pinned exactly in `data/footprint.txt`, which CI
measures on `nightly-2026-08-06` with `tools/footprint.sh` and fails on any move, growth or
shrinkage. The board's are pinned in `data/onboard.txt`, measured at the commit their build line
names; CI has no board, so it checks only that this page shows them.

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
- **Time and painted stack** are measured on a board, below: every call a replay makes, timed and
  its stack painted. A painted stack is a lower bound, the deepest the calls made reached; the
  walk above is an upper bound, every path the code has. No M0 board was timed.

Why each function has the form it has, and what the forms not taken would have cost, is in
[DESIGN.md, "Measured cost, by function"](../DESIGN.md#measured-cost-by-function), together with
host timings, which are taken on one machine and pinned nowhere.

## Time on a target

An ARK FPV flight controller, an STM32H743: a Cortex-M7 at 400 MHz, instruction and data caches
on, the filter and the stack in DTCM. Built `rustc_1.99.0-nightly_(7608eb7b0_2026-08-05)` at
`c0e4d26f83`, `opt-level = 3` with no LTO, the
profile the stack frames above are measured in, for the `thumbv7em-none-eabihf` target: a
single-precision FPU, `f64` in library calls, flush-to-zero off.

The calls are the replay's (`onboard/`, #41): every call the harness makes into the filter on
the thirteen corpus logs and `hover_outage`, written down on the host and made again on the
board, plus a trace built to reach the paths the corpus does not (`onboard/examples/paths.rs`).
Each call is timed alone by the cycle counter, interrupts masked, net of the
35 cycles the timing itself takes, and the stack beneath it painted. After
every call the board checks its outcome and a digest of the state and covariance against the
host's, and refuses the run on any difference: every figure is of the arithmetic the host ran,
bit for bit.

| entry point | worst, µs | mean, µs | worst cold, µs | calls | denormal inputs | painted stack | walked stack |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `predict` | 278.7 | 119.0 | 279.8 | 3447657 | 0 | 10588 | 10640 |
| `fuse_gnss_position` | 369.2 | 345.1 | 370.3 | 74123 | 0 | 10548 | 10600 |
| `fuse_gnss_geodetic` | 1148.0 | 305.3 | 1147.5 | 100 | 0 | 10876 | 10928 |
| `fuse_gnss_velocity` | 352.0 | 303.3 | 352.3 | 74222 | 0 | 11300 | 11352 |
| `fuse_baro_altitude` | 177.2 | 159.7 | 175.4 | 111222 | 0 | 9364 | 9416 |
| `fuse_mag_heading` | 180.4 | 160.8 | 179.1 | 87124 | 0 | 9452 | 9504 |
| `fuse_gnss_heading` | 190.1 | 169.8 | 189.7 | 629 | 0 | 9588 | 9640 |
| `fuse_course` | 166.5 | 151.7 | 167.2 | 19 | 0 | 9596 | 9648 |
| `fuse_stationary` | 175.6 | 174.8 | 175.2 | 10 | 0 | 11212 | 11264 |
| `predicted_validity` | 79.9 | 78.3 | 80.3 | 15 | 0 | 8436 | 8488 |
| `initialize` | 149.2 | 120.3 | 149.7 | 16 | 0 | 4960 | 5312 |
| `initialize_coarse` | 52.3 | 52.3 | 54.2 | 1 | 0 | 5696 | 6048 |
| `initialize_from` | 22.4 | 22.0 | 23.7 | 2 | 0 | 2844 | 2896 |
| `StaticWindow::push` | 28.3 | 13.0 | 31.7 | 9673 | 0 | 548 | — |

The worst is over every call on every trace. *Cold* invalidates both caches before each call; it
moves the worst cases little, and the longer the call the less, since each refills the caches once
and then runs from them. The painted stack is beneath the call into the entry point, and the walked is
`chain.` above: every entry point's painted stack is under its walk.

The worst paths, reached on purpose by the `paths` trace:

| path | worst, µs | calls |
| --- | --- | --- |
| a 6.4 s coast, then a hold on the same step | 265.5 | 1 |
| a 10 s coast, then a hold | 267.1 | 1 |
| an unaided step and a hold | 258.8 | 10 |
| every source at the oldest age the history holds | 376.6 | 6 |
| a position 100 m out, rejected until adopted | 350.1 | 3280 |
| a velocity 30 m/s out, rejected until adopted | 352.1 | 3280 |
| a magnetic heading half a circle out | 259.6 | 3780 |
| the first geodetic fix, which places the origin | 1148.0 | 1 |
| `predicted_validity` over a 6.4 s horizon | 79.9 | 1 |
| a heading of 3e38 rad | 13.0 | 1 |

A coast is (22′) in one exact step, so a gap costs what a step does at any length. `wrap_pi`'s
`fmodf` takes at most 242 cycles on the angles the filter forms itself, inside
(−4π, 4π), and at most 665 on any `f32` a caller hands it
([DESIGN.md, "Execution time bounded by constants"](../DESIGN.md#execution-time-bounded-by-constants)).

Two other builds, on the traces all three ran (`4b473e91`, `a299e722`, `cd7e0001`, `093e806a`
and `paths`), worst case in µs:

| entry point | `primary` | `opt-level = "s"`, fat LTO | `-C target-cpu=cortex-m7` |
| --- | --- | --- | --- |
| `predict` | 267.1 | 569.6 | 247.5 |
| `fuse_gnss_position` | 366.7 | 704.8 | 316.2 |
| `fuse_gnss_velocity` | 350.2 | 608.0 | 219.8 |
| `fuse_mag_heading` | 179.8 | 324.3 | 154.2 |
| `fuse_gnss_geodetic` | 1148.0 | 1150.3 | 314.6 |
| `StaticWindow::push` | 28.3 | 30.1 | 4.4 |

`opt-level = "s"` with fat LTO runs the filter's own arithmetic at about half the speed, and inlines the entry points into the caller:
the deepest stack beneath its dispatcher is 14972 bytes. `cortex-m7` turns
on the M7's double-precision FPU, which the `f64` of a start's window and of a geodetic fix use.

## Memory

| type | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `Eskf`, the filter | 4688 | 4688 |
| of which the state history of (23′) | 1544 | 1544 |
| of which the covariance `P` | 900 | 900 |
| of which `Diagnostics` | 1160 | 1160 |
| of which the yaw estimator of (45)–(52) | 456 | 456 |
| of which the barometric offset of (30′) | 64 | 64 |
| `State`, the estimate `state()` returns | 72 | 72 |
| `Config` | 268 | 268 |
| `StaticWindow`, at any rate and length | 912 | 912 |
| `StaticSample` | 80 | 80 |
| `Startup`, a start worked out and checked before it commits, on a start's stack | 1088 | 1088 |

Sizes in bytes.

## Stack

The deepest stack a call into each entry point takes, in bytes, before the caller's frames.

| entry point | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `predict` | 10812 | 10640 |
| `fuse_gnss_position` | 10812 | 10600 |
| `fuse_gnss_geodetic` | 11148 | 10928 |
| `fuse_gnss_velocity` | 11580 | 11352 |
| `fuse_baro_altitude` | 9652 | 9416 |
| `fuse_mag_heading` | 9756 | 9504 |
| `fuse_gnss_heading` | 9892 | 9640 |
| `fuse_course` | 9908 | 9648 |
| `fuse_stationary` | 11484 | 11264 |
| `predicted_validity` | 8868 | 8488 |
| `initialize` | 5476 | 5312 |
| `initialize_coarse` | 6212 | 6048 |
| `initialize_from` | 3024 | 2896 |
| **the deepest of them** | **11580** | **11352** |

The deepest path is `fuse_gnss_velocity` calling the three-measurement update, which calls
`nalgebra`'s 15 × 15 matrix product, which calls the soft-float multiply on `thumbv6m` and
`memcpy` on `thumbv7em`. `tools/footprint.py --path` prints any entry point's path, frame by frame.

Each entry point's own frame, and the frames beneath it that set its depth:

| function | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `predict` | 80 | 72 |
| `propagate_or_coast`, the step it calls | 2168 | 2160 |
| `hold_if_unaided`, the hold it calls beside the step | 1304 | 1304 |
| `YawEstimator::predict`, the yaw estimator it steps beside both | 256 | 152 |
| `fuse_gnss_position` | 1384 | 1336 |
| `fuse_gnss_geodetic` | 336 | 328 |
| `fuse_gnss_velocity` | 1456 | 1432 |
| `fuse_baro_altitude` | 1264 | 1256 |
| `fuse_mag_heading` | 120 | 104 |
| `fuse_gnss_heading` | 248 | 248 |
| `fuse_course` | 256 | 256 |
| `fuse_stationary` | 1360 | 1344 |
| `predicted_validity` | 2784 | 2744 |
| `initialize` | 2224 | 2216 |
| `initialize_coarse` | 3504 | 3488 |
| `initialize_from` | 1928 | 1856 |
| beneath the three heading entry points: the heading update they share | 1264 | 1240 |
| beneath `fuse_gnss_velocity`: the update of (23)–(27), three measurements | 8120 | 8000 |
| beneath `fuse_gnss_position` (and so a geodetic fix): two, the horizontal pair | 7424 | 7344 |
| beneath `fuse_gnss_position`'s height, `fuse_baro_altitude` and the heading update: one | 6384 | 6240 |
| beside the update: the observation formed at the measurement's time, (23′) | 1464 | 1440 |
| beside the update: committing its result, or handing it to an adoption | 1112 | 1064 |
| beside the update: adopting a position, from `fuse_gnss_position` | 1976 | 1992 |
| beside the update: adopting a velocity, from `fuse_gnss_velocity` | 1904 | 1896 |
| beneath the update: the attitude reset of (41) | 456 | 448 |
| beneath the update: the injection of (39)–(40) | 168 | 88 |
| beneath `predict`: one sample, (9)–(22) | 1648 | 1440 |
| beneath that: the covariance step of (22) | 2832 | 2760 |
| beneath that and the reset of (41): symmetry, (42) | 128 | 8 |
| beneath `predict`, across a gap: the coast of (22′) | 1960 | 1920 |
| beneath `predicted_validity`: the covariance carried over the horizon | 1864 | 1840 |
| beneath `initialize` and `initialize_coarse`: the initial covariance | 1120 | 1080 |

## Flash

| bytes | `thumbv6m` `3` | `thumbv6m` `s` | `thumbv7em` `3` | `thumbv7em` `s` |
| --- | --- | --- | --- | --- |
| `.text` | 188334 | 127526 | 208020 | 134108 |
| of which `libm` | 19532 | 11196 | 20752 | 12620 |
| of which `compiler_builtins` | 10334 | 11630 | 7674 | 9096 |
| of which `nalgebra`, out of line | 15486 | 2130 | 3956 | 924 |
| `.rodata` | 4455 | 4535 | 4599 | 4679 |

The column heads are target and `opt-level`. `compiler_builtins` is software floating point on
`thumbv6m`; on `thumbv7em` it is the `f64` arithmetic a single-precision FPU lacks, beside
`memcpy`, 64-bit division and `fmodf` on both. Most of what `nalgebra` costs is not in its row:
its generics are inlined into the filter's functions and counted there.
