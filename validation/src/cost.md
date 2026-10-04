# Cost on a target

**What does the filter cost a microcontroller?** The whole filter, `Eskf`, is
{{footprint thumbv6m-none-eabi size.eskf.Eskf}} bytes on a Cortex-M0 and allocates nothing
else, so that is the RAM to plan for beyond the stack. The deepest stack is a GNSS velocity
update, {{footprint thumbv6m-none-eabi stack_peak}} bytes, and linking every entry point takes
{{footprint thumbv6m-none-eabi text.s}} bytes of flash at `opt-level = "s"`. Execution time on
hardware is not measured yet (#41).

Every figure on this page is pinned exactly in `data/footprint.txt`, which CI measures on
`{{footprint toolchain}}` with `tools/footprint.sh` and fails on any move, growth or shrinkage.
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
| `Eskf`, the filter | {{footprint thumbv6m-none-eabi size.eskf.Eskf}} | {{footprint thumbv7em-none-eabihf size.eskf.Eskf}} |
| of which the state history of (23′) | {{footprint thumbv6m-none-eabi size.history.History}} | {{footprint thumbv7em-none-eabihf size.history.History}} |
| of which the covariance `P` | {{footprint thumbv6m-none-eabi size.state.Covariance}} | {{footprint thumbv7em-none-eabihf size.state.Covariance}} |
| of which `Diagnostics` | {{footprint thumbv6m-none-eabi size.health.Diagnostics}} | {{footprint thumbv7em-none-eabihf size.health.Diagnostics}} |
| of which the yaw estimator of (45)–(52) | {{footprint thumbv6m-none-eabi size.gsf.YawEstimator}} | {{footprint thumbv7em-none-eabihf size.gsf.YawEstimator}} |
| of which the barometric offset of (30′) | {{footprint thumbv6m-none-eabi size.state.Offset}} | {{footprint thumbv7em-none-eabihf size.state.Offset}} |
| `State`, the estimate `state()` returns | {{footprint thumbv6m-none-eabi size.state.State}} | {{footprint thumbv7em-none-eabihf size.state.State}} |
| `Config` | {{footprint thumbv6m-none-eabi size.config.Config}} | {{footprint thumbv7em-none-eabihf size.config.Config}} |
| `StaticWindow`, at any rate and length | {{footprint thumbv6m-none-eabi size.init.StaticWindow}} | {{footprint thumbv7em-none-eabihf size.init.StaticWindow}} |
| `StaticSample` | {{footprint thumbv6m-none-eabi size.init.StaticSample}} | {{footprint thumbv7em-none-eabihf size.init.StaticSample}} |
| `Startup`, a start worked out and checked before it commits, on a start's stack | {{footprint thumbv6m-none-eabi size.eskf.start.Startup}} | {{footprint thumbv7em-none-eabihf size.eskf.start.Startup}} |

Sizes in bytes.

## Stack

The deepest stack a call into each entry point takes, in bytes, before the caller's frames.

| entry point | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `predict` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.predict}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.predict}} |
| `fuse_gnss_position` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_gnss_position}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_gnss_position}} |
| `fuse_gnss_geodetic` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_gnss_geodetic}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_gnss_geodetic}} |
| `fuse_gnss_velocity` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_gnss_velocity}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_gnss_velocity}} |
| `fuse_baro_altitude` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_baro_altitude}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_baro_altitude}} |
| `fuse_mag_heading` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_mag_heading}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_mag_heading}} |
| `fuse_gnss_heading` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_gnss_heading}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_gnss_heading}} |
| `fuse_course` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_course}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_course}} |
| `fuse_stationary` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.fuse_stationary}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.fuse_stationary}} |
| `predicted_validity` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.predicted_validity}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.predicted_validity}} |
| `initialize` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.initialize}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.initialize}} |
| `initialize_coarse` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.initialize_coarse}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.initialize_coarse}} |
| `initialize_from` | {{footprint thumbv6m-none-eabi chain.eskf.Eskf.initialize_from}} | {{footprint thumbv7em-none-eabihf chain.eskf.Eskf.initialize_from}} |
| **the deepest of them** | **{{footprint thumbv6m-none-eabi stack_peak}}** | **{{footprint thumbv7em-none-eabihf stack_peak}}** |

The deepest path is `fuse_gnss_velocity` calling the three-measurement update, which calls
`nalgebra`'s 15 × 15 matrix product, which calls the soft-float multiply on `thumbv6m` and
`memcpy` on `thumbv7em`. `tools/footprint.py --path` prints any entry point's path, frame by frame.

Each entry point's own frame, and the frames beneath it that set its depth:

| function | `thumbv6m` | `thumbv7em` |
| --- | --- | --- |
| `predict` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.predict}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.predict}} |
| `propagate_or_coast`, the step it calls | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.propagate_or_coast}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.propagate_or_coast}} |
| `hold_if_unaided`, the hold it calls beside the step | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.hold_if_unaided}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.hold_if_unaided}} |
| `YawEstimator::predict`, the yaw estimator it steps beside both | {{footprint thumbv6m-none-eabi frame.gsf.YawEstimator.predict}} | {{footprint thumbv7em-none-eabihf frame.gsf.YawEstimator.predict}} |
| `fuse_gnss_position` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_gnss_position}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_gnss_position}} |
| `fuse_gnss_geodetic` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_gnss_geodetic}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_gnss_geodetic}} |
| `fuse_gnss_velocity` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_gnss_velocity}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_gnss_velocity}} |
| `fuse_baro_altitude` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_baro_altitude}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_baro_altitude}} |
| `fuse_mag_heading` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_mag_heading}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_mag_heading}} |
| `fuse_gnss_heading` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_gnss_heading}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_gnss_heading}} |
| `fuse_course` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_course}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_course}} |
| `fuse_stationary` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_stationary}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_stationary}} |
| `predicted_validity` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.predicted_validity}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.predicted_validity}} |
| `initialize` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.initialize}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.initialize}} |
| `initialize_coarse` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.initialize_coarse}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.initialize_coarse}} |
| `initialize_from` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.initialize_from}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.initialize_from}} |
| beneath the three heading entry points: the heading update they share | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.fuse_heading}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.fuse_heading}} |
| beneath `fuse_gnss_velocity`: the update of (23)–(27), three measurements | {{footprint thumbv6m-none-eabi frame.update.update.3}} | {{footprint thumbv7em-none-eabihf frame.update.update.3}} |
| beneath `fuse_gnss_position` (and so a geodetic fix): two, the horizontal pair | {{footprint thumbv6m-none-eabi frame.update.update.2}} | {{footprint thumbv7em-none-eabihf frame.update.update.2}} |
| beneath `fuse_gnss_position`'s height, `fuse_baro_altitude` and the heading update: one | {{footprint thumbv6m-none-eabi frame.update.update.1}} | {{footprint thumbv7em-none-eabihf frame.update.update.1}} |
| beside the update: the observation formed at the measurement's time, (23′) | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.observe.3}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.observe.3}} |
| beside the update: committing its result, or handing it to an adoption | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.apply_or_recover}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.apply_or_recover}} |
| beside the update: adopting a position, from `fuse_gnss_position` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.adopt_position.3}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.adopt_position.3}} |
| beside the update: adopting a velocity, from `fuse_gnss_velocity` | {{footprint thumbv6m-none-eabi frame.eskf.Eskf.adopt_velocity}} | {{footprint thumbv7em-none-eabihf frame.eskf.Eskf.adopt_velocity}} |
| beneath the update: the attitude reset of (41) | {{footprint thumbv6m-none-eabi frame.update.reparameterize}} | {{footprint thumbv7em-none-eabihf frame.update.reparameterize}} |
| beneath the update: the injection of (39)–(40) | {{footprint thumbv6m-none-eabi frame.update.inject}} | {{footprint thumbv7em-none-eabihf frame.update.inject}} |
| beneath `predict`: one sample, (9)–(22) | {{footprint thumbv6m-none-eabi frame.propagate.propagate}} | {{footprint thumbv7em-none-eabihf frame.propagate.propagate}} |
| beneath that: the covariance step of (22) | {{footprint thumbv6m-none-eabi frame.propagate.propagate_covariance}} | {{footprint thumbv7em-none-eabihf frame.propagate.propagate_covariance}} |
| beneath that and the reset of (41): symmetry, (42) | {{footprint thumbv6m-none-eabi frame.math.enforce_symmetry.15}} | {{footprint thumbv7em-none-eabihf frame.math.enforce_symmetry.15}} |
| beneath `predict`, across a gap: the coast of (22′) | {{footprint thumbv6m-none-eabi frame.propagate.coast}} | {{footprint thumbv7em-none-eabihf frame.propagate.coast}} |
| beneath `predicted_validity`: the covariance carried over the horizon | {{footprint thumbv6m-none-eabi frame.propagate.project}} | {{footprint thumbv7em-none-eabihf frame.propagate.project}} |
| beneath `initialize` and `initialize_coarse`: the initial covariance | {{footprint thumbv6m-none-eabi frame.init.initial_covariance}} | {{footprint thumbv7em-none-eabihf frame.init.initial_covariance}} |

## Flash

| bytes | `thumbv6m` `3` | `thumbv6m` `s` | `thumbv7em` `3` | `thumbv7em` `s` |
| --- | --- | --- | --- | --- |
| `.text` | {{footprint thumbv6m-none-eabi text.3}} | {{footprint thumbv6m-none-eabi text.s}} | {{footprint thumbv7em-none-eabihf text.3}} | {{footprint thumbv7em-none-eabihf text.s}} |
| of which `libm` | {{footprint thumbv6m-none-eabi text_libm.3}} | {{footprint thumbv6m-none-eabi text_libm.s}} | {{footprint thumbv7em-none-eabihf text_libm.3}} | {{footprint thumbv7em-none-eabihf text_libm.s}} |
| of which `compiler_builtins` | {{footprint thumbv6m-none-eabi text_compiler_builtins.3}} | {{footprint thumbv6m-none-eabi text_compiler_builtins.s}} | {{footprint thumbv7em-none-eabihf text_compiler_builtins.3}} | {{footprint thumbv7em-none-eabihf text_compiler_builtins.s}} |
| of which `nalgebra`, out of line | {{footprint thumbv6m-none-eabi text_nalgebra.3}} | {{footprint thumbv6m-none-eabi text_nalgebra.s}} | {{footprint thumbv7em-none-eabihf text_nalgebra.3}} | {{footprint thumbv7em-none-eabihf text_nalgebra.s}} |
| `.rodata` | {{footprint thumbv6m-none-eabi rodata.3}} | {{footprint thumbv6m-none-eabi rodata.s}} | {{footprint thumbv7em-none-eabihf rodata.3}} | {{footprint thumbv7em-none-eabihf rodata.s}} |

The column heads are target and `opt-level`. `compiler_builtins` is software floating point on
`thumbv6m`; on `thumbv7em` it is the `f64` arithmetic a single-precision FPU lacks, beside
`memcpy`, 64-bit division and `fmodf` on both. Most of what `nalgebra` costs is not in its row:
its generics are inlined into the filter's functions and counted there.
