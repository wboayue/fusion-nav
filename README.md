# fusion-nav

Embedded-first inertial navigation using a 15-state Error-State Kalman Filter (ESKF).

`fusion-nav` estimates 3D attitude, velocity, and position by fusing IMU measurements with GNSS,
barometric altitude, and magnetometer observations. It is `no_std`, allocation-free, and aimed at
flight controllers, UAVs, and other embedded navigation.

This page is the overview and a quick start.
[GUIDE.md](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md) is how to use the filter in
detail, and the [documents](#documents) below answer the rest.

![The simulator's gnss_outage flight: position error against truth stays inside the filter's own 3-sigma band, which widens while GNSS is lost and closes at the first fix](https://raw.githubusercontent.com/wboayue/fusion-nav/main/validation/figures/gnss_outage/error_position.png)

*A simulated flight that loses GNSS for 20 s. The line is the position error against truth, the
gray band the filter's own ±3σ, the shading its health, `Status` (see
[health reporting](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#health-reporting)): yellow
`Degraded`, red `DeadReckoning`. The band widens as the error grows, and the first fix puts the
error back inside a band that then narrows.
[VALIDATION.md](https://github.com/wboayue/fusion-nav/blob/main/VALIDATION.md) has the rest,
against simulated truth, real UAV flights and PX4's EKF2.*

## Why this crate

* **Readable mathematics.** The code cites a numbered equation for every step it takes, in
  [EQUATIONS.md](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md), and a table maps
  each equation to the function implementing it. No generated code.
* **Frames in the types.** Passing an ENU vector where NED is expected is a compile error, and a
  PX4 or ArduPilot attitude seeds with no conversion. See
  [conventions](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#conventions).
* **Health travels with the estimate.** `state()` returns the solution with its `Status` and a
  validity flag per quantity, and every call returns a typed outcome: accepted, rejected,
  adopted, or refused with a reason.
* **Pure Rust, one dependency** (`nalgebra`). `no_std`, allocation-free, no C++ toolchain, and no
  reachable panic, which CI
  [checks](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#the-library-cannot-panic) by
  linking the whole API for two Cortex-M targets.
* **Validation you can rerun.** Seeded simulations scored against truth, and the covariance's
  honesty tested on 50 seeds, gate CI with no hardware. Real PX4 logs replay beside EKF2, and one
  script regenerates every published figure.
* **Configuration derived, not demanded.** `replay --derive` prints a `Config` from your own log.
  The accuracy your mission needs is the one thing you must supply.

[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md) says what each claim rests on
and what would falsify it.

## Quick start

```rust,no_run
use fusion_nav::prelude::*;
# let (next_still_sample, imu) = (StaticSample::default, ImuSample::default());
# let (lat_e7, lon_e7, height_mm, eph, epv) = (473_977_420, 85_455_940, 488_000, 1.5, 3.0);
# let arrived = Timestamp::from_micros(12_500_000);

let mut filter = Eskf::default();

// Start from samples taken while the vehicle sits still, folded in as they arrive.
let mut window = StaticWindow::new();
while !window.is_long_enough(filter.config()) {
    if let Err(_refusal) = window.push(next_still_sample()) { /* dropped; log it */ }
}
if let Alignment::Coarse(_) = filter.initialize(&window)? {
    /* a short or moving window: running, and `Status::Aligning` until attitude converges */
}

loop {
    // Every IMU sample, on the caller's clock.
    if !filter.predict(imu).is_propagated() { /* log the gap */ }

    // Every measurement, at the time it was taken, with its own noise. The first fix
    // places the navigation origin, so fuse only a real one.
    let fix = Geodetic::from_degrees_e7(lat_e7, lon_e7, height_mm);
    let noise = PositionNoise::horizontal_vertical(eph, epv);
    let taken = arrived.before(Seconds::from_secs(0.110)); // arrival, less the receiver's latency
    let antenna = Position::zero(); // the antenna's offset from the IMU, in body axes
    if !filter.fuse_gnss_geodetic(taken, fix, noise, antenna).is_accepted() {
        /* diagnostics() has the detail */
    }

    // The estimate carries its own health.
    let s = filter.state();
    if s.validity.horizontal_position { /* use s.position */ }
}
# Ok::<(), InitError>(())
```

[GUIDE.md](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md) covers each step in full:
[initialization](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#initialization) and its
other entry points, every [measurement](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#measurements)
and its noise, the antenna offset and latency, and what every outcome means.

`fusion_nav::prelude` carries the whole integration surface. Four examples show it in full: where
the time comes from, sources at their own rates, every outcome handled and logged where it is
returned. `embedded` is the `no_std` one, with no `println!`.

```console
$ cargo run --example basic         # the integration loop on its own
$ cargo run --example degradation   # dropouts, diagnostics, an application-driven reset
$ cargo build --example embedded --target thumbv7em-none-eabihf   # the loop on a microcontroller
$ cargo run --example replay        # a recorded flight in, the estimate out, as CSV
```

`replay` also runs real PX4 logs, and `cargo run --example simulate` generates seeded flights with
analytic ground truth. Hand `replay` that truth as a third argument and it scores itself: RMSE,
NEES, and how often it called an estimate usable while the error said otherwise.

```console
$ cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv
```

See [data/README.md](https://github.com/wboayue/fusion-nav/blob/main/data/README.md).

## How it fits together

```text
  every IMU sample                          every measurement, at its own time
  ────────────────                          ──────────────────────────────────
  predict(imu) ──► Propagation              fuse_*(time, z, noise)
       │           Propagated · Coasted ·        │ refused ──► Fusion::NotFinite, …
       │           refused                       ▼
       ▼                                    gate: does z agree with the estimate then?
  nominal state + covariance  ◄── correct ─ accept ──► Fusion::Accepted
       │                                    reject ──► Fusion::Rejected
       │                      ◄── adopt ─── └─ locked out too long ──► Fusion::Reset
       ▼
  state() ──► position, velocity, attitude, biases
              + Status + validity per quantity
```

Every call returns an outcome, and every output carries its health.
[DESIGN.md](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#architecture) draws the same
two paths with the module that owns each step.

## Why an ESKF?

No single sensor gives you navigation. An IMU is fast and self-contained, but integrating it
drifts without bound. GNSS is absolute but slow, noisy, and sometimes absent. A barometer gives
height only; a magnetometer gives heading only and is easily disturbed.

The errors are also coupled, and each step integrates once more. Followed from a constant bias:

```text
  gyroscope bias ──► attitude error ──► velocity error ──► position error
     constant           grows ∝ t         grows ∝ t²          grows ∝ t³
                            │                 ▲
                            └─ gravity tipped ┘
                               into the wrong axis

  accelerometer bias ─────────────────► velocity error ──► position error
     constant                             grows ∝ t           grows ∝ t²
```

A Kalman filter that carries all of these quantities together models that coupling in its
covariance, so a GNSS position fix corrects not only position but the attitude and IMU biases
that made it drift. The error-state form keeps attitude as a quaternion and estimates only a small
3-D correction to it, rather than treating the quaternion's four components as independent. See
[DESIGN.md](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#error-state-kalman-filter).

### When you do not need one

`fusion-nav` complements the lighter Fusion filters: `fusion-ahrs` estimates attitude alone, and
`fusion-altitude` adds altitude and vertical velocity to it.

* **attitude only:** `fusion-ahrs`, substantially simpler and cheaper.
* **attitude, altitude, vertical velocity** (stabilization, altitude hold): `fusion-ahrs` with
  `fusion-altitude`.
* **3D position or velocity:** `fusion-nav`. It owns its attitude rather than consuming one from
  `fusion-ahrs`, because attitude uncertainty is coupled to velocity and position uncertainty.

[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md) compares `fusion-nav` against
other Rust crates and PX4 / ArduPilot.

## Health at a glance

A filter that rejects every measurement because it is itself wrong dead-reckons while looking
confident, so `state()` returns the solution together with its health. Three questions, three
answers:

| question | answer | read it |
| --- | --- | --- |
| how bad is the worst thing? | `state().status` | for a mode change, a failsafe, a log line |
| which outputs can I use now? | `state().validity`, one flag per quantity | before using a quantity in control |
| will they still be good if I take off now? | `predicted_validity()` | in an arming check |

`Status` reads, most severe first, `DeadReckoning`, `Aligning`, `Degraded`, `Healthy`. A source
locked out by the gate for too long is recovered by adopting its next measurement. See
[health reporting](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#health-reporting) and
[recovery](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#recovery-from-gate-lockout).

## Limitations

Known, and stated rather than discovered in flight.
[GUIDE.md](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#limitations) says what each
costs and links the issue that removes it.

* A measurement's latency is the caller's to know.
* GNSS height carries the low frequencies, so a barometer steadier than PX4's drift rate is
  trusted less than it could be.
* No magnetic-field states: calibrate the magnetometer for hard and soft iron first.
* With no heading sensor, heading waits for the vehicle to accelerate, and a wrong one is believed
  until it does.
* In-motion alignment is coarse.
* Correlation times are configured, not measured.
* An IMU gap is coasted on an assumption of no acceleration or turn.
* The position hold assumes the vehicle stays put: a car or fixed-wing sets `Config::hold = None`.
* Position is a local tangent plane, so far from the origin `-p_D` is not height.

Features out of scope (wind, terrain, optical flow, airspeed, ...) are listed in
[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#non-goals).

## Coming from PX4 or ArduPilot

A tuned EKF2 or EKF3 does not carry across by renaming: measurement noise is a per-call argument,
gates are percentiles, and IMU noise is a density.
[MIGRATING.md](https://github.com/wboayue/fusion-nav/blob/main/MIGRATING.md) maps each parameter
onto what this crate takes.

## Documents

| question | document |
| --- | --- |
| how do I use it, step by step? | [GUIDE.md](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md): conventions, initialization, measurements, health, recovery, limitations |
| how do my PX4 or ArduPilot parameters map? | [MIGRATING.md](https://github.com/wboayue/fusion-nav/blob/main/MIGRATING.md): what is not a rename, and what is left out |
| why does this crate exist, and what did it decide? | [GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md): positioning, differentiators, decisions, non-goals |
| how is it built, and where do its numbers come from? | [DESIGN.md](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md): architecture, modules, defaults and their evidence, measured cost |
| what does it compute? | [EQUATIONS.md](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md): the mathematics, numbered, with an equation-to-code map |
| what does a word mean here? | [GLOSSARY.md](https://github.com/wboayue/fusion-nav/blob/main/GLOSSARY.md): innovation, NEES, bias, specific force, and what PX4 and ArduPilot call the same things |
| how good is it? | [VALIDATION.md](https://github.com/wboayue/fusion-nav/blob/main/VALIDATION.md): accuracy, honesty, robustness and cost, regenerated from the runs |
| how is it tested on logs? | [data/README.md](https://github.com/wboayue/fusion-nav/blob/main/data/README.md): the replay harness and the PX4 log corpus |

## License

Licensed under the MIT License.
