# fusion-nav

Embedded-first inertial navigation: attitude, velocity and position from an IMU, GNSS, a barometer
and a magnetometer.

`fusion-nav` is a 15-state error-state Kalman filter (ESKF) written in Rust. It is `no_std` and
allocation-free, and aimed at flight controllers, UAVs and other embedded navigation. New to the
vocabulary? [GLOSSARY.md](https://github.com/wboayue/fusion-nav/blob/main/GLOSSARY.md) defines it.

![The simulator's gnss_outage flight: position error against truth stays inside the filter's own 3-sigma band, which widens while GNSS is lost and closes at the first fix](https://raw.githubusercontent.com/wboayue/fusion-nav/main/validation/figures/gnss_outage/error_position.png)

*A simulated flight that loses GNSS for 20 s. The line is the position error against truth. The gray
band is ±3σ, the range the filter expects its own error to stay within: it widens while GNSS is
gone, and the first fix brings the error back inside it.
[VALIDATION.md](https://github.com/wboayue/fusion-nav/blob/main/VALIDATION.md) has the rest, against
simulated truth, real UAV flights and PX4's EKF2.*

## Is it for you?

* **3D position or velocity** from an IMU and GNSS: `fusion-nav`.
* **Attitude only:** `fusion-ahrs`, substantially simpler and cheaper.
* **Attitude, altitude and vertical velocity** (stabilization, altitude hold): `fusion-ahrs` with
  `fusion-altitude`.

`fusion-nav` owns its attitude rather than consuming one from `fusion-ahrs`, because attitude
uncertainty is coupled to velocity and position uncertainty.

Coming from PX4 or ArduPilot? A tuned EKF2 or EKF3 does not carry across by renaming:
[MIGRATING.md](https://github.com/wboayue/fusion-nav/blob/main/MIGRATING.md) maps each parameter
onto what this crate takes. [GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md)
compares `fusion-nav` against other Rust crates and both autopilots.

## Why this crate

* **Readable mathematics.** The code cites a numbered equation for every step it takes, in
  [EQUATIONS.md](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md), and a table maps
  each equation to the function implementing it. No generated code.
* **Frames in the types.** Passing an ENU vector where NED is expected is a compile error, and a PX4
  or ArduPilot attitude seeds with no conversion. See
  [conventions](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#conventions).
* **Health travels with the estimate.** `state()` returns the solution with its `Status` and a
  validity flag per quantity, and every call returns a typed outcome: accepted, rejected, reset to
  the measurement, or refused with a reason.
* **Pure Rust, one dependency** (`nalgebra`). `no_std`, allocation-free, no C++ toolchain, and no
  reachable panic, which
  [CI's panic check](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#the-library-cannot-panic)
  proves by linking the whole API for two Cortex-M targets.
* **Validation you can rerun.** CI scores seeded simulations against truth, and tests on 50 seeds
  whether the filter's error bars hold, with no hardware. Real PX4 logs replay beside EKF2, and one
  script regenerates every published figure.
* **Configuration derived, not demanded.** The `replay` example's `--derive` prints a `Config` from
  your own flight log. The accuracy your mission needs is the one thing you must supply.

[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md) says what each claim rests on
and what would falsify it.

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

Two paths. Every IMU sample is integrated forward by `predict`. Every other measurement is first
checked against the estimate, the gate: one that agrees corrects it, one that does not is rejected,
and a source rejected for too long is reset to its next measurement (adopted). Every call returns an
outcome, and every output carries its health.
[DESIGN.md](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#architecture) draws the same
two paths with the module that owns each step.

## Quick start

The crate is not on crates.io yet:

```toml
[dependencies]
fusion-nav = { git = "https://github.com/wboayue/fusion-nav" }
```

The default feature `magnetic-model` links a magnetic declination table (about 2.6 KB of flash).
`defmt` adds `defmt::Format` for logging on a microcontroller.

The filter starts from about two seconds of IMU samples with the vehicle at rest, which levels it
and measures the gyroscope bias. It then predicts on every IMU sample and fuses each measurement as
it arrives. The `read_*` and `poll_*` functions stand for your drivers.

```rust,no_run
use fusion_nav::prelude::*;
# struct Pvt { lat_e7: i32, lon_e7: i32, height_mm: i32, h_acc_mm: u32, v_acc_mm: u32, arrived: Timestamp }
# fn read_still_sample() -> StaticSample { StaticSample::default() }
# fn read_imu() -> ImuSample { ImuSample::default() }
# fn poll_gnss() -> Option<Pvt> { None }

let mut filter = Eskf::default();

// Start from samples taken while the vehicle sits still.
let mut window = StaticWindow::new();
while !window.is_long_enough(filter.config()) {
    if let Err(_refusal) = window.push(read_still_sample()) { /* dropped; log it */ }
}
if let Alignment::Coarse(_) = filter.initialize(&window)? {
    /* it moved, or the window was short: running, `Status::Aligning` until attitude converges */
}

// Where the GNSS antenna sits relative to the IMU: forward, right, down, in meters.
let antenna = Position::body(0.05, 0.0, -0.12);
// The receiver's latency, from its documentation or your logs.
let latency = Seconds::from_secs(0.110);

loop {
    // Every IMU sample.
    if !filter.predict(read_imu()).is_propagated() { /* log the gap */ }

    // Each GNSS fix, when one arrives. Fuse only a real fix: the first places the origin.
    if let Some(pvt) = poll_gnss() {
        let fix = Geodetic::from_degrees_e7(pvt.lat_e7, pvt.lon_e7, pvt.height_mm);
        // Bound the receiver's own accuracy figures, as PX4 and ArduPilot do.
        let (eph, epv) = (pvt.h_acc_mm as f32 * 1e-3, pvt.v_acc_mm as f32 * 1e-3);
        let (horizontal, vertical) = (SigmaBounds::new(0.5, 100.0), SigmaBounds::new(0.75, 100.0));
        let noise = PositionNoise::clamped(eph, epv, horizontal, vertical);
        let taken = pvt.arrived.before(latency);
        if !filter.fuse_gnss_geodetic(taken, fix, noise, antenna).is_accepted() {
            /* diagnostics() has the detail */
        }
    }

    // The estimate carries its own health.
    let s = filter.state();
    if s.validity.horizontal_position { /* use s.position */ }
}
# Ok::<(), InitError>(())
```

[GUIDE.md](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md) covers each step in full:
[initialization](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#initialization) and its
other entry points, every
[measurement](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#measurements) and its noise,
and what every outcome means.

Four examples run the whole integration surface, `fusion_nav::prelude`. `basic` is the loop above as
a program to copy, and `embedded` is the `no_std` one, with no `println!`.

```console
$ cargo run --example basic         # the integration loop on its own
$ cargo run --example degradation   # dropouts, diagnostics, an application-driven reset
$ cargo build --example embedded --target thumbv7em-none-eabihf   # the loop on a microcontroller
$ cargo run --example replay        # a recorded flight in, the estimate out, as CSV
```

`replay` also runs real PX4 logs, and `cargo run --example simulate` generates seeded flights with
known ground truth. Hand `replay` that truth as a third argument and it scores itself: the error,
whether the filter's error bars held, and how often it called an estimate usable while the error
said otherwise.

```console
$ cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv
```

[The replay harness and log corpus](https://github.com/wboayue/fusion-nav/blob/main/data/README.md)
explains the formats.

## Health at a glance

A filter that is itself wrong rejects correct measurements and drifts on the IMU alone while looking
confident. So every estimate carries its health, and three questions have three answers:

| question | answer | read it |
| --- | --- | --- |
| how bad is the worst thing? | `state().status` | for a mode change, a failsafe, a log line |
| which outputs can I use now? | `state().validity`, one flag per quantity | before using a quantity in control |
| will they still be good if I take off now? | `predicted_validity()` | in an arming check |

`Status` is, most severe first:

| `Status` | meaning |
| --- | --- |
| `DeadReckoning` | no GNSS position or velocity accepted for a while; horizontal position is unusable |
| `Aligning` | attitude has not converged yet |
| `Degraded` | a source has timed out |
| `Healthy` | everything fused is still accepted |

A source rejected for too long is reset to its next measurement, per source and on by default. See
[health reporting](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#health-reporting) and
[recovery](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#recovery-from-gate-lockout).

## Why an ESKF?

No single sensor gives you navigation. An IMU is fast and self-contained, but integrating it drifts
without bound. GNSS is absolute but slow, noisy, and sometimes absent. A barometer gives height
only; a magnetometer gives heading only and is easily disturbed.

The errors are also coupled. A small gyroscope bias tilts the attitude, the tilt puts gravity into
the wrong axis, and position error grows with the cube of time. A Kalman filter that carries all of
these together lets a GNSS fix correct the attitude and biases that caused the drift, not only the
position. The error-state form keeps attitude as a quaternion and estimates only a small correction
to it.
[DESIGN.md](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#error-state-kalman-filter)
draws the chain.

## Limitations

The main ones, stated up front.
[GUIDE.md](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#limitations) lists all of them,
says what each costs, and links the issue that would remove it.

* You supply each measurement's latency; the filter cannot measure it.
* Calibrate the magnetometer for hard and soft iron first. The filter cannot detect an uncalibrated
  one.
* Without a magnetometer or a second GNSS antenna, heading waits for the vehicle to accelerate, and
  a wrong magnetometer heading is believed until it does.
* A start while moving gives a rough attitude at first.
* The barometer's reference is allowed to drift at PX4's rate, so over the long term height follows
  GNSS, even when the barometer is steadier.
* Without GNSS the filter assumes the vehicle stays put. A car or a fixed-wing sets
  `Config::hold = None`.
* Position is on a flat plane about the starting point, so far from it the reported height parts
  from the true one: 8 cm at 1 km, 7.8 m at 10 km.

Features out of scope (wind, terrain, optical flow, airspeed, ...) are listed in
[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#non-goals).

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

Licensed under the [MIT License](https://github.com/wboayue/fusion-nav/blob/main/LICENSE).
