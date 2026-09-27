# fusion-nav

Embedded-first inertial navigation using a 15-state Error-State Kalman Filter (ESKF).

> **Status: complete and aided by GNSS position and velocity, the barometer and magnetic
> heading.** Equations (1)–(44) are implemented, bar two: the three-axis magnetometer of
> (31)–(33), which is
> [out of scope](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#magnetometer-without-magnetic-field-states)
> rather than pending, and (5′), whose in-motion levelling term is measured and reported but not
> yet subtracted. Initialization levels, takes a heading and a gyroscope bias, and sets
> its covariance; `predict` propagates the state *and* its uncertainty, (9)–(22); every
> `fuse_*` corrects both through the innovation gate of (37)–(38) — `fuse_gnss_position`,
> `fuse_gnss_geodetic`, `fuse_gnss_velocity` and `fuse_baro_altitude` by (23)–(30),
> `fuse_mag_heading` by (34)–(36). Yaw is the one attitude component any of them observes
> directly; roll and pitch are corrected as far as the covariance carries an observation into
> them. Where no aiding arrives, the uncertainty grows without bound and `Validity` says so:
> each flag goes false as its own variance passes `Config::accuracy`, 3.83 s in for tilt at
> the default noise on an unaided start. Accuracy is measured rather than asserted — see
> [VALIDATION.md](https://github.com/wboayue/fusion-nav/blob/main/VALIDATION.md) for the
> figures, against simulated truth and beside PX4's EKF2 on real flights, and
> [data/README.md](https://github.com/wboayue/fusion-nav/blob/main/data/README.md) for the
> harness. The API is not yet frozen.

`fusion-nav` estimates 3D attitude, velocity, and position by fusing IMU measurements with GNSS,
barometric altitude, and magnetometer observations. It is `no_std`, allocation-free, and aimed at
flight controllers, UAVs, and other embedded navigation.

```text
   IMU ──► state propagation ──┐
                               │      ┌──────────────────┐     attitude
  GNSS ──► position, velocity ─┤      │    fusion-nav    │     position NED
  Baro ──► altitude ───────────┼────► │  15-state ESKF   │ ──► velocity NED
   Mag ──► heading ────────────┘      └──────────────────┘     accelerometer bias
                                                               gyroscope bias
```

## Why an ESKF?

No single sensor gives you navigation. An IMU is fast and self-contained but integrating it drifts
without bound: a small gyroscope bias becomes an attitude error, and an accelerometer bias becomes
velocity error that grows linearly and position error that grows quadratically. GNSS is absolute
but slow, noisy, and sometimes absent. A barometer gives height only; a magnetometer gives heading
only and is easily disturbed.

The errors are also coupled. A small attitude error projects gravity into the wrong axis, and
that becomes acceleration error, then velocity error, then position error.

A Kalman filter that carries all of these quantities together models that coupling through its
covariance, so a GNSS position fix corrects not only position but the attitude and IMU biases that
caused it to drift. The error-state form keeps attitude as a quaternion and estimates only a small
3-D correction to it, which avoids treating the quaternion's four components as independent. See
[DESIGN.md](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#error-state-kalman-filter).

### When you do not need one

`fusion-nav` complements the lighter Fusion filters:

* **attitude only** — `fusion-ahrs`, substantially simpler and cheaper.
* **attitude, altitude, vertical velocity** (stabilization, altitude hold) — `fusion-ahrs` with
  `fusion-altitude`.
* **3D position or velocity** — `fusion-nav`. It owns its attitude rather than consuming one from
  `fusion-ahrs`, because attitude uncertainty is coupled to velocity and position uncertainty.

[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md) compares `fusion-nav` against other Rust crates and PX4 / ArduPilot.

## Quick start

```rust,no_run
use fusion_nav::prelude::*;
# let (static_window, imu) = ([StaticSample::default(); 800], ImuSample::default());
# let (lat_e7, lon_e7, height_mm, h_acc_mm, v_acc_mm) = (473_977_420, 85_455_940, 488_000, 1_500, 3_000);
# let arrived = Timestamp::from_micros(12_500_000);

let mut filter = Eskf::new(Config::default());

// Initialize from a window of samples taken while the vehicle sits still. A short or
// moving window still starts the filter, as `Alignment::Coarse`.
if let Alignment::Coarse(_) = filter.initialize(&static_window)? {
    /* running, but reports `Status::Aligning` until attitude converges */
}

loop {
    // High-rate propagation on every IMU sample: the increments an integrating driver hands
    // over, with its timestamp, or `ImuSample::from_rates(time, gyro, accel, interval)` from
    // a rate gyroscope. The outcome is #[must_use]: a step too long to integrate is coasted
    // on an assumption, or refused with `Config::coast` off.
    if !filter.predict(imu).is_propagated() { /* log the gap */ }

    // Measurement updates whenever a sensor delivers, each with its own noise. Bound a
    // receiver's accuracy the way PX4 and ArduPilot do, and fuse only a real fix —
    // the first one places the navigation origin. The names are a u-blox PVT's.
    let fix = Geodetic::from_degrees_e7(lat_e7, lon_e7, height_mm);
    let (eph, epv) = (h_acc_mm as f32 * 1e-3, v_acc_mm as f32 * 1e-3);
    let (horizontal, vertical) = (SigmaBounds::new(0.5, 100.0), SigmaBounds::new(0.75, 100.0));
    let noise = PositionNoise::clamped(eph, epv, horizontal, vertical);
    // The time the fix describes, on the IMU's clock: when it arrived, less the receiver's
    // latency. PX4's EKF2_GPS_DELAY is that latency, 110 ms by default.
    let taken = arrived.before(Seconds::from_secs(0.110));
    if !filter.fuse_gnss_geodetic(taken, fix, noise).is_accepted() {
        /* diagnostics() has the detail */
    }

    // The estimate carries its own health.
    let s = filter.state();
    if s.validity.horizontal_position { /* use s.position */ }
}
# Ok::<(), InitError>(())
```

`fusion_nav::prelude` carries the whole integration surface. Three runnable programs show it in
full, and a fourth is the embedded counterpart, `no_std` with no `println!`: where the time comes
from, sources at their own rates, every outcome handled and logged where it is returned.

```console
$ cargo run --example basic         # the integration loop on its own
$ cargo run --example degradation   # dropouts, diagnostics, an application-driven reset
$ cargo build --example embedded --target thumbv7em-none-eabihf   # the loop on a microcontroller
$ cargo run --example replay        # a recorded flight in, the estimate out, as CSV
```

`replay` also runs real PX4 logs, and `cargo run --example simulate` generates seeded flights with
analytic ground truth. Hand `replay` that truth as a third argument and it scores itself against
it — RMSE, NEES, and how often it called an estimate usable while the error said otherwise:

```console
$ cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv
```

See [data/README.md](https://github.com/wboayue/fusion-nav/blob/main/data/README.md).

## Conventions

Fixed, not configurable. ENU and FLU input converts at the boundary, through constructors that
do it — `Position::enu(e, n, u).to_ned()`, `AngularRate::flu(..)` — so the exact signed
permutation is written once, in the crate.

```text
navigation (NED)        body (FRD)
+x  North               +x  Forward
+y  East                +y  Right
+z  Down                +z  Down
```

* Down is positive, so gravity has a **positive** z component in the navigation frame and a level,
  stationary accelerometer reads negative z.
* Attitude is a unit quaternion rotating body to navigation, Hamilton convention, scalar first.
  `Attitude`'s constructors name the convention they take; see [seeding an attitude](#seeding-an-attitude).
* Positions and velocities are in the navigation frame; IMU measurements in the body frame.

Frames are in the types, and constructors name them: `Position::ned(n, e, d)`,
`AngularRate::body(x, y, z)`. Passing a `Position<Enu>` where NED is expected is a compile error.
Units are SI and named only where a source commonly supplies something else:
`Radians::from_degrees`, `AngularRate::body_deg_per_s`, `Geodetic::from_degrees_e7`. Noise is
built `from_sigma` or `from_variance`, so a receiver's σ cannot arrive as a variance.
Components come out as plain numbers: `.x()`, `.to_array()`, or `.vector()` for `nalgebra`.

The filter never reads a clock. Every `ImuSample` carries a `Timestamp` on the caller's clock and
the intervals its increments were integrated over; the step between samples is differenced from
the timestamps, in integer microseconds, and a seed names its time too. A driver that reads its
IMU in batches hands them over one at a time or summed, `earlier.accumulate(later)`.

## Initialization

The preferred start is a quasi-static window: the vehicle still, gravity the only specific force.
Each `StaticSample` is an `ImuSample` plus an optional magnetometer reading, barometer reading
and GNSS velocity. From the window the filter takes:

* **roll and pitch** from the averaged accelerometer
* **heading** from the magnetometer if the window carries one, levelled by that roll and
  pitch. Without one, heading is unobserved: stillness says nothing about the rotation about
  gravity. `validity.heading` is false and `Status` stays `Aligning` until the first
  `fuse_mag_heading` is accepted, however still the window was — `Initialization::sigma_yaw` is a
  prior on a yaw nobody measured, and the covariance alone cannot tell the two apart, which is
  why the first accepted heading is adopted rather than fused. Leaving `Aligning` is one-way: it reports a
  start that has not been resolved, while `validity.tilt` and `validity.heading` stay live and go
  false again as an unaided covariance grows past `Config::accuracy`. The two read different bars:
  `Aligning` ends at the fixed `ALIGNED_TILT` (3°, PX4's) and `ALIGNED_HEADING` (30°), and
  `validity` at whatever the mission asks
* **gyroscope bias** from the averaged gyroscope, but only from a window taken at rest, which is
  what makes the bias observable rather than the vehicle's own turn rate. A window taken in
  motion starts it at zero, as both PX4 and ArduPilot do at every start. The accelerometer bias
  is not separable from tilt at rest either way, and starts at zero
* **the barometric reference** `α₀` — the altitude the barometer read at the origin, with the
  variance of that reading. The filter goes on estimating it, since a barometer's reference
  drifts. A window with no barometer samples fixes none, and the first altitude once position is
  established reads one from the estimate instead

A window taken **at rest** establishes `α₀`, including one too short to align an attitude from: a
vehicle sitting on the ground has an honest reference whatever the window length, and that altitude
is what zero will mean. A window taken in motion — a restart at altitude, most obviously — keeps the
reference the flight began with rather than calling its own altitude the ground. A start that
leaves no reference at all — in motion, with no barometer, or an `initialize_from` seed — takes one
from the estimate at the first altitude once position is established, as PX4 does, correlated with
the height it was read against; `fuse_baro_altitude` returns `Fusion::NoReference` until then.
`set_baro_reference(α₀, σ)` names one instead, for a reference known better than the estimate, and
`Config::baro_reference_from_estimate = false` leaves that to the caller; it returns `false` for a
value that is not a number or a σ that is not positive.

A window that is short or moving is **not refused**. It gives a coarse start: attitude
uncertainty bounded by what that window itself supports — equations (5)–(6) level its *averages*,
so what widens the prior is how far those averages are from what a still vehicle reads, how far
the vehicle turned while they were being taken, and how far the window's two halves disagree,
rather than the worst sample in it — with `Status::Aligning` until tilt and heading are within
`ALIGNED_TILT` and `ALIGNED_HEADING`. A window that is only *short*, taken with
the vehicle at rest, starts from the static figures and establishes what it saw: stillness is
measured from the window, never read off the alignment. A filter that will not start is worth less than one
that starts and says how much to trust it — refusing would rule out moving decks, hand launches,
and restarts at altitude.

`StaticSample::velocity` is what a moving window has that a still one does not need. Two GNSS
velocities in the window give `ā_n`, the vehicle's own acceleration, which is the part of the
specific force that is not gravity — and `Coarse::NotStationary` reports it beside the motion it
measured. Levelling with it, equation (5′), is not built: a coarse start bounds tilt by how far
its *averaged* specific force is from gravity, and a real `ā_n` is part of what puts it there.

| entry point | for |
| ----------- | --- |
| `initialize(window)` | the usual case; `Alignment::Static` if the window was genuinely still, `Alignment::Coarse` with what it measured otherwise |
| `initialize_coarse(imu)` | no window at all — one sample of gravity, and the filter runs |
| `initialize_from(state, covariance, time)` | an estimate the application already holds: a companion AHRS such as `fusion-ahrs`, the last flight's saved state. [Seeding an attitude](#seeding-an-attitude) is where its convention gets named |

`alignment_of(window)` reports what `initialize` would make of a window without touching the
filter, for an application that would rather wait for stillness than start coarsely.

A **seed** is checked where a window is not, because it crosses a boundary the filter does not
control — another estimator, or storage that may be stale. `initialize_from` returns
`InitError::NotFinite` for a NaN or an infinity, and `InitError::InvalidVariance` for a variance on
the covariance diagonal that is not strictly positive: the bar every `fuse_*` puts on `R`. Zero is
the one that arrives in practice, from a warm start whose diagonal was never populated, and it is
not a tight prior but a claim of perfect knowledge — nothing would ever correct that quantity, and
`validity()` would report it good immediately. A refused seed leaves the filter uninitialized
rather than poisoned.

After a coarse start the first GNSS position and first GNSS velocity are **adopted rather than
fused**, reported as `Fusion::Reset`. A vehicle that initialized while moving has no position or
velocity for the gate to judge a fix against. The first magnetic heading is adopted the same way
whenever initialization left yaw unobserved — after any coarse start, and after a static window
that carried no magnetometer, since stillness observes tilt and never yaw. That one steps the
attitude, by up to half a circle. This happens once per quantity; everything after is fused
normally.

Stillness is still worth arranging where available: initialization quality dominates
early-flight performance. See [initialization](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#initialization).

### Seeding an attitude

A quaternion carries no frames, so `Attitude` has no `From<UnitQuaternion>` and every constructor
names the convention it takes. Nothing else in initialization is like this: a seed is the one input
with no residual to expose a wrong one.

| the source publishes | constructor |
| -------------------- | ----------- |
| body FRD to NED — PX4 `vehicle_attitude.q`, ArduPilot `get_quat_body_to_ned`, `fusion-ahrs` on `Convention::Ned` | `Attitude::body_to_ned(q)`, which converts nothing |
| NED to body FRD, the stored inverse | `Attitude::ned_to_body(q)` |
| body FLU to ENU — ROS REP 103 | `Attitude::flu_to_enu(q)` |
| body FLU to NWU — Madgwick-family, `fusion-ahrs` on its default | `Attitude::flu_to_nwu(q)` |

Where both frames differ the conversion is two-sided, `q_ned←frd = r_nav ⊗ q ⊗ r_body⁻¹`; rotating
only the navigation frame reports the same heading with the vehicle upside down, which a level
bench check agrees with. The constructors' rustdoc carries the conventions, their sources, and what
a wrong one costs.

```rust
use fusion_nav::prelude::*;
use nalgebra::UnitQuaternion;

let mut filter = Eskf::new(Config::default());

// What a companion AHRS published: 2.9° nose up, heading 63°.
let q = UnitQuaternion::from_euler_angles(0.0, 0.05, 1.1);

// PX4's `vehicle_attitude.q` and ArduPilot's `get_quat_body_to_ned` are body FRD to NED
// already, which is this crate's convention too, so this constructor converts nothing.
let state = State {
    attitude: Attitude::body_to_ned(q),
    ..State::default()
};

// The seed's quality is the caller's to state, and the covariance is how: these are the
// AHRS's own sigmas, not the static-window figures in `Initialization`.
let covariance = Covariance::from_sigmas([
    5.0, 5.0, 5.0, // position, meters
    0.5, 0.5, 0.5, // velocity, meters per second
    0.035, 0.035, 0.087, // tilt, tilt, heading — radians
    0.1, 0.1, 0.1, // accelerometer bias
    0.01, 0.01, 0.01, // gyroscope bias
]);

// When the seed is valid, on the clock the IMU's timestamps are on.
let time = Timestamp::from_micros(12_500_000);
assert_eq!(filter.initialize_from(state, covariance, time)?, Alignment::Seeded);
assert!((filter.state().attitude.euler_angles().2 - 1.1).abs() < 1.0e-6);
# Ok::<(), InitError>(())
```

## Running the filter

### Propagation

Call `predict(imu)` on every IMU sample. The step `dt` is the time from the last sample's
timestamp to this one's, and it is what the health timers and the gap test read; the increments
are integrated over their own intervals. The result is `#[must_use]`:

| `Propagation` | meaning |
| ------------- | ------- |
| `Propagated` | state advanced across the sample |
| `Coasted { dt }` | `dt` exceeded `Config::max_predict_dt`, so the sample was not integrated: position advanced on the estimated velocity and the covariance grew by what `Config::coast` allows ([equation (22′)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#coasting-across-a-gap)) |
| `StepTooLong { dt, limit }` | the same, with `Config::coast` off; state unchanged, but health timers advanced because the time really passed |
| `InvalidStep { dt }` | the sample is not after the last: `dt` zero or negative; nothing moved, the clock included |
| `NotFinite` | the **sample** carried a NaN or an infinity; state unchanged, health timers advanced as above |
| `InvalidInterval { interval }` | an integration interval under a microsecond, or longer than `Config::max_predict_dt`; as `NotFinite` |
| `StateNotFinite` | the propagated **state** did, so it was discarded; a finite sample can still overflow f32 through (11)–(14) |
| `NotInitialized` | no state to propagate |

### Measurements

| method | measurement |
| ------ | ----------- |
| `fuse_gnss_geodetic(time, fix, noise)` | latitude, longitude, height; converted about the filter's origin |
| `fuse_gnss_position(time, position, noise)` | NED position about the filter's origin, for a caller that converts itself |

Both GNSS position calls return a `GnssFusion`, a `Fusion` for each half of the fix —
`horizontal` and `height` — because the two are gated apart: a height the estimate disagrees with
is rejected without costing the horizontal fix beside it, and `diagnostics()` carries each half
as its own source, `gnss_position` and `gnss_height`. `is_accepted()` on it asks for both.
| `fuse_gnss_velocity(time, velocity, noise)` | NED velocity |
| `fuse_baro_altitude(time, altitude, noise)` | altitude, relative to `α₀` |
| `fuse_mag_heading(time, field, noise)` | body-frame field, reduced to a heading and fused as one scalar |

`time` is when the measurement was taken, on the clock the IMU's samples are timed on, and it is
an argument for the reason the noise is: a receiver's latency is a property of that receiver and
that fix. A measurement older than `LATENCY_HORIZON` or later than the state by more than
`Config::max_predict_dt` is refused as `OutOfHorizon { age }`, and the age tells a latency past the
horizon from a clock on another epoch.

The noise is an argument, not configuration, because the accuracy of a fix is a property of that
fix. Build it the way the source reports it: `PositionNoise::horizontal_vertical(eph, epv)` and
`VelocityNoise::from_speed_accuracy(sacc)` take a receiver's standard deviations, `from_variance`
takes a covariance diagonal as ROS carries it. Bound a receiver's figures first —
`PositionNoise::clamped` and `VelocityNoise::clamped` take a `SigmaBounds` per axis, and their
documentation writes PX4's and ArduPilot's rules as one call each, since neither production
autopilot fuses a receiver's figures raw. Both floor it, against a receiver whose accuracy
stays small under multipath while the fix is metres wrong; ArduPilot also caps, and PX4 caps
horizontal position only while GNSS is its sole horizontal aid. Where an axis was not measured at
all — a two-dimensional fix, a solution with no vertical velocity — `horizontal_vertical` is the
constructor instead, since it leaves that axis' σ alone where `clamped` would cap it back into a
measurement. The magnetometer must already be calibrated for hard and soft iron: `noise` is on
the heading rather than on the field, and the filter widens it by the tilt it levelled with —
equation (36′) — but nothing in it can find a hard-iron offset. Heading is true only once
`set_magnetic_declination` names the site's declination, zero until then; set it before
initializing, since the window's heading reads it, and again when a first fix says where the
vehicle is.

Every measurement passes through an innovation gate first. The result carries the test ratio, so
a rejection is diagnosable. Reading it is optional: `diagnostics()` keeps the ratio, the counts
and the timer per source, and counts the refusals below with the reason for the latest one, so
nothing here is lost by discarding the return value. Read it where the response is per call —
a `Reset` steps the state, and a refusal says the measurement never reached the gate:

| `Fusion` | meaning |
| -------- | ------- |
| `Accepted { test_ratio }` | fused; ratio ≤ 1 |
| `Rejected { test_ratio }` | gated out; ratio > 1, state unchanged |
| `Reset` | adopted outright: the quantity was never established, or its source was locked out past its [recovery](#recovery-from-gate-lockout) timeout; steps the state |
| `NoReference` | barometer altitude with no `α₀` and no established position to read one against, or a geodetic fix that cannot place an origin |
| `NotFinite` | a NaN or infinity in the measurement or its noise; discarded |
| `InvalidNoise` | a zero or negative variance in the noise — no sensor has one, and `S` would be singular or worse; discarded |
| `OutOfHorizon { age }` | `time` older than `LATENCY_HORIZON`, or ahead of the state by more than `Config::max_predict_dt`; discarded |
| `StateInvalid` | the filter's own covariance or correction could not support an update — `S` not positive-definite, or f32 overflow; nothing committed, and the measurement is not at fault |
| `NotInitialized` | no state to fuse against |

### The navigation origin

Position is NED meters about an origin the filter holds. The first `fuse_gnss_geodetic` places
it: under the current estimate after a static start or a seed, so nothing steps, and at the fix
itself after a coarse start, where the fix is adopted. Check the fix type first — a receiver
without a fix often reports latitude and longitude zero, and that would become the origin. `set_origin` names one instead, such as a
surveyed home; call it after initializing, since a static start clears it.

Read it back with `origin()`, and use it for anything else held in latitude and longitude — a
waypoint, a geofence — so it lands in the estimate's frame. `geodetic_position()` is the estimate
converted back.

## Health reporting

A single rejection needs no action — that is what the gate is for. Sustained rejection is
different: if the filter itself is wrong, correct measurements look inconsistent, all are
rejected, and the filter silently dead-reckons while looking confident. So health travels with
the estimate: `filter.state()` returns the solution together with its status and validity, and a
solution cannot be read without them.

### `Status` — how bad is the worst thing

| `Status` | meaning |
| -------- | ------- |
| `Healthy` | every source that has been fused is still accepted, and attitude has converged |
| `Aligning` | running and aided, but attitude has not converged — a coarse start still learning, or a heading no magnetometer has observed yet |
| `Degraded` | a source has timed out; others still aid the solution |
| `DeadReckoning` | nothing is aiding; position and velocity drift without bound |

When several apply the most severe wins, in the order `DeadReckoning` > `Aligning` > `Degraded` >
`Healthy`. Only sources that have ever been accepted count, so a vehicle with no GNSS is not
`Degraded` for lacking it.

### `validity` — which outputs can I use

`state().validity` has one flag per quantity: `tilt`, `heading`, `horizontal_position`,
`vertical_position`, `horizontal_velocity`, `vertical_velocity`. Each is the covariance measured
against `Config::accuracy`, plus the requirement that the quantity was ever established — a tight
prior on a number nobody set is not validity. A coarse start with an adopted GNSS fix has valid
position while its attitude is still `Aligning`; `Status` alone cannot say that. Heading is the
same rule applied to attitude: a vehicle with no magnetometer has valid `tilt` and never valid
`heading`, until one is fused.

`Config::accuracy` is the one group of numbers meant to be supplied rather than derived: a survey
platform and a racing quadrotor disagree about what "good enough" means. It moves `validity` and
nothing else — `Status::Aligning` reads fixed bars, so asking for 1° of roll does not also make the
filter wait for 1° before it calls the start resolved. A bar tighter than the prior
`Initialization` starts from is never met, and that quantity is invalid from the first epoch.

### `predicted_validity` — will it be good if I take off now

On the ground, heading may be unobservable and GNSS may not have a fix yet, so `validity` says
no about a filter that would be navigating a second after takeoff. `predicted_validity()` answers
a different question: whether each quantity would **still** be good `Accuracy::horizon` from now
with nothing fusing, **or** a source that constrains it is currently being accepted. Use it for
arming checks. (ArduPilot's `pred_horiz_pos_rel` is the second clause; neither it nor PX4
publishes the first.)

The two clauses cover the two ways an arming check goes wrong. The projection propagates `P`
forward by the same equations `predict` runs and tests each quantity at the far end, so a tilt
that is inside its bar now and will not be in a second reads false here and true from
`validity()`. The aiding clause is what a projection cannot supply: before the first fix there is
no horizontal position to propagate, and that fixes are arriving is the whole answer.

Tilt is where it matters most, because nothing aids it — a static window brings it in and
gyroscope-bias uncertainty takes it back out, on a schedule only the covariance knows. At the
default noise an unaided start holds tilt for 3.83 s, so a horizon under that arms and one over
it does not.

`Accuracy::horizon` is the one number in the crate no data could settle: how long after arming
you need the estimate. It defaults to 1 s. Set it to zero and nothing is projected, leaving the
aiding clause on its own — the current answer widened by what is being accepted, which is what
`predicted_validity()` meant before it could project at all.

### Detail

* `is_aligned()` — whether attitude has converged, on the same bar `Status::Aligning` uses: read
  from the covariance against `ALIGNED_TILT` and `ALIGNED_HEADING`, so promotion is measured rather
  than timed, and not against `Config::accuracy`, which is the mission's.
* `diagnostics()` — per source: test ratio, time since last acceptance, consecutive rejections,
  and how many measurements were refused before the gate and why. A source that only ever refuses
  reads as "never accepted", like one that was never connected, and the refusal count is what
  tells them apart. Also carries what `predict` refused and `floored`, neither of which is per
  source. For logging and tuning; not on the hot path.
  `floored` counts variances raised to the floor of equation (42′), and is meant to stay at zero:
  the floor is set well below anything the filter reaches, so a count climbing there says a
  covariance is being driven toward zero by an `R` far tighter than what the measurement observes.
* `covariance()` — the 15 × 15 covariance, indexed by name: `p.variance(ErrorState::AttitudeZ)`.
* `baro_reference()` — `α₀` as currently estimated, if a start established one. It moves as the
  barometer and GNSS height disagree; see equation (30′).

### Logging an outcome

Every outcome, `Status` and `Validity` implement `Display`, one line each and without the
source, which the outcome does not know:

```rust
use core::fmt::Write;
use fusion_nav::prelude::*;

fn fuse(filter: &mut Eskf, log: &mut impl Write, at: Timestamp, fix: Geodetic) -> core::fmt::Result {
    let noise = PositionNoise::horizontal_vertical(1.5, 3.0);
    let outcome = filter.fuse_gnss_geodetic(at, fix, noise);
    if !outcome.is_accepted() {
        writeln!(log, "gnss position {outcome}")?; // gnss position rejected, ratio 2.70
    }
    let state = filter.state();
    writeln!(log, "{} {}", state.status, state.validity) // degraded att:TH pos:HV vel:-V
}
```

With the `defmt` feature the same types, and `State` and `Diagnostics` besides, implement
`defmt::Format`. It is off by default, so the default build keeps its one dependency.
The numbers are printed in fixed point through integer formatting, because core's `f32`
formatting can panic; see [the library cannot panic](#the-library-cannot-panic).

## Recovery from gate lockout

A gate rejects what disagrees with the estimate, so an estimate that has gone wrong — a logging
dropout at speed, a covariance shrunk around an error it cannot see — rejects the measurements
that would correct it. The filter recovers: a source rejected for longer than `Config::recovery`
allows has its next measurement adopted rather than discarded, reported as `Fusion::Reset` and
counted in `SourceHealth::recovered`. The timeouts are PX4's, and `Recovery`'s documentation
says where each comes from and how each source recovers.

An application that owns the decision — a controller that cannot take a step, a failsafe that
would rather land — turns off the sources it owns, or all of them, and resets the state itself:

```rust
use fusion_nav::prelude::*;

let config = Config {
    recovery: Recovery {
        gnss_position: None, // the application resets position itself
        ..Recovery::default()
    },
    ..Config::default()
};
# let _ = config;
let everything_off = Config { recovery: Recovery::OFF, ..Config::default() };
# let _ = everything_off;
```

`reset_position_to(fix, noise)` and `reset_velocity_to(fix, noise)` are that application's
tools. Both return `false`, changing nothing, for a fix or a noise a `fuse_*` would have refused:
a reset writes the noise onto the covariance diagonal with no gate in the way. See
[rejection handling](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#rejection-handling-recover-by-default-opt-out-per-source).

## The library cannot panic

Checked in CI, not remembered: `panic-check/run.sh` links the whole public API for
`thumbv7em-none-eabihf` and `thumbv6m-none-eabi` with fat LTO and fails if any reference to
`core::panicking` survives. That covers an `unwrap` and an `expect`, and equally a slice index
or a `nalgebra` matrix index nobody wrote down. On `thumbv6m` a panic is a `udf` instruction and
the vehicle is a brick, which is why bad input comes back as `Propagation::InvalidStep` or
`Fusion::InvalidNoise` rather than as an assertion.

It is a real link rather than a scan of the source, so it proves reachability rather than
absence of a keyword. `panic-check/src/main.rs` calls every entry point through
`core::hint::black_box`, and the script refuses to run if any `pub fn` in `src/` is missing from
it — a new entry point joins the gate or CI stops. A failure names the function that can panic,
not just the symbol it reached.

Two boundaries, both real:

* **`debug-assertions = false`**, the release default. Three `debug_assert!`s in `src/eskf.rs`
  restate a condition the lines above them just checked, and with assertions on they are panics
  like any other — the gate fails, by design. They are a test-time check on the crate's own
  reasoning, not a runtime guard, which is why they are `debug_assert!` and not `if`.
* **`opt-level = 3` or `"s"`.** At `"z"` and `1`, LLVM stops proving that `nalgebra`'s statically
  sized `Matrix3 * Vector3` indexes in bounds and leaves the check in as dead code, reached from
  `LocalOrigin::to_ned`. That is the optimizer giving up, not a path this crate can take, and
  gating it would put CI at the mercy of someone else's codegen.

The `Display` impls are in the gate too, and are why none of them prints an `f32` with `{}`:
core's float formatting reaches `core::panicking` on both targets at both levels, which the gate
measured the first time one was linked. So they print fixed point through integer formatting,
which passes, and `run.sh` refuses a `Display` impl that `panic-check/src/main.rs` does not
format. That covers what this crate prints and nothing the caller does: a `{}` or `{:?}` of an
`f32` in application code, including the derived `Debug` of any type here, brings the path back.
`defmt` sends floats as raw bytes and formats them on the host, so it never takes the path.

The crate is also `#![forbid(unsafe_code)]`, `no_std`, and allocation-free.

## Limitations

Known, and stated here rather than discovered in flight. Some are deliberate; the rest link the issue that removes them.

* **A measurement's latency is the caller's to know.** Each is fused at the time it is given,
  against the state as it was then, so a late fix costs nothing, but the filter cannot measure how
  late a receiver is: PX4 takes a parameter and ArduPilot the driver's figure. A wrong one is an error
  that grows with speed, and a receiver with none that fits is worse than fused as current: one
  corpus log rejects 396 fixes at PX4's 110 ms and 267 at none. Anything older than
  `LATENCY_HORIZON`, 0.3 s, is refused. See
  [measurement latency](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#measurement-latency).
* **Barometer drift costs height where there is none.** The reference is estimated and allowed to
  walk, at PX4's rate by default, so GNSS height carries the low frequencies and a barometer
  more stable than that is trusted less than it could be. See
  [barometric reference as an estimated offset](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#barometric-reference-as-an-estimated-offset).
* **No magnetic-field states.** Hard- and soft-iron calibration is the application's job; an
  uncalibrated magnetometer gives a heading bias the filter cannot detect.
* **Heading needs a magnetometer.** It is the only heading source the filter has, so a vehicle
  without one never leaves `Aligning` and never reports `validity.heading`, however good the rest
  of the estimate is. `Aligning` hides `Degraded`, so such a vehicle's source timeouts stop
  showing in `Status` too and have to be read from `diagnostics()`. Yaw from course over ground,
  dual-antenna GNSS heading and a GSF yaw estimator are the answers, all unbuilt. See
  [alignment beyond the static window](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#alignment-beyond-the-static-window).
* **In-motion alignment is coarse.** A moving start runs and reports `Aligning`, but full
  alignment of a bare vehicle in motion is not yet built; `initialize_from` covers a held
  estimate. See [alignment beyond the static window](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#alignment-beyond-the-static-window).
* **Correlation times are configured, not measured.** Each source is fused at the variance its
  correlation with the last reading leaves ([equation (24′)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#correlated-measurements)),
  with `τ` from `Config::correlation`, whose defaults are the corpus's medians. A sensor whose
  error persists longer than its `τ` still shrinks the covariance below what it supports, and one
  reporting a σ too small is gated on that σ and weighted less besides. See
  [correlated measurement error](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#correlated-measurement-error-as-equivalent-white-noise).
* **An IMU gap is coasted on an assumption.** Across a step longer than `Config::max_predict_dt`
  the filter assumes the vehicle neither accelerated nor turned, and prices what it may have done
  with `Config::coast`'s two densities, set from one VTOL log's gaps at 30 m/s. A vehicle that
  manoeuvres harder than that inside a gap can still be turned down by the gate afterwards,
  until `Config::recovery` adopts a fix.
* **No lever arms.** GNSS position and velocity are taken as the IMU's, so an uncorrected antenna
  offset `r` reads rotation as velocity (`ω × r`), and the filter believes it. The caller corrects
  it with `angular_rate()`, the bias-corrected `ω`, whose documentation writes the correction out;
  PX4 applies it inside the filter. See [#25](https://github.com/wboayue/fusion-nav/issues/25).
* **One set of timeouts for every source.** `Config::timeouts` applies one threshold to every
  source, and any accepted source counts as aiding. See [#56](https://github.com/wboayue/fusion-nav/issues/56).
* **Local tangent plane.** Position is Cartesian NED about a fixed origin. The geodetic
  conversion is exact at any range ([equation (43)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#geodetic-origin)), but a plane
  leaves a curved Earth: `d` from the origin it sits `d²/2R` above the surface, 8 cm at 1 km and
  7.8 m at 10 km, so `-p_D` far out is not height. The barometer model does not correct for it,
  and [equation (30)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#barometric-altitude)
  records the measurement behind that.

Features out of scope (wind, terrain, optical flow, airspeed, ...) are listed in
[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#non-goals).

## Further reading

* [GLOSSARY.md](https://github.com/wboayue/fusion-nav/blob/main/GLOSSARY.md) — the vocabulary the other four assume: innovation, NEES, bias,
  specific force, consistency against accuracy, and what PX4 and ArduPilot call the same things
* [DESIGN.md](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md) — architecture, state definition, measurement models, gating, embedded
  budget, scope
* [EQUATIONS.md](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md) — the mathematics, numbered, with an equation-to-code map
* [GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md) — positioning, differentiators, decisions, open questions
* [data/README.md](https://github.com/wboayue/fusion-nav/blob/main/data/README.md) — the replay harness and PX4 log corpus

## License

Licensed under the MIT License.
