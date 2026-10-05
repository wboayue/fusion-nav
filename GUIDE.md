# Guide

How to use `fusion-nav` in detail. [README.md](https://github.com/wboayue/fusion-nav/blob/main/README.md) is the overview and quick start,
and [MIGRATING.md](https://github.com/wboayue/fusion-nav/blob/main/MIGRATING.md) maps PX4 and ArduPilot parameters onto this crate.

## Conventions

Fixed, not configurable. ENU and FLU input converts at the boundary, through constructors that
do it (`Position::enu(e, n, u).to_ned()`, `AngularRate::flu(..)`), so the exact signed
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
  `Attitude`'s constructors name the convention they take; see
  [seeding an attitude](#seeding-an-attitude).
* Positions and velocities are in the navigation frame; IMU measurements in the body frame.

Frames are in the types, and constructors name them: `Position::ned(n, e, d)`,
`AngularRate::body(x, y, z)`. Passing a `Position<Enu>` where NED is expected is a compile error.
Units are SI and named only where a source commonly supplies something else:
`Radians::from_degrees`, `AngularRate::body_deg_per_s`, `Geodetic::from_degrees_e7`. Noise is
built `from_sigma` or `from_variance`, so a receiver's σ cannot arrive as a variance.

Components cross as plain numbers, `.x()` or `.to_array()` out and `from_array` in, with the frame
still the type parameter: `Position::<Ned>::from_array(p)`. No `nalgebra` type is public. It is
0.x, so a public `Vector3` would pin every integrator to this crate's version; a vector's array
converts to and from any version's, and glam's, with `.into()`. A quaternion is a `Quaternion` with
named fields instead, since the libraries disagree on the order of four numbers.

The filter never reads a clock. Every `ImuSample` carries a `Timestamp` on the caller's clock and
the intervals its increments were integrated over. The step between samples is differenced from
the timestamps, in integer microseconds, and a seed names its time too. A driver that reads its
IMU in batches hands them over one at a time or summed, `earlier.accumulate(later)`.

## Initialization

Three entry points start the filter, and each reports an `Alignment`:

| entry point | for |
| ----------- | --- |
| `initialize(window)` | the usual case; `Alignment::Static` if the window was genuinely still, `Alignment::Coarse` with what it measured otherwise |
| `initialize_coarse(imu)` | no window at all: one sample of gravity, and the filter runs |
| `initialize_from(state, covariance, time)` | an estimate the application already holds: a companion AHRS such as `fusion-ahrs`, the last flight's saved state. [Seeding an attitude](#seeding-an-attitude) is where its convention gets named |

The preferred start is a quasi-static window: the vehicle still, gravity the only specific force.
Each `StaticSample` is an `ImuSample` plus an optional magnetometer reading, barometer reading
and GNSS velocity. From the window the filter takes:

* **roll and pitch** from the averaged accelerometer.
* **heading** from the magnetometer, leveled by roll and pitch. Without one, heading is
  unobserved: `validity.heading` is false and `Status` is `Aligning` until a first heading is
  accepted (adopted, see below).
* **gyroscope bias** from the averaged gyroscope, but only from a window taken at rest, which is
  what makes the bias observable rather than the vehicle's own turn rate. A window taken in
  motion starts it at zero, as both PX4 and ArduPilot do at every start. The accelerometer bias
  is not separable from tilt at rest either way, and starts at zero.
* **the barometric reference** `α₀`: the altitude the barometer read at the origin, with that
  reading's variance, and what zero altitude will mean. A window taken **at rest** sets it, even
  one too short to align an attitude from, since a vehicle on the ground has an honest reference
  whatever the window length. A window taken in motion (a restart at altitude, most obviously)
  keeps the reference the flight began with rather than calling its own altitude the ground. The
  filter goes on estimating `α₀`, since a barometer's reference drifts.

A start that leaves no reference (in motion, with no barometer, or an `initialize_from` seed)
reads one from the estimate at the first altitude once position is established, as PX4 does,
correlated with the height it was read against. Until then `fuse_baro_altitude` returns
`Fusion::NoReference`. `set_baro_reference(α₀, σ)` names one instead, for a reference known better
than the estimate, and `Config::baro_reference_from_estimate = false` leaves that to the caller.
`set_baro_reference` returns `false` for a value that is not a number or a σ that is not positive.

The window is a `StaticWindow`, which keeps what the samples reduce to rather than the samples, so
it costs under a kilobyte at any IMU rate. A slice of buffered samples converts with
`StaticWindow::try_from`, and `try_extend` folds in any iterator of samples. A window only grows,
so one that moved is started over; `window.is_at_rest(&config)` says after every sample whether
it has. `alignment_of(window)` reports what `initialize` would make of a window without touching
the filter, for an application that would rather wait for stillness than start coarsely.
`window.noise(&config)` reports the white noise a still window measured on each sensor: a floor
under what `Config::imu` and a barometer's `R` should be, never a replacement. It asks for no
filter and changes nothing unless the application writes it into a `Config`.

A short or moving window is **not refused**: it starts coarse, with `Status::Aligning` until tilt
and heading converge. Its attitude prior widens with how far the window's averages sit from a
still vehicle's, how far it turned, and how far its halves disagree; equations (5)–(6) level
averages, not the worst sample. A window that is only *short*, taken at rest, starts from the
static figures and establishes what it saw: stillness is measured from the window, never read off
the alignment. A filter that starts and says how much to trust it is worth more than one that
will not start, and refusing would rule out moving decks, hand launches, and restarts at
altitude.

`StaticSample::velocity` is what a moving window has that a still one does not need. Two GNSS
velocities in the window give `ā_n`, the vehicle's own acceleration: the part of the specific
force that is not gravity. `Coarse::NotStationary` reports it beside the motion it measured.
Leveling with it, equation (5′), is not built: a coarse start bounds tilt by how far its
*averaged* specific force is from gravity, and a real `ā_n` is part of what puts it there.

A **seed** is checked where a window is not, because it crosses a boundary the filter does not
control: another estimator, or storage that may be stale. `initialize_from` returns
`InitError::NotFinite` for a NaN or an infinity, and `InitError::InvalidVariance` for a variance
on the covariance diagonal that is not strictly positive, the bar every `fuse_*` puts on `R`.

Zero is the variance that arrives in practice, from a warm start whose diagonal was never
populated. It is not a tight prior but a claim of perfect knowledge: nothing would ever correct
that quantity, and `validity()` would report it good immediately. A refused seed leaves the filter
uninitialized rather than poisoned.

After a coarse start the first GNSS position and first GNSS velocity are **adopted rather than
fused**, reported as `Fusion::Reset`: a vehicle that initialized while moving has no position or
velocity for the gate to judge a fix against. The first heading, magnetic, GNSS, course or the
[yaw estimator](#yaw-without-a-heading-sensor)'s, is
adopted the same way whenever initialization left yaw unobserved: after any coarse start, and
after a static window with no magnetometer, since stillness observes tilt and never yaw. A prior
on a yaw nobody measured (`Initialization::sigma_yaw`) looks to the covariance like a
measurement, which is why that heading is adopted. It steps the attitude, by up to half a circle.
Each quantity is adopted once; everything after is fused normally.

Stillness is still worth arranging where available: initialization quality dominates
early-flight performance. See
[initialization](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#initialization).

### Seeding an attitude

A quaternion carries no frames, so `Attitude` has no `From<Quaternion>` and every constructor
names the convention it takes. Nor does `fusion_nav::Quaternion` convert from an array: PX4's
`q[4]` puts the scalar first, while ROS, Eigen, glam's `to_array` and `nalgebra`'s `From<[f32; 4]>`
put it last, so its fields are named and the order is written where it is read. Nothing else in
initialization is like this: a seed is the one input with no residual to expose a wrong one.

| the convention | in | out |
| -------------- | -- | --- |
| body FRD to NED: PX4 `vehicle_attitude.q`, ArduPilot `get_quat_body_to_ned`, `fusion-ahrs` on `Convention::Ned` | `Attitude::from_body_to_ned(q)`, which converts nothing | `attitude.body_to_ned()` |
| NED to body FRD, the stored inverse | `Attitude::from_ned_to_body(q)` | `attitude.ned_to_body()` |
| body FLU to ENU: ROS REP 103 | `Attitude::from_flu_to_enu(q)` | `attitude.flu_to_enu()` |
| body FLU to NWU: Madgwick-family, `fusion-ahrs` on its default | `Attitude::from_flu_to_nwu(q)` | `attitude.flu_to_nwu()` |

The edge is symmetric: whatever enters in a convention can leave in it. Vectors go the same way,
`Position::enu(..).to_ned()` in and `position.to_enu()` out, `AngularRate::flu(..)` in and
`rate.to_flu()` out, and `AttitudeVariance::to_enu` orders the attitude variances for an ENU
covariance.

Where both frames differ the conversion is two-sided, `q_ned←frd = r_nav ⊗ q ⊗ r_body⁻¹`.
Rotating only the navigation frame reports the same heading with the vehicle upside down, which a
level bench check agrees with. The constructors' rustdoc carries the conventions, their sources,
and what a wrong one costs.

```rust
use fusion_nav::Quaternion;
use fusion_nav::prelude::*;

let mut filter = Eskf::default();

// What a companion AHRS published, `q[4]` scalar first: 2.9° nose up, heading 63°.
let published = [0.8522581, -0.0130658, 0.0213109, 0.5225239];
let q = Quaternion { w: published[0], x: published[1], y: published[2], z: published[3] };

// PX4's `vehicle_attitude.q` and ArduPilot's `get_quat_body_to_ned` are body FRD to NED
// already, which is this crate's convention too, so this constructor converts nothing.
let state = State {
    attitude: Attitude::from_body_to_ned(q),
    ..State::default()
};

// The seed's quality is the caller's to state, and the covariance is how: these are the
// AHRS's own sigmas, not the static-window figures in `Initialization`.
let covariance = Covariance::from_sigmas([
    5.0, 5.0, 5.0, // position, meters
    0.5, 0.5, 0.5, // velocity, meters per second
    0.035, 0.035, 0.087, // tilt, tilt, heading, radians
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

Without horizontal aiding nothing observes tilt, so it grows on the schedule `ImuNoise` sets. Once
no GNSS position or velocity has been judged within `Timeouts::dead_reckoning_after`, which a
start that has heard none meets from its first step, and the tilt σ has passed 3°, a committed
step also fuses a **position hold**: the estimate's own position from
when the hold engaged, at `Config::hold`'s σ (10 m), five times a second
([equation (28″)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#holding-tilt-without-aiding)).
Bounding position bounds velocity, which is what lets the accelerometer level the filter. It is an
assumption, not a sensor: `Status` stays `DeadReckoning`, horizontal position and velocity each
stay invalid until measured again, and the first GNSS fix the gate turns down after it is adopted
at once. `Config::hold = None` turns it off. PX4's fake position and ArduPilot's `AID_NONE` do
the same; this one is fused as correlated, (24′), which keeps the covariance honest for a vehicle
that stays put. One that flies on is among the [limitations](#limitations).

### Yaw without a heading sensor

A multirotor with no magnetometer and no second antenna still gets a heading, and calls nothing
for it. Beside the filter runs a **yaw estimator**
([equations (45)–(52)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#yaw-without-a-heading-sensor)):
five yaw hypotheses, each predicting the GNSS velocity from the IMU, weighed by how well it
does. `predict` steps it and `fuse_gnss_velocity` weighs it. PX4 and ArduPilot fly this vehicle
the same way (`EKFGSF_yaw`).

Its answer reaches the filter in two cases, both adoptions and both counted in
`diagnostics().yaw_estimator`:

* **No heading yet.** Once the hypotheses agree to within 15°, their yaw is adopted as the
  first heading (`adopted`), and the filter leaves `Aligning`. That takes a horizontal
  acceleration: a hover says nothing about yaw. After an IMU gap it also takes ten seconds
  of GNSS velocity first.
* **A heading GNSS contradicts.** A magnetometer that is wrong turns every acceleration the
  wrong way, and GNSS velocity is rejected for it. After a second of that, with the estimator
  more than 25° from the filter's yaw, its yaw replaces the filter's and the velocity is adopted
  (`recovered`). `Recovery::yaw_estimator` is that second, and `None` turns it off. A
  dual-antenna heading that is being accepted is never overruled, and a magnetometer that was
  is not adopted back until it agrees with the new heading.

`Config::yaw_estimator = false` turns the whole estimator off, for a vehicle that always has a
heading source or a processor that would rather not run it: its state is part of `Eskf` either
way, and it costs cycles only when on
([cost](https://github.com/wboayue/fusion-nav/blob/main/validation/cost.md)).

### Measurements

| method | measurement |
| ------ | ----------- |
| `fuse_gnss_geodetic(time, fix, noise, antenna)` | latitude, longitude, height; converted about the filter's origin |
| `fuse_gnss_position(time, position, noise, antenna)` | NED position about the filter's origin, for a caller that converts itself |
| `fuse_gnss_velocity(time, velocity, noise, antenna)` | NED velocity |
| `fuse_baro_altitude(time, altitude, noise)` | altitude, relative to `α₀` |
| `fuse_mag_heading(time, field, noise)` | body-frame field, reduced to a heading and fused as one scalar |
| `fuse_gnss_heading(time, heading, noise)` | true heading from a dual-antenna receiver, the mounting angle already removed |
| `fuse_course(time, sideslip)` | a constraint rather than a reading: the nose points along the estimated velocity, to within `sideslip`. Call it after `fuse_gnss_velocity`, with that fix's `time`. Fixed-wing and ground vehicles; never a multirotor |
| `fuse_stationary(time, noise)` | a claim rather than a reading: the vehicle is still, so velocity is zero on every axis. Call it while a landed detector or the application knows it; it holds tilt on a bench with no GNSS, and a claim the gate turns down is never adopted |

Both GNSS position calls return a `GnssFusion`, a `Fusion` for each half of the fix, `horizontal`
and `height`, because the two are gated apart. A height the estimate disagrees with is rejected
without costing the horizontal fix beside it, and `diagnostics()` carries each half as its own
source, `gnss_position` and `gnss_height`. `is_accepted()` on it asks for both.

The noise is an argument, not configuration, because the accuracy of a fix is a property of that
fix. Build it the way the source reports it: `PositionNoise::horizontal_vertical(eph, epv)` and
`VelocityNoise::from_speed_accuracy(sacc)` take a receiver's standard deviations, and
`from_variance` takes a covariance diagonal as ROS carries it.

* **Bound a receiver's figures first.** `PositionNoise::clamped` and `VelocityNoise::clamped` take
  a `SigmaBounds` per axis, and their documentation writes PX4's and ArduPilot's rules as one call
  each; neither production autopilot fuses a receiver's figures raw. Both floor them, against a
  receiver whose accuracy stays small under multipath while the fix is meters wrong. ArduPilot
  also caps, and PX4 caps horizontal position only while GNSS is its sole horizontal aid.
* **An axis not measured at all** (a two-dimensional fix, a solution with no vertical velocity)
  takes `horizontal_vertical` instead, which leaves that axis' σ alone where `clamped` would cap
  it back into a measurement.
* **The magnetometer must already be calibrated** for hard and soft iron. `noise` is on the
  heading rather than the field, and the filter widens it by the tilt it leveled with, equation
  (36′), but nothing in it can find a hard-iron offset.
* **A dual-antenna heading** is bounded like a fix: `HeadingNoise::clamped`, with PX4's or
  ArduPilot's floor.
* **Declination** makes a heading true. The filter reads it from a World Magnetic Model table
  wherever it places its origin, at the first geodetic fix, and turns a heading only the
  magnetometer has referred to north by the difference (the `magnetic-model` feature, on by
  default, about 2.6 KB of flash). `set_magnetic_declination` overrides it for good, and
  `Geodetic::magnetic_declination` is the same lookup for a caller that knows its site before the
  first fix.

`antenna` is where the GNSS antenna sits relative to the IMU, in body axes:
`Position::body(forward, right, down)` or `Position::flu(..)`, and `Position::zero()` for one on
top of it. A fix measures the antenna, so the filter refers it to the IMU by its own attitude
and, for a velocity, by the rate the vehicle was turning when the fix was taken, equations (28′)
and (29′). From PX4 it is `SENS_GPS0_OFF*` less `EKF2_IMU_POS*`. It is an argument rather than a
setting, as on PX4's GNSS message, because a second receiver sits somewhere else.

`time` is when the measurement was taken, on the clock the IMU's samples are timed on. It is an
argument because a receiver's latency is a property of that receiver and that fix. A measurement
older than `LATENCY_HORIZON`, or later than the state by more than `Config::max_predict_dt`, is
refused as `OutOfHorizon { age }`, and the age tells a latency past the horizon from a clock on
another epoch.

Every measurement passes through an innovation gate first, and the result carries the test ratio,
so a rejection is diagnosable. Reading it is optional: `diagnostics()` keeps the ratio, the counts
and the timer per source, and counts the refusals below with the reason for the latest one, so
discarding the return value loses nothing. Read it where the response is per call: a `Reset`
steps the state, and a refusal says the measurement never reached the gate.

| `Fusion` | meaning |
| -------- | ------- |
| `Accepted { test_ratio }` | fused; ratio ≤ 1 |
| `Rejected { test_ratio }` | gated out; ratio > 1, state unchanged |
| `Reset` | adopted outright: the quantity was never established, or its source was locked out past its [recovery](#recovery-from-gate-lockout) timeout; steps the state |
| `NoReference` | barometer altitude with no `α₀` and no established position to read one against, a geodetic fix that cannot place an origin, or a course with no GNSS velocity accepted recently |
| `Unobservable` | a GNSS heading or course the geometry cannot give: body x within 30° of vertical, or a course while the vehicle is too slow against its velocity's uncertainty; discarded |
| `NotFinite` | a NaN or infinity in the measurement or its noise; discarded |
| `InvalidNoise` | a zero or negative variance in the noise: no sensor has one, and `S` would be singular or worse; discarded |
| `OutOfHorizon { age }` | `time` older than `LATENCY_HORIZON`, or ahead of the state by more than `Config::max_predict_dt`; discarded |
| `StateInvalid` | the filter's own covariance or correction could not support an update (`S` not positive-definite, or f32 overflow); nothing committed, and the measurement is not at fault |
| `NotInitialized` | no state to fuse against |

### The navigation origin

Position is NED meters about an origin the filter holds. The first `fuse_gnss_geodetic` places
it: under the current estimate after a static start or a seed, so nothing steps, and at the fix
itself after a coarse start, where the fix is adopted. Check the fix type first: a receiver
without a fix often reports latitude and longitude zero, and that would become the origin.
`set_origin` names one instead, such as a surveyed home; call it after initializing, since a
static start clears it.

Read it back with `origin()`, and use it for anything else held in latitude and longitude (a
waypoint, a geofence) so it lands in the estimate's frame. `geodetic_position()` is the estimate
converted back.

## Health reporting

A single rejection needs no action: that is what the gate is for. Sustained rejection is
different. If the filter itself is wrong (a logging dropout at speed, a covariance shrunk around
an error it cannot see), correct measurements look inconsistent, all are rejected, and the filter
silently dead-reckons while looking confident. So health travels with the estimate:
`filter.state()` returns the solution together with its status and validity, and a solution
cannot be read without them.

Three questions, three answers:

| question | answer | read it |
| --- | --- | --- |
| how bad is the worst thing? | `state().status` | for a mode change, a failsafe, a log line |
| which outputs can I use now? | `state().validity`, one flag per quantity | before using a quantity in control |
| will they still be good if I take off now? | `predicted_validity()` | in an arming check |

### `Status` — how bad is the worst thing

| `Status` | meaning |
| -------- | ------- |
| `DeadReckoning` | neither GNSS position nor velocity has been accepted for `Config::timeouts.dead_reckoning_after`; horizontal position is dead reckoned or held by the position hold, unusable either way, whatever the barometer and magnetometer still hold |
| `Aligning` | running and aided, but attitude has not converged: a coarse start still learning, or a heading nothing has observed yet |
| `Degraded` | a source has timed out; horizontal position is still aided |
| `Healthy` | every source that has been fused is still accepted, and attitude has converged; the course constraint, the stationary claim, the position hold and the yaw estimator, which read no sensor of their own, are not counted |

When several apply the most severe wins, in the order of the table: `DeadReckoning` > `Aligning` >
`Degraded` > `Healthy`. So a vehicle waiting for its first GNSS fix reads `DeadReckoning`, not
`Aligning`, while its barometer and magnetometer arrive. Only sources that have ever been accepted
count toward `Degraded`, so a vehicle with no magnetometer is not `Degraded` for lacking one. Each
source times out against its own rate, after two and a half of its measured periods
(`SourceHealth::period`, `SourceHealth::timeout`): a 20 Hz barometer that stops is noticed in an
eighth of a second, a 1 Hz receiver in two and a half. `DeadReckoning` is horizontal, as PX4's and
ArduPilot's are; `validity` answers per quantity.

`Aligning` is left once and never returns: it reports a start that has not been resolved. It ends
at fixed bars, `ALIGNED_TILT` (3°, PX4's) and `ALIGNED_HEADING` (30°), read from the covariance,
so promotion is measured rather than timed. `validity.tilt` and `validity.heading` stay live
instead, and go false again as an unaided covariance grows past `Config::accuracy`.

### `validity` — which outputs can I use

`state().validity` has one flag per quantity: `tilt`, `heading`, `horizontal_position`,
`vertical_position`, `horizontal_velocity`, `vertical_velocity`. Each is the covariance measured
against `Config::accuracy`, plus the requirement that the quantity was ever established: a tight
prior on a number nobody set is not validity. A coarse start with an adopted GNSS fix has valid
position while its attitude is still `Aligning`, which `Status` alone cannot say. Heading follows
the same rule: a vehicle with no magnetometer has valid `tilt` and no valid `heading` until
one is fused, or the yaw estimator supplies one.

`Config::accuracy` is the one group of numbers meant to be supplied rather than derived: a survey
platform and a racing quadrotor disagree about what "good enough" means. It moves `validity` and
nothing else. `Status::Aligning` reads its fixed bars, so asking for 1° of roll does not make the
filter wait for 1° before it calls the start resolved. A bar tighter than the prior
`Initialization` starts from is never met, and that quantity is invalid from the first epoch.

### `predicted_validity` — will it be good if I take off now

On the ground, heading may be unobservable and GNSS may not have a fix yet, so `validity` says no
about a filter that would be navigating a second after takeoff. `predicted_validity()` answers
whether each quantity would **still** be good `Accuracy::horizon` from now with nothing fusing,
**or** a source that constrains it is being accepted. Use it for arming checks. (ArduPilot's
`pred_horiz_pos_rel` is the second clause; neither it nor PX4 publishes the first.)

The two clauses cover the two ways an arming check goes wrong. The projection propagates `P`
forward by the same equations `predict` runs and tests each quantity at the far end, so a tilt
inside its bar now and outside it in a second reads false here and true from `validity()`. The
aiding clause is what a projection cannot supply: before the first fix there is no horizontal
position to propagate, and that fixes are arriving is the whole answer.

Tilt is where it matters most, because nothing aids it: a static window brings it in, and the
gyroscope's noise and bias uncertainty take it back out on a schedule only the covariance knows.
At default noise an unaided start keeps valid tilt for 10.3 s after a 2 s window that measured
its gyroscope, and 4.84 s after one whose gyroscope never scattered, with the position hold off.
The hold engages at that bar and is not projected. A horizon shorter than that arms; a longer one does not.

`Accuracy::horizon` is the one number in the crate no data could settle: how long after arming
you need the estimate. It defaults to 1 s. At zero nothing is projected, leaving the aiding clause
on its own: the current answer widened by what is being accepted.

### Detail

* `is_aligned()`: whether attitude has converged, on the bar `Status::Aligning` uses.
* `diagnostics()`: per source, the test ratio, time since last acceptance, consecutive
  rejections, and how many measurements were refused before the gate and why. A source that only
  ever refuses reads as "never accepted", like one never connected; the refusal count tells them
  apart. It also carries what `predict` refused, and `floored`, neither of which is per source.
  For logging and tuning; not on the hot path.
  `floored` counts variances raised to the floor of equation (42′), and is meant to stay at zero.
  The floor sits well below anything the filter reaches, so a climbing count says a covariance is
  being driven toward zero by an `R` far tighter than what the measurement observes.
* `covariance()`: the 15 × 15 covariance, indexed by name: `p.variance(ErrorState::AttitudeZ)`.
* `baro_reference()`: `α₀` as currently estimated, if a start established one. It moves as the
  barometer and GNSS height disagree; see equation (30′).

## Recovery from gate lockout

A filter locked out by its own gate, as [Health reporting](#health-reporting) describes, recovers:
a source rejected for longer than `Config::recovery` allows has its next measurement adopted
rather than discarded, reported as `Fusion::Reset` and counted in `SourceHealth::recovered`. The
timeouts are PX4's, and `Recovery`'s documentation says where each comes from and how each source
recovers.

An application that owns the decision (a controller that cannot take a step, a failsafe that
would rather land) turns off the sources it owns, or all of them, and resets the state itself:

```rust
use fusion_nav::prelude::*;

let config = Config {
    recovery: Recovery {
        gnss_position: None, // the application resets position itself
        ..Recovery::default()
    },
    ..Config::default()
};
let filter = Eskf::new(config)?;
# let _ = filter;
let everything_off = Config { recovery: Recovery::OFF, ..Config::default() };
# let _ = everything_off;
# Ok::<(), ConfigError>(())
```

`None` is how a timeout says off. `Eskf::new` checks every value in a `Config` against its bound
and returns a `ConfigError` naming the first field outside it and the value it held.
`Config::validate` is the same check on its own, and its documentation says what each refused
value would have done.

`reset_position_to(fix, noise)` and `reset_velocity_to(fix, noise)` are that application's tools.
Both return `false`, changing nothing, for a fix or a noise a `fuse_*` would have refused: a reset
writes the noise onto the covariance diagonal with no gate in the way. See
[rejection handling](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#rejection-handling-recover-by-default-opt-out-per-source).

## Logging an outcome

Every outcome, `Status` and `Validity` implement `Display`: one line each, without the source,
which the outcome does not know.

```rust
use core::fmt::Write;
use fusion_nav::prelude::*;

fn fuse(filter: &mut Eskf, log: &mut impl Write, at: Timestamp, fix: Geodetic) -> core::fmt::Result {
    let noise = PositionNoise::horizontal_vertical(1.5, 3.0);
    let outcome = filter.fuse_gnss_geodetic(at, fix, noise, Position::zero());
    if !outcome.is_accepted() {
        writeln!(log, "gnss position {outcome}")?; // gnss position rejected, ratio 2.70
    }
    let state = filter.state();
    writeln!(log, "{} {}", state.status, state.validity) // degraded att:TH pos:HV vel:-V
}
```

With the `defmt` feature the same types, and `State` and `Diagnostics` besides, implement
`defmt::Format`. It is off by default, so the default build keeps its one dependency. Numbers
print in fixed point through integer formatting, because core's `f32` formatting can panic; see
[the library cannot panic](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#the-library-cannot-panic).

## Limitations

Known, and stated here rather than discovered in flight. Some are deliberate; the rest link the
issue that removes them, where one is open.

* **A measurement's latency is the caller's to know.** Each is fused at the time it is given,
  against the state as it was then, so a late fix costs nothing. But the filter cannot measure how
  late a receiver is: PX4 takes a parameter and ArduPilot the driver's figure. A wrong latency is
  an error that grows with speed, and a receiver that no single latency fits can be worse at its
  configured one than fused as current. Anything older than `LATENCY_HORIZON`, 0.3 s, is refused.
  See
  [measurement latency](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#measurement-latency).
* **Barometer drift costs height where there is none.** The reference is estimated and allowed to
  walk, at PX4's rate by default, so GNSS height carries the low frequencies and a barometer
  more stable than that is trusted less than it could be. See
  [barometric reference as an estimated offset](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#barometric-reference-as-an-estimated-offset).
* **No magnetic-field states.** Hard- and soft-iron calibration is the application's job; an
  uncalibrated magnetometer gives a heading bias the filter cannot detect.
* **Heading with no heading sensor waits for the vehicle to accelerate.** A multirotor with
  neither a magnetometer nor a second antenna gets its heading from the
  [yaw estimator](#yaw-without-a-heading-sensor), which needs GNSS velocity and a horizontal
  maneuver. Until then it reads `Aligning` with no `validity.heading`, and since `Aligning`
  hides `Degraded`, its source timeouts have to be read from `diagnostics()`; one that takes
  off and holds station stays there. The course constraint serves a vehicle that points where
  it goes, and its heading is no better than the sideslip it is told to allow. See
  [yaw without a heading sensor](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#yaw-without-a-heading-sensor-a-second-estimator-inside).
* **A heading sensor that is wrong is believed until the vehicle accelerates.** The yaw
  estimator replaces a heading GNSS velocity contradicts, a second after the contradiction
  shows. Before that the filter flies on the sensor, and its covariance says the heading is
  good.
* **In-motion alignment is coarse.** A moving start runs and reports `Aligning`, but full
  alignment of a bare vehicle in motion is not built: leveling with the vehicle's own
  acceleration, equation (5′), is
  [#59](https://github.com/wboayue/fusion-nav/issues/59). `initialize_from` covers a held
  estimate. See
  [alignment beyond the static window](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#alignment-beyond-the-static-window).
* **Correlation times are configured, not measured.** Each source is fused at the variance its
  correlation with the last reading leaves
  ([equation (24′)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#correlated-measurements)),
  with `τ` from `Config::correlation`, whose defaults are read off the corpus: medians across
  logs, and one log each for the dual-antenna heading and the course, the only logs that carry
  them. A sensor whose error persists longer than its `τ` still shrinks the covariance below what
  it supports, and one reporting a σ too small is gated on that σ and weighted less besides. A `τ` read off a log is a lower bound: an estimator the filter does not bias is
  [#195](https://github.com/wboayue/fusion-nav/issues/195). See
  [correlated measurement error](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#correlated-measurement-error-as-equivalent-white-noise).
* **An IMU gap is coasted on an assumption.** Across a step longer than `Config::max_predict_dt`
  the filter assumes the vehicle neither accelerated nor turned, and prices what it may have done
  with `Config::coast`'s two densities, set from one VTOL log's gaps at 30 m/s. A vehicle that
  maneuvers harder than that inside a gap can still be turned down by the gate afterwards,
  until `Config::recovery` adopts a fix. `cargo run --example replay -- --derive <log>` prints
  the densities a vehicle's own logged gaps need, among the rest of a `Config` derived from the
  log ([deriving a `Config`](https://github.com/wboayue/fusion-nav/blob/main/data/README.md#deriving-a-config)).
* **The position hold assumes the vehicle stays put.** A car or a fixed-wing, which keeps moving
  without GNSS, should set `Config::hold = None`. While it holds, a multirotor that keeps
  flying reads partly as tilt error, which the 3° gate and (24′) keep small, and the position it
  reports is pulled toward where the hold engaged. That is why position stays invalid
  until a fix is accepted or adopted after it. A long outage flown through also leaves the
  position covariance overconfident: the simulated circuit passes its consistency bound through
  a 20 s gap and fails it at 40 s, where the filter without the hold passes (#214). See
  [holding tilt without aiding](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#holding-tilt-without-aiding).
* **Local tangent plane.** Position is Cartesian NED about a fixed origin. The geodetic
  conversion is exact at any range
  ([equation (43)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#geodetic-origin)),
  but a plane leaves a curved Earth: `d` from the origin it sits `d²/2R` above the surface, 8 cm
  at 1 km and 7.8 m at 10 km, so `-p_D` far out is not height. The barometer model does not
  correct for it, and [equation (30)](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#barometric-altitude)
  records the measurement behind that.

Features out of scope (wind, terrain, optical flow, airspeed, ...) are listed in
[GOALS.md](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#non-goals).
