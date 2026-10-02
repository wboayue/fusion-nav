# Glossary

Terms this repository uses without explaining, for a reader who has not worked on estimators
before. Each entry says what the word means and points at the document that owns the thing:
the mathematics is [EQUATIONS.md](EQUATIONS.md), the structure [DESIGN.md](DESIGN.md), the
positioning [GOALS.md](GOALS.md), the harness [data/README.md](data/README.md). Nothing here is
normative: an entry that disagrees with one of those is wrong.

It defines this crate's vocabulary rather than the field's. The exception is the last section,
for a reader arriving from PX4 or ArduPilot, where their word and ours name different things and
the difference is the kind that costs a day.

## Frames and quantities

* **Navigation frame**, **body frame**: the two coordinate systems every vector belongs to.
  The navigation frame is fixed to the earth at the origin; the body frame is bolted to the
  vehicle and turns with it. A number without a frame is not a quantity, which is why
  `src/units.rs` puts the frame in the type.
* **NED**, **ENU**: North-East-Down and East-North-Up, two conventions for the navigation
  frame. This crate is NED, so **down is positive** and gravity has a positive `z`.
  ROS and most geographic software are ENU. See
  [conventions](EQUATIONS.md#notation-and-conventions).
* **FRD**, **FLU**: Forward-Right-Down and Forward-Left-Up, the same disagreement for the body
  frame. This crate is FRD; many IMU breakout boards are FLU.
* **Attitude**: the vehicle's orientation: the rotation that carries a body-frame vector into
  the navigation frame. Equivalently roll, pitch and yaw, or a quaternion, or a 3 × 3 rotation
  matrix.
* **Roll, pitch, yaw**: rotations about the forward, right and down axes. **Tilt** is roll and
  pitch together, the part gravity can measure; **heading** is yaw, the part it cannot. Their
  uncertainties are about **navigation** axes (tilt about north and east, heading about down),
  so they are the body-axis attitude variances only near level; `AttitudeVariance` in
  `src/state.rs` owns the difference.
* **Quaternion**: four numbers representing a rotation, used instead of Euler angles because
  they have no gimbal lock and compose cheaply. The cost is conventions that look alike and are
  not. **Hamilton** vs JPL is settled here as Hamilton, and **scalar-first** vs scalar-last
  storage is not settled at all: `Quaternion`'s fields are named `w`, `x`, `y`, `z`, so a caller
  writes the order down rather than assuming one. What `Attitude`'s constructors name is the
  frame and the direction (`from_body_to_ned`, `from_ned_to_body`, `from_flu_to_enu`,
  `from_flu_to_nwu`), because a stored inverse or an ENU quaternion taken as body-to-NED produces
  a filter that runs and reports health while flying an attitude that is, in the level case, a
  half turn out.
* **Specific force**: what an accelerometer actually measures: acceleration minus gravity, in
  body axes. A stationary level vehicle reads `[0, 0, −γ]`, not zero, which is what makes
  levelling from the accelerometer possible at all. See
  [initialization](EQUATIONS.md#initialization).
* **Levelling**, **alignment**: recovering the initial attitude before the filter can run.
  Levelling is the tilt half, from gravity; alignment is the whole job, including heading.
  A **static** (quasi-stationary) window gives the good answer, a **coarse** one the usable
  answer with the uncertainty to match. See
  [alignment beyond the static window](GOALS.md#alignment-beyond-the-static-window).
* **Declination**: the angle between magnetic north, which a magnetometer measures, and true
  north, which the navigation frame uses. It varies by location and by year. Read from a
  **WMM** (World Magnetic Model) table at the origin unless the caller sets it,
  [the decision](GOALS.md#magnetic-declination-from-a-table-read-where-the-origin-is-placed).
* **Barometric reference**, `α₀`: the barometric altitude that corresponds to the navigation
  origin, so a barometer reading becomes a height. Set by a still start or read from the estimate, then
  estimated as an **offset** beside the covariance, since a barometer drifts with the weather,
  [equation (30′)](EQUATIONS.md#barometric-offset).
* **Course**, **sideslip**: course is the direction of the horizontal velocity, where the
  vehicle is going; heading is where its nose points; sideslip is the angle between them. A
  fixed-wing in coordinated flight holds sideslip near zero, a crosswind adds a crab angle, and a
  multirotor has no relation between the two at all. The course constraint is equation (35″).
* **Dual-antenna heading**, **moving baseline**: a GNSS receiver with two antennas measuring the
  direction of the line between them, a true heading independent of any magnetic field.
  Equation (35′).
* **Geodetic coordinates**, **ECEF**, **local tangent plane**: latitude/longitude/height on the
  WGS-84 ellipsoid; an earth-centred Cartesian frame; and the flat NED frame this filter works
  in, pinned to a geodetic **origin**. Converting between them is
  [equations (43)–(44)](EQUATIONS.md#geodetic-origin); the flat approximation's cost is in the
  README's limitations.

## The filter

* **Inertial navigation**, **dead reckoning**: integrating gyroscope and accelerometer readings
  to carry position, velocity and attitude forward with no outside reference. It is exact for an
  instant and hopeless over a minute, because every error integrates: an attitude error tips
  gravity into the horizontal channel and integrates twice; on the simulator's `gnss_outage`,
  [20 s without GNSS](VALIDATION.md) is metres. The term is not the status:
  `Status::DeadReckoning` means no *horizontal* aiding, so a filter fusing only a barometer and a
  magnetometer reports it while still corrected in height and heading.
* **Kalman filter**: the recursive estimator underneath all of this: carry a state estimate and
  a covariance, **predict** both forward with a model, **correct** both when a measurement
  arrives, weighting the two by how much each claims to be trusted.
* **Extended Kalman filter (EKF)**: a Kalman filter on a nonlinear system, linearized about the
  current estimate at each step. `H` and `F` are that linearization.
* **Error-state Kalman filter (ESKF)**: the variant this crate implements. The **nominal
  state** (16 values) integrates the IMU directly and carries no covariance. The **error state**
  (15 values) is the small difference between the nominal state and the truth, and *that* is
  what the Kalman filter estimates. Attitude is why: a three-component attitude error keeps the
  covariance non-singular while the quaternion keeps its unit norm. See
  [state definitions](EQUATIONS.md#state-definitions) and
  [DESIGN.md](DESIGN.md#error-state-kalman-filter).
* **Predict**, **propagate**: carry the state and its covariance forward by one IMU step: the
  nominal state by [equations (9)–(15)](EQUATIONS.md#nominal-state-propagation), the covariance
  by [(20)–(22)](EQUATIONS.md#covariance-propagation), through the
  [error-state dynamics](EQUATIONS.md#error-state-dynamics) of (16)–(19). The estimate gets worse
  and the covariance says so.
* **Update**, **fuse**, **correct**: fold one measurement in,
  [equations (23)–(27)](EQUATIONS.md#measurement-update).
* **Aiding**: any measurement from outside the IMU that constrains the drift: GNSS, barometer,
  magnetometer. A filter that is *aided* is being corrected; an *unaided* one is dead reckoning,
  whatever its covariance looked like a second ago.
* **Coast**: carrying the state across a gap in the IMU stream longer than
  `Config::max_predict_dt`, which no single sample can describe. The filter assumes no
  acceleration and no rotation over the gap and prices that assumption into the covariance,
  [equation (22′)](EQUATIONS.md#coasting-across-a-gap); `Propagation::Coasted` reports it.
* **Injection and reset**: the ESKF's extra step: the estimated error is added into the nominal
  state, then the error state is zeroed and the covariance rotated by the **reset Jacobian**
  `G`. This is why the error state's prior is always zero.
  [Equations (39)–(41)](EQUATIONS.md#error-injection-and-reset).
* **Bias**: the slowly varying offset an inertial sensor adds to every reading. Estimated here
  as six states, three per sensor, because an unestimated gyroscope bias is an attitude error
  that grows linearly and then a position error that grows cubically. **Drift** is what the
  *solution* does when a bias is not estimated; the bias is the cause, the drift the symptom.
* **Random walk**: the model for how a bias moves: the integral of white noise, so its
  uncertainty grows linearly with time rather than staying put. `ImuNoise::gyro_bias_walk` and
  `accel_bias_walk` are its strength. Measuring it honestly takes an **Allan variance** soak of
  several hours, which is why `GOALS.md` lists it as a number the user supplies.
* **Observability**: whether the available measurements can actually determine a state. It is
  not about noise: a stationary vehicle never observes yaw at all, however long it sits, and an
  accelerometer bias is not separable from tilt at rest. This is why heading waits for a
  magnetometer and why a coarse start marks position and velocity `Unestablished` rather than
  trusting a prior.

## Uncertainty

* **Covariance**, `P`: the filter's own account of how wrong it might be: a 15 × 15 matrix whose
  diagonal holds each state's variance and whose off-diagonal terms hold the correlations. The
  correlations are what let a GNSS position fix correct velocity and attitude.
* **Variance**, **σ (sigma)**: squared spread and spread. A state's σ is the square root of its
  diagonal entry in `P`; **3σ** is the interval a Gaussian falls inside 99.7 % of the time.
* **Three kinds of noise number, easily confused**: `Q`, the **process noise**, which is how
  much uncertainty propagation adds per step (`Config::imu`); `R`, the **measurement noise**,
  which is how bad a *particular* measurement is and therefore arrives as an argument to each
  `fuse_*` rather than living in `Config`; and `P0`, the initial covariance
  (`Initialization::sigma_*`), a prior on the state and not a property of any sensor. `P` is none
  of the three: it is the estimate's own uncertainty, which the other three set and move.
* **Noise density**, **spectral density**: the units `ImuNoise` states, `rad s⁻¹/√Hz` and
  friends. They look odd because the noise is continuous-time: variance accumulates linearly with
  time, so the σ over an interval `Δt` is the density times `√Δt`. Doubling the sample rate does
  not double the drift.
* **Noise floor**, **window noise**: the white noise a still initialization window measured on
  each sensor, as `StaticWindow::noise` reports it (`WindowNoise`), by
  [equation (8″)](EQUATIONS.md#what-a-still-window-measures-of-its-sensors): a lower bound on `Q`
  and `R`, since a vehicle on the ground is quieter than in flight, with no airflow and vibration
  only from motors at idle. Not the **floor** of
  [equation (42′)](EQUATIONS.md#numerical-conditioning), which bounds a variance in `P` from below
  to keep it a covariance.
* **Consider state**: a quantity carried in the covariance whose uncertainty is priced but which
  is never corrected. Measured and rejected for the barometric reference in favour of an
  estimated one, [the decision](GOALS.md#barometric-reference-as-an-estimated-offset).
* **Kalman gain**, `K`: how much of a measurement's disagreement to believe, set by the ratio of
  the filter's uncertainty to the total. Confident filter, ignored measurement; uncertain filter,
  adopted measurement.
* **Joseph form**: the algebraically equivalent but numerically stabler way of writing the
  covariance update, [equation (27)](EQUATIONS.md#measurement-update). It costs more arithmetic
  and keeps `P` symmetric and positive definite in `f32`, which the short form does not.
* **Positive definite**, **symmetry enforcement**: a covariance must be symmetric with positive
  variances, or it is not a covariance. Rounding erodes both, so
  [equation (42)](EQUATIONS.md#numerical-conditioning) re-symmetrizes after every operation and
  (42′) floors the variances.
* **Overconfident**, **conservative**: a covariance smaller than the true error, or larger.
  Overconfidence is the dangerous direction twice over: it makes the filter ignore the
  measurements that would fix it (see **gate lockout**) and it makes `Validity` vouch for an
  output that is not good enough.
* **Marginal** vs **joint**: a test on one axis at a time, reading only `P`'s diagonal, against
  a test on a whole block, reading its correlations. `in3s` and `nees_*` below are the same
  distinction; a covariance with the right variances and the wrong correlations passes the first
  and fails the second.

## Measurement updates and gating

* **Observation model**, `h(x)`: what the filter expects a sensor to read given its current
  state. **`H`** is that model's Jacobian: the matrix saying how a small error in each state
  moves the prediction. [Observation models](EQUATIONS.md#observation-models).
* **Innovation**, `y` (also **residual**): measured minus predicted, `z − h(x̂)`,
  [equation (23)](EQUATIONS.md#measurement-update). `EQUATIONS.md` and the code write `y`; the
  replay harness's per-fusion columns write `ν`, the other common spelling for the same quantity.
  It is the only thing a measurement ever tells the filter, and the only quantity available for
  checking a filter that has no truth to compare against.
* **Innovation covariance**, `S`: how large the innovation should be if both the filter and the
  sensor are telling the truth: `H P Hᵀ + R`. The filter's uncertainty and the sensor's, added.
* **Mahalanobis distance**: distance measured in σ rather than in metres, `yᵀ S⁻¹ y` under the
  square root. It is what makes "is 3 m a lot?" answerable: it depends on `S`.
* **NIS**, normalized innovation squared: that distance squared, `ε = yᵀ S⁻¹ y`,
  [equation (37)](EQUATIONS.md#innovation-gating). Under the hypothesis that the filter and the
  sensor are both honest it is **chi-square** distributed with `dim(z)` **degrees of freedom**,
  which is what turns it into a test.
* **Gate**, **gating**: rejecting a measurement whose `ε` exceeds a threshold `γ` taken from
  that chi-square distribution at a chosen **percentile**. A 99 % gate rejects one good
  measurement in a hundred by construction; that is the price of catching the bad ones. The
  default is `Percentile::P999`, 99.9 %; [its evidence](DESIGN.md#gates).
* **Test ratio**, `r = ε / γ`: the gate's verdict as one dimensionless number, so `r > 1` means
  rejected whatever the observation's dimension, and GNSS position, barometer and heading are
  comparable on one scale, [equation (38)](EQUATIONS.md#innovation-gating). PX4 logs a quantity
  of the same name for a different construction; see **innovation test ratio** below.
* **Gate lockout**: the failure mode gating creates. If the *filter* is wrong rather than the
  measurement, every correct measurement looks inconsistent, all of them are rejected, and the
  filter dead-reckons while reporting confidence. This crate reports it and, past a per-source
  timeout, recovers by **adoption**; see [gate lockout](EQUATIONS.md#gate-lockout) and
  [rejection handling](GOALS.md#rejection-handling-recover-by-default-opt-out-per-source).
* **Adoption**: taking a measurement as the state outright instead of fusing it, the
  zero-information limit of the update. Used once for a quantity the start never established,
  and again for a source locked out past its `Config::recovery` timeout.
* **Recovery**: adoption after gate lockout, per source, on the timeouts `Config::recovery` fixes
  in advance. `Recovery::OFF` turns it off and leaves the decision to the application,
  [rejection handling](GOALS.md#rejection-handling-recover-by-default-opt-out-per-source).
* **Latency**: the age of a measurement when it is fused. GNSS solutions are 100–200 ms stale.
  Each `fuse_*` takes the time the measurement was taken and fuses it against the state as it was
  then, [equation (23′)](EQUATIONS.md#delayed-measurements); knowing the latency is the caller's,
  which is in the README's limitations.
* **History**, **horizon**: the recent past of the nominal state that (23′) reads, 32 entries
  10 ms apart. A measurement older than `LATENCY_HORIZON` (0.3 s) is past it and returns
  `Fusion::OutOfHorizon` rather than being fused against a guess.
* **Correlation time**, `τ`, **equivalent white noise**: how long a sensor's error persists.
  Readings closer together than `τ` share most of their error, so fusing each as independent
  would average away an error that is still there. (24′) fuses them with the variance of the
  white noise that would carry the same information, larger than the reading's own `R`;
  `Config::correlation` holds one `τ` per source.
  [Correlated measurements](EQUATIONS.md#correlated-measurements).
* **Lever arm**, **antenna offset**: where the GNSS antenna sits relative to the IMU, in body
  axes. A rotating vehicle moves its antenna even when the IMU is still, so the offset enters
  `H`, [equations (28′) and (29′)](EQUATIONS.md#gnss-position). Passed per call, since a second
  receiver has its own, [the decision](GOALS.md#sensor-offsets-as-per-call-arguments).

## Scoring a run

* **Ground truth**: the true trajectory, known only where it was generated or surveyed. The PX4
  corpus has none, which is why `examples/simulate.rs` exists. See
  [three questions](GOALS.md#three-questions-three-kinds-of-source).
* **Consistency** vs **accuracy**: two different questions, and the distinction the validation
  plan is built on. Consistency asks whether the filter's errors match the uncertainty it
  claims, and needs no truth. Accuracy asks how close it is, and needs truth. A filter can be
  consistent and inaccurate, or accurate and overconfident; a benchmark that collapses the two
  certifies something it never tested.
* **RMSE**: root-mean-square error, the score keys `pos_h`, `pos_v` and `vel`. An average, so it
  hides excursions, which is why `pos_h_max` is pinned beside it.
* **NEES**, normalized estimation error squared: the Mahalanobis distance of the *error* against
  `P`, which needs truth. Divided by its degrees of freedom it should average **1**: above 1 the
  filter is overconfident, below 1 conservative. The `nees_pos`, `nees_vel` and `nees_att` keys.
* **ANEES**: NEES averaged over many independent runs at each epoch, compared against a
  chi-square bound. One run's epochs share their error, so no bound can be put on its average;
  N independent runs at one epoch can. `data/anees.sh` flies each scenario on 50 seeds and
  `data/anees.txt` holds the bounds. See [the covariance's own
  promise](data/README.md#the-covariances-own-promise).
* **`in3s`**: the fraction of axis-epochs where the error sat inside 3σ, over all 15 states. The
  marginal companion to `nees_*`, and the only key that reaches the bias states.
* **`false_valid`**: how often the filter said an output was usable while the truth error was
  outside the accuracy the mission asked for. It reads the filter's own verdict and falsifies it
  in the shape the filter states it, per axis; re-deriving either half would test a copy of the
  claim. See [scoring against truth](data/README.md#scoring-against-truth).
* **Epoch**: one IMU sample time, and the row unit of the replay output. Every score above is a
  mean over the epochs that had a truth row (`scored`).
* **Ceiling**: a measured bound per score key per scenario in `data/scenarios.txt`, asserted in
  CI. It ratchets in both directions and needs a sentence when it moves. What it cannot catch is
  a filter that got more accurate and more overconfident at once, which is ANEES's job. See
  [ceilings](data/README.md#ceilings-and-what-they-gate).
* **Corpus**, **manifest**, **scenario**: the data and its bookkeeping. A corpus is a body of
  flights: real PX4 logs, fetched rather than committed, or the seeded flights
  `examples/simulate.rs` generates with truth beside them. The manifest pins each fetched log by
  checksum *and* by the output replaying it must produce. A scenario is one generated flight, and
  exists only if it covers something no other one does.
* **RTK**, **SITL**: real-time kinematic GNSS, centimetre fixes from carrier phase against a base
  station; and software in the loop, a PX4 build flying a simulated vehicle. A SITL log in the
  corpus is synthetic data without the simulator's truth.

## Words this crate uses in a particular way

* **`Status`**: one enum answering *how bad is the worst thing*, most-severe-first:
  `DeadReckoning` > `Aligning` > `Degraded` > `Healthy`.
* **`Validity`**: six per-quantity flags answering *which outputs can I use*, derived from `P`
  against `Config::accuracy`. `Status` is the summary, `Validity` the detail, and neither
  substitutes for the other; the reasoning is
  [per-quantity validity](GOALS.md#per-quantity-validity-not-one-ladder).
* **`Accuracy`**: what the *mission* needs from each output, and the one knob the filter cannot
  derive for the caller. It moves `Validity` and nothing else.
* **Aligning**: the filter is running, but its attitude has not converged: a coarse start, a
  vague seed, or a heading no source has observed yet. It latches: once resolved it never returns, because read live it flaps.
* **Seed**: a start from an estimate the application already holds (`Eskf::initialize_from`),
  rather than from a window. It vouches for every quantity, so no first measurement is adopted
  after it; recovery from gate lockout still is.
* **Unestablished**: a quantity that was never observed, as distinct from one that has gone
  stale. A prior is not an estimate.
* **Differentiator**: one of the commitments in
  [GOALS.md](GOALS.md#differentiators) that set this crate apart, cited by number (1–4, 6, 7).
* **Stub**: a function whose signature and doc comment exist while its mathematics does not.
  Marked with a `**Stub.**` paragraph, and the marker is kept accurate.

## Coming from PX4 or ArduPilot

Where the two estimators this crate is measured against use a word differently. Source citations
are `file:line` at PX4 `c4e4ef98e9` and ArduPilot `368dc0c428`; where the claim belongs to another
document, the entry points there instead of repeating it.

* **Fusion time horizon**, **output predictor**: PX4 fuses at a *delayed* horizon and runs a
  separate fast predictor forward to the present
  (`src/modules/ekf2/EKF/output_predictor/output_predictor.h:54-59`), which is how it absorbs
  sensor latency. This filter has neither: it runs at the present and fuses each measurement
  against a history of the state at the time it was taken, equation (23′) of `EQUATIONS.md`,
  so a verdict is returned by the call that offered it. See
  [measurement latency](GOALS.md#measurement-latency).
* **Reset**: the near-miss. There, a reset is *recovery*: states are set to a measurement after an
  aiding timeout, and a counter is published so consumers can step their own state
  (`xy_reset_counter` and friends, `msg/versioned/VehicleLocalPosition.msg`). Here, `Fusion::Reset`
  is **adoption**, and recovery is only one of its two uses: the other is the first measurement of
  a quantity initialization never established (position and velocity after a coarse start,
  heading wherever the window observed none), which PX4 does as it starts fusing a source rather
  than as a reset. `SourceHealth::recovered` counts the first kind apart
  ([rejection handling](GOALS.md#rejection-handling-recover-by-default-opt-out-per-source)).
* **Innovation test ratio**: the same name and nearly the same number. Both test per component
  against the diagonal of `S` and differ in how they group the components: PX4 each axis at 5σ,
  ArduPilot the horizontal pair as one sum against the summed variances and the vertical alone.
  This crate's is joint over the whole observation, which is what makes a percentile mean what it
  names, except for a GNSS fix, whose horizontal pair and height are two observations, as they are
  in both platforms. `Gates`'s doc comment owns the comparison and its citations.
* **`filter_control_status`**, **`nav_filter_status`**: their per-quantity validity bits. The
  counterpart is `Validity`, and `Eskf::predicted_validity` answers ArduPilot's
  `pred_horiz_pos_rel` question. `Status` is *not* the counterpart: it is a one-glance severity
  summary with no equivalent there. See
  [per-quantity validity](GOALS.md#per-quantity-validity-not-one-ladder).
* **Tilt align, yaw align**: their alignment flags, latched as this crate latches
  `Status::Aligning`. `ALIGNED_TILT` takes the stricter of the two tilt-variance bars they
  publish. `ALIGNED_HEADING` has none to take: both latch yaw on the magnetometer reset rather
  than on a variance, so its doc comment says what the 30° is chosen against instead.
* **GSF yaw estimator**: a Gaussian Sum Filter recovering yaw from IMU and GNSS velocity, which
  is how both fly without a magnetometer. Not built here: #165; the README's limitations say
  what its absence costs a multirotor.
* **Lane**, **core**: ArduPilot runs several EKF3 instances on different IMUs and switches
  between them on relative error (`libraries/AP_NavEKF3/AP_NavEKF3.h:329-337`). `Eskf` is one
  instance and does no such selection; running several and choosing is the application's.
* **Magnetic field states**: both can estimate earth- and body-frame field states, selected by
  ArduPilot's `EK3_MAG_CAL` (`libraries/AP_NavEKF3/AP_NavEKF3.cpp:275-281`) and PX4's
  `EKF2_MAG_TYPE` (`src/modules/ekf2/params_magnetometer.yaml:5-23`), both of which choose
  between heading fusion and the three-component fusion that learns the field. This crate fuses
  heading only and estimates no field states, which makes magnetometer calibration a precondition
  rather than something the filter learns. See
  [the decision](GOALS.md#magnetometer-without-magnetic-field-states).
* **Barometer bias**: both track the reference rather than fixing it, and neither carries it in
  the EKF's state vector: PX4 runs a dedicated one-state estimator per height source, ArduPilot
  slews a `baroHgtOffset` outside the covariance. Here the error in `α₀` is estimated inside
  the covariance, as an offset appended to the error state for the update only, so it is not a
  component of `State`. See [the decision](GOALS.md#barometric-reference-as-an-estimated-offset).
* **`EKF2_*`, `EK3_*` parameters**: dozens of tunables, most describing the hardware rather than
  the mission. `Config` is deliberately small, and the ambition is smaller still:
  [configuration derived, not demanded](GOALS.md#7-configuration-derived-not-demanded).
