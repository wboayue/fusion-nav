//! Filter tuning.
//!
//! Every default here is a **placeholder** chosen to make the shape of the API concrete,
//! and none has been validated against flight data — with three exceptions:
//! [`Timeouts::degraded_after`] and [`Initialization`]'s stationarity tolerances, both
//! read off the PX4 replay corpus, and [`ImuNoise`], which follows the defaults PX4 and
//! ArduPilot ship.

use crate::units::{Meters, MetersPerSecond, MetersPerSecond2, Radians, RadiansPerSecond, Seconds};

/// Standard gravity, m s⁻². The `γ` of equations (5) and (11).
///
/// The WGS-84 standard value, and a constant rather than a derived or configured one. Local
/// gravity varies by about 0.5 % between the equator and the poles, and the filter holds a
/// geodetic origin whose latitude would give it — but the origin is placed by the first GNSS
/// fix, which can arrive after propagation has begun, and a propagation constant that changes
/// mid-flight is the self-retuning `GOALS.md`'s differentiator 7 rules out. The derivation
/// belongs to the offline tool that prints a `Config` from a log (#51).
///
/// What the error costs: 0.03 m s⁻² at the equator, entering (11) as a systematic vertical
/// specific-force error. The accelerometer bias state absorbs a constant offset, so it shows
/// up as a bias estimate wrong by that much rather than as vertical drift.
pub const GRAVITY: f32 = 9.806_65;

/// Tilt uncertainty, per axis, at which a start counts as resolved: the bar behind
/// [`Eskf::is_aligned`](crate::Eskf::is_aligned) and so
/// [`Status::Aligning`](crate::Status::Aligning).
///
/// A constant rather than a field of [`Accuracy`], because it answers a different question.
/// [`Accuracy`] is what the mission needs from an output; this is whether the start has
/// produced an attitude the filter can work from, and no mission changes that. Both
/// production estimators answer it with an internal constant nobody configures: PX4 declares
/// tilt alignment at 3° (`getTiltVariance() < sq(radians(3.f))`,
/// `src/modules/ekf2/EKF/control.cpp:73-78` at `c4e4ef98e9`), ArduPilot at 5°
/// (`tiltErrorVariance < sq(radians(5.0))`,
/// `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp:520-525` at `368dc0c428`). This takes the
/// stricter of the two.
pub const ALIGNED_TILT: Radians = Radians::from_degrees(3.0);

/// Heading uncertainty at which a start counts as resolved, once a heading has been
/// established at all: the yaw half of [`ALIGNED_TILT`]'s test.
///
/// No published counterpart exists to cite. Both estimators latch yaw alignment on the
/// magnetometer reset rather than comparing a variance; the only yaw-variance bar either
/// publishes is 15°, for accepting the GSF yaw estimator (`EKFGSF_yaw_err_max`,
/// `src/modules/ekf2/EKF/common.h:396` at `c4e4ef98e9`; `GSF_YAW_ACCURACY_THRESHOLD_DEG`,
/// `libraries/AP_NavEKF3/AP_NavEKF3_core.h:78` at `368dc0c428`), and a static start's own
/// prior, [`Initialization::sigma_yaw`] at 20°, would fail it.
///
/// So 30° is chosen under one hard constraint: it has to clear `sigma_yaw`. A caller who
/// configures `sigma_yaw` above it gets a static start that never leaves
/// [`Aligning`](crate::Status::Aligning) until a heading measurement brings yaw down, which
/// the type system cannot refuse.
pub const ALIGNED_HEADING: Radians = Radians::from_degrees(30.0);

/// IMU noise, as the continuous-time densities of equations (16)–(21).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuNoise {
    /// Gyroscope white noise, rad s⁻¹ / √Hz.
    pub gyro_white: f32,
    /// Accelerometer white noise, m s⁻² / √Hz.
    pub accel_white: f32,
    /// Gyroscope bias random walk, rad s⁻² / √Hz.
    pub gyro_bias_walk: f32,
    /// Accelerometer bias random walk, m s⁻³ / √Hz.
    pub accel_bias_walk: f32,
}

impl Default for ImuNoise {
    /// PX4 EKF2's defaults, which ArduPilot's EK3 independently agrees with to within a
    /// factor of two on every term.
    ///
    /// These are deliberately far above what an IMU datasheet or an Allan variance plot
    /// gives for the sensor alone, because the process noise of a real airframe absorbs
    /// what the model leaves out: vibration, scale-factor and cross-axis error, timing
    /// jitter, and the coning and sculling a first-order propagation does not capture.
    ///
    /// Datasheet-grade figures — 10 to 15 times tighter on every term — would make the
    /// covariance claim a precision the estimate does not have, and an overconfident
    /// covariance gates out measurements that were fine. Two independent production
    /// estimators agreeing is not the same evidence as a replay of our own, so these
    /// remain subject to the validation in `GOALS.md`, but they are the right order of
    /// magnitude to start from.
    fn default() -> Self {
        Self {
            gyro_white: 1.5e-2,
            accel_white: 3.5e-1,
            gyro_bias_walk: 1.0e-3,
            accel_bias_walk: 1.0e-2,
        }
    }
}

/// How often a gate rejects a measurement that was fine: the percentile of the chi-square
/// distribution its threshold sits at.
///
/// Named rather than continuous, because each name is an exact tabulated quantile and a
/// continuous `p` would need an inverse normal CDF and the Wilson–Hilferty approximation at
/// three degrees of freedom — code a reader has to check, for percentiles nobody asks for.
/// `#[non_exhaustive]` because it is an input, constructed and rarely matched, so a new
/// percentile costs callers nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Percentile {
    /// 95 %: one good measurement in twenty rejected.
    P95,
    /// 99 %: one in a hundred.
    P99,
    /// 99.9 %: one in a thousand.
    P999,
}

/// Innovation gate threshold `γ` of equation (37), for an observation of `DOF` dimensions.
///
/// The degrees of freedom are a property of the observation rather than a choice, so they
/// are part of the type: a threshold for a one-dimensional observation cannot be written
/// into a three-dimensional source's field, and [`Gate::at`] exists only for the dimensions
/// this crate observes. The only free parameter left is the [`Percentile`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gate<const DOF: usize>(f32);

impl<const DOF: usize> Gate<DOF> {
    /// A threshold given as a number, for a caller with a reason to leave the [`Percentile`]
    /// table; `None` unless it is finite and strictly positive.
    ///
    /// Each refusal is a gate that fails silently. NaN compares false against every `ε` and
    /// accepts everything; `+∞` does the same while reporting a test ratio of zero; zero or
    /// less rejects everything, and zero makes the ratio of (38) a division by zero.
    pub const fn new(threshold: f32) -> Option<Self> {
        if threshold.is_finite() && threshold > 0.0 {
            Some(Self(threshold))
        } else {
            None
        }
    }

    /// The threshold `γ`, which a test ratio of 1 corresponds to.
    pub const fn threshold(self) -> f32 {
        self.0
    }
}

impl Gate<1> {
    /// The chi-square quantile at `percentile` for one degree of freedom: barometric
    /// altitude, GNSS height, magnetic heading.
    ///
    /// Checked in this module's tests against the closed-form CDF, not against another table.
    pub const fn at(percentile: Percentile) -> Self {
        Self(match percentile {
            Percentile::P95 => 3.841_459,
            Percentile::P99 => 6.634_897,
            Percentile::P999 => 10.827_566,
        })
    }
}

impl Gate<2> {
    /// The chi-square quantile at `percentile` for two degrees of freedom: the horizontal half
    /// of a GNSS position fix.
    ///
    /// `−2 ln(1 − p)` exactly, since two degrees of freedom is the one case with a closed-form
    /// CDF, `F(x) = 1 − e^(−x/2)`; checked against it in this module's tests.
    pub const fn at(percentile: Percentile) -> Self {
        Self(match percentile {
            Percentile::P95 => 5.991_465,
            Percentile::P99 => 9.210_34,
            Percentile::P999 => 13.815_511,
        })
    }
}

impl Gate<3> {
    /// The chi-square quantile at `percentile` for three degrees of freedom: GNSS velocity.
    ///
    /// Checked in this module's tests against the closed-form CDF, not against another table.
    pub const fn at(percentile: Percentile) -> Self {
        Self(match percentile {
            Percentile::P95 => 7.814_728,
            Percentile::P99 => 11.344_867,
            Percentile::P999 => 16.266_236,
        })
    }
}

/// Innovation gate thresholds `γ`, one per observation, from equation (37).
///
/// The filter reports `r = ε / γ`, so these set what `r = 1` means. Built from a percentile,
/// then adjusted per source where a caller has a reason:
///
/// ```
/// use fusion_nav::{Gate, Gates, Percentile};
///
/// let mut gates = Gates::at(Percentile::P99);
/// gates.baro_altitude = Gate::<1>::at(Percentile::P999); // a barometer that sees prop wash
/// ```
///
/// A threshold of the wrong dimension does not compile:
///
/// ```compile_fail
/// # use fusion_nav::{Gate, Gates, Percentile};
/// let mut gates = Gates::default();
/// gates.baro_altitude = Gate::<3>::at(Percentile::P99); // expected `Gate<1>`, found `Gate<3>`
/// ```
///
/// Every threshold is compared against the joint `ε = yᵀ S⁻¹ y` over all of the
/// observation's components, not one component at a time. Both production estimators test
/// per component against the diagonal of `S` only: PX4 each axis at 5σ
/// (`src/modules/ekf2/EKF/ekf.h:1142` at `c4e4ef98e9`), which fits its fusing the axes one by
/// one (`EKF/position_fusion.cpp:62-68`); ArduPilot the horizontal pair as one sum against the
/// summed variances and the vertical alone
/// (`libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp:904-906` and `:1023` at `368dc0c428`).
/// The joint test is what makes a [`Percentile`] mean what it names: only `ε` is chi-square
/// with `dim(z)` degrees of freedom, so only it rejects exactly `1 − p` of good measurements,
/// where a per-component test's rate depends on correlations nothing states. And it keeps
/// those correlations — a yaw error couples north and east, so the region `S` describes is an
/// ellipsoid rather than a box aligned with the navigation axes.
///
/// A GNSS position fix is the exception, split into a horizontal `Gate<2>` and a vertical
/// `Gate<1>` as ArduPilot splits it, because the joint test's cost is all or nothing: a
/// height the estimate disagrees with rejects a good horizontal fix. That cost is measured.
/// `2c42096b` is a stationary vehicle whose barometer and receiver drift ~20 m apart in
/// height; with both fused under one `Gate<3>` it rejects 3945 of its 4616 fixes, and the
/// horizontal pair alone fails a `Gate<2>` at [`Percentile::P999`] on none of them. The
/// split keeps the joint test within each half, so north and east are still tested as the
/// ellipse `S` describes; what it gives up is the north–down and east–down correlation, which
/// a position fix barely carries. See [`GnssFusion`](crate::GnssFusion).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gates {
    /// GNSS position, horizontal: north and east.
    pub gnss_position: Gate<2>,
    /// GNSS position, vertical: the height half of the same fix.
    pub gnss_height: Gate<1>,
    /// GNSS velocity.
    pub gnss_velocity: Gate<3>,
    /// Barometric altitude.
    pub baro_altitude: Gate<1>,
    /// Magnetic heading.
    pub mag_heading: Gate<1>,
}

impl Gates {
    /// Every source gated at the same percentile, each at its own degrees of freedom.
    pub const fn at(percentile: Percentile) -> Self {
        Self {
            gnss_position: Gate::<2>::at(percentile),
            gnss_height: Gate::<1>::at(percentile),
            gnss_velocity: Gate::<3>::at(percentile),
            baro_altitude: Gate::<1>::at(percentile),
            mag_heading: Gate::<1>::at(percentile),
        }
    }
}

impl Default for Gates {
    /// The 99.9th percentile: the tightest gate that costs nothing measurable on good data.
    ///
    /// GNSS position was replayed at 95 %, 99 %, 99.9 % and a 5σ equivalent (`γ` = 31.81 at
    /// three degrees of freedom, the two-sided tail of 5σ in one), tested as one joint
    /// three-axis `Gate<3>` rather than the split [`Gates`] describes. The figures below are
    /// that test's; the percentile applies to both halves of the split unchanged.
    ///
    /// The corpus cannot tell them apart. No log rejects a fix at any of the four, because
    /// PX4's `eph` and `epv` are far wider than the innovations they come with: the mean test
    /// ratio at 95 % is 0.0023–0.0449 across the three logs carrying GNSS and the largest is
    /// 0.33, where a consistent `R` would put the mean near 0.38.
    ///
    /// Those figures move as the filter gains aiding, and the direction is the one to expect:
    /// every source that tightens `P` tightens `S = H P Hᵀ + R`, so the same innovation reads
    /// as a larger ratio. They were 0.005–0.02 and 0.21 when position was the only gated
    /// source. `nis_gnss_pos=` on the `summary` line is the maintained form of this claim —
    /// per log, pinned in `data/manifest.txt`, and stated as a distribution rather than as a
    /// distance from a threshold that itself moves.
    ///
    /// The simulator, whose GNSS errors are exactly the Gaussian `R` claims, can, and it prices
    /// the tight end. On `mission`, 95 % rejects 30 of 915 good fixes and 99 % rejects 4,
    /// with horizontal RMSE 0.716 m and 0.702 m and vertical 0.796 m and 0.749 m. 99.9 %
    /// rejects 1 and 5σ none, and both give the same 0.702 m and 0.749 m that 99 % does. So
    /// 95 % buys worse accuracy, and one good fix in thirty turned away, for protection
    /// against outliers that nothing here contains.
    ///
    /// Between 99.9 % and 5σ the good data says nothing, and only hostile measurements can
    /// decide, which is #60's. 99.9 % stays the tighter of the two. It is still tighter than
    /// both production estimators, which test each axis at 5σ (PX4
    /// `src/modules/ekf2/EKF/common.h:348-374` at `c4e4ef98e9`; ArduPilot `*_I_GATE_DEFAULT
    /// 500`, in hundredths of σ, `libraries/AP_NavEKF3/AP_NavEKF3.cpp:34-36` at `368dc0c428`):
    /// a one-axis outlier fails `γ` = 16.27 at 4.0σ.
    fn default() -> Self {
        Self::at(Percentile::P999)
    }
}

/// How long a source may go unaccepted before the status degrades.
///
/// The filter reports and stops there; it does not reset itself. See
/// [gate lockout](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#gate-lockout).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Timeouts {
    /// Beyond this, a source counts as timed out and the status becomes
    /// [`Degraded`](crate::Status::Degraded).
    ///
    /// Must clear the slowest source's update period with margin, or ordinary jitter
    /// reads as a fault. Roughly two and a half missed updates from the slowest source
    /// is a reasonable rule.
    pub degraded_after: Seconds,
    /// Beyond this with no source accepted at all, the status becomes
    /// [`DeadReckoning`](crate::Status::DeadReckoning).
    pub dead_reckoning_after: Seconds,
}

impl Default for Timeouts {
    /// Sized for a 1 Hz GNSS, the slowest source in common use.
    ///
    /// Not the 1.0 s that period suggests: on a PX4 log whose fix intervals run
    /// 0.988-1.024 s, a 1.0 s threshold puts 37 of 122 fixes over it and flaps the status
    /// between `Healthy` and `Degraded` 76 times in 124 seconds. 2.5 s clears two missed
    /// fixes and still leaves half the window to
    /// [`dead_reckoning_after`](Timeouts::dead_reckoning_after).
    fn default() -> Self {
        Self {
            degraded_after: Seconds::from_secs(2.5),
            dead_reckoning_after: Seconds::from_secs(5.0),
        }
    }
}

/// Quasi-static initialization, equations (5)–(8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Initialization {
    /// How long the vehicle must have been still.
    ///
    /// A duration rather than a sample count, because the same count means very
    /// different things across IMU rates: 100 samples is 2 s at 50 Hz and 0.25 s at
    /// 400 Hz, and a quarter second is too short to average sensor noise down or to
    /// tell stillness from a slow drift. The five logs in `data/manifest.txt` alone span
    /// 50 Hz to 250 Hz.
    pub min_duration: Seconds,
    /// Largest angular rate magnitude still considered stationary.
    ///
    /// Compared against the **peak** over the window, not a filtered value, so at the
    /// same number this is the stricter test: one vibration spike is enough to fail it.
    pub max_gyro_rate: RadiansPerSecond,
    /// Largest departure of the specific-force magnitude from gravity still considered
    /// stationary. Peak over the window, as with
    /// [`max_gyro_rate`](Self::max_gyro_rate).
    pub max_accel_deviation: MetersPerSecond2,
    /// Initial position standard deviation.
    pub sigma_position: Meters,
    /// Initial velocity standard deviation.
    pub sigma_velocity: MetersPerSecond,
    /// Initial roll and pitch standard deviation. Gravity determines these well.
    pub sigma_tilt: Radians,
    /// Initial yaw standard deviation. Much larger than
    /// [`sigma_tilt`](Self::sigma_tilt): yaw inherits the magnetometer's calibration error.
    pub sigma_yaw: Radians,
    /// Initial accelerometer bias standard deviation.
    pub sigma_accel_bias: MetersPerSecond2,
    /// Initial gyroscope bias standard deviation.
    pub sigma_gyro_bias: RadiansPerSecond,
}

impl Default for Initialization {
    /// The stationarity tolerances are PX4 EKF2's — 15°/s and 20% of gravity — chosen
    /// against the replay corpus.
    ///
    /// Tighter ones fail parked vehicles: at 0.05 rad s⁻¹ and 0.5 m s⁻², four of the five
    /// logs in `data/manifest.txt` are called moving while sitting on the ground, on peaks
    /// of 0.026–0.172 rad s⁻¹ and 0.15–1.04 m s⁻² that are idle vibration and prop wash,
    /// not motion. A tolerance that calls a parked quadrotor moving does not protect the
    /// alignment, it just denies it. At these values four of the five align statically,
    /// and the fifth — peak deviation 6.2 m s⁻² — stays coarse, correctly: 6 m s⁻² is a
    /// vehicle being handled, not a vehicle vibrating.
    fn default() -> Self {
        Self {
            min_duration: Seconds::from_secs(2.0),
            max_gyro_rate: RadiansPerSecond::from_rad_per_s(0.262),
            max_accel_deviation: MetersPerSecond2::from_m_per_s2(1.961),
            sigma_position: Meters::from_meters(1.0),
            sigma_velocity: MetersPerSecond::from_m_per_s(0.1),
            sigma_tilt: Radians::from_radians(0.02),
            sigma_yaw: Radians::from_radians(0.35),
            sigma_accel_bias: MetersPerSecond2::from_m_per_s2(0.1),
            sigma_gyro_bias: RadiansPerSecond::from_rad_per_s(0.01),
        }
    }
}

/// How good an estimate has to be before the filter calls it valid.
///
/// The one group of numbers here that is **meant** to be supplied rather than derived: a
/// survey platform and a racing quadrotor disagree about what "good enough" means, and no
/// amount of flight data settles it. Everything else in [`Config`] is a property of the
/// hardware or the mathematics; this is a property of the mission.
///
/// Each is a standard deviation, compared against the covariance the filter carries. For
/// limits that differ between axes, read [`Eskf::covariance`](crate::Eskf::covariance)
/// directly — these are the coarse per-quantity bar.
///
/// Nothing here moves [`Status`](crate::Status). Whether the start has been resolved is
/// [`ALIGNED_TILT`] and [`ALIGNED_HEADING`]'s question, so a survey platform can ask for 1° of
/// roll without also waiting for 1° before the filter stops reporting
/// [`Aligning`](crate::Status::Aligning).
///
/// A bar tighter than the prior [`Initialization`] starts from is accepted and never met: that
/// quantity is invalid from the first epoch, while the filter aligns and reports
/// [`Healthy`](crate::Status::Healthy) as usual. On a replay it reads `attitude_lost=0.00` —
/// out of service the moment the start resolved — beside an `aligned_at=` that has not moved.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Accuracy {
    /// Roll and pitch.
    pub tilt: Radians,
    /// Heading.
    pub heading: Radians,
    /// Position, horizontally and vertically.
    pub position: Meters,
    /// Velocity, horizontally and vertically.
    pub velocity: MetersPerSecond,
    /// How far ahead [`Eskf::predicted_validity`](crate::Eskf::predicted_validity) asks:
    /// how long after arming the vehicle needs the estimate to still be good.
    ///
    /// The one number in this crate that no data could settle. Every other default here was
    /// measured or could be, by replaying what real vehicles do; this one is a statement
    /// about the mission ahead rather than about any sensor, and a multirotor that will be
    /// under a GNSS fix within a second disagrees completely with a fixed-wing hand-launched
    /// into a minute of dead reckoning.
    ///
    /// Read as a duration of *unaided* flight: the covariance is propagated this far with
    /// nothing fusing and each quantity tested at the far end. A value that is not a positive
    /// duration projects nothing, which leaves `predicted_validity` its other clause — the
    /// current answer widened by whatever source is being accepted — rather than reducing it
    /// to [`validity`](crate::Eskf::validity).
    ///
    /// Out past 6.4 s the projection takes longer steps rather than more of them and drifts
    /// further onto the optimistic side; `propagate.rs`'s `MAX_PROJECTION_STEPS` measures by
    /// how much.
    pub horizon: Seconds,
}

impl Default for Accuracy {
    /// Attitude clears the prior a static alignment starts from, by the margin an unaided
    /// filter takes to drift through. All four are **placeholders** for a mission nobody has
    /// named: tilt and heading are [`ALIGNED_TILT`] and [`ALIGNED_HEADING`] because a bar
    /// some estimator uses is a better starting point than none, and position and velocity
    /// are loose enough to admit a 1 Hz GNSS solution.
    ///
    /// Exactly those, not rounded near them. A mission bar a hair under the alignment bar
    /// opens a sliver where a start aligns and is out of service at the same epoch, for a
    /// reason nobody chose: 0.052 rad against 3°'s 0.0524 did exactly that.
    ///
    /// A bar equal to the prior it is compared against is no bar at all: with
    /// [`tilt`](Accuracy::tilt) at [`Initialization::sigma_tilt`](Initialization::sigma_tilt)
    /// exactly and [`heading`](Accuracy::heading) at
    /// [`Initialization::sigma_yaw`](Initialization::sigma_yaw) exactly, compared with `<=`, a
    /// static start passes by zero margin and the first `Q` of equation (21) takes it away
    /// again. On every static log in `data/manifest.txt` that read as a valid attitude for
    /// exactly one sample.
    ///
    /// What these buy, measured on a static start at [`ImuNoise`]'s defaults: 3.82 s of unaided
    /// propagation before tilt leaves the bar, 35.8 s before heading does
    /// (`an_unaided_start_holds_its_attitude_for_the_margin_the_defaults_buy`). Neither is the
    /// `σ_g² t` the white-noise density alone gives, which would be 10.4 s and 674 s — the
    /// gyroscope-bias prior enters attitude through equation (20)'s `−I Δt` and grows as
    /// `σ_βg² t²`, overtaking the white-noise term inside two seconds.
    ///
    /// What those two times move is [`Validity`](crate::Validity), and nothing else.
    /// [`Status`](crate::Status) is already answering on the aiding timers by then — an unaided
    /// filter reports [`DeadReckoning`](crate::Status::DeadReckoning) from
    /// [`Timeouts::dead_reckoning_after`], or from its first step if no source was ever accepted
    /// — and [`Aligning`](crate::Status::Aligning) reads the alignment bars rather than these.
    /// So these are a claim about which outputs a controller may still use, which is the
    /// question [`Accuracy`] exists to answer. Supply your own numbers.
    ///
    /// [`horizon`](Accuracy::horizon) is 1 s, and it is a placeholder in a stronger sense
    /// than the other four: they are bars some estimator uses, while no estimator publishes
    /// this one at all. A second is the shortest horizon that is not simply
    /// [`validity`](crate::Eskf::validity) asked twice — long enough that the gyroscope-bias
    /// term of (20) has started to tell on tilt, short enough that a vehicle expecting a
    /// GNSS fix on takeoff is not failed for the gap before it.
    fn default() -> Self {
        Self {
            tilt: ALIGNED_TILT,
            heading: ALIGNED_HEADING,
            position: Meters::from_meters(5.0),
            velocity: MetersPerSecond::from_m_per_s(1.0),
            horizon: Seconds::from_secs(1.0),
        }
    }
}

/// Everything the filter is tuned by.
///
/// Construct by updating the default:
///
/// ```
/// use fusion_nav::{Config, Seconds, Timeouts};
///
/// let config = Config {
///     timeouts: Timeouts {
///         degraded_after: Seconds::from_secs(1.5),  // a 5 Hz GNSS can be stricter
///         ..Timeouts::default()
///     },
///     ..Config::default()
/// };
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    /// IMU noise densities.
    pub imu: ImuNoise,
    /// Innovation gate thresholds.
    pub gates: Gates,
    /// Fusion timeouts feeding [`Status`](crate::Status).
    pub timeouts: Timeouts,
    /// Static initialization.
    pub init: Initialization,
    /// How good an estimate must be to count as valid.
    pub accuracy: Accuracy,
    /// Largest `dt` [`Eskf::predict`](crate::Eskf::predict) will propagate over.
    ///
    /// Beyond this the step is refused and the state left alone, because the
    /// discretization of equations (9)–(22) is a first-order approximation over a short
    /// interval and one IMU sample cannot describe a long one. The filter reports and
    /// stops there, as it does for a locked-out gate: whether to reset, coast, or abort
    /// is the application's call.
    ///
    /// Gaps come from logging dropouts, a scheduler overrun, or a sensor that genuinely
    /// stopped, and the filter cannot tell which. The default passes normal operation on
    /// every log in `data/manifest.txt` — IMU rates of 50 Hz to 250 Hz, whose longest
    /// interval short of a dropout is 90.5 ms, twice in the 2 h log's 1.4 M samples —
    /// while catching real SD-card dropouts of 0.34 s and up. The margin at that worst
    /// case is 10 ms, so a slower log than any in the corpus would need this raised.
    pub max_predict_dt: Seconds,
    /// Magnetic declination at the operating site, added to magnetic heading to give
    /// true heading. Equation (6).
    pub magnetic_declination: Radians,
}

impl Default for Config {
    // Written out rather than derived: `Seconds::default()` is zero, which would make
    // `max_predict_dt` refuse every step.
    fn default() -> Self {
        Self {
            imu: ImuNoise::default(),
            gates: Gates::default(),
            timeouts: Timeouts::default(),
            init: Initialization::default(),
            accuracy: Accuracy::default(),
            max_predict_dt: Seconds::from_secs(0.1),
            magnetic_declination: Radians::ZERO,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::ComplexField;

    /// The chi-square CDF at `x` for one or three degrees of freedom, by Simpson's rule.
    ///
    /// Substituting `x = t²` removes the one-dof density's singularity at zero and leaves the
    /// same constant for both: `F(x) = √(2/π) ∫₀^√x t^(k−1) e^(−t²/2) dt`. Computed rather than
    /// looked up, so the table is checked against the distribution and not against a copy.
    fn chi_square_cdf(dof: i32, x: f64) -> f64 {
        const STEPS: usize = 10_000;
        let upper = ComplexField::sqrt(x);
        let h = upper / STEPS as f64;
        let density = |t: f64| t.powi(dof - 1) * ComplexField::exp(-t * t / 2.0);
        let interior: f64 = (1..STEPS)
            .map(|i| density(i as f64 * h) * if i % 2 == 1 { 4.0 } else { 2.0 })
            .sum();
        let integral = h / 3.0 * (density(0.0) + interior + density(upper));
        ComplexField::sqrt(2.0 / core::f64::consts::PI) * integral
    }

    const PERCENTILES: [(Percentile, f64); 3] = [
        (Percentile::P95, 0.95),
        (Percentile::P99, 0.99),
        (Percentile::P999, 0.999),
    ];

    #[test]
    fn each_threshold_is_the_quantile_it_names() {
        // The three-significant-figure values this table replaced (7.81, 3.84) miss by 1e-4,
        // so the bound separates a quantile from a rounded one.
        for (percentile, p) in PERCENTILES {
            let one = Gate::<1>::at(percentile).threshold();
            let three = Gate::<3>::at(percentile).threshold();
            let two = Gate::<2>::at(percentile).threshold();
            let f1 = chi_square_cdf(1, f64::from(one));
            let f2 = 1.0 - ComplexField::exp(-f64::from(two) / 2.0);
            let f3 = chi_square_cdf(3, f64::from(three));
            assert!((f1 - p).abs() < 1e-6, "1 dof at {p}: F({one}) = {f1}");
            assert!((f2 - p).abs() < 1e-6, "2 dof at {p}: F({two}) = {f2}");
            assert!((f3 - p).abs() < 1e-6, "3 dof at {p}: F({three}) = {f3}");
        }
    }

    #[test]
    fn a_threshold_that_would_fail_silently_is_refused() {
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.0, -0.0, -1.0] {
            assert_eq!(Gate::<1>::new(bad), None, "{bad}");
        }
        assert_eq!(Gate::<3>::new(11.34).map(Gate::threshold), Some(11.34));
        assert_eq!(
            Gate::<1>::new(f32::MIN_POSITIVE).map(Gate::threshold),
            Some(f32::MIN_POSITIVE)
        );
    }

    #[test]
    fn the_default_is_the_999th_percentile() {
        assert_eq!(Gates::default(), Gates::at(Percentile::P999));
        assert_eq!(
            Gates::default().gnss_position,
            Gate::<2>::at(Percentile::P999)
        );
        assert_eq!(
            Gates::default().baro_altitude,
            Gate::<1>::at(Percentile::P999)
        );
    }
}
