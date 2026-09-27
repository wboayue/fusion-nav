//! Filter tuning.
//!
//! Every default here is a **placeholder** chosen to make the shape of the API concrete,
//! and none has been validated against flight data — with three exceptions:
//! [`SourceHealth::timeout`](crate::SourceHealth::timeout)'s missed-update count and
//! [`Initialization`]'s stationarity tolerances, both read off the PX4 replay corpus, and
//! [`ImuNoise`], which follows the defaults PX4 and ArduPilot ship.

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

/// How old a measurement may be and still be fused: the span of the past the filter can place
/// a measurement in. Older than this, a `fuse_*` refuses it as
/// [`Fusion::OutOfHorizon`](crate::Fusion::OutOfHorizon).
///
/// 0.3 s is what both production estimators hold. PX4 sizes its buffers to one and a half
/// times `EKF2_DELAY_MAX`, whose default is 200 ms
/// (`src/modules/ekf2/EKF/estimator_interface.cpp:591-592`, `src/modules/ekf2/module.yaml:26-34`
/// at `c4e4ef98e9`), and ArduPilot caps a receiver's lag at 250 ms, "the max value the EKF has
/// been tested for" (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:70-71` at `368dc0c428`). The
/// corpus's receivers are configured at 110 ms, one at 33.
pub const LATENCY_HORIZON: Seconds = Seconds::from_secs(0.3);

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
    /// White noise at ten times PX4 EKF2's density, measured to be what this filter needs;
    /// bias walks at ArduPilot EK3's density.
    ///
    /// These are deliberately far above what an IMU datasheet or an Allan variance plot
    /// gives for the sensor alone, because the process noise of a real airframe absorbs
    /// what the model leaves out: vibration, scale-factor and cross-axis error, timing
    /// jitter, and the coning and sculling a first-order propagation does not capture.
    /// Datasheet-grade figures would make the covariance claim a precision the estimate
    /// does not have, and an overconfident covariance gates out measurements that were fine.
    ///
    /// Both estimators add a bias walk as `(σ Δt)²` per prediction step, so their σ is
    /// per-step at their own rate and the density it implies is `σ √Δt`: PX4 at
    /// `src/modules/ekf2/EKF/covariance.cpp:152-153,166-167` with a 10 ms step
    /// (`module.yaml:22`, `EKF2_PREDICT_US`), at `c4e4ef98e9`; ArduPilot at
    /// `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1065,1071` with a 12 ms step
    /// (`AP_NavEKF3_core.cpp:36`, `EKF_TARGET_DT`), at `368dc0c428`, its bias states being
    /// `b Δt`. PX4's 1.0e-3 rad s⁻² and 3.0e-3 m s⁻³ (`params_gyro_bias.yaml:19`,
    /// `params_accel_bias.yaml:19`) come to 1.0e-4 and 3.0e-4; ArduPilot's copter 1.0e-3 and
    /// 2.0e-2 (`AP_NavEKF3.cpp:30-31`) to 1.1e-4 and 2.2e-3. These are ArduPilot's, the
    /// larger pair.
    ///
    /// What the conversion is worth: read unconverted as a density, the accelerometer-bias
    /// walk is 33 times PX4's in σ, and on `2c42096b` — grounded for two hours, where only
    /// `tilt · g + b_a` is observed — `σ_ba` grows to 0.63 m s⁻², the bias estimate walks to
    /// 0.56 and drags tilt from 0.95° to a peak of 5.2°, against EKF2's 1.08°. Converted, the
    /// ten-minute mean tilt holds between 0.89° and 1.11°, and the 2.5° peak is a 3.5 g knock
    /// at 4757 s. PX4's own pair gives 2.4°. A ceiling on `σ_ba` at PX4's 0.35 gives 3.3°
    /// and a clamp on the bias at its 0.4 gives 4.4°, and neither is reached once the walk
    /// is converted.
    ///
    /// The white noise is PX4's per-step σ taken as a density, which is ten times the density
    /// PX4's 1.5e-2 rad s⁻¹ and 0.35 m s⁻² stand for at its 10 ms step (`module.yaml:118-131`),
    /// and the factor is kept because replay measured it being spent. Scaling
    /// `accel_white` down breaks the one thing each source can say: at 0.3×, `2c42096b` — a
    /// real airframe vibrating on the ground — reads a tilt peak of 4.6° against 2.5° (EKF2
    /// never leaves 1.08°), and still 4.5° with PX4's `R` floors applied, so the vibration
    /// needs it; `a299e722` ends `Degraded`, rejecting 493 velocity solutions against 283,
    /// which PX4's floors cure (43, `Healthy`), so the receiver's raw `R` needs it.
    /// `gnss_latency` does not: with each fix fused at the time it was taken, (23′), it reads
    /// `false_valid` 0 at 0.3× as at 1×, where fused as current it read 888 at 0.3×. Scaling
    /// `gyro_white` down to 0.7× improves `tilt` and `yaw` on every scenario, but `harsh_imu`'s
    /// `nees_att` crosses 1 (1.07), and on the corpus `f16771dd` grows a 14.1° tilt at
    /// t = 51 s where EKF2 reads 2.6°.
    /// The simulator's IMU is 58 times quieter than this figure, so the scenarios favouring
    /// less gyroscope noise are the simulator's preference rather than an airframe's.
    ///
    /// So the factor stands for two things this filter does not model: the floors both
    /// estimators put under a receiver's reported accuracy (#105), and vibration, which PX4
    /// meets only by inflating accelerometer noise on clipping (`covariance.cpp:125-133`). A
    /// change that models either is the moment to measure this again.
    fn default() -> Self {
        Self {
            gyro_white: 1.5e-2,
            accel_white: 3.5e-1,
            gyro_bias_walk: 1.1e-4,
            accel_bias_walk: 2.2e-3,
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
    /// Dual-antenna GNSS heading.
    pub gnss_heading: Gate<1>,
    /// Course constraint.
    pub course: Gate<1>,
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
            gnss_heading: Gate::<1>::at(percentile),
            course: Gate::<1>::at(percentile),
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

/// How long the filter may go without horizontal aiding before the status reports it
/// dead-reckoning.
///
/// What the status *says*; what the filter *does* about a source it keeps rejecting is
/// [`Recovery`]'s. When a single source counts as timed out is not configured: each source's
/// own measured period sets it, [`SourceHealth::timeout`](crate::SourceHealth::timeout), capped
/// here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Timeouts {
    /// Beyond this with neither GNSS position nor GNSS velocity accepted, the status becomes
    /// [`DeadReckoning`](crate::Status::DeadReckoning), whatever else is still arriving. Also
    /// the longest any source's [`timeout`](crate::SourceHealth::timeout) runs.
    ///
    /// Horizontal, because that is the error nothing else bounds: a barometer holds height and
    /// a magnetometer heading, and position still drifts. PX4's `inertial_dead_reckoning` is
    /// the same test (`src/modules/ekf2/EKF/ekf_helper.cpp:804-903` at `c4e4ef98e9`), cleared
    /// only by horizontal position or velocity aiding, and ArduPilot's `dead_reckoning` flag
    /// reads horizontal sources alone (`libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp:811` at
    /// `368dc0c428`). A mission question rather than a measured one: how long a vehicle may
    /// navigate on its inertial solution is what it is flying for.
    pub dead_reckoning_after: Seconds,
}

impl Default for Timeouts {
    /// PX4's `EKF2_NOAID_TOUT`, the time it allows inertial dead reckoning before reporting
    /// the horizontal solution invalid (`src/modules/ekf2/EKF/common.h:519` at `c4e4ef98e9`).
    fn default() -> Self {
        Self {
            dead_reckoning_after: Seconds::from_secs(5.0),
        }
    }
}

/// How long each source's measurement error persists: the time constant `τ` of equation
/// (24′), per source. `None` fuses that source as white, which is (24) as written.
///
/// Equation (24) treats each measurement's error as independent of the last, and almost no
/// source in the corpus is. Fused as white, the lag-1 autocorrelation of every real log's
/// innovations (the `acf1_` keys of the replay `summary` line) is positive on GNSS position on
/// every log but the RTK receiver's, and on the barometer and magnetometer on nearly every
/// one. A filter that fuses such measurements as white averages down an error it cannot
/// observe, and its covariance claims the averaging worked. (24′) fuses each at the variance
/// that makes a run of them carry what they actually carry, which is less the shorter the
/// interval is against `τ`.
///
/// Inflating `R` rather than flooring `P`, and in the gain rather than the gate, is measured; the
/// [decision](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#correlated-measurement-error-as-equivalent-white-noise)
/// records what each alternative read.
///
/// Per source, like [`Recovery`] and [`Gates`], because a source is what the gate judges and
/// what (24′) times. Configured rather than derived, as [`ImuNoise`] is: it is a property of
/// the sensor, and the filter cannot measure it in flight without retuning itself. The replay
/// harness measures it offline, `τ = −T / ln ρ` at a log's sample interval `T` and its `acf1_`
/// value `ρ`. That reading is a lower bound: an innovation is whiter than the error behind it,
/// because the filter follows part of that error.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Correlation {
    /// GNSS horizontal position.
    pub gnss_position: Option<Seconds>,
    /// GNSS height.
    pub gnss_height: Option<Seconds>,
    /// GNSS velocity, all three axes.
    pub gnss_velocity: Option<Seconds>,
    /// Barometric altitude. The reference's drift is (30′)'s, not this: this is the noise
    /// about it.
    pub baro_altitude: Option<Seconds>,
    /// Magnetic heading, with the levelling variance of (36′) it carries.
    pub mag_heading: Option<Seconds>,
    /// Dual-antenna GNSS heading.
    pub gnss_heading: Option<Seconds>,
    /// Course constraint: the sideslip the constraint allows, which persists as long as the
    /// wind and the vehicle's trim do.
    pub course: Option<Seconds>,
}

impl Correlation {
    /// Every measurement independent of the last: equation (24) with nothing added.
    pub const WHITE: Self = Self {
        gnss_position: None,
        gnss_height: None,
        gnss_velocity: None,
        baro_altitude: None,
        mag_heading: None,
        gnss_heading: None,
        course: None,
    };
}

impl Default for Correlation {
    /// The corpus's: `τ = −T / ln ρ` from each real log's `acf1_` for the source, read with
    /// every measurement fused as white, at that log's own sample interval, and the median over
    /// the real logs whose autocorrelation is positive (the SITL log excluded, and the RTK
    /// log's GNSS, a receiver whose innovations alternate):
    ///
    /// | source | logs | range, s | median, s |
    /// | ------ | ---- | -------- | --------- |
    /// | GNSS horizontal position | 8 | 2.1–15.8 | 4.2 |
    /// | GNSS height | 8 | 3.7–70 | 14 |
    /// | GNSS velocity | 5 | 0.31–2.2 | 0.50 |
    /// | barometer | 10 | 0.006–4.2 | 0.20 |
    /// | magnetometer | 10 | 0.006–14.6 | 1.2 |
    /// | dual-antenna GNSS heading | 1 | 0.25 | 0.25 |
    /// | course constraint | 1 | 1.37 | 1.37 |
    ///
    /// Three receivers' velocity innovations and one magnetometer's alternate in sign, which no
    /// `τ` describes, and are left out rather than read as white.
    ///
    /// GNSS heading rests on one log, `a299e722`, the only one whose EKF2 fused a dual-antenna
    /// yaw: `acf1_gnss_yaw` 0.6696 at 10 Hz. The course constraint rests on one too,
    /// `093e806a`, the fixed-wing, replayed as a vehicle without a magnetometer
    /// (`--without mag --course 3`): `acf1_course` 0.4818 at 1 Hz, `τ` 1.37 s, and 0.4965 with
    /// its magnetometer, 1.43 s. What persists there is the sideslip, which the constraint cannot observe, and fused
    /// white the simulator's `no_mag` read a heading NEES of 2.7 on a sideslip it averaged as
    /// noise.
    fn default() -> Self {
        Self {
            gnss_position: Some(Seconds::from_secs(4.2)),
            gnss_height: Some(Seconds::from_secs(14.0)),
            gnss_velocity: Some(Seconds::from_secs(0.5)),
            baro_altitude: Some(Seconds::from_secs(0.2)),
            mag_heading: Some(Seconds::from_secs(1.2)),
            gnss_heading: Some(Seconds::from_secs(0.25)),
            course: Some(Seconds::from_secs(1.4)),
        }
    }
}

/// How long each source may be rejected before the filter adopts it again: automatic recovery from
/// [gate lockout](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#gate-lockout), one
/// switch per source. `None` turns that source's recovery off.
///
/// A source recovers when the gate rejects one of its measurements and none has been accepted
/// for at least this long — since initialization, for a source never accepted. That
/// measurement is then adopted rather than discarded: [`Fusion::Reset`](crate::Fusion::Reset),
/// counted in [`SourceHealth::recovered`](crate::SourceHealth::recovered). The state becomes
/// the measurement and its covariance block the measurement's, which is what undoes the
/// overconfidence that locked the gate rather than only moving the estimate. Only a rejection
/// triggers it: a source that is silent or refused has nothing to adopt, and a stream of NaN
/// must not step the state.
///
/// On by default, because a filter that reports lockout and stops hands every integrator the
/// same timeout-and-reset loop to write. An application that owns its failsafe policy — a
/// controller that cannot take a step, an operator alert instead of a reset — turns off the
/// source it owns, or all of them with [`Recovery::OFF`], and drives
/// [`Eskf::reset_position_to`](crate::Eskf::reset_position_to) itself. The decision is [rejection
/// handling](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#rejection-handling-recover-by-default-opt-out-per-source).
///
/// Per source, not per quantity, because a source is what the gate rejects: the fields mirror
/// [`Gates`] and [`Diagnostics`](crate::Diagnostics).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Recovery {
    /// GNSS horizontal position: north and east are adopted, height is left alone.
    pub gnss_position: Option<Seconds>,
    /// GNSS height: the down axis is adopted, and the barometric reference is dropped so that
    /// the next altitude reads it again against the adopted height — unless
    /// [`Config::baro_reference_from_estimate`] is off, which leaves the reference the
    /// caller's and keeps it.
    pub gnss_height: Option<Seconds>,
    /// GNSS velocity: all three axes are adopted.
    pub gnss_velocity: Option<Seconds>,
    /// Barometric altitude: the reference `α₀` is read again from the estimate, as the first
    /// one is when a start leaves none, and the state does not move. Needs an established
    /// position to read it against, and [`Config::baro_reference_from_estimate`] on: with it
    /// off the reference is the caller's, and a barometer that disagrees stays rejected.
    pub baro_altitude: Option<Seconds>,
    /// Magnetic heading: adopted as the first heading is, with the `R` of (36′) — but only
    /// while neither GNSS position nor velocity is fresh
    /// ([`SourceHealth::is_fresh`](crate::SourceHealth::is_fresh)), since with those arriving a magnetometer that disagrees
    /// for this long is more likely disturbed than right. Nor while a GNSS heading is
    /// accepted, for the same reason.
    pub mag_heading: Option<Seconds>,
    /// Dual-antenna GNSS heading: adopted as the first heading is, with the caller's `R`. No
    /// guard: it is the absolute heading reference when the vehicle carries one.
    pub gnss_heading: Option<Seconds>,
    /// Course constraint: adopted as the first heading is — but only while neither magnetic
    /// nor GNSS heading is fresh, since with either
    /// arriving a course that disagrees this long is sideslip the caller did not allow for,
    /// a crosswind or a multirotor crabbing, rather than a wrong heading.
    pub course: Option<Seconds>,
}

impl Recovery {
    /// No automatic recovery at all: the filter reports lockout and the application decides.
    pub const OFF: Self = Self {
        gnss_position: None,
        gnss_height: None,
        gnss_velocity: None,
        baro_altitude: None,
        mag_heading: None,
        gnss_heading: None,
        course: None,
    };
}

impl Default for Recovery {
    /// PX4's timeouts, at `c4e4ef98e9`: `reset_timeout_max`, 7 s, for horizontal position,
    /// velocity and heading, and `hgt_fusion_timeout_max`, 5 s, for height
    /// (`src/modules/ekf2/EKF/common.h:515-517`). Each is applied the way PX4 applies it —
    /// position at `aid_sources/gnss/gps_control.cpp:204-213`, velocity at `:146-156`, heading
    /// at `aid_sources/magnetometer/mag_control.cpp:276-289`, whose guard against fresh
    /// horizontal aiding [`mag_heading`](Self::mag_heading) keeps — with one departure.
    ///
    /// PX4 resets height only when every height source is failing (`ekf_helper.cpp:48-57`),
    /// and otherwise stops fusing the one that disagrees, because its barometer is the height
    /// reference. Here GNSS height is the absolute and the barometer's offset is estimated,
    /// equation (30′), so a GNSS height rejected for 5 s is adopted even while the barometer
    /// is accepted, and a barometer rejected for 5 s has its reference read again: the
    /// absolute wins either way.
    ///
    /// The two heading sources PX4 either lacks or does not recover take the same 7 s, by the
    /// rule recovery follows everywhere here rather than by a measurement. PX4 stops fusing a
    /// GNSS yaw that has failed for `reset_timeout_max` and resets nothing
    /// (`aid_sources/gnss/gnss_yaw_control.cpp:78-81`); ArduPilot realigns it on the ground
    /// after 10 s (`AP_NavEKF3_MagFusion.cpp:433-435` at `368dc0c4`). PX4 has no course
    /// constraint at all; ArduPilot's in-flight course realignment is itself a reset
    /// (`realignYawGPS`, `AP_NavEKF3_MagFusion.cpp:145-218`).
    fn default() -> Self {
        Self {
            gnss_position: Some(Seconds::from_secs(7.0)),
            gnss_height: Some(Seconds::from_secs(5.0)),
            gnss_velocity: Some(Seconds::from_secs(7.0)),
            baro_altitude: Some(Seconds::from_secs(5.0)),
            mag_heading: Some(Seconds::from_secs(7.0)),
            gnss_heading: Some(Seconds::from_secs(7.0)),
            course: Some(Seconds::from_secs(7.0)),
        }
    }
}

/// What the filter assumes across an IMU gap it coasts over. Equation (22′).
///
/// A gap longer than [`Config::max_predict_dt`] has no measurement describing it, so the
/// filter coasts: position advances on the estimated velocity, nothing else moves, and the
/// covariance grows by (22) plus an acceleration nobody measured. Refusing the step instead
/// leaves a moving vehicle's state stale by `v Δt` under a covariance that did not grow, and
/// every fix after the gap is then tens of σ away and turned down until
/// [`Config::recovery`] adopts one — the lockout coasting exists to prevent.
///
/// On by default, per [rejection
/// handling](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#rejection-handling-recover-by-default-opt-out-per-source):
/// a correction the filter can make honestly is made, and `Config::coast = None` is the
/// opt-out for an application that owns the gap itself.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coast {
    /// Density of the acceleration the vehicle may have pulled during the gap, m s⁻² / √Hz,
    /// added to the velocity block of (21) for the gap's duration: `σ_v = a √Δt`, 3.5 m/s over
    /// a 3.1 s gap at the default.
    pub acceleration: f32,
    /// Density of the rotation the vehicle may have made during the gap, rad s⁻¹ / √Hz,
    /// added to the attitude block of (21) for the gap's duration: 10° over a 3.1 s gap at
    /// the default. All three axes, because a vehicle that turns also banks.
    pub rotation: f32,
}

impl Default for Coast {
    /// Measured on the two sources with gaps at speed, and set at twice the smallest value
    /// either needed. Both are properties of an airframe's manoeuvres, so a vehicle more
    /// agile than these is the reason to raise them.
    ///
    /// `4b473e91`, a VTOL at 30 m/s with eight logging dropouts of 1.0–3.1 s, is what sets
    /// both, since in the simulator any value passes. Refused, its gaps cost 12 recoveries,
    /// 39 rejected positions and 28 velocities. Coasted with `rotation` at zero, the first
    /// fix after every gap is accepted at any `acceleration`, and what follows it is not: the
    /// course turns 44° across the 3.1 s gap at 954 s, the heading innovation afterwards sits
    /// at −0.60 rad under an `S` that did not grow, and the stale heading steers velocity off
    /// until the gate turns it down (7 recoveries at `acceleration` 1.0). With `rotation` at
    /// 0.02 or more and `acceleration` at 2.0 no gap causes a rejection or a recovery; the
    /// two positions and two velocities still rejected are fixes timestamped inside a gap,
    /// fused before the IMU sample that ends it. `acceleration` at 1.0 still needs `rotation`
    /// at 0.1, and at 0.5 leaves 7 recoveries at any `rotation`.
    ///
    /// `logging_dropout` (1.2 s at 20 m/s in a turn) passes at `acceleration` 0.5 or more
    /// whatever the rotation, and reads the same across the whole range: `pos_h_max` 23.25 m
    /// refused, 2.74 m coasted, `false_valid` 1946 → 0.
    fn default() -> Self {
        Self {
            acceleration: 2.0,
            rotation: 0.1,
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
    ///
    /// The whole tilt uncertainty of a levelled start, including the share an accelerometer
    /// bias explains: that share, `σ_βa / γ`, is carried as the tilt's correlation with the
    /// bias rather than added to it, and where it exceeds this figure it is the prior, with
    /// the window's own scatter across gravity added as the share the bias does not explain
    /// (equation (8)). At the defaults it does, 0.0204 rad against 0.02.
    pub sigma_tilt: Radians,
    /// Initial yaw standard deviation. Much larger than
    /// [`sigma_tilt`](Self::sigma_tilt): yaw inherits the magnetometer's calibration error.
    pub sigma_yaw: Radians,
    /// Initial accelerometer bias standard deviation.
    ///
    /// 0.2 m/s², the switch-on bias both production estimators assume: PX4's
    /// `EKF2_ABIAS_INIT` (`src/modules/ekf2/EKF/common.h:340`, `c4e4ef98`), and ArduPilot's
    /// `0.2 · EK3_ACC_BIAS_LIM` at its default of 1.0
    /// (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:616`, `AP_NavEKF3_core.h:52`,
    /// `AP_NavEKF3.cpp:567`, `368dc0c4`). A static window cannot measure it, since (5) reads a
    /// horizontal bias as tilt, which is why the tilt carries its correlation with it.
    ///
    /// At 0.1 the `harsh_imu` scenario's 0.186 m/s² sat at 1.86σ, and its attitude was
    /// overconfident on 50 seeds wherever (24′) did not inflate the covariance past it: 2281
    /// epochs over the family-wise bound at `Correlation::WHITE`, none at 0.2 with the
    /// correlation. On the corpus it is worth most on `7ce66f0d`, the hand launch levelled
    /// 12° wrong: 69 recoveries → 28, and aligned at 17.7 s rather than 32.7.
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
            sigma_accel_bias: MetersPerSecond2::from_m_per_s2(0.2),
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
    /// Tilt, per axis: the attitude's σ about north and about east, each held to this. See
    /// [`AttitudeVariance`](crate::AttitudeVariance).
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
    /// What these buy, measured on a static start at [`ImuNoise`]'s defaults: 3.83 s of unaided
    /// propagation before tilt leaves the bar, 37.8 s before heading does
    /// (`an_unaided_start_holds_its_attitude_for_the_margin_the_defaults_buy`). Neither is the
    /// `σ_g² t` the white-noise density alone gives, which would be 10.4 s and 674 s — the
    /// gyroscope-bias prior enters attitude through equation (20)'s `−I Δt` and grows as
    /// `σ_βg² t²`, overtaking the white-noise term inside two seconds.
    ///
    /// What those two times move is [`Validity`](crate::Validity), and nothing else.
    /// [`Status`](crate::Status) is already answering on the aiding timers by then — an unaided
    /// filter reports [`DeadReckoning`](crate::Status::DeadReckoning) from
    /// [`Timeouts::dead_reckoning_after`] without horizontal aiding, or from its first step if
    /// none was ever accepted — and [`Aligning`](crate::Status::Aligning) reads the alignment bars rather than these.
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
///         dead_reckoning_after: Seconds::from_secs(2.0),  // a multirotor in close quarters
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
    /// Automatic recovery from gate lockout, per source.
    pub recovery: Recovery,
    /// How long each source's measurement error persists, equation (24′).
    pub correlation: Correlation,
    /// Static initialization.
    pub init: Initialization,
    /// How good an estimate must be to count as valid.
    pub accuracy: Accuracy,
    /// Largest `dt` [`Eskf::predict`](crate::Eskf::predict) will integrate one IMU sample
    /// over.
    ///
    /// Beyond this the sample is not integrated, because the discretization of equations
    /// (9)–(22) is a first-order approximation over a short interval and one sample cannot
    /// describe a long one. The filter coasts across the gap instead, by [`Config::coast`], or
    /// with that off refuses the step and leaves the state where it was.
    ///
    /// Gaps come from logging dropouts, a scheduler overrun, or a sensor that genuinely
    /// stopped, and the filter cannot tell which. The default passes normal operation on
    /// every log in `data/manifest.txt` — IMU rates of 50 Hz to 250 Hz, whose longest
    /// interval short of a dropout is 90.5 ms, twice in the 2 h log's 1.4 M samples —
    /// while catching real SD-card dropouts of 0.34 s and up. The margin at that worst
    /// case is 10 ms, so a slower log than any in the corpus would need this raised.
    ///
    /// The same bound holds a measurement timed after the state, which is carried forward to
    /// its time on the estimated velocity and the last sample's rate: the longest the filter
    /// extrapolates on one sample either way. Later than this, a `fuse_*` refuses it as
    /// [`Fusion::OutOfHorizon`](crate::Fusion::OutOfHorizon).
    pub max_predict_dt: Seconds,
    /// Coasting across a step longer than [`max_predict_dt`](Self::max_predict_dt), equation
    /// (22′). `None` refuses the step instead, as
    /// [`Propagation::StepTooLong`](crate::Propagation::StepTooLong).
    pub coast: Option<Coast>,
    /// Random walk of the barometric offset, m s⁻¹ / √Hz: the `q_b` of equation (30′).
    ///
    /// `α₀`, the reference a barometer's altitude is measured against, is estimated rather
    /// than held constant, and this is how fast the filter lets it move. A barometer's
    /// reference drifts with the weather and the sensor, which no window can measure, so it
    /// is configured as [`ImuNoise`] is.
    ///
    /// The default is PX4's `baro_bias_nsd`, `src/modules/ekf2/EKF/common.h:347` at
    /// `c4e4ef98e9`, and the corpus's one drifting barometer agrees with it: `2c42096b`'s
    /// climbs about 12 m start to end over 2 h, and at 0.02 the filter rejects 20 of its GNSS
    /// heights, at 0.05 five, at this value none. Zero is a constant reference, which on that
    /// log rejects 3825 — the estimate settles on the barometer and then refuses the receiver
    /// the barometer has drifted away from.
    ///
    /// What it costs is height where the barometer does not drift. The simulator's never
    /// does, and there `mission` scores 0.170 m of vertical RMSE here against 0.085 at zero:
    /// the offset walks away from what the barometer knew, and GNSS height takes over the low
    /// frequencies. A barometer characterized on the bench as more stable than this is the
    /// reason to lower it.
    pub baro_offset_walk: f32,
    /// Whether a filter holding no barometric reference takes one from the estimate, at the
    /// first altitude once position is established. Equations (30) and (30′).
    ///
    /// On by default, because every start that leaves no reference — one in motion, a window
    /// with no barometer, a seed — otherwise discards the barometer for the whole flight:
    /// `cd7e0001`, a coarse start, refuses all 3530 of its altitudes, as `2c42096b` refused
    /// 35575 while it started coarse. PX4 does the same at
    /// `baro_height_control.cpp:79` at `c4e4ef98`. See
    /// [`Eskf::fuse_baro_altitude`](crate::Eskf::fuse_baro_altitude) for how the reference is
    /// seeded.
    ///
    /// Off also keeps [`Recovery`] from reading a new one, for a barometer or a GNSS height
    /// locked out: the caller's reference outlives both.
    ///
    /// Off is for an application that names its own reference with
    /// [`Eskf::set_baro_reference`](crate::Eskf::set_baro_reference) — a surveyed pad, say —
    /// and would rather altitudes were refused with
    /// [`Fusion::NoReference`](crate::Fusion::NoReference) until it does than referred to the
    /// estimate's height at whatever moment the first reading arrived.
    pub baro_reference_from_estimate: bool,
}

impl Default for Config {
    // Written out rather than derived: `Seconds::default()` is zero, which would make
    // `max_predict_dt` refuse every step.
    fn default() -> Self {
        Self {
            imu: ImuNoise::default(),
            gates: Gates::default(),
            timeouts: Timeouts::default(),
            recovery: Recovery::default(),
            correlation: Correlation::default(),
            init: Initialization::default(),
            accuracy: Accuracy::default(),
            max_predict_dt: Seconds::from_secs(0.1),
            coast: Some(Coast::default()),
            baro_offset_walk: 0.13,
            baro_reference_from_estimate: true,
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
