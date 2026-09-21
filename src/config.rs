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

/// Chi-square gate thresholds `γ`, one per observation, from equation (37).
///
/// The filter reports `r = ε / γ`, so these set what `r = 1` means.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gates {
    /// GNSS position, 3 degrees of freedom.
    pub gnss_position: f32,
    /// GNSS velocity, 3 degrees of freedom.
    pub gnss_velocity: f32,
    /// Barometric altitude, 1 degree of freedom.
    pub baro_altitude: f32,
    /// Magnetic heading, 1 degree of freedom.
    pub mag_heading: f32,
}

impl Default for Gates {
    fn default() -> Self {
        // 95th percentile of chi-square at 3 and 1 degrees of freedom.
        Self {
            gnss_position: 7.81,
            gnss_velocity: 7.81,
            baro_altitude: 3.84,
            mag_heading: 3.84,
        }
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Accuracy {
    /// Roll and pitch. Also the bar for alignment:
    /// [`Status::Aligning`](crate::Status::Aligning) lasts until tilt and heading are
    /// both within these.
    pub tilt: Radians,
    /// Heading.
    pub heading: Radians,
    /// Position, horizontally and vertically.
    pub position: Meters,
    /// Velocity, horizontally and vertically.
    pub velocity: MetersPerSecond,
}

impl Default for Accuracy {
    /// Attitude clears the prior a static alignment starts from, by the margin an unaided
    /// filter takes to drift through. Position and velocity are **placeholders** — loose
    /// enough to admit a 1 Hz GNSS solution, and nothing more considered than that.
    ///
    /// A bar equal to the prior it is compared against is no bar at all, which is what these
    /// were until covariance propagation landed: [`tilt`](Accuracy::tilt) was
    /// [`Initialization::sigma_tilt`](Initialization::sigma_tilt) exactly and
    /// [`heading`](Accuracy::heading) was
    /// [`Initialization::sigma_yaw`](Initialization::sigma_yaw) exactly, compared with `<=`,
    /// so a static start passed by zero margin and the first `Q` of equation (21) took it
    /// away again. It reads `aligned_at=0.00` on every static log in `data/manifest.txt` and
    /// `never` one step later — a filter that reported convergence for exactly one sample.
    ///
    /// 3° of tilt is what PX4 declares tilt alignment at (`getTiltVariance() <
    /// sq(radians(3.f))`, `src/modules/ekf2/EKF/control.cpp:73-78` at `c4e4ef98e9`);
    /// ArduPilot uses 5° (`tiltErrorVariance < sq(radians(5.0))`,
    /// `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp:520-525` at `368dc0c428`). Heading has no
    /// published counterpart — both estimators latch yaw alignment on the magnetometer reset
    /// rather than comparing a variance — so 30° is a mission number under one hard
    /// constraint: it has to clear `sigma_yaw`, since a heading whose prior is 20° can never
    /// be valid under a 20° bar.
    ///
    /// What they buy, measured on a static start at [`ImuNoise`]'s defaults: 3.79 s of unaided
    /// propagation before tilt leaves the bar, 35.4 s before heading does
    /// (`an_unaided_start_holds_its_attitude_for_the_margin_the_defaults_buy`). Neither is the
    /// `σ_g² t` the white-noise density alone gives, which would be 10.4 s and 657 s — the
    /// gyroscope-bias prior enters attitude through equation (20)'s `−I Δt` and grows as
    /// `σ_βg² t²`, overtaking the white-noise term inside two seconds. An unaided filter
    /// therefore reports [`Degraded`](crate::Status::Degraded) at 2.5 s,
    /// [`Aligning`](crate::Status::Aligning) at 3.8 s and
    /// [`DeadReckoning`](crate::Status::DeadReckoning) from 5 s, which is the precedence
    /// working as intended: the most severe thing true of the estimate is what it reports.
    /// Supply your own numbers; that is what [`Accuracy`] is for.
    ///
    /// Neither bar latches. Nothing in propagation shrinks a covariance, so the crossing is
    /// one-way and [`Status`](crate::Status) moves to
    /// [`Aligning`](crate::Status::Aligning) once rather than flapping — the corpus shows one
    /// added transition per log, not a sequence. PX4 and ArduPilot both latch instead
    /// (`tilt_align` and `tiltAlignComplete` are only ever tested while false), which is the
    /// cheaper answer to a covariance that moves in both directions; when the update of
    /// (23)–(28) makes ours do that, `aligned_at` is the measurement that says whether
    /// latching is needed here too.
    fn default() -> Self {
        Self {
            tilt: Radians::from_radians(0.052),
            heading: Radians::from_radians(0.52),
            position: Meters::from_meters(5.0),
            velocity: MetersPerSecond::from_m_per_s(1.0),
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
    /// discretization of equations (9)-(22) is a first-order approximation over a short
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
