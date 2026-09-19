//! Filter tuning.
//!
//! Every default here is a **placeholder** chosen to make the shape of the API concrete,
//! and none has been validated against flight data — with two exceptions:
//! [`Timeouts::degraded_after`] and [`Initialization`]'s stationarity tolerances, both
//! corrected after replaying the PX4 corpus, and [`ImuNoise`], re-baselined against the
//! defaults PX4 and ArduPilot ship.

use crate::units::{Radians, Seconds};

/// Standard gravity, m s⁻².
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
    /// The previous defaults here were datasheet-grade — 10 to 15 times tighter on every
    /// term — which would make the covariance claim a precision the estimate does not
    /// have, and an overconfident covariance gates out measurements that were fine.
    /// Two independent production estimators agreeing is not the same evidence as a
    /// replay of our own, so these remain subject to the validation in `GOALS.md`, but
    /// they are the right order of magnitude to start from.
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
    /// The previous default of 1.0 s was exactly that period: replaying a PX4 log whose
    /// fix intervals ran 0.988-1.024 s put 37 of 122 of them over the threshold, and the
    /// status flapped between `Healthy` and `Degraded` 76 times in 124 seconds. 2.5 s
    /// clears two missed fixes and still leaves half the window to
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
    /// tell stillness from a slow drift. Sample rates from 50 Hz to 400 Hz appear in
    /// real logs.
    pub min_duration: Seconds,
    /// Largest angular rate magnitude, rad s⁻¹, still considered stationary.
    ///
    /// Compared against the **peak** over the window, not a filtered value, so at the
    /// same number this is the stricter test: one vibration spike is enough to fail it.
    pub max_gyro_rate: f32,
    /// Largest departure of the specific-force magnitude from gravity, m s⁻², still
    /// considered stationary. Peak over the window, as with
    /// [`max_gyro_rate`](Self::max_gyro_rate).
    pub max_accel_deviation: f32,
    /// Initial position standard deviation, m.
    pub sigma_position: f32,
    /// Initial velocity standard deviation, m s⁻¹.
    pub sigma_velocity: f32,
    /// Initial roll and pitch standard deviation, rad. Gravity determines these well.
    pub sigma_tilt: Radians,
    /// Initial yaw standard deviation, rad. Much larger than
    /// [`sigma_tilt`](Self::sigma_tilt): yaw inherits the magnetometer's calibration error.
    pub sigma_yaw: Radians,
    /// Initial accelerometer bias standard deviation, m s⁻².
    pub sigma_accel_bias: f32,
    /// Initial gyroscope bias standard deviation, rad s⁻¹.
    pub sigma_gyro_bias: f32,
}

impl Default for Initialization {
    /// The stationarity tolerances are PX4 EKF2's — 15°/s and 20% of gravity — corrected
    /// from the replay corpus.
    ///
    /// The previous 0.05 rad s⁻¹ and 0.5 m s⁻² failed four of the five logs in
    /// `data/manifest.txt`, on vehicles that were sitting on the ground: peaks of
    /// 0.026–0.172 rad s⁻¹ and 0.15–1.04 m s⁻², which is idle vibration and prop wash,
    /// not motion. A tolerance that calls a parked quadrotor moving does not protect the
    /// alignment, it just denies it. At these values four of the five align statically,
    /// and the fifth — peak deviation 6.2 m s⁻² — stays coarse, correctly: 6 m s⁻² is a
    /// vehicle being handled, not a vehicle vibrating.
    fn default() -> Self {
        Self {
            min_duration: Seconds::from_secs(2.0),
            max_gyro_rate: 0.262,
            max_accel_deviation: 1.961,
            sigma_position: 1.0,
            sigma_velocity: 0.1,
            sigma_tilt: Radians::from_radians(0.02),
            sigma_yaw: Radians::from_radians(0.35),
            sigma_accel_bias: 0.1,
            sigma_gyro_bias: 0.01,
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
    /// Roll and pitch, rad. Also the bar for alignment:
    /// [`Status::Aligning`](crate::Status::Aligning) lasts until tilt and heading are
    /// both within these.
    pub sigma_tilt: Radians,
    /// Heading, rad.
    pub sigma_heading: Radians,
    /// Position, m, horizontally and vertically.
    pub sigma_position: f32,
    /// Velocity, m s⁻¹, horizontally and vertically.
    pub sigma_velocity: f32,
}

impl Default for Accuracy {
    /// Attitude matches what a good static alignment gives, so a filter that started
    /// still is aligned from its first sample. Position and velocity are **placeholders**
    /// — loose enough to admit a 1 Hz GNSS solution, and nothing more considered than
    /// that.
    fn default() -> Self {
        Self {
            sigma_tilt: Radians::from_radians(0.02),
            sigma_heading: Radians::from_radians(0.35),
            sigma_position: 5.0,
            sigma_velocity: 1.0,
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
    /// every log in `data/manifest.txt` — whose worst ordinary interval is 65 ms across
    /// rates from 50 Hz to 400 Hz — while catching real SD-card dropouts of 0.34 s and up.
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
