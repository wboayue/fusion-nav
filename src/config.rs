//! Filter tuning.
//!
//! Every default here is a **placeholder** chosen to make the shape of the API concrete.
//! None has been validated against flight data.

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
    fn default() -> Self {
        Self {
            gyro_white: 1.0e-3,
            accel_white: 3.0e-2,
            gyro_bias_walk: 1.0e-4,
            accel_bias_walk: 1.0e-3,
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
    pub degraded_after: Seconds,
    /// Beyond this with no source accepted at all, the status becomes
    /// [`DeadReckoning`](crate::Status::DeadReckoning).
    pub dead_reckoning_after: Seconds,
}

impl Default for Timeouts {
    fn default() -> Self {
        Self {
            degraded_after: Seconds::from_secs(1.0),
            dead_reckoning_after: Seconds::from_secs(5.0),
        }
    }
}

/// Quasi-static initialization, equations (5)–(8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Initialization {
    /// Samples required in the static window.
    pub min_samples: usize,
    /// Largest angular rate, rad s⁻¹, still considered stationary.
    pub max_gyro_rate: f32,
    /// Largest deviation of the accelerometer magnitude from gravity, m s⁻², still
    /// considered stationary.
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
    fn default() -> Self {
        Self {
            min_samples: 100,
            max_gyro_rate: 0.05,
            max_accel_deviation: 0.5,
            sigma_position: 1.0,
            sigma_velocity: 0.1,
            sigma_tilt: Radians::from_radians(0.02),
            sigma_yaw: Radians::from_radians(0.35),
            sigma_accel_bias: 0.1,
            sigma_gyro_bias: 0.01,
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
///         degraded_after: Seconds::from_secs(0.5),
///         ..Timeouts::default()
///     },
///     ..Config::default()
/// };
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Config {
    /// IMU noise densities.
    pub imu: ImuNoise,
    /// Innovation gate thresholds.
    pub gates: Gates,
    /// Fusion timeouts feeding [`Status`](crate::Status).
    pub timeouts: Timeouts,
    /// Static initialization.
    pub init: Initialization,
    /// Magnetic declination at the operating site, added to magnetic heading to give
    /// true heading. Equation (6).
    pub magnetic_declination: Radians,
}
