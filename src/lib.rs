#![doc = include_str!("../README.md")]
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

mod config;
mod eskf;
mod frames;
mod geodetic;
mod health;
mod init;
mod math;
mod observation;
mod propagate;
mod state;
mod units;
mod update;

// The prelude is the one list of public types; the root adds back the three it leaves out.
#[doc(inline)]
pub use prelude::*;

pub use frames::Frame;
pub use state::{CovarianceMatrix, STATES};

/// Everything needed to write an integration loop, in one import.
///
/// ```
/// use fusion_nav::prelude::*;
///
/// let mut filter = Eskf::new(Config::default());
/// let dt = Seconds::from_secs(0.0025);
///
/// // The barometer in the window is what fixes the reference the fusion below is
/// // relative to. Without it that call refuses with `Fusion::NoReference`.
/// let still = StaticSample {
///     imu: ImuSample {
///         gyro: AngularRate::body(0.0, 0.0, 0.0),
///         accel: Acceleration::body(0.0, 0.0, -GRAVITY),
///     },
///     baro: Some(Altitude::from_meters(112.0)),
///     ..StaticSample::default()
/// };
/// filter.initialize(&[still; 800], dt)?;
/// assert_eq!(filter.predict(still.imu, dt), Propagation::Propagated);
///
/// // An IMU that has stopped producing numbers is refused rather than propagated: a
/// // non-finite sample reaches the quaternion and then the covariance, and never leaves.
/// let broken = ImuSample {
///     gyro: AngularRate::body(f32::NAN, 0.0, 0.0),
///     accel: still.imu.accel,
/// };
/// assert_eq!(filter.predict(broken, dt), Propagation::NotFinite);
///
/// // Two metres above the reference the window fixed. An altitude far from what the
/// // filter expects is `Fusion::Rejected` instead — the gate of (37) runs on every
/// // measurement, and a 52 m step from a vehicle that has not moved does not pass it.
/// let outcome = filter.fuse_baro_altitude(
///     Altitude::from_meters(114.0),
///     AltitudeNoise::from_sigma(2.0),
/// );
/// assert!(outcome.is_accepted());
///
/// // `R` describes that one measurement, so it travels with it — and it has to be a
/// // variance some sensor could have. Zero or negative is refused, not fused.
/// assert_eq!(
///     filter.fuse_baro_altitude(
///         Altitude::from_meters(114.0),
///         AltitudeNoise::from_variance(0.0),
///     ),
///     Fusion::InvalidNoise,
/// );
/// # Ok::<(), InitError>(())
/// ```
///
/// Deliberately excluded, because their names are too generic to glob-import safely:
/// [`Frame`], [`STATES`], and [`CovarianceMatrix`]. Import those by path.
pub mod prelude {
    pub use crate::config::{
        ALIGNED_HEADING, ALIGNED_TILT, Accuracy, Config, GRAVITY, Gate, Gates, ImuNoise,
        Initialization, Percentile, Timeouts,
    };
    pub use crate::eskf::Eskf;
    pub use crate::frames::{Body, Enu, Ned};
    pub use crate::geodetic::{Geodetic, LocalOrigin};
    pub use crate::health::{
        Diagnostics, Fusion, GnssFusion, Innovation, Propagation, PropagationHealth, Refusal,
        SourceHealth, Status, Validity,
    };
    pub use crate::init::{Alignment, Coarse, InitError, StaticSample};
    pub use crate::propagate::ImuSample;
    pub use crate::state::{Covariance, ErrorState, State};
    pub use crate::units::{
        Acceleration, Altitude, AltitudeNoise, AngularRate, Attitude, HeadingNoise, MagField,
        Meters, MetersPerSecond, MetersPerSecond2, Position, PositionNoise, Radians,
        RadiansPerSecond, Seconds, Velocity, VelocityNoise,
    };
}
