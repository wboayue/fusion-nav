//! Embedded-first inertial navigation using a 15-state Error-State Kalman Filter (ESKF).
//!
//! # Status: API sketch
//!
//! **No estimation mathematics is implemented.** [`Eskf`] has the intended signatures and
//! keeps its own health bookkeeping, but `predict` propagates nothing and every `fuse_*`
//! accepts unconditionally with a zero test ratio. This crate currently exists to let the
//! shape of the API be written against and argued with.
//!
//! See `README.md` for usage, `DESIGN.md` for the architecture, `EQUATIONS.md` for the
//! mathematics each method cites, and `GOALS.md` for positioning.
//!
//! # Shape
//!
//! ```
//! use fusion_nav::prelude::*;
//!
//! let mut filter = Eskf::new(Config::default());
//! let dt = Seconds::from_secs(0.0025); // 400 Hz IMU
//!
//! // A vehicle sitting still: no rotation, gravity the only specific force. 800 samples
//! // at 400 Hz is the 2 s `Initialization::min_duration` wants.
//! let still = StaticSample {
//!     imu: ImuSample {
//!         gyro: AngularRate::body(0.0, 0.0, 0.0),
//!         accel: Acceleration::body(0.0, 0.0, -GRAVITY),
//!     },
//!     ..StaticSample::default()
//! };
//!
//! // Initialization reports what it achieved rather than refusing what it dislikes. A
//! // window that is short or moving gives `Alignment::Coarse`, and the filter runs and
//! // says `Status::Aligning` until attitude converges.
//! assert_eq!(filter.initialize(&[still; 800], dt)?, Alignment::Static);
//!
//! // Nothing in that window carried a barometer, so there is no reference altitude and
//! // `fuse_baro_altitude` would refuse. See `StaticSample::baro`.
//!
//! assert!(filter.predict(ImuSample::default(), dt).is_propagated());
//!
//! // GNSS in latitude and longitude. The filter holds the navigation origin: the first
//! // fix places it, under the estimate, so every later fix converts about the same point.
//! let outcome = filter.fuse_gnss_geodetic(
//!     Geodetic::from_degrees(47.397_742, 8.545_594, 488.0),
//!     PositionNoise::horizontal_vertical(1.5, 3.0),
//! );
//! assert!(outcome.is_accepted());
//! assert!(filter.origin().is_some());
//!
//! // Nothing in that window carried a magnetometer either, so nothing observed the
//! // rotation about gravity: heading is not valid, and `Aligning` says the attitude has
//! // not converged rather than the aiding having failed.
//! let state = filter.state();
//! assert_eq!(state.status, Status::Aligning);
//! assert!(state.validity.tilt && !state.validity.heading);
//!
//! // The first accepted magnetic heading is what establishes yaw.
//! let outcome = filter.fuse_mag_heading(
//!     MagField::body(0.22, 0.0, 0.44),
//!     HeadingNoise::from_sigma(0.1),
//! );
//! assert!(outcome.is_accepted());
//! assert_eq!(filter.state().status, Status::Healthy);
//! # Ok::<(), fusion_nav::InitError>(())
//! ```
//!
//! Frames are type parameters, so a `Position<Enu>` where NED is expected is a compile
//! error rather than a flight anomaly. Units are named by every constructor.
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
mod propagate;
mod state;
mod units;

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
/// let outcome = filter.fuse_baro_altitude(
///     Altitude::from_meters(60.0),
///     AltitudeNoise::from_sigma(2.0),
/// );
/// assert!(outcome.is_accepted());
///
/// // `R` describes that one measurement, so it travels with it — and it has to be a
/// // variance some sensor could have. Zero or negative is refused, not fused.
/// assert_eq!(
///     filter.fuse_baro_altitude(
///         Altitude::from_meters(60.0),
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
    pub use crate::config::{Accuracy, Config, GRAVITY, Gates, ImuNoise, Initialization, Timeouts};
    pub use crate::eskf::Eskf;
    pub use crate::frames::{Body, Enu, Ned};
    pub use crate::geodetic::{Geodetic, LocalOrigin};
    pub use crate::health::{
        Diagnostics, Fusion, Propagation, PropagationHealth, Refusal, SourceHealth, Status,
        Validity,
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
