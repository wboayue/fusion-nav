//! Embedded-first inertial navigation using a 15-state Error-State Kalman Filter (ESKF).
//!
//! # Status: API sketch
//!
//! **No estimation mathematics is implemented.** [`Eskf`] has the intended signatures and
//! keeps its own health bookkeeping, but `predict` propagates nothing and every `fuse_*`
//! accepts unconditionally with a zero test ratio. This crate currently exists to let the
//! shape of the API be written against and argued with.
//!
//! See `README.md` for the architecture, `EQUATIONS.md` for the mathematics each method
//! cites, and `GOALS.md` for positioning.
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
//!         gyro: AngularRate::from_rad_per_s(0.0, 0.0, 0.0),
//!         accel: Acceleration::from_m_per_s2(0.0, 0.0, -GRAVITY),
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
//! let outcome = filter.fuse_gnss_position(
//!     Position::<Ned>::from_meters(12.0, -3.0, -40.0),
//!     PositionVariance::isotropic(1.5),
//! );
//! assert!(outcome.is_accepted());
//!
//! let state = filter.state();
//! assert_eq!(state.status, Status::Healthy);
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
mod health;
mod state;
mod units;

pub use config::{Accuracy, Config, GRAVITY, Gates, ImuNoise, Initialization, Timeouts};
pub use eskf::{Alignment, Coarse, Eskf, ImuSample, InitError, StaticSample};
pub use frames::{Body, Enu, Frame, Ned};
pub use health::{Diagnostics, Fusion, Propagation, SourceHealth, Status, Validity};
pub use state::{Covariance, CovarianceMatrix, ErrorState, STATES, State};
pub use units::{
    Acceleration, Altitude, AltitudeVariance, AngularRate, Attitude, HeadingVariance, MagField,
    Position, PositionVariance, Radians, Seconds, Velocity, VelocityVariance,
};

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
///         gyro: AngularRate::from_rad_per_s(0.0, 0.0, 0.0),
///         accel: Acceleration::from_m_per_s2(0.0, 0.0, -GRAVITY),
///     },
///     baro: Some(Altitude::from_meters(112.0)),
///     mag: None,
/// };
/// filter.initialize(&[still; 800], dt)?;
/// let _ = filter.predict(ImuSample::default(), dt);
///
/// let outcome = filter.fuse_baro_altitude(
///     Altitude::from_meters(60.0),
///     AltitudeVariance::from_m2(4.0),
/// );
/// assert!(outcome.is_accepted());
/// # Ok::<(), InitError>(())
/// ```
///
/// Deliberately excluded, because their names are too generic to glob-import safely:
/// [`Frame`], [`STATES`], and [`CovarianceMatrix`]. Import those by path.
pub mod prelude {
    pub use crate::config::{Accuracy, Config, GRAVITY, Gates, ImuNoise, Initialization, Timeouts};
    pub use crate::eskf::{Alignment, Coarse, Eskf, ImuSample, InitError, StaticSample};
    pub use crate::frames::{Body, Enu, Ned};
    pub use crate::health::{Diagnostics, Fusion, Propagation, SourceHealth, Status, Validity};
    pub use crate::state::{Covariance, ErrorState, State};
    pub use crate::units::{
        Acceleration, Altitude, AltitudeVariance, AngularRate, Attitude, HeadingVariance, MagField,
        Position, PositionVariance, Radians, Seconds, Velocity, VelocityVariance,
    };
}
