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
//! use fusion_nav::{Config, Eskf, ImuSample, Ned, Position, PositionVariance};
//! use fusion_nav::{Seconds, StaticSample, Status};
//!
//! let mut filter = Eskf::new(Config::default());
//! filter.initialize(&[StaticSample::default(); 100])?;
//!
//! filter.predict(ImuSample::default(), Seconds::from_secs(0.0025));
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

pub use config::{Config, GRAVITY, Gates, ImuNoise, Initialization, Timeouts};
pub use eskf::{Eskf, ImuSample, InitError, StaticSample};
pub use frames::{Body, Enu, Frame, Ned};
pub use health::{Diagnostics, Fusion, SourceHealth, Status};
pub use state::{Covariance, CovarianceMatrix, ErrorState, STATES, State};
pub use units::{
    Acceleration, Altitude, AltitudeVariance, AngularRate, Attitude, HeadingVariance, MagField,
    Position, PositionVariance, Radians, Seconds, Velocity, VelocityVariance,
};
