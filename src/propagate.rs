//! Propagation input. Equations (9)–(22) are to be implemented here; see the
//! equation-to-code table in `EQUATIONS.md`.
//!
//! **Stub.** Only the IMU sample lives here today. [`Eskf::predict`](crate::Eskf::predict)
//! advances the health timers and propagates nothing.

use crate::frames::Body;
use crate::units::{Acceleration, AngularRate};

/// One IMU measurement, uncorrected. The filter subtracts its own bias estimates,
/// equation (9).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    /// Angular rate, body frame.
    pub gyro: AngularRate<Body>,
    /// Specific force, body frame. A level, stationary vehicle reads `(0, 0, -g)`.
    pub accel: Acceleration<Body>,
}

impl ImuSample {
    /// Whether every number in the sample is finite.
    ///
    /// Written once and called from both places a sample enters the filter, so that
    /// [`Eskf::initialize`](crate::Eskf::initialize) and
    /// [`Eskf::predict`](crate::Eskf::predict) refuse the same sample.
    pub(crate) fn is_finite(self) -> bool {
        self.gyro.is_finite() && self.accel.is_finite()
    }
}
