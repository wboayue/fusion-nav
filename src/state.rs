//! The navigation state estimate and the error-state covariance.

use nalgebra::{SMatrix, SVector};

use crate::frames::{Body, Ned};
use crate::health::Status;
use crate::units::{Acceleration, AngularRate, Attitude, Position, Velocity};

/// Dimension of the error state: three each of position, velocity, attitude,
/// accelerometer bias, and gyroscope bias.
pub const STATES: usize = 15;

/// The navigation estimate, returned by [`Eskf::state`](crate::Eskf::state).
///
/// Small and `Copy`, so reading it every control cycle costs nothing. The [`status`] is a
/// field rather than a separate accessor so that the trust level travels with the numbers
/// it qualifies. Nothing forces a caller to read it — `state().position` compiles — but it
/// is in hand rather than behind a second call that is easy not to make.
///
/// [`status`]: State::status
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    /// Rotation from body to NED.
    pub attitude: Attitude,
    /// Position relative to the navigation origin.
    pub position: Position<Ned>,
    /// Velocity in the navigation frame.
    pub velocity: Velocity<Ned>,
    /// Estimated accelerometer bias, to be subtracted from raw measurements.
    pub accel_bias: Acceleration<Body>,
    /// Estimated gyroscope bias, to be subtracted from raw measurements.
    pub gyro_bias: AngularRate<Body>,
    /// Whether the estimate is currently aided, and by how much.
    pub status: Status,
}

/// Index of an error-state component within the covariance.
///
/// The ordering is that of equation (2):
/// `[δp δv δθ δβa δβg]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(usize)]
pub enum ErrorState {
    /// North position error.
    PositionNorth = 0,
    /// East position error.
    PositionEast = 1,
    /// Down position error.
    PositionDown = 2,
    /// North velocity error.
    VelocityNorth = 3,
    /// East velocity error.
    VelocityEast = 4,
    /// Down velocity error.
    VelocityDown = 5,
    /// Attitude error about body x. Local (body-frame) perturbation.
    AttitudeX = 6,
    /// Attitude error about body y.
    AttitudeY = 7,
    /// Attitude error about body z.
    AttitudeZ = 8,
    /// Accelerometer bias error, body x.
    AccelBiasX = 9,
    /// Accelerometer bias error, body y.
    AccelBiasY = 10,
    /// Accelerometer bias error, body z.
    AccelBiasZ = 11,
    /// Gyroscope bias error, body x.
    GyroBiasX = 12,
    /// Gyroscope bias error, body y.
    GyroBiasY = 13,
    /// Gyroscope bias error, body z.
    GyroBiasZ = 14,
}

impl ErrorState {
    /// The index of this component, for indexing [`Covariance::as_matrix`].
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// The 15 x 15 error covariance `P`, in the [`ErrorState`] ordering.
///
/// 900 bytes in `f32`. Returned by reference for that reason.
pub type CovarianceMatrix = SMatrix<f32, STATES, STATES>;

/// The error covariance, with named access to its components.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Covariance(CovarianceMatrix);

impl Covariance {
    /// All entries zero. Not a valid filter prior; see equation (8).
    pub fn zero() -> Self {
        Self(CovarianceMatrix::zeros())
    }

    /// A diagonal covariance from per-component standard deviations, in the
    /// [`ErrorState`] ordering. Equation (8).
    pub fn from_sigmas(sigmas: [f32; STATES]) -> Self {
        let variances = SVector::<f32, STATES>::from_iterator(sigmas.iter().map(|s| s * s));
        Self(CovarianceMatrix::from_diagonal(&variances))
    }

    /// Wrap a matrix. Not checked for symmetry or positive-definiteness.
    pub const fn from_matrix(p: CovarianceMatrix) -> Self {
        Self(p)
    }

    /// One entry, named rather than indexed.
    pub fn get(&self, row: ErrorState, column: ErrorState) -> f32 {
        self.0[(row.index(), column.index())]
    }

    /// The variance of one error-state component.
    pub fn variance(&self, state: ErrorState) -> f32 {
        self.get(state, state)
    }

    /// The whole matrix, for callers that want to do their own algebra.
    pub const fn as_matrix(&self) -> &CovarianceMatrix {
        &self.0
    }
}

impl Default for Covariance {
    fn default() -> Self {
        Self::zero()
    }
}
