//! The navigation state estimate and the error-state covariance.

use nalgebra::{SMatrix, SVector};

use crate::frames::{Body, Ned};
use crate::health::{Status, Validity};
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
    /// Which parts of this estimate are good enough to use.
    ///
    /// Alongside [`status`](Self::status) rather than behind an accessor, for the same
    /// reason: the trust travels with the numbers it qualifies. `status` is the summary a
    /// human reads; this is what a controller branches on.
    pub validity: Validity,
}

impl State {
    /// Whether every number in the estimate is finite.
    ///
    /// Two callers with one question: [`Eskf::initialize_from`](crate::Eskf::initialize_from)
    /// asks it of a seed the application supplied, and
    /// [`Eskf::predict`](crate::Eskf::predict) asks it of what propagation produced, since
    /// (11)–(14) can overflow f32 from finite inputs. Beside the type rather than in either
    /// caller's module, because neither owns it.
    ///
    /// The quaternion is unit by construction, so only its finiteness is in question.
    pub(crate) fn is_finite(&self) -> bool {
        let q = self.attitude.quaternion();
        [q.w, q.i, q.j, q.k].iter().all(|v| v.is_finite())
            && self.position.is_finite()
            && self.velocity.is_finite()
            && self.accel_bias.is_finite()
            && self.gyro_bias.is_finite()
    }
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

    /// Replace one component's variance and drop its correlations with everything else.
    ///
    /// The reset of a quantity the filter is adopting rather than correcting: the new
    /// error came from the measurement, so it carries the measurement's variance and is
    /// uncorrelated with the errors the filter accumulated before it. Zeroing the row and
    /// column is what makes the second part true; leaving them would let the old
    /// correlation pull the reset value straight back.
    ///
    /// One component rather than a block, because the magnetic heading of (34)–(36)
    /// adopts yaw alone and leaves the tilt it was levelled by exactly as it was.
    pub(crate) fn reset_state(&mut self, state: ErrorState, variance: f32) {
        let i = state.index();
        for k in 0..STATES {
            self.0[(i, k)] = 0.0;
            self.0[(k, i)] = 0.0;
        }
        self.0[(i, i)] = variance;
    }

    /// [`reset_state`](Self::reset_state) over a whole block, for the position and
    /// velocity of [`Fusion::Reset`](crate::Fusion::Reset), which are adopted three
    /// components at a time.
    pub(crate) fn reset_block(&mut self, states: [ErrorState; 3], variances: [f32; 3]) {
        for (state, variance) in states.iter().zip(variances) {
            self.reset_state(*state, variance);
        }
    }

    /// The whole matrix, for callers that want to do their own algebra.
    pub const fn as_matrix(&self) -> &CovarianceMatrix {
        &self.0
    }

    /// Whether every entry is finite.
    ///
    /// Two callers with one question, as [`State::is_finite`] has:
    /// [`Eskf::initialize_from`](crate::Eskf::initialize_from) asks it of a seed the
    /// application supplied, and [`Eskf::predict`](crate::Eskf::predict) asks it of what (22)
    /// produced, since `F P Fᵀ + Q` overflows f32 from finite inputs for the same reason
    /// (11)–(14) do.
    pub(crate) fn is_finite(&self) -> bool {
        self.0.iter().all(|entry| entry.is_finite())
    }
}

impl Default for Covariance {
    fn default() -> Self {
        Self::zero()
    }
}

/// The barometric offset's share of the augmented covariance of equation (30′): `P_xb`, its
/// cross-covariance with the error state in the [`ErrorState`] ordering, and `P_bb`, its own
/// variance.
///
/// Beside [`Covariance`] rather than inside it, so that the 15 × 15 a caller reads stays the
/// covariance of the navigation state. The offset `b` is the error in `α₀`, the barometric
/// reference, and is never a component of [`State`]: its estimate is folded into `α₀` at
/// every update, so all it keeps between them is this row.
///
/// All zero while there is no reference: nothing then correlates with it and nothing reads it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Offset {
    /// `P_xb`.
    pub(crate) cross: SVector<f32, STATES>,
    /// `P_bb`.
    pub(crate) variance: f32,
}

impl Offset {
    /// An offset uncorrelated with the error state, of variance `variance`: a reference
    /// measured apart from the estimate, as a window at rest or a caller measures one.
    pub(crate) fn independent(variance: f32) -> Self {
        Self {
            cross: SVector::zeros(),
            variance,
        }
    }

    /// Drop the offset's correlation with one error-state component, as
    /// [`Covariance::reset_state`] drops that component's correlations with the rest.
    pub(crate) fn decorrelate(&mut self, state: ErrorState) {
        self.cross[state.index()] = 0.0;
    }

    /// Whether every entry is finite.
    pub(crate) fn is_finite(&self) -> bool {
        self.variance.is_finite() && self.cross.iter().all(|entry| entry.is_finite())
    }
}
