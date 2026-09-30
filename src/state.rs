//! The navigation state estimate and the error-state covariance.

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::frames::{Body, Ned};
use crate::health::{Status, Validity};
use crate::math::enforce_symmetry;
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// The index of this component, for indexing [`Covariance::to_rows`].
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// The 15 x 15 error covariance `P`, in the [`ErrorState`] ordering.
///
/// 900 bytes in `f32`, so the filter hands it around by reference.
pub(crate) type CovarianceMatrix = SMatrix<f32, STATES, STATES>;

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

    /// From rows, `p[row][column]` in the [`ErrorState`] ordering. Not checked for symmetry
    /// or positive-definiteness here; [`Eskf::initialize_from`](crate::Eskf::initialize_from)
    /// checks what it needs of a seed.
    pub fn from_rows(p: [[f32; STATES]; STATES]) -> Self {
        Self(CovarianceMatrix::from_fn(|row, column| p[row][column]))
    }

    /// Wrap a matrix the filter computed.
    pub(crate) const fn from_matrix(p: CovarianceMatrix) -> Self {
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
    pub(crate) fn reset_state(&mut self, state: ErrorState, variance: f32) {
        let i = state.index();
        for k in 0..STATES {
            self.0[(i, k)] = 0.0;
            self.0[(k, i)] = 0.0;
        }
        self.0[(i, i)] = variance;
    }

    /// Replace the attitude block, `P_θθ`, leaving its correlations with the other states.
    pub(crate) fn set_attitude_block(&mut self, block: Matrix3<f32>) {
        let theta = ErrorState::AttitudeX.index();
        self.0
            .fixed_view_mut::<3, 3>(theta, theta)
            .copy_from(&block);
    }

    /// Replace `P_θβa`, the attitude's cross-covariance with the accelerometer bias, and its
    /// transpose, leaving both diagonal blocks as they were.
    pub(crate) fn set_attitude_accel_bias_block(&mut self, block: Matrix3<f32>) {
        let (theta, beta) = (
            ErrorState::AttitudeX.index(),
            ErrorState::AccelBiasX.index(),
        );
        self.0.fixed_view_mut::<3, 3>(theta, beta).copy_from(&block);
        self.0
            .fixed_view_mut::<3, 3>(beta, theta)
            .copy_from(&block.transpose());
    }

    /// [`reset_state`](Self::reset_state) along a direction of the attitude block rather
    /// than one of its axes: the component `uᵀδθ` gets `variance` and loses its correlations,
    /// and everything orthogonal to it is left as it was.
    ///
    /// Heading adoption needs it because heading is rotation about navigation down, which
    /// is the body axis `u = R(q̂)ᵀe₃` and not `δθ_z` unless the vehicle is level — at 90°
    /// of pitch, resetting `δθ_z` would reset a tilt. With `M = I − uuᵀ` the projection onto
    /// the rest, the attitude rows become `M P_θ·`, the block `M P_θθ M + variance · uuᵀ`,
    /// and `u = eᵢ` is [`reset_state`](Self::reset_state) exactly. `u` must be a unit vector.
    pub(crate) fn reset_attitude_direction(&mut self, u: Vector3<f32>, variance: f32) {
        let theta = ErrorState::AttitudeX.index();
        let m = Matrix3::identity() - u * u.transpose();
        let rows = m * self.0.fixed_rows::<3>(theta);
        self.0.fixed_rows_mut::<3>(theta).copy_from(&rows);
        let columns = self.0.fixed_columns::<3>(theta) * m;
        self.0.fixed_columns_mut::<3>(theta).copy_from(&columns);
        let block = self.0.fixed_view::<3, 3>(theta, theta) + u * u.transpose() * variance;
        self.0
            .fixed_view_mut::<3, 3>(theta, theta)
            .copy_from(&block);
        // (42): `M P M` rounds differently on either side of the diagonal.
        enforce_symmetry(&mut self.0);
    }

    /// [`reset_state`](Self::reset_state) over several states, for the position and
    /// velocity of [`Fusion::Reset`](crate::Fusion::Reset): three components on a first
    /// adoption, the horizontal pair or the height alone on a recovery.
    pub(crate) fn reset_block<const N: usize>(
        &mut self,
        states: [ErrorState; N],
        variances: [f32; N],
    ) {
        for (state, variance) in states.iter().zip(variances) {
            self.reset_state(*state, variance);
        }
    }

    /// The whole matrix as rows, `p[row][column]` in the [`ErrorState`] ordering, for callers
    /// that do their own algebra.
    ///
    /// Rows rather than `nalgebra`'s storage, which is an array of columns, because `P` is
    /// symmetric only to its last bit: a caller indexing columns as rows would read the
    /// transpose, and nothing would say so. The same trap is on the way into `nalgebra`, whose
    /// `From<[[f32; 15]; 15]>` also reads columns: `SMatrix::from_fn(|i, j| p[i][j])` is the
    /// conversion. A copy, 900 bytes on the caller's stack; [`get`](Self::get) reads one entry
    /// without it.
    pub fn to_rows(&self) -> [[f32; STATES]; STATES] {
        core::array::from_fn(|row| core::array::from_fn(|column| self.0[(row, column)]))
    }

    /// The whole matrix, for the filter's own algebra.
    pub(crate) const fn as_matrix(&self) -> &CovarianceMatrix {
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

/// How uncertain the attitude is about each navigation axis, in radians squared: tilt about
/// north and east, heading about down.
///
/// The covariance cannot be read for these directly. Its attitude block is the covariance of
/// `δθ`, a rotation vector in **body** axes (equations (2)–(4)), and (36) states the
/// navigation-frame rotation it stands for as `R(q̂) δθ`. So the body x and y diagonal is tilt,
/// and z heading, only while the vehicle is level: at 90° of pitch body x is vertical, and
/// `P[δθ_x]` is the heading's variance. These are the diagonal of `R(q̂) P_θθ R(q̂)ᵀ`, which
/// holds at any attitude, and which is what PX4 publishes too — `getTiltVariance` and
/// `getYawVar` read the same components of its navigation-frame covariance
/// (`src/modules/ekf2/EKF/ekf_helper.cpp:938-947` at `c4e4ef98e9`).
///
/// `heading` is exactly `H P Hᵀ` for the heading Jacobian of (36), whose row is the third of
/// `R(q̂)`: a heading source and this struct agree on what heading uncertainty is.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AttitudeVariance {
    /// About north: the tilt a roll reads while the vehicle faces north.
    pub tilt_north: f32,
    /// About east: the tilt a pitch reads while the vehicle faces north.
    pub tilt_east: f32,
    /// About down.
    pub heading: f32,
}

impl AttitudeVariance {
    /// Resolve `covariance`'s attitude block on the navigation axes of `attitude`.
    ///
    /// The attitude is an argument rather than read from a filter because the covariance
    /// is: [`Eskf::predicted_validity`](crate::Eskf::predicted_validity) asks this of a
    /// covariance projected forward, which the nominal attitude it was projected from
    /// still describes.
    pub(crate) fn of(attitude: &Attitude, covariance: &Covariance) -> Self {
        let r = attitude.quaternion().to_rotation_matrix().into_inner();
        let theta = ErrorState::AttitudeX.index();
        let p_theta = covariance.as_matrix().fixed_view::<3, 3>(theta, theta);
        let ned = r * p_theta * r.transpose();
        Self {
            tilt_north: ned[(0, 0)],
            tilt_east: ned[(1, 1)],
            heading: ned[(2, 2)],
        }
    }

    /// The three variances about east, north and up, the axes ROS's ENU frame orders a
    /// `nav_msgs/Odometry` orientation covariance in: the horizontal pair swapped, and the
    /// sign of an axis, which a variance does not carry, dropped.
    pub fn to_enu(self) -> [f32; 3] {
        [self.tilt_east, self.tilt_north, self.heading]
    }

    /// The body-frame attitude block whose navigation-frame covariance is these three
    /// variances and nothing correlated between them: `R(q̂)ᵀ diag(·) R(q̂)`, the inverse of
    /// [`of`](Self::of). Equation (8).
    ///
    /// Symmetrized by (42), since the product is not exactly: rounded, `R(q̂)ᵀ D R(q̂)` differs
    /// from its transpose in the last bit off the diagonal, and this block is committed at a
    /// start with no propagation after it to repair that.
    pub(crate) fn in_body(self, attitude: &Attitude) -> Matrix3<f32> {
        let r = attitude.quaternion().to_rotation_matrix().into_inner();
        let ned =
            Matrix3::from_diagonal(&Vector3::new(self.tilt_north, self.tilt_east, self.heading));
        let mut body = r.transpose() * ned * r;
        enforce_symmetry(&mut body);
        body
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

    /// The offset of a reference read from the estimate, `α̂₀ = α + p̂_D`: its error is
    /// `−δp_D` plus the reading's noise, so `P_xb = −P[:, D]` and `P_bb = P_DD + R_m`.
    /// Equation (30′).
    pub(crate) fn from_estimate(covariance: &Covariance, noise_variance: f32) -> Self {
        let down = ErrorState::PositionDown;
        Self {
            cross: -covariance.as_matrix().column(down.index()),
            variance: covariance.variance(down) + noise_variance,
        }
    }

    /// Drop the offset's correlation with one error-state component, as
    /// [`Covariance::reset_state`] drops that component's correlations with the rest.
    pub(crate) fn decorrelate(&mut self, state: ErrorState) {
        self.cross[state.index()] = 0.0;
    }

    /// Drop the offset's correlation with the attitude component `uᵀδθ`, as
    /// [`Covariance::reset_attitude_direction`] drops that component's correlations with the
    /// rest.
    pub(crate) fn decorrelate_attitude_direction(&mut self, u: Vector3<f32>) {
        let theta = ErrorState::AttitudeX.index();
        let m = Matrix3::identity() - u * u.transpose();
        let cross = m * self.cross.fixed_rows::<3>(theta);
        self.cross.fixed_rows_mut::<3>(theta).copy_from(&cross);
    }

    /// Whether every entry is finite.
    pub(crate) fn is_finite(&self) -> bool {
        self.variance.is_finite() && self.cross.iter().all(|entry| entry.is_finite())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;
    use nalgebra::UnitQuaternion;

    fn attitude_of(roll: f32, pitch: f32, yaw: f32) -> Attitude {
        Attitude::from_quaternion(UnitQuaternion::from_euler_angles(roll, pitch, yaw))
    }

    #[test]
    fn the_array_is_rows_in_and_out() {
        // Asymmetric, so an array read as columns anywhere reads the transpose and fails.
        let mut rows = [[0.0; STATES]; STATES];
        rows[ErrorState::PositionNorth.index()][ErrorState::GyroBiasZ.index()] = 1.0;
        let covariance = Covariance::from_rows(rows);
        assert_eq!(
            covariance.get(ErrorState::PositionNorth, ErrorState::GyroBiasZ),
            1.0
        );
        assert_eq!(
            covariance.get(ErrorState::GyroBiasZ, ErrorState::PositionNorth),
            0.0
        );
        assert_eq!(covariance.to_rows(), rows);
    }

    #[test]
    fn enu_order_swaps_the_tilts_and_keeps_heading_last() {
        let variance = AttitudeVariance {
            tilt_north: 1.0,
            tilt_east: 2.0,
            heading: 3.0,
        };
        assert_eq!(variance.to_enu(), [2.0, 1.0, 3.0]);
    }

    /// Found by the adversarial suite (#44): rounded, `R D Rᵀ` and `M P M` are not their own
    /// transposes, and both are committed with nothing after them to repair it. Several
    /// attitudes, because whether the last bit differs depends on the rotation.
    #[test]
    fn the_attitude_blocks_a_start_and_a_heading_reset_write_are_exactly_symmetric() {
        let variance = AttitudeVariance {
            tilt_north: 4.1e-4,
            tilt_east: 3.7e-4,
            heading: 0.29,
        };
        for (roll, pitch, yaw) in [(0.3, -0.2, 1.1), (0.01, 0.02, -2.9), (-0.7, 0.4, 0.5)] {
            let attitude = attitude_of(roll, pitch, yaw);
            let block = variance.in_body(&attitude);
            assert_eq!(
                block,
                block.transpose(),
                "in_body at {roll}, {pitch}, {yaw}"
            );

            let mut p = with_attitude_block(block);
            let down = attitude
                .quaternion()
                .inverse_transform_vector(&Vector3::z());
            p.reset_attitude_direction(down, 0.05);
            let p = p.as_matrix();
            assert_eq!(*p, p.transpose(), "reset at {roll}, {pitch}, {yaw}");
        }
    }

    fn with_attitude_block(block: Matrix3<f32>) -> Covariance {
        let mut p = CovarianceMatrix::identity();
        let theta = ErrorState::AttitudeX.index();
        p.fixed_view_mut::<3, 3>(theta, theta).copy_from(&block);
        Covariance::from_matrix(p)
    }

    #[test]
    fn level_and_facing_north_the_body_diagonal_is_tilt_and_heading() {
        let p = with_attitude_block(Matrix3::from_diagonal(&Vector3::new(1.0, 2.0, 3.0)));
        let variance = AttitudeVariance::of(&Attitude::default(), &p);
        assert_eq!((variance.tilt_north, variance.tilt_east), (1.0, 2.0));
        assert_eq!(variance.heading, 3.0);
    }

    #[test]
    fn on_its_tail_body_x_is_the_heading_axis() {
        // Pitched 90° nose-up, body x points up, so the variance about it is heading's.
        // Mutating `of` to read the body diagonal reports it as tilt instead.
        let p = with_attitude_block(Matrix3::from_diagonal(&Vector3::new(1.0, 2.0, 3.0)));
        let variance = AttitudeVariance::of(&attitude_of(0.0, FRAC_PI_2, 0.0), &p);
        assert!((variance.heading - 1.0).abs() < 1e-6, "{variance:?}");
        assert!((variance.tilt_east - 2.0).abs() < 1e-6, "{variance:?}");
        assert!((variance.tilt_north - 3.0).abs() < 1e-6, "{variance:?}");
    }

    #[test]
    fn in_body_is_the_inverse_of_of() {
        let attitude = attitude_of(0.4, 1.2, -2.0);
        let ned = AttitudeVariance {
            tilt_north: 1e-4,
            tilt_east: 4e-4,
            heading: 0.1,
        };
        let p = with_attitude_block(ned.in_body(&attitude));
        let back = AttitudeVariance::of(&attitude, &p);
        assert!((back.tilt_north - ned.tilt_north).abs() < 1e-7, "{back:?}");
        assert!((back.tilt_east - ned.tilt_east).abs() < 1e-7, "{back:?}");
        assert!((back.heading - ned.heading).abs() < 1e-6, "{back:?}");
    }

    #[test]
    fn a_reset_along_a_body_axis_is_reset_state() {
        let mut p = CovarianceMatrix::from_fn(|i, j| 0.01 * (1 + i.min(j)) as f32);
        p.fill_diagonal(1.0);
        let mut along = Covariance::from_matrix(p);
        along.reset_attitude_direction(Vector3::z(), 0.25);
        let mut axis = Covariance::from_matrix(p);
        axis.reset_state(ErrorState::AttitudeZ, 0.25);
        assert_eq!(along, axis);
    }

    #[test]
    fn a_reset_along_a_direction_leaves_the_orthogonal_components_alone() {
        // `u` between body y and z: the reset component gets the variance and nothing else,
        // and the component orthogonal to `u` in that plane keeps its variance.
        let u = Vector3::new(0.0, 0.6, 0.8);
        let w = Vector3::new(0.0, 0.8, -0.6);
        let mut p = CovarianceMatrix::from_fn(|i, j| 0.01 * (1 + i.min(j)) as f32);
        p.fill_diagonal(1.0);
        let theta = ErrorState::AttitudeX.index();
        let before = p.fixed_view::<3, 3>(theta, theta).into_owned();
        let mut covariance = Covariance::from_matrix(p);
        covariance.reset_attitude_direction(u, 0.25);
        let q = covariance.as_matrix();
        let block = q.fixed_view::<3, 3>(theta, theta).into_owned();

        assert!(((u.transpose() * block * u)[0] - 0.25).abs() < 1e-6);
        assert!((u.transpose() * block * w)[0].abs() < 1e-6);
        assert!(((w.transpose() * block * w)[0] - (w.transpose() * before * w)[0]).abs() < 1e-6);
        for k in (0..STATES).filter(|k| !(theta..theta + 3).contains(k)) {
            let cross = u.dot(&q.fixed_view::<3, 1>(theta, k).into_owned());
            assert!(
                cross.abs() < 1e-6,
                "the reset component correlates with {k}: {cross}"
            );
        }
    }
}
