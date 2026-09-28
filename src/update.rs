//! The measurement update every observation shares: equations (23)–(27), the gate of
//! (37)–(38), and the injection and reset of (39)–(41).
//!
//! Nothing here knows which sensor it is correcting from. An observation arrives as its
//! innovation, its Jacobian and its noise — [`Observation`] — and the modules under
//! `observation/` are what build one; `Eskf` is what commits the result. So the whole update
//! is testable against a synthetic `H` with no filter around it.

use nalgebra::{Cholesky, Const, Matrix3, SMatrix, SVector, Vector3};

use crate::config::Gate;
use crate::health::Innovation;
use crate::math::{enforce_symmetry, exp_quat, skew};
use crate::propagate::Transition;
use crate::state::{Covariance, CovarianceMatrix, ErrorState, Offset, STATES, State};
use crate::units::{Acceleration, AngularRate, Attitude, Position, Seconds, Velocity};

/// One measurement, reduced to what (23)–(27) read: `y`, `H` and `R_m` for an `M`-dimensional
/// observation.
///
/// The observation model `h(x̂)` is not here. It is evaluated once, by whoever forms `y`, since
/// that is the only place (23) reads it.
pub(crate) struct Observation<const M: usize> {
    /// `y = z − h(x̂)`, equation (23).
    pub(crate) y: SVector<f32, M>,
    /// `H`, the Jacobian of `h` with respect to the error state.
    pub(crate) h: SMatrix<f32, M, STATES>,
    /// `H_b`, the Jacobian of `h` with respect to the barometric offset of (30′): one for
    /// a barometric altitude, zero for everything else.
    pub(crate) h_b: SVector<f32, M>,
    /// The diagonal of `R_m`. Every noise type this crate takes is a diagonal, so the
    /// off-diagonal entries are never carried.
    pub(crate) r_m: SVector<f32, M>,
    /// The diagonal of the `R` the gain and (27) are computed with: `r_m` as the observation
    /// module forms it, and (24′)'s `R̃` once [`correlated`](Self::correlated) has priced in a
    /// measurement's error persisting from the last. The gate reads `r_m`, because one
    /// measurement's innovation variance is `H P Hᵀ + R_m` however correlated the next is.
    pub(crate) r_gain: SVector<f32, M>,
}

impl<const M: usize> Observation<M> {
    /// The same observation with its gain computed at `R̃ = R_m · inflation`, equation (24′);
    /// `inflation` is [`correlation_inflation`](crate::math::correlation_inflation)'s.
    pub(crate) fn correlated(self, inflation: f32) -> Self {
        Self {
            r_gain: self.r_m * inflation,
            ..self
        }
    }

    /// The same observation of a state `age` in the past, expressed in today's error.
    /// Equation (23′): `H exp(−A τ)` to second order.
    ///
    /// A negative `age` is a state ahead of the present, and the same series carries it back.
    pub(crate) fn delayed(self, age: Seconds, a: &Transition) -> Self {
        let tau = age.as_secs();
        let ha = self.h * a;
        let haa = ha * a;
        Self {
            h: self.h - ha * tau + haa * (0.5 * tau * tau),
            ..self
        }
    }
}

/// What one update produced, before anything is committed.
///
/// The state and covariance of an accepted update travel together for the reason
/// `Propagated` pairs them: `Eskf` commits both or neither.
#[allow(
    clippy::large_enum_variant,
    reason = "no allocator to box into, and it is returned once per measurement"
)]
pub(crate) enum Update {
    /// The measurement passed the gate. The corrected nominal state and its covariance.
    Accepted {
        state: State,
        covariance: Covariance,
        offset: Offset,
        /// `δb̂`, the correction to the barometric offset.
        offset_correction: f32,
        ratio: f32,
        innovation: Innovation,
    },
    /// The measurement failed the gate. Nothing past the gate was computed.
    Rejected { ratio: f32, innovation: Innovation },
    /// The filter's own numbers could not support an update: `S` was not positive-definite,
    /// or the correction overflowed f32. The measurement is not at fault.
    Invalid,
}

/// Gate a measurement and, if it passes, fold it into the state. Equations (23)–(27) and
/// (37)–(41).
///
/// `S` is factored by Cholesky, and the factor serves both the gate and the gain:
/// `ε = yᵀ S⁻¹ y` and `K = P Hᵀ S⁻¹` each need a solve against `S`, and neither needs `S⁻¹`
/// itself. A correlated measurement is the exception, gated on its own `R_m` and gained on the
/// larger `R̃` of (24′), so its gain is solved against a second factor; where `r_gain` is
/// `r_m` the second factor is the first. The second costs 272 bytes of `update::<3>`'s frame
/// on `thumbv6m` and 64 on `thumbv7em`, 320 and 64 of `update::<1>`'s, measured by building
/// each without it.
///
/// The factorization is also the check that `S` is positive-definite. With `R_m > 0` and `P`
/// positive semi-definite it always is, so a failure means `P` has lost that property in f32 —
/// the filter's fault rather than the measurement's, reported as [`Update::Invalid`] rather
/// than gated.
///
/// The gate runs **before** the gain. A rejected measurement costs one factorization and one
/// solve, and nothing it could have changed has been computed.
///
/// The gain is formed as `(S⁻¹ H P)ᵀ`, which is `P Hᵀ S⁻¹` because `P` and `S` are both
/// symmetric; that is what lets one solve against `S` produce it.
///
/// The covariance update is Joseph form, (27), which Solà recommends over his own `(I − KH)P`
/// in his footnote 26. Both are exact in exact arithmetic; in f32 the short form subtracts
/// two nearly equal matrices and can leave `P` indefinite, which the next `S` inherits. The
/// Joseph form is a sum of two positive semi-definite terms, so rounding cannot take it below
/// zero. The unit tests show the difference on an ill-conditioned `P` rather than asserting
/// it here.
///
/// Every quantity is the sixteen-state one of (30′), the error state and the barometric
/// offset `b`, and is taken in blocks: `P`, `P_xb` and `P_bb`; `K` as `K_x` and `K_b`; and
/// `A = I − K H` as `A_xx`, `a_xb`, `a_bx` and `a_bb`, so that (27) is `A P Aᵀ + K R Kᵀ` block by
/// block. The augmented matrix written out is the obvious form, and the tests compare against
/// exactly that. Formed here, it cost 4168 bytes more of stack on `thumbv6m` — a 1024-byte
/// 16 × 16 for each 900-byte temporary, and a copy in and out of it.
///
/// The frame is the largest in the crate: `update::<3>` is 8088 bytes on `thumbv6m-none-eabi`
/// and 7960 on `thumbv7em-none-eabihf` at `opt-level = 3`, against 2832 for
/// `propagate_covariance`, the largest single frame propagation reaches. Most of it is (27),
/// whose `A_xx`, its two products and `K R Kᵀ` are each a 900-byte 15 × 15; the offset's blocks
/// are vectors, and cost 968 bytes over the fifteen-state update. That is comfortable on the
/// STM32H7 class `DESIGN.md` names and nearly all the RAM of an 8 KB Cortex-M0 part. #41, stack
/// high-water on hardware, is what would say a less obvious form is worth writing.
///
/// `M` is what the rest scales with, and a scalar source is cheaper rather than free:
/// `update::<1>` takes 6384 bytes on `thumbv6m` and 6240 on `thumbv7em`, so neither scalar
/// source moves the crate's high-water mark — `update::<3>` still sets it. The two scalar
/// sources share that one monomorphization: the barometer of (30) paid for it, and the
/// magnetic heading of (34)–(36) added 1204 bytes of `.text` linking the whole public API for
/// `thumbv6m` under fat LTO, 2.4 %, against the 4.1 % the barometer cost when it brought
/// `update::<1>` into existence. A dimension is what costs flash, not a source. That is the
/// price of the dimension being a type parameter, which is what makes a `Gate<M>` of the
/// wrong dimension a compile error (#58); trading it back for a runtime `M` is #41's call to
/// make with hardware numbers, not one to take on a 2 KB estimate.
pub(crate) fn update<const M: usize>(
    state: &State,
    covariance: &Covariance,
    offset: &Offset,
    observation: &Observation<M>,
    gate: Gate<M>,
) -> Update {
    let p = covariance.as_matrix();
    let Offset {
        cross: c,
        variance: v,
    } = *offset;
    let Observation {
        y,
        h,
        h_b,
        r_m,
        r_gain,
    } = observation;
    let correlated = r_gain != r_m;
    let (r_m, r_gain) = (
        SMatrix::<f32, M, M>::from_diagonal(r_m),
        SMatrix::<f32, M, M>::from_diagonal(r_gain),
    );

    // `H P` of the augmented state, by blocks: `[H P_xx + H_b P_bx, H P_xb + H_b P_bb]`.
    let hp_x = h * p + h_b * c.transpose();
    let hp_b = h * c + h_b * v;

    let hph = hp_x * h.transpose() + hp_b * h_b.transpose();
    let s = hph + r_m; // (24)
    let Some(gate_factor) = Cholesky::new(s) else {
        return Update::Invalid;
    };
    let innovation = Innovation::new(y, &s);

    let ratio = test_ratio(nis(y, &gate_factor), gate);
    if ratio > 1.0 {
        return Update::Rejected { ratio, innovation };
    }
    // (24′)'s `S̃`, for a source whose `R` the gain reads apart from the gate.
    let s_factor = if correlated {
        let Some(factor) = Cholesky::new(hph + r_gain) else {
            return Update::Invalid;
        };
        factor
    } else {
        gate_factor
    };

    let k_x = s_factor.solve(&hp_x).transpose(); // (25)
    let k_b = s_factor.solve(&hp_b).transpose();
    let dx = k_x * y; // (26), with no prior term: the error state was reset to zero
    let db = k_b.dot(&y.transpose());

    // (27), Joseph form, with `A = I − K H` and `P` both taken in blocks; see above.
    let a_xx = CovarianceMatrix::identity() - k_x * h;
    let a_xb = -(k_x * h_b);
    let a_bx = -(k_b * h);
    let a_bb = 1.0 - k_b.dot(&h_b.transpose());
    // The outer products `a_xb cᵀ` and `(AP)_xb a_xbᵀ` are added in place, by `ger`, rather
    // than formed: each would be another 900-byte 15 × 15 on the crate's largest frame.
    let mut ap_xx = a_xx * p;
    ap_xx.ger(1.0, &a_xb, &c, 1.0);
    let ap_xb = a_xx * c + a_xb * v;
    let ap_bx = a_bx * p + a_bb * c.transpose();
    let ap_bb = a_bx.dot(&c.transpose()) + a_bb * v;
    let kr_x = k_x * r_gain;
    let kr_b = k_b * r_gain;
    let cross = ap_xx * a_bx.transpose() + ap_xb * a_bb + kr_x * k_b.transpose();
    let variance = ap_bx.dot(&a_bx) + ap_bb * a_bb + kr_b.dot(&k_b);
    // The cross-covariance is taken from the rows above rather than averaged with its
    // transpose, which would cost a third 15 × 15 product; (42) for the block is
    // `reparameterize`'s.
    let mut p = ap_xx * a_xx.transpose() + kr_x * k_x.transpose();
    p.ger(1.0, &ap_xb, &a_xb, 1.0);

    // `Exp(δθ̂)` of (39) is taken unchecked, so a correction that overflowed is refused before
    // it reaches the quaternion; see `exp_quat`.
    if !dx.iter().all(|value| value.is_finite()) || !db.is_finite() {
        return Update::Invalid;
    }
    let state = inject(state, &dx);
    // (41), applied to the block and the offset's row apart: returned together, as a pair,
    // the two cost a 900-byte copy of `P` on this frame.
    let g_theta = reset_jacobian(attitude_error(&dx));
    let offset = reparameterize_offset(&Offset { cross, variance }, g_theta);
    let covariance = reparameterize(p, g_theta);

    if !state.is_finite() || !covariance.is_finite() || !offset.is_finite() {
        return Update::Invalid;
    }
    Update::Accepted {
        state,
        covariance,
        offset,
        offset_correction: db,
        ratio,
        innovation,
    }
}

/// `ε = yᵀ S⁻¹ y`, the normalized innovation squared of equation (37), through the Cholesky
/// factor of `S` rather than its inverse.
fn nis<const M: usize>(y: &SVector<f32, M>, s_factor: &Cholesky<f32, Const<M>>) -> f32 {
    y.dot(&s_factor.solve(y))
}

/// `r = ε / γ`, the test ratio of equation (38): 1 at the threshold whatever the dimension.
///
/// `γ` needs no guard. [`Gate`] cannot hold zero, a negative, NaN or an infinity, which is
/// the whole list of values that would make this division lie.
fn test_ratio<const M: usize>(epsilon: f32, gate: Gate<M>) -> f32 {
    epsilon / gate.threshold()
}

/// Compose the estimated error into the nominal state. Equations (39) and (40).
///
/// Attitude composes on the right, as (15)'s increment does, because `δθ` is the local
/// (body-frame) error of equation (2). The quaternion is renormalized for the same reason
/// (15) renormalizes: a product of two unit quaternions drifts off the manifold in f32.
fn inject(state: &State, dx: &SVector<f32, STATES>) -> State {
    let block = |at: ErrorState| -> Vector3<f32> { dx.fixed_rows::<3>(at.index()).clone_owned() };

    let mut attitude = state.attitude.body_to_ned() * exp_quat(block(ErrorState::AttitudeX));
    attitude.renormalize();

    State {
        attitude: Attitude::from_body_to_ned(attitude),
        position: Position::from_vector(state.position.vector() + block(ErrorState::PositionNorth)),
        velocity: Velocity::from_vector(state.velocity.vector() + block(ErrorState::VelocityNorth)),
        accel_bias: Acceleration::from_vector(
            state.accel_bias.vector() + block(ErrorState::AccelBiasX),
        ),
        gyro_bias: AngularRate::from_vector(
            state.gyro_bias.vector() + block(ErrorState::GyroBiasX),
        ),
        ..*state
    }
}

/// `δθ̂`, the attitude block of the correction.
fn attitude_error(dx: &SVector<f32, STATES>) -> Vector3<f32> {
    dx.fixed_rows::<3>(ErrorState::AttitudeX.index())
        .clone_owned()
}

/// `G_θ`, the attitude block of the reset Jacobian `G` that carries the covariance across
/// the injection of (39). Equation (41).
///
/// `G` is the exact reset Jacobian, `I − [½δθ̂]ₓ` in its attitude block, rather than the
/// identity Solà gives as the usual approximation ((285)–(288)). Injecting `δθ̂` moves the
/// point the attitude error is measured from, so the error that remains is expressed about a
/// rotated reference; `G` is that rotation to first order. The approximation drops a term of
/// the size of the correction, which is small after an ordinary update and is not after the
/// first heading a coarse start fuses — the one place a large attitude correction is
/// expected. The exact block costs one 3 × 3 skew.
///
/// The block it is applied to, and what that costs, are [`reparameterize`].
fn reset_jacobian(delta_theta: Vector3<f32>) -> Matrix3<f32> {
    Matrix3::identity() - skew(0.5 * delta_theta)
}

/// `G P Gᵀ` with `G` the identity outside its attitude block. Equation (41).
///
/// Applied to the attitude rows and columns only, which is all of `G P Gᵀ` that differs from
/// `P`. The full product is the more obvious form and costs a 15 × 15 `G` and two more
/// temporaries of `P`'s size: at `opt-level = 3` it measured 864 bytes more of stack for
/// `update` on `thumbv6m-none-eabi`, on a frame that is already the largest in the crate. The
/// difference is quoted rather than the two totals, which move by tens of bytes with codegen
/// — adding `update::<1>` for (30) moved this one without touching a line of it.
///
/// Which `G_θ` to use is the caller's, because the two resets in the crate move the nominal
/// attitude by different amounts: [`update`] injects a correction and uses (41)'s first-order
/// Jacobian, while the heading adoption behind
/// [`Fusion::Reset`](crate::Fusion::Reset) turns the nominal by up to π and passes the exact
/// rotation. The block is the same either way, and so is the reason it has to move: the `δθ`
/// of (2) is referenced to the nominal's body axes, so turning the nominal turns the axes
/// every attitude row and column is written in.
///
/// (42) runs last, as after every covariance operation: both products of (27) and this one
/// drift off symmetry in f32.
pub(crate) fn reparameterize(mut p: CovarianceMatrix, g_theta: Matrix3<f32>) -> Covariance {
    let theta = ErrorState::AttitudeX.index();

    // `G` from the left rotates the attitude rows, `Gᵀ` from the right the attitude columns.
    let rows = g_theta * p.fixed_rows::<3>(theta);
    p.fixed_rows_mut::<3>(theta).copy_from(&rows);
    let columns = p.fixed_columns::<3>(theta) * g_theta.transpose();
    p.fixed_columns_mut::<3>(theta).copy_from(&columns);
    enforce_symmetry(&mut p);
    Covariance::from_matrix(p)
}

/// [`reparameterize`] for the barometric offset's row of (30′), `G P_xb`.
///
/// One side only: the offset is a height and does not turn with the body, so, like the bias
/// blocks, only its correlations with attitude are rewritten in the new axes.
pub(crate) fn reparameterize_offset(offset: &Offset, g_theta: Matrix3<f32>) -> Offset {
    let theta = ErrorState::AttitudeX.index();
    let mut offset = *offset;
    let cross = g_theta * offset.cross.fixed_rows::<3>(theta);
    offset.cross.fixed_rows_mut::<3>(theta).copy_from(&cross);
    offset
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    const TOLERANCE: f32 = 1.0e-6;

    /// A covariance with every error-state variance at `variance` and no correlations.
    fn diagonal(variance: f32) -> Covariance {
        Covariance::from_matrix(CovarianceMatrix::from_diagonal_element(variance))
    }

    /// `H` observing the position block directly, as (28) does, without importing it: the
    /// core is tested apart from any observation model.
    fn observes_position() -> SMatrix<f32, 3, STATES> {
        let mut h = SMatrix::<f32, 3, STATES>::zeros();
        h.fixed_view_mut::<3, 3>(0, ErrorState::PositionNorth.index())
            .copy_from(&Matrix3::identity());
        h
    }

    /// `H` observing one scalar: down position, as a barometer would.
    fn observes_down() -> SMatrix<f32, 1, STATES> {
        let mut h = SMatrix::<f32, 1, STATES>::zeros();
        h[(0, ErrorState::PositionDown.index())] = 1.0;
        h
    }

    fn position(y: [f32; 3], r_m: f32) -> Observation<3> {
        Observation {
            y: SVector::from(y),
            h: observes_position(),
            h_b: SVector::zeros(),
            r_m: SVector::from([r_m; 3]),
            r_gain: SVector::from([r_m; 3]),
        }
    }

    fn gate<const M: usize>(threshold: f32) -> Gate<M> {
        Gate::new(threshold).expect("a positive, finite threshold")
    }

    fn is_symmetric(p: &CovarianceMatrix) -> bool {
        (p - p.transpose()).iter().all(|entry| *entry == 0.0)
    }

    fn accepted(outcome: Update) -> (State, Covariance, f32) {
        match outcome {
            Update::Accepted {
                state,
                covariance,
                ratio,
                ..
            } => (state, covariance, ratio),
            Update::Rejected { ratio, .. } => panic!("rejected at r = {ratio}"),
            Update::Invalid => panic!("invalid"),
        }
    }

    #[test]
    fn a_consistent_measurement_shrinks_what_it_observes_and_leaves_p_symmetric() {
        let prior = diagonal(4.0);
        let outcome = update(
            &State::default(),
            &prior,
            &Offset::default(),
            &position([0.5, -0.5, 0.2], 1.0),
            gate(7.8),
        );
        let (state, covariance, _) = accepted(outcome);

        for axis in [
            ErrorState::PositionNorth,
            ErrorState::PositionEast,
            ErrorState::PositionDown,
        ] {
            // Scalar Kalman: 4 · 1 / (4 + 1).
            assert!(
                (covariance.variance(axis) - 0.8).abs() < TOLERANCE,
                "{axis:?}"
            );
        }
        // Uncorrelated with position, so nothing to learn and nothing lost.
        assert_eq!(covariance.variance(ErrorState::VelocityNorth), 4.0);
        assert!(is_symmetric(covariance.as_matrix()));
        // Gain 4/5 of the innovation.
        assert!((state.position.vector() - Vector3::new(0.4, -0.4, 0.16)).norm() < TOLERANCE);
    }

    #[test]
    fn the_gate_reads_the_fixs_own_variance_and_the_gain_the_larger_one() {
        // A fix fused at (24′)'s variance, nine times its own: the gate asks whether this one
        // fix is consistent, which its own `R_m` answers, while the gain is what a run of
        // such fixes is worth.
        let prior = diagonal(1.0);
        let own = position([1.5, 0.0, 0.0], 1.0);
        let correlated = Observation {
            r_gain: SVector::from([9.0; 3]),
            ..position([1.5, 0.0, 0.0], 1.0)
        };
        let (_, _, white_ratio) = accepted(update(
            &State::default(),
            &prior,
            &Offset::default(),
            &own,
            gate(7.8),
        ));
        let (state, covariance, ratio) = accepted(update(
            &State::default(),
            &prior,
            &Offset::default(),
            &correlated,
            gate(7.8),
        ));

        assert_eq!(ratio, white_ratio);
        // Scalar Kalman at R = 9: a tenth of the innovation, and 1 · 9 / (1 + 9) of the
        // variance.
        assert!((state.position.vector()[0] - 0.15).abs() < TOLERANCE);
        assert!((covariance.variance(ErrorState::PositionNorth) - 0.9).abs() < TOLERANCE);
    }

    #[test]
    fn a_correlation_carries_the_correction_to_what_was_not_observed() {
        // Velocity north fully correlated with position north: a position fix must move it.
        let mut p = *diagonal(1.0).as_matrix();
        let (pn, vn) = (
            ErrorState::PositionNorth.index(),
            ErrorState::VelocityNorth.index(),
        );
        p[(pn, vn)] = 0.9;
        p[(vn, pn)] = 0.9;
        let (state, covariance, _) = accepted(update(
            &State::default(),
            &Covariance::from_matrix(p),
            &Offset::default(),
            &position([1.0, 0.0, 0.0], 1.0),
            gate(7.8),
        ));

        assert!((state.velocity.x() - 0.45).abs() < TOLERANCE); // 0.9 / (1 + 1)
        assert!(covariance.variance(ErrorState::VelocityNorth) < 1.0);
    }

    #[test]
    fn zero_innovation_moves_nothing_and_still_shrinks_p() {
        let start = State::default();
        let prior = diagonal(4.0);
        let (state, covariance, ratio) = accepted(update(
            &start,
            &prior,
            &Offset::default(),
            &position([0.0; 3], 1.0),
            gate(7.8),
        ));

        assert_eq!(state, start);
        assert_eq!(ratio, 0.0);
        assert!(covariance.variance(ErrorState::PositionNorth) < 4.0);
    }

    #[test]
    fn joseph_form_survives_an_ill_conditioned_p_that_the_short_form_does_not() {
        // A nearly exact measurement of a very uncertain position: `KH` is within f32
        // rounding of the identity on the observed block, so `(I − KH)P` subtracts two
        // numbers that agree to every bit and keeps whatever rounding left behind.
        let mut p = *diagonal(1.0e6).as_matrix();
        let (pn, vn) = (
            ErrorState::PositionNorth.index(),
            ErrorState::VelocityNorth.index(),
        );
        p[(pn, vn)] = 999_999.0;
        p[(vn, pn)] = 999_999.0;
        let prior = Covariance::from_matrix(p);
        let observation = Observation {
            y: SVector::from([0.0]),
            h: {
                let mut h = SMatrix::<f32, 1, STATES>::zeros();
                h[(0, pn)] = 1.0;
                h
            },
            h_b: SVector::zeros(),
            r_m: SVector::from([1.0e-6]),
            r_gain: SVector::from([1.0e-6]),
        };

        let (_, joseph, _) = accepted(update(
            &State::default(),
            &prior,
            &Offset::default(),
            &observation,
            gate::<1>(3.84),
        ));
        let joseph = joseph.as_matrix();
        assert!(is_symmetric(joseph));
        assert!(
            Cholesky::new(*joseph).is_some(),
            "Joseph form stays positive-definite"
        );

        // The short form, from the same `K`.
        let s = observation.h * p * observation.h.transpose()
            + SMatrix::<f32, 1, 1>::from_diagonal(&observation.r_m);
        let k = p * observation.h.transpose() * s.try_inverse().expect("S is 1 x 1 and > 0");
        let short = (CovarianceMatrix::identity() - k * observation.h) * p;
        // Positive-definiteness alone, not symmetry: f32 rounding leaves the short form
        // slightly asymmetric on its own, so an assertion that accepted either would pass
        // without the short form ever losing the property Joseph form is chosen for.
        // Cholesky reads the lower triangle only, so asymmetry does not decide it.
        assert!(
            Cholesky::new(short).is_none(),
            "the short form stays positive-definite here, so this fixture shows nothing"
        );
    }

    #[test]
    fn the_gate_boundary_is_a_ratio_of_exactly_one() {
        // dim 1: y = 2, S = 0.5 + 0.5 = 1, so ε = 4 exactly.
        let scalar = Observation {
            y: SVector::from([2.0]),
            h: observes_down(),
            h_b: SVector::zeros(),
            r_m: SVector::from([0.5]),
            r_gain: SVector::from([0.5]),
        };
        let prior = diagonal(0.5);
        let (_, _, ratio) = accepted(update(
            &State::default(),
            &prior,
            &Offset::default(),
            &scalar,
            gate::<1>(4.0),
        ));
        assert_eq!(ratio, 1.0);

        // dim 3: y = (1, 2, 2), S = I, so ε = 9 exactly.
        let vector = position([1.0, 2.0, 2.0], 0.5);
        let (_, _, ratio) = accepted(update(
            &State::default(),
            &prior,
            &Offset::default(),
            &vector,
            gate::<3>(9.0),
        ));
        assert_eq!(ratio, 1.0);

        // Just past it, each rejects.
        assert!(matches!(
            update(&State::default(), &prior, &Offset::default(), &scalar, gate::<1>(3.99)),
            Update::Rejected { ratio, .. } if ratio > 1.0
        ));
        assert!(matches!(
            update(&State::default(), &prior, &Offset::default(), &vector, gate::<3>(8.99)),
            Update::Rejected { ratio, .. } if ratio > 1.0
        ));
    }

    #[test]
    fn the_test_ratio_is_joint_over_the_correlated_innovation() {
        // Position north and east strongly correlated: an innovation along the correlation
        // is ordinary, one across it is not, though each component is the same size.
        let mut p = *diagonal(1.0).as_matrix();
        p[(0, 1)] = 0.9;
        p[(1, 0)] = 0.9;
        let prior = Covariance::from_matrix(p);
        let ratio = |y| match update(
            &State::default(),
            &prior,
            &Offset::default(),
            &position(y, 0.1),
            gate(7.8),
        ) {
            Update::Accepted { ratio, .. } | Update::Rejected { ratio, .. } => ratio,
            Update::Invalid => panic!("invalid"),
        };

        assert!(ratio([1.0, -1.0, 0.0]) > 5.0 * ratio([1.0, 1.0, 0.0]));
    }

    #[test]
    fn the_published_innovation_is_y_and_the_diagonal_of_s() {
        let prior = diagonal(4.0);
        let Update::Accepted { innovation, .. } = update(
            &State::default(),
            &prior,
            &Offset::default(),
            &position([0.5, -0.5, 0.2], 1.0),
            gate(7.8),
        ) else {
            panic!("expected an acceptance");
        };
        assert_eq!(innovation.values(), &[0.5, -0.5, 0.2]);
        assert_eq!(innovation.variances(), &[5.0, 5.0, 5.0]);
    }

    #[test]
    fn a_rejection_carries_its_ratio_and_innovation() {
        let prior = diagonal(1.0);
        let Update::Rejected { ratio, innovation } = update(
            &State::default(),
            &prior,
            &Offset::default(),
            &position([10.0, 0.0, 0.0], 1.0),
            gate(7.8),
        ) else {
            panic!("expected a rejection");
        };
        assert!((ratio - 50.0 / 7.8).abs() < 1.0e-4); // ε = 100 / 2
        assert_eq!(innovation.values(), &[10.0, 0.0, 0.0]);
    }

    #[test]
    fn an_indefinite_s_is_invalid_rather_than_gated() {
        // A negative variance on the observed axis, larger than `R_m`: `S` < 0.
        let mut p = *diagonal(1.0).as_matrix();
        p[(2, 2)] = -2.0;
        let outcome = update(
            &State::default(),
            &Covariance::from_matrix(p),
            &Offset::default(),
            &Observation {
                y: SVector::from([0.1]),
                h: observes_down(),
                h_b: SVector::zeros(),
                r_m: SVector::from([1.0]),
                r_gain: SVector::from([1.0]),
            },
            gate::<1>(3.84),
        );
        assert!(matches!(outcome, Update::Invalid));
    }

    #[test]
    fn a_correction_that_overflows_is_invalid_rather_than_committed() {
        // An innovation well inside a gate that wide, whose correction `K y = 5e31` is more
        // than an ulp of the position it is added to, at `f32::MAX`.
        let start = State {
            position: Position::ned(f32::MAX, 0.0, 0.0),
            ..State::default()
        };
        let outcome = update(
            &start,
            &diagonal(1.0e36),
            &Offset::default(),
            &position([1.0e32, 0.0, 0.0], 1.0e36),
            Gate::<3>::new(f32::MAX).expect("finite"),
        );
        assert!(matches!(outcome, Update::Invalid));
    }

    #[test]
    fn injection_composes_attitude_on_the_right_and_adds_the_rest() {
        let start = State {
            attitude: Attitude::from_body_to_ned(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.5)),
            ..State::default()
        };
        let mut dx = SVector::<f32, STATES>::zeros();
        dx[ErrorState::AttitudeX.index()] = 0.01; // roll, about body x
        dx[ErrorState::GyroBiasZ.index()] = 0.002;
        dx[ErrorState::AccelBiasY.index()] = -0.03;

        let injected = inject(&start, &dx);
        let expected = start.attitude.body_to_ned() * exp_quat(Vector3::new(0.01, 0.0, 0.0));
        assert!(injected.attitude.body_to_ned().angle_to(&expected) < TOLERANCE);
        assert_eq!(injected.gyro_bias.z(), 0.002);
        assert_eq!(injected.accel_bias.y(), -0.03);
    }

    #[test]
    fn the_reset_jacobian_is_the_identity_without_an_attitude_correction() {
        let p = *diagonal(2.0).as_matrix();
        assert_eq!(
            reparameterize(p, reset_jacobian(Vector3::zeros())).as_matrix(),
            &p
        );
    }

    #[test]
    fn the_reset_jacobian_rotates_only_the_attitude_block() {
        let mut p = *diagonal(2.0).as_matrix();
        let (theta, pn) = (
            ErrorState::AttitudeX.index(),
            ErrorState::PositionNorth.index(),
        );
        p[(theta, pn)] = 0.5;
        p[(pn, theta)] = 0.5;
        let reset = reparameterize(p, reset_jacobian(Vector3::new(0.0, 0.0, 0.2)));
        let reset = reset.as_matrix();

        // `(I − [½δθ]ₓ)` about z mixes attitude x and y: the x–position correlation leaks
        // into y, and x keeps it.
        assert_eq!(reset[(theta, pn)], 0.5);
        assert!((reset[(theta + 1, pn)] - (-0.1 * 0.5)).abs() < TOLERANCE);
        // Rows outside the attitude block are untouched.
        assert_eq!(reset[(pn, pn)], 2.0);
        assert!(is_symmetric(reset));
    }

    /// A covariance with the offset correlated to down position and to vertical velocity,
    /// and nothing correlated with attitude, so that no attitude correction arises and (41)
    /// leaves the result as (27) produced it.
    fn correlated_with_the_offset() -> (Covariance, Offset) {
        let mut p = *diagonal(4.0).as_matrix();
        let (pd, vd) = (
            ErrorState::PositionDown.index(),
            ErrorState::VelocityDown.index(),
        );
        p[(pd, vd)] = 1.5;
        p[(vd, pd)] = 1.5;
        let mut cross = SVector::<f32, STATES>::zeros();
        cross[pd] = -2.0;
        cross[vd] = 0.5;
        (
            Covariance::from_matrix(p),
            Offset {
                cross,
                variance: 3.0,
            },
        )
    }

    /// (23)–(27) on the sixteen-state covariance, written as the equations read: what the
    /// block form in `update` must reproduce.
    fn augmented_joseph<const M: usize>(
        covariance: &Covariance,
        offset: &Offset,
        observation: &Observation<M>,
    ) -> (SMatrix<f32, 16, 16>, SVector<f32, 16>) {
        let mut p = SMatrix::<f32, 16, 16>::zeros();
        p.fixed_view_mut::<STATES, STATES>(0, 0)
            .copy_from(covariance.as_matrix());
        p.fixed_view_mut::<STATES, 1>(0, STATES)
            .copy_from(&offset.cross);
        p.fixed_view_mut::<1, STATES>(STATES, 0)
            .copy_from(&offset.cross.transpose());
        p[(STATES, STATES)] = offset.variance;
        let mut h = SMatrix::<f32, M, 16>::zeros();
        h.fixed_columns_mut::<STATES>(0).copy_from(&observation.h);
        h.set_column(STATES, &observation.h_b);
        let r = SMatrix::<f32, M, M>::from_diagonal(&observation.r_m);
        let s = h * p * h.transpose() + r;
        let k = p * h.transpose() * s.try_inverse().expect("S is positive-definite");
        let i_kh = SMatrix::<f32, 16, 16>::identity() - k * h;
        (
            i_kh * p * i_kh.transpose() + k * r * k.transpose(),
            k * observation.y,
        )
    }

    fn assert_matches_augmented<const M: usize>(observation: &Observation<M>, gate: Gate<M>) {
        let (prior, offset) = correlated_with_the_offset();
        let (want, dx) = augmented_joseph(&prior, &offset, observation);
        let Update::Accepted {
            covariance,
            offset,
            offset_correction,
            ..
        } = update(&State::default(), &prior, &offset, observation, gate)
        else {
            panic!("a consistent measurement");
        };
        let close = |a: f32, b: f32| (a - b).abs() < 1e-4 * (1.0 + b.abs());
        for i in 0..STATES {
            for j in 0..STATES {
                assert!(
                    close(covariance.as_matrix()[(i, j)], want[(i, j)]),
                    "P[{i},{j}]"
                );
            }
            assert!(close(offset.cross[i], want[(i, STATES)]), "P_xb[{i}]");
        }
        assert!(close(offset.variance, want[(STATES, STATES)]), "P_bb");
        assert!(close(offset_correction, dx[STATES]), "δb");
    }

    #[test]
    fn the_block_update_is_the_sixteen_state_joseph_update() {
        let mut h = SMatrix::<f32, 1, STATES>::zeros();
        h[(0, ErrorState::PositionDown.index())] = 1.0;
        let baro = Observation {
            y: SVector::from([0.7]),
            h,
            h_b: SVector::from([1.0]),
            r_m: SVector::from([0.5]),
            r_gain: SVector::from([0.5]),
        };
        assert_matches_augmented(&baro, gate::<1>(100.0));

        // A source the offset does not enter still moves its cross-covariance, through the
        // height the two are correlated with.
        assert_matches_augmented(&position([0.3, -0.2, 0.9], 1.0), gate::<3>(100.0));
    }

    #[test]
    fn an_offset_reset_rotates_its_attitude_entries_and_nothing_else() {
        let (theta, pd) = (
            ErrorState::AttitudeX.index(),
            ErrorState::PositionDown.index(),
        );
        let mut cross = SVector::<f32, STATES>::zeros();
        cross[theta] = 0.5;
        cross[pd] = -2.0;
        let offset = Offset {
            cross,
            variance: 3.0,
        };
        let reset = reparameterize_offset(&offset, reset_jacobian(Vector3::new(0.0, 0.0, 0.2)));
        assert_eq!(reset.cross[theta], 0.5);
        assert!((reset.cross[theta + 1] - (-0.1 * 0.5)).abs() < TOLERANCE);
        assert_eq!(reset.cross[pd], -2.0);
        assert_eq!(reset.variance, 3.0);
    }
}
