//! The primitives the equations share: `[u]ₓ` and `Exp(φ)` from the operators table, the
//! `wrap(·)` of (35), both halves of (42) — the symmetry enforcement and the diagonal
//! variance floor — and the inflation of (24′).
//!
//! Nothing here holds state or reads configuration, which is why it is a module of its own:
//! each function is checkable against its definition without a filter around it.

use core::f32::consts::{PI, TAU};

use nalgebra::{ComplexField, Matrix3, Quaternion, SMatrix, UnitQuaternion, Vector3};

use crate::state::{CovarianceMatrix, ErrorState, Offset, STATES};
use crate::units::Seconds;

/// The most (24′) multiplies a variance by: a measurement fused at a millionth of its
/// information adds none worth counting. Any variance under about 3e32 stays finite times this,
/// so (27)'s `K R Kᵀ` does too; one above it overflows and the update is refused as
/// `StateInvalid`: a σ of 1.7e16, which no sensor reports.
const MAX_INFLATION: f32 = 1.0e6;

/// `(1 + ρ) / (1 − ρ)`, `ρ = exp(−Δt / τ)`: what (24′) multiplies a correlated measurement's
/// variance by, `Δt` after the previous one.
///
/// Written as `(2 − x) / x` with `x = 1 − ρ = −expm1(−Δt / τ)`: at `Δt ≪ τ`, `ρ` rounds to 1 in
/// `f32` and `1 − ρ` to zero, where `expm1` keeps the digits. The factor is 1 for a long
/// interval, where measurements are independent again, and `2τ / Δt` for a short one, so a
/// source reporting faster than its error changes buys no more per second than one reporting
/// at `τ`. At `Δt = 0`, or a `τ` too long for `f32`, the same error arrives twice: `ρ` is 1,
/// the factor unbounded, and it saturates at [`MAX_INFLATION`].
///
/// 1 with no interval to measure (the first measurement) and for a `τ` that is `None`, not a
/// number or not positive: white, which is (24) as written.
pub(crate) fn correlation_inflation(interval: Option<Seconds>, tau: Option<Seconds>) -> f32 {
    let (Some(dt), Some(tau)) = (interval, tau) else {
        return 1.0;
    };
    let (dt, tau) = (dt.as_secs(), tau.as_secs());
    if !(dt >= 0.0 && tau > 0.0) {
        return 1.0;
    }
    let x = -ComplexField::exp_m1(-dt / tau);
    ((2.0 - x) / x).min(MAX_INFLATION)
}

/// The skew-symmetric matrix `[u]ₓ` of the operators table, so that `[u]ₓ v = u × v`.
///
/// Written out rather than taken from `Vector3::cross_matrix`, which is the same matrix:
/// the sign convention is what a reader checks this against, and it should be on the page
/// next to the equations that use it.
#[rustfmt::skip]
pub(crate) fn skew(u: Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(
         0.0, -u.z,  u.y,
         u.z,  0.0, -u.x,
        -u.y,  u.x,  0.0,
    )
}

/// `Exp(φ)`, the rotation vector to quaternion map of the operators table, used by (15)
/// and (39).
///
/// `Exp(φ) = [cos(‖φ‖/2), (φ/‖φ‖) sin(‖φ‖/2)]`, evaluated through the scale factor
/// `sin(‖φ‖/2)/‖φ‖` so that the vector part is one multiplication of `φ`.
///
/// Not `UnitQuaternion::from_scaled_axis`, which is the same map but substitutes the
/// identity for `‖φ‖ ≤ 2 f32::EPSILON` (2.4 × 10⁻⁷ rad), discarding the first-order term
/// rather than evaluating it. The series below keeps it, and a rotation increment the
/// filter is asked to compose is never silently dropped.
///
/// The result is unit by the Pythagorean identity, so it is taken unchecked; normalizing
/// would divide by 1 ± ε. (15) renormalizes after composition, where the error does grow.
///
/// `φ` must be finite, and callers owe that check. Unchecked construction is what makes a
/// non-finite `φ` dangerous rather than merely wrong: `‖φ‖` is then NaN, the small-angle
/// test is false, and the result is a `UnitQuaternion` that is not a unit quaternion — every
/// rotation after it is NaN with nothing reporting so. The refusal cannot live here, since
/// the return type has no channel to refuse through; it belongs where there is a typed
/// outcome, and [`Eskf::predict`](crate::Eskf::predict) is one — it refuses a non-finite
/// sample as [`Propagation::NotFinite`](crate::Propagation::NotFinite), so the `φ` that
/// (15) builds from a gyroscope is finite before it arrives. The `δθ̂` (39) injects comes
/// from the correction rather than from a sensor, so it owes its own check.
pub(crate) fn exp_quat(phi: Vector3<f32>) -> UnitQuaternion<f32> {
    let angle = phi.norm();
    let half = 0.5 * angle;

    // `sin(‖φ‖/2)/‖φ‖` is 0/0 at φ = 0 — the branch every implementation gets wrong. Its
    // series is ½ − ‖φ‖²/48 + ‖φ‖⁴/3840; at the threshold the dropped term is 2.6 × 10⁻¹⁶,
    // eight orders of magnitude inside an f32 ulp of ½.
    let scale = if angle < SMALL_ANGLE {
        0.5 - angle * angle / 48.0
    } else {
        ComplexField::sin(half) / angle
    };

    UnitQuaternion::new_unchecked(Quaternion::new(
        ComplexField::cos(half),
        scale * phi.x,
        scale * phi.y,
        scale * phi.z,
    ))
}

/// The rotation angle below which [`exp_quat`] takes the series form of its scale factor.
const SMALL_ANGLE: f32 = 1.0e-3;

/// `wrap(·)` of the operators table: an angle reduced to `(-π, π]`. Used by (6) and (35).
///
/// The interval is half-open, so `wrap_pi(-π)` is `+π` and `wrap_pi(π)` is `π`. Which end
/// is closed is arbitrary; that exactly one of them is, is not, and it makes the function
/// idempotent — an angle already in range comes back bit-identical.
///
/// Reduced by remainder and one correction rather than by `angle − 2π ⌈(angle − π)/2π⌉`,
/// which subtracts nearby quantities and so can land outside the interval it computes: at
/// `-3π` that form returns 3.1415930, one ulp above `π`. Here `%` is exact (IEEE `fmod`
/// rounds nothing), and each correction subtracts quantities within a factor of two of
/// each other, which is exact as well. So for a finite angle the range is a guarantee, not
/// a tolerance. A non-finite angle stays non-finite: `±∞ % 2π` is NaN, and NaN fails both
/// comparisons, so the guarantee is on the caller's finiteness and not on this function.
pub(crate) fn wrap_pi(angle: f32) -> f32 {
    let remainder = angle % TAU; // exact, and in (-2π, 2π)
    if remainder > PI {
        remainder - TAU
    } else if remainder <= -PI {
        remainder + TAU
    } else {
        remainder
    }
}

/// Equation (42): `P ← ½(P + Pᵀ)`, which (42) asks for after every covariance operation.
///
/// Swept over the upper triangle in place rather than written as the equation reads,
/// `*p = (*p + p.transpose()) * 0.5`, which materializes two 15×15 temporaries. At
/// `opt-level = 3` the equation form's stack frame measures 1884 bytes on
/// `thumbv6m-none-eabi` and 1820 on `thumbv7em-none-eabihf`, against 108 and 0 for the
/// sweep. (42) runs after every covariance operation, so that frame sits under `predict`
/// beneath the temporaries propagation needs of its own, and 1884 bytes is a quarter of
/// the RAM on an 8 KB Cortex-M0 part.
///
/// The two forms agree bit for bit — `a + a` and a multiplication by ½ are both exact in
/// binary floating point — so an already-symmetric `P` is unchanged either way and the
/// cheaper one costs no accuracy.
pub(crate) fn enforce_symmetry<const N: usize>(p: &mut SMatrix<f32, N, N>) {
    // Both indices stay below `N`, which is the dimension of `P`, so neither the read nor
    // the write can be out of range: `nalgebra` indexing panics, and nothing in `src/` may.
    for i in 0..N {
        for j in (i + 1)..N {
            let mean = 0.5 * (p[(i, j)] + p[(j, i)]);
            p[(i, j)] = mean;
            p[(j, i)] = mean;
        }
    }
}

/// The smallest variance each error state may hold, in the `ErrorState` ordering
/// `[δp δv δθ δβa δβg]`. Equation (42′).
///
/// One floor per group rather than one number for the matrix, because the fifteen states
/// carry five units — m², (m/s)², rad², (m s⁻²)² and (rad/s)² — and a single "small positive
/// value" would be a different claim in each of them. Both production estimators floor per
/// group for that reason: PX4 `constrainStateVariances` at
/// `src/modules/ekf2/EKF/covariance.cpp:250-289` (`c4e4ef98e9`), with `kGyroBiasVarianceMin`
/// and `kAccelBiasVarianceMin` at `EKF/ekf.h:494-495`, and ArduPilot `ConstrainVariances` at
/// `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1878-1935` (`368dc0c428`), whose
/// `POS_STATE_MIN_VARIANCE` and `VEL_STATE_MIN_VARIANCE` are `1e-4` at
/// `AP_NavEKF3_core.h:84-85`.
///
/// These are PX4's values, and the corpus says they sit below anything an honest source
/// drives the filter to: across the thirteen logs of `data/manifest.txt` and the thirteen
/// scenarios of `examples/simulate.rs`, the smallest variance any state reaches at an epoch
/// is 1.9e-4 m² of position on `89a498ce`, an RTK receiver, 1.7e-6 (rad/s)² of gyroscope
/// bias on the same log and on `2b2ad123`, the other RTK log, 4.8e-5 rad² of attitude on
/// `gnss_heading`, 7.3e-4 (m s⁻²)² of accelerometer bias on `093e806a` and 5.6e-4 (m/s)² of
/// velocity on `cd7e0001`. Two to six decades of headroom, so
/// [`Diagnostics::floored`](crate::Diagnostics::floored) reads zero on all thirteen,
/// `cd7e0001` included, whose receiver reports a 0.43 mm/s velocity after touchdown and is
/// fused raw. Measured as σ² from the replay output's six-decimal σ columns.
#[rustfmt::skip]
const FLOOR: [f32; STATES] = [
    1e-6, 1e-6, 1e-6, // δp, m²
    1e-6, 1e-6, 1e-6, // δv, (m/s)²
    1e-9, 1e-9, 1e-9, // δθ, rad²
    1e-9, 1e-9, 1e-9, // δβa, (m s⁻²)²
    1e-9, 1e-9, 1e-9, // δβg, (rad/s)²
];

/// Raise the barometric offset's variance to its floor, the position group's, since `b` of
/// (30′) is a height. Equation (42′). Returns whether it was raised, for the same count
/// [`floor_diagonal`] keeps.
pub(crate) fn floor_offset(offset: &mut Offset) -> bool {
    // NaN fails the comparison and is left alone, as in `floor_diagonal`.
    let floor = FLOOR[ErrorState::PositionDown.index()];
    let raised = offset.variance < floor;
    if raised {
        offset.variance = floor;
    }
    raised
}

/// Whether any variance on the diagonal sits below its floor.
///
/// The same table [`floor_diagonal`] enforces, asked as a question rather than applied, for
/// the one caller that refuses such a covariance instead of repairing it:
/// [`Eskf::initialize_from`](crate::Eskf::initialize_from) is the only path that writes a
/// covariance in whole, so a diagonal below the floor there is a seed that was never
/// populated rather than a variance the filter drove down.
///
/// NaN is not below the floor by this test, and does not need to be: the caller checks
/// finiteness first and reports [`InitError::NotFinite`](crate::InitError::NotFinite),
/// which names the right fault.
pub(crate) fn below_floor(p: &CovarianceMatrix) -> bool {
    // The index stays below `STATES`, the dimension of `P` and the length of `FLOOR`.
    FLOOR
        .iter()
        .enumerate()
        .any(|(i, floor)| p[(i, i)] < *floor)
}

/// Equation (42′): `P_ii ← max(P_ii, σ²_i)`, the diagonal variance floor. Returns how many
/// entries it raised.
///
/// A variance that reaches zero is a state the filter claims to know exactly, and the claim
/// is self-sealing: `K = P Hᵀ S⁻¹` is zero in that row, so no measurement can ever move it
/// again, and (22) grows it back only through a `Q` the position row of (16) does not have.
/// In `f32` that is reachable by rounding rather than by arithmetic — the Joseph form of (27)
/// keeps `P` positive semi-definite, and semi-definite includes zero.
///
/// Raising a diagonal entry adds a positive semi-definite diagonal matrix to `P`, so it
/// preserves the property (27) was chosen to protect and can only widen the `S` of (24). The
/// floor can make the filter more uncertain than it should be and never more confident, which
/// is why it is a constant of the arithmetic and not a [`Config`](crate::Config) field: there
/// is no mission whose answer is a different number, only a filter whose numbers should never
/// reach this one. [`FLOOR`] records what the corpus says about that.
pub(crate) fn floor_diagonal(p: &mut CovarianceMatrix) -> u32 {
    let mut raised = 0;

    // The index stays below `STATES`, the dimension of `P` and the length of `FLOOR`, so
    // neither the matrix nor the table can be indexed out of range: both panic, and nothing
    // in `src/` may.
    for (i, floor) in FLOOR.iter().enumerate() {
        // NaN fails this comparison and is left alone. `predict` and `update` both refuse a
        // covariance that is not finite, and refusing is more honest than flooring a NaN
        // into a number that reads as an estimate.
        if p[(i, i)] < *floor {
            p[(i, i)] = *floor;
            raised += 1;
        }
    }
    raised
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::SVector;

    fn inflation(dt: f32, tau: f32) -> f32 {
        correlation_inflation(Some(Seconds::from_secs(dt)), Some(Seconds::from_secs(tau)))
    }

    #[test]
    fn the_inflation_of_24_prime_is_its_definition() {
        // At Δt = τ, ρ = 1/e and the factor is (e + 1)/(e − 1) = 2.1639.
        let rho = (-1.0f32).exp();
        assert!((inflation(0.5, 0.5) - (1.0 + rho) / (1.0 - rho)).abs() < 1e-5);
    }

    #[test]
    fn the_inflation_limits_are_white_and_two_tau_over_the_interval() {
        assert_eq!(inflation(1e3, 1.0), 1.0);
        // Where `1 − exp` keeps two digits of `1 − ρ`, and `expm1` all of them.
        let short = inflation(1e-4, 14.0);
        assert!((short / (2.0 * 14.0 / 1e-4) - 1.0).abs() < 1e-3, "{short}");
    }

    #[test]
    fn the_same_error_twice_is_worth_nothing_and_stays_finite() {
        // Two measurements with no step between them, and a `τ` no interval is short against:
        // `ρ` is 1 in both, and the factor saturates rather than reaching (27) as infinity,
        // where `K R Kᵀ` would read `0 · ∞`.
        for (dt, tau) in [(0.0, 4.2), (0.2, f32::INFINITY), (0.2, f32::MAX)] {
            assert_eq!(inflation(dt, tau), MAX_INFLATION, "Δt {dt}, τ {tau}");
        }
    }

    #[test]
    fn with_nothing_to_measure_or_nothing_configured_the_measurement_is_white() {
        let dt = Some(Seconds::from_secs(0.2));
        for (interval, tau) in [
            (None, Some(Seconds::from_secs(4.2))),
            (dt, None),
            (dt, Some(Seconds::ZERO)),
            (dt, Some(Seconds::from_secs(-1.0))),
            (dt, Some(Seconds::from_secs(f32::NAN))),
        ] {
            assert_eq!(correlation_inflation(interval, tau), 1.0);
        }
    }

    use crate::state::ErrorState;

    /// Vectors with mixed signs, a zero component, and magnitudes either side of one.
    const VECTORS: [Vector3<f32>; 5] = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.3, -1.7, 2.5),
        Vector3::new(-4.0, 0.25, 0.0),
        Vector3::new(-0.01, -0.02, -0.03),
    ];

    #[test]
    fn the_skew_matrix_is_the_cross_product() {
        for u in VECTORS {
            for v in VECTORS {
                assert_eq!(skew(u) * v, u.cross(&v), "u = {u:?}, v = {v:?}");
            }
        }
    }

    #[test]
    fn the_skew_matrix_is_skew_symmetric() {
        for u in VECTORS {
            assert_eq!(skew(u).transpose(), -skew(u), "u = {u:?}");
        }
    }

    #[test]
    fn exp_agrees_with_the_library_for_ordinary_angles() {
        let rotations = [
            Vector3::new(0.0, 0.0, core::f32::consts::FRAC_PI_2),
            Vector3::new(0.1, -0.2, 0.3),
            Vector3::new(PI, 0.0, 0.0),
            Vector3::new(-1.5, 2.5, -0.5),
            // 400 Hz, 30 deg/s: the size of increment (15) actually composes.
            Vector3::new(0.0, 1.3e-3, 0.0),
        ];
        for phi in rotations {
            let ours = exp_quat(phi).into_inner();
            let theirs = UnitQuaternion::from_scaled_axis(phi).into_inner();
            assert!(
                (ours - theirs).norm() < 1e-6,
                "phi = {phi:?}: {ours:?} vs {theirs:?}"
            );
        }
    }

    #[test]
    fn exp_stays_finite_and_unit_where_the_scale_factor_is_zero_over_zero() {
        for phi in [
            Vector3::zeros(),
            Vector3::new(1.0e-9, 0.0, 0.0),
            Vector3::new(-1.0e-20, 1.0e-20, 0.0),
            Vector3::new(5.0e-4, -5.0e-4, 5.0e-4),
        ] {
            let q = exp_quat(phi).into_inner();
            assert!(q.norm().is_finite(), "phi = {phi:?}: {q:?}");
            assert!((q.norm() - 1.0).abs() < 1e-6, "phi = {phi:?}: {q:?}");
        }
    }

    #[test]
    fn exp_keeps_the_first_order_term_the_library_drops() {
        // Inside nalgebra's epsilon branch, where `from_scaled_axis` returns the identity.
        let phi = Vector3::new(1.0e-9, 0.0, 0.0);
        let ours = exp_quat(phi).into_inner();

        // Exp(φ) ≈ [1, φ/2] to first order, and φ/2 is exact here.
        assert_eq!(ours.i, 5.0e-10);
        assert_eq!(ours.w, 1.0);

        // The comparison this function exists for. If it ever fails, nalgebra has stopped
        // truncating and `exp_quat` has lost its reason to be hand-written.
        assert_eq!(
            UnitQuaternion::from_scaled_axis(phi),
            UnitQuaternion::identity()
        );
    }

    #[test]
    fn exp_of_zero_is_the_identity() {
        assert_eq!(exp_quat(Vector3::zeros()), UnitQuaternion::identity());
    }

    #[test]
    fn wrap_closes_the_interval_at_plus_pi() {
        assert_eq!(wrap_pi(PI), PI);
        assert_eq!(wrap_pi(-PI), PI);
    }

    #[test]
    fn wrap_moves_an_angle_by_whole_turns_only() {
        // input, expected. Either side of +pi, either side of -pi, and beyond +-2pi.
        let cases = [
            (PI - 0.01, PI - 0.01),
            (PI + 0.01, -PI + 0.01),
            (-PI + 0.01, -PI + 0.01),
            (-PI - 0.01, PI - 0.01),
            (TAU + 0.5, 0.5),
            (-TAU - 0.5, -0.5),
            (3.0 * PI, PI),
            // The half-open interval is a statement about the boundary, and `3.0 * PI` in
            // f32 is not on it: its exact reduction lands an ulp inside the open end, so
            // this comes back as -π rather than the +π that `wrap_pi(-PI)` gives. The two
            // are the same angle. Only an input that reduces exactly can be moved.
            (-3.0 * PI, -PI),
            (0.0, 0.0),
        ];
        for (angle, expected) in cases {
            let wrapped = wrap_pi(angle);
            assert!(
                (wrapped - expected).abs() < 1e-5,
                "wrap_pi({angle}) = {wrapped}, expected {expected}"
            );
            assert!(
                wrapped > -PI && wrapped <= PI,
                "wrap_pi({angle}) = {wrapped} is outside (-pi, pi]"
            );
        }
    }

    #[test]
    fn wrap_lands_inside_the_interval_at_every_magnitude() {
        // The full f32 exponent range, four mantissas each, both signs: the range is
        // claimed as a guarantee, so it is checked over more than the angles (35) sends.
        for exponent in 0..255u32 {
            for mantissa in [0, 1, 0x2a_aaaa, 0x7f_ffff] {
                let magnitude = f32::from_bits((exponent << 23) | mantissa);
                for angle in [magnitude, -magnitude] {
                    let wrapped = wrap_pi(angle);
                    assert!(
                        wrapped > -PI && wrapped <= PI,
                        "wrap_pi({angle:e}) = {wrapped} is outside (-pi, pi]"
                    );
                }
            }
        }
    }

    #[test]
    fn wrap_carries_a_non_finite_angle_through_rather_than_inventing_one() {
        // The interval is a guarantee about finite input, and the caller owns finiteness.
        // Pinned so the caveat in the doc comment has a test under it.
        assert!(wrap_pi(f32::NAN).is_nan());
        assert!(wrap_pi(f32::INFINITY).is_nan());
        assert!(wrap_pi(f32::NEG_INFINITY).is_nan());
    }

    #[test]
    fn wrap_leaves_an_angle_already_in_range_alone() {
        let mut angle = -PI + 0.001;
        while angle < PI {
            assert_eq!(wrap_pi(angle), angle);
            angle += 0.1;
        }
    }

    /// `p[(i, j)] != p[(j, i)]` everywhere off the diagonal.
    fn asymmetric() -> CovarianceMatrix {
        CovarianceMatrix::from_fn(|i, j| (i * 15 + j) as f32 * 0.125)
    }

    #[test]
    fn symmetry_enforcement_is_idempotent() {
        let mut p = asymmetric();
        enforce_symmetry(&mut p);
        let once = p;
        enforce_symmetry(&mut p);
        assert_eq!(p, once);
        assert_eq!(p, p.transpose());
    }

    #[test]
    fn symmetry_enforcement_leaves_a_symmetric_matrix_bit_identical() {
        // Products are symmetric, and the values are not powers of two, so any rounding
        // in the averaging would show.
        let mut p = CovarianceMatrix::from_fn(|i, j| ((i + 1) * (j + 1)) as f32 * 0.3);
        let before = p;
        enforce_symmetry(&mut p);
        assert_eq!(p, before);
    }

    #[test]
    fn symmetry_enforcement_averages_the_two_halves() {
        let mut p = asymmetric();
        let before = p;
        enforce_symmetry(&mut p);
        assert_eq!(p[(2, 7)], 0.5 * (before[(2, 7)] + before[(7, 2)]));
        assert_eq!(p.diagonal(), before.diagonal());
    }

    #[test]
    fn the_floor_raises_a_collapsed_diagonal_to_its_own_floor_and_counts_every_entry() {
        let mut p = CovarianceMatrix::zeros();
        assert_eq!(floor_diagonal(&mut p), STATES as u32);
        for i in 0..STATES {
            assert_eq!(p[(i, i)], FLOOR[i], "state {i}");
        }
    }

    #[test]
    fn the_floor_leaves_the_variances_the_filter_actually_reaches_alone() {
        // The smallest variance per group anywhere in the corpus or the scenarios, in the
        // `ErrorState` ordering. These are [`FLOOR`]'s own evidence as literals beside the
        // assertion that reads them, and the one place they are copied: they move when it
        // does, which is whenever a log or a scenario is added or dropped.
        #[rustfmt::skip]
        let smallest = [
            5.1e-3, 5.1e-3, 5.1e-3,
            2.6e-3, 2.6e-3, 2.6e-3,
            2.6e-4, 2.6e-4, 2.6e-4,
            3.5e-3, 3.5e-3, 3.5e-3,
            1.6e-5, 1.6e-5, 1.6e-5,
        ];
        let mut p = CovarianceMatrix::from_diagonal(&SVector::from(smallest));
        let before = p;

        assert_eq!(floor_diagonal(&mut p), 0);
        assert_eq!(p, before);
    }

    #[test]
    fn each_floor_sits_against_the_state_it_was_measured_for() {
        // `FLOOR` is positional, and nothing else ties it to the ordering its comments
        // name: `math.rs` reads the table by index and never mentions `ErrorState`. The
        // length is the compiler's, so what is left to pin is which group each value
        // belongs to. One per group, at its first component.
        assert_eq!(FLOOR[ErrorState::PositionNorth.index()], 1e-6);
        assert_eq!(FLOOR[ErrorState::VelocityNorth.index()], 1e-6);
        assert_eq!(FLOOR[ErrorState::AttitudeX.index()], 1e-9);
        assert_eq!(FLOOR[ErrorState::AccelBiasX.index()], 1e-9);
        assert_eq!(FLOOR[ErrorState::GyroBiasX.index()], 1e-9);
    }

    #[test]
    fn the_below_floor_test_agrees_with_the_floor_it_names() {
        // One table, two readings: what `floor_diagonal` would raise is exactly what
        // `below_floor` reports. Survives the mutation that gives `below_floor` a constant
        // of its own, which would pass every test written against one group alone.
        for i in 0..STATES {
            let mut p = CovarianceMatrix::from_diagonal_element(1.0);
            p[(i, i)] = FLOOR[i] * 0.5;
            assert!(below_floor(&p), "state {i} below its floor");

            p[(i, i)] = FLOOR[i];
            assert!(!below_floor(&p), "state {i} exactly at its floor");
        }

        assert!(!below_floor(&CovarianceMatrix::from_diagonal_element(1.0)));
        assert!(below_floor(&CovarianceMatrix::zeros()));

        // What the seed check refused before the floor existed, still refused.
        let mut negative = CovarianceMatrix::from_diagonal_element(1.0);
        negative[(4, 4)] = -1.0;
        assert!(below_floor(&negative));
    }

    #[test]
    fn the_floor_is_per_group_rather_than_one_number_for_the_matrix() {
        // Survives the mutation that replaces `FLOOR` with a single scalar: 1e-7 is below
        // the position and velocity floors and above the attitude and bias ones, so one
        // number cannot produce this answer whichever value it takes.
        let mut p = CovarianceMatrix::from_diagonal_element(1e-7);
        assert_eq!(floor_diagonal(&mut p), 6);

        assert_eq!(p[(0, 0)], 1e-6);
        assert_eq!(p[(5, 5)], 1e-6);
        assert_eq!(p[(6, 6)], 1e-7);
        assert_eq!(p[(14, 14)], 1e-7);
    }

    #[test]
    fn the_floor_touches_nothing_off_the_diagonal() {
        // Correlations are what carry a collapsed variance into the other states, so a
        // floor that raised them would be inventing information rather than withholding a
        // claim. Symmetry is the visible half of that: this matrix is symmetric, and
        // flooring the diagonal has to leave it so.
        let mut p = CovarianceMatrix::from_fn(|i, j| if i == j { 0.0 } else { 0.25 });
        let before = p;

        assert_eq!(floor_diagonal(&mut p), STATES as u32);
        for i in 0..STATES {
            for j in 0..STATES {
                if i != j {
                    assert_eq!(p[(i, j)], before[(i, j)], "({i}, {j})");
                }
            }
        }
        assert_eq!(p, p.transpose());
    }

    #[test]
    fn the_floor_leaves_a_nan_variance_for_the_finiteness_check_to_refuse() {
        let mut p = CovarianceMatrix::zeros();
        p[(3, 3)] = f32::NAN;

        assert_eq!(floor_diagonal(&mut p), (STATES - 1) as u32);
        assert!(p[(3, 3)].is_nan());
    }
}
