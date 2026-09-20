//! The primitives the equations share: `[u]ₓ` and `Exp(φ)` from the operators table, the
//! `wrap(·)` of (35), and the symmetry enforcement of (42).
//!
//! Nothing here holds state or reads configuration, which is why it is a module of its own:
//! each function is checkable against its definition without a filter around it.
//!
//! (42)'s other half, the diagonal variance floor, is not here. It only means something
//! once a covariance shrinks, so it lands with the update that first shrinks one.

//! No filter path calls any of this yet, so each function carries its own
//! `expect(dead_code)` naming the equation that will. `expect` rather than `allow`, and one
//! per function rather than one for the module, so that each stage's first caller fails the
//! build until it deletes the line: a module-wide allowance stays satisfied while any one
//! function is still unwired, and would cover a later unused item by accident.

use core::f32::consts::{PI, TAU};

use nalgebra::{ComplexField, Matrix3, Quaternion, UnitQuaternion, Vector3};

use crate::state::{CovarianceMatrix, STATES};

/// The skew-symmetric matrix `[u]ₓ` of the operators table, so that `[u]ₓ v = u × v`.
///
/// Written out rather than taken from `Vector3::cross_matrix`, which is the same matrix:
/// the sign convention is what a reader checks this against, and it should be on the page
/// next to the equations that use it.
#[rustfmt::skip]
#[cfg_attr(not(test), expect(dead_code, reason = "(16)-(19) and (41) are unwritten"))]
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
#[cfg_attr(not(test), expect(dead_code, reason = "(15) and (39) are unwritten"))]
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

/// `wrap(·)` of the operators table: an angle reduced to `(-π, π]`. Used by (35).
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
#[cfg_attr(not(test), expect(dead_code, reason = "(35) is unwritten"))]
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
#[cfg_attr(not(test), expect(dead_code, reason = "(22) and (27) are unwritten"))]
pub(crate) fn enforce_symmetry(p: &mut CovarianceMatrix) {
    // Both indices stay below `STATES`, which is the dimension of `P`, so neither the read
    // nor the write can be out of range: `nalgebra` indexing panics, and nothing in `src/`
    // may.
    for i in 0..STATES {
        for j in (i + 1)..STATES {
            let mean = 0.5 * (p[(i, j)] + p[(j, i)]);
            p[(i, j)] = mean;
            p[(j, i)] = mean;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
