//! Heading observation models every heading source shares, and the two that arrive as an
//! angle rather than a field: a dual-antenna GNSS heading and the course constraint.
//! Equations (36), (35′) and (35″).
//!
//! A magnetic heading forms its own innovation, (34)–(35), and prices its leveling, (36′);
//! see `mag.rs`. What it shares with these is (36), the row that reads yaw out of the error
//! state.

use nalgebra::{RealField, SMatrix, SVector, UnitQuaternion, Vector3};

use crate::math::wrap_pi;
use crate::state::{Covariance, ErrorState, STATES, State};
use crate::units::{HeadingNoise, Radians};
use crate::update::Observation;

/// The least horizontal share of the forward axis, `cos 60°`, below which the heading of that
/// axis is refused as [`Fusion::Unobservable`](crate::Fusion::Unobservable): body x within 30°
/// of vertical. PX4's bar for a GNSS yaw reset, `fabsf(ant_vec_ef(2)) > cosf(radians(30))`
/// (`EKF/aid_sources/gnss/gnss_yaw_control.cpp:221` at `c4e4ef98`).
///
/// (36) itself has no singularity there; the angle it is compared with does. A tailsitter in
/// hover points body x at the sky, and the direction its horizontal part names is noise.
const FORWARD_HORIZONTAL_MIN: f32 = 0.5;

/// `sin 15°`, the most course uncertainty the constraint will read an angle from: ArduPilot's
/// `GPS_VEL_YAW_ALIGN_MAX_ANG_ERR` (`AP_NavEKF3_core.h:125` at `368dc0c4`), which it applies
/// as `asin(σ_v / |v|) < 15°` (`AP_NavEKF3_MagFusion.cpp:165-166`). `σ_χ ≈ σ_⊥ / v_h` for
/// small angles, so the test here is the same one on the estimate's own velocity covariance.
const COURSE_SIGMA_MAX: f32 = 0.258_819;

/// `H = [0 0 e₃ᵀ R(q̂) 0 0]`, the Jacobian of a heading. Equation (36).
///
/// The error state of (2) is a local, body-frame rotation vector, so the navigation-frame
/// rotation it stands for is `R(q̂) δθ`; yaw is rotation about navigation down, which
/// makes the third row of `R(q̂)` the row that reads yaw out of the error state.
///
/// Yaw as the down component of a rotation vector is exact at zero tilt and degrades as
/// `1/cos θ`. What it buys is the singularity: reading yaw as an Euler angle instead is
/// undefined at 90° of pitch, and a filter that stops having a heading Jacobian when the
/// vehicle points at the sky is worse than one whose Jacobian is a few percent small.
///
/// Every heading source uses this row: the magnetometer's (35), the GNSS heading of (35′) and
/// the course constraint of (35″), which adds a velocity block of its own.
pub(crate) fn heading_jacobian(state: &State) -> SMatrix<f32, 1, STATES> {
    let rotation = state.attitude.quaternion().to_rotation_matrix();
    let mut h = SMatrix::<f32, 1, STATES>::zeros();
    h.fixed_view_mut::<1, 3>(0, ErrorState::AttitudeX.index())
        .copy_from(&rotation.matrix().row(2));
    h
}

/// Body x in navigation axes, `R(q̂) e₁`: the axis a heading is the direction of.
fn forward(state: &State) -> Vector3<f32> {
    state.attitude.quaternion() * Vector3::x()
}

/// Whether the forward axis is far enough from vertical to have a heading. See
/// [`FORWARD_HORIZONTAL_MIN`].
pub(crate) fn has_heading(state: &State) -> bool {
    let f = forward(state);
    f.x * f.x + f.y * f.y >= FORWARD_HORIZONTAL_MIN * FORWARD_HORIZONTAL_MIN
}

/// `ψ̂`, the heading of the forward axis: `atan2` of its east and north components.
///
/// A left rotation about navigation down turns every vector's horizontal part by the same
/// angle, so this moves one for one with the rotation (36) reads, at any tilt. The adoption's
/// `Exp(y e₃) ⊗ q̂` is exactly that rotation, which is why an innovation formed against this
/// angle is one the adoption removes whole.
fn forward_heading(state: &State) -> f32 {
    heading_of(state.attitude.quaternion())
}

/// [`forward_heading`] of an attitude that is not a [`State`]'s: a yaw hypothesis's, (45)–(52).
pub(crate) fn heading_of(attitude: UnitQuaternion<f32>) -> f32 {
    let f = attitude * Vector3::x();
    RealField::atan2(f.y, f.x)
}

/// A dual-antenna GNSS heading as the update reads it. Equation (35′).
///
/// `y = wrap(ψ_m − ψ̂)`, `H` from (36), and `R` the caller's alone. Nothing here is leveled
/// with the estimated attitude, so the (36′) a magnetic heading carries has no counterpart:
/// the receiver measures the baseline's direction in the navigation frame directly.
///
/// PX4 predicts the heading of the antenna baseline through the full quaternion and
/// differentiates that exactly (`EKF/python/ekf_derivation/derivation.py:598-604` at
/// `c4e4ef98`), so its `H` carries tilt. This takes (36) for the reason the magnetometer does:
/// the exact Jacobian corrects tilt from a scalar heading, and on the magnetometer it measured
/// `mission` at 9.0° of tilt against 0.58. ArduPilot fuses the same scalar yaw
/// (`fuseEulerYaw`, `AP_NavEKF3_MagFusion.cpp:946` at `368dc0c4`).
///
/// The baseline is assumed along body x. A receiver whose antennas are mounted otherwise
/// reports the heading of *their* line, and the caller subtracts the mounting angle first,
/// as PX4 does with `EKF2_GPS_YAW_OFF` and ArduPilot with `get_mb_yaw_offset`.
pub(crate) fn gnss_observation(
    state: &State,
    heading: Radians,
    noise: HeadingNoise,
) -> Observation<1> {
    let r_m = SVector::<f32, 1>::new(noise.variance());
    Observation {
        y: SVector::<f32, 1>::new(wrap_pi(heading.as_radians() - forward_heading(state))),
        h: heading_jacobian(state),
        h_b: SVector::<f32, 1>::zeros(),
        r_m,
        r_gain: r_m,
    }
}

/// `∂χ/∂v`, how the course `χ = atan2(v_E, v_N)` moves with the estimated velocity:
/// `(−v_E, v_N, 0) / v_h²`, zero where there is no horizontal speed to take a direction of.
fn course_gradient(state: &State) -> Vector3<f32> {
    let v = state.velocity.vector();
    let v_h2 = v.x * v.x + v.y * v.y;
    if v_h2 > 0.0 {
        Vector3::new(-v.y / v_h2, v.x / v_h2, 0.0)
    } else {
        Vector3::zeros()
    }
}

/// The variance of the estimated course, `σ_χ² = ∇χᵀ P_vv ∇χ`, or `None` where it names no
/// direction: no horizontal speed, or `σ_χ` over [`COURSE_SIGMA_MAX`].
///
/// That is the speed threshold, and it is a consequence rather than a setting. The course is
/// uncertain by the cross-track velocity uncertainty over the speed, so a vehicle with an RTK
/// velocity has a course at walking pace and one on a 0.5 m/s receiver needs about 2 m/s.
pub(crate) fn course_variance(state: &State, covariance: &Covariance) -> Option<f32> {
    let gradient = course_gradient(state);
    let v = ErrorState::VelocityNorth.index();
    let p_vv = covariance.as_matrix().fixed_view::<3, 3>(v, v);
    let variance = gradient.dot(&(p_vv * gradient));
    let observable = gradient != Vector3::zeros()
        && variance.is_finite()
        && variance <= COURSE_SIGMA_MAX * COURSE_SIGMA_MAX;
    observable.then_some(variance)
}

/// The course constraint as the update reads it: the vehicle points where it is going, to
/// within `sideslip`. Equation (35″).
///
/// `h(x) = ψ − χ`, the forward axis's heading less the estimated velocity's, measured as zero
/// with variance `σ_β²`. So `y = wrap(χ̂ − ψ̂)`, and `H` is (36)'s attitude row beside
/// `−∇χ` in the velocity block.
///
/// Formed from the estimated velocity rather than from a GNSS velocity handed in, because that
/// velocity has already been fused: a course taken from the same sample would count its
/// cross-track error twice, once in (29) and again here as if independent. Read off the state,
/// the velocity's uncertainty reaches `S` through `P`, correlations included, and `R` is the
/// one thing the constraint adds: how far the vehicle's nose may point from its track.
///
/// That is a property of the vehicle and the air it flies in, so the caller supplies it. A
/// fixed-wing in coordinated flight, or a ground vehicle on a road, holds it to a few degrees;
/// a crosswind adds its crab angle. A multirotor has no such relation at all, which is why
/// `GOALS.md` rules course out for one rather than this module pricing it.
pub(crate) fn course_observation(state: &State, sideslip: HeadingNoise) -> Observation<1> {
    let v = state.velocity.vector();
    let course = RealField::atan2(v.y, v.x);
    let mut h = heading_jacobian(state);
    h.fixed_view_mut::<1, 3>(0, ErrorState::VelocityNorth.index())
        .copy_from(&(-course_gradient(state)).transpose());
    let r_m = SVector::<f32, 1>::new(sideslip.variance());
    Observation {
        y: SVector::<f32, 1>::new(wrap_pi(course - forward_heading(state))),
        h,
        h_b: SVector::<f32, 1>::zeros(),
        r_m,
        r_gain: r_m,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observation::mag::tests::attitude_of;
    use crate::units::Velocity;
    use core::f32::consts::PI;

    fn state(roll: f32, pitch: f32, yaw: f32, velocity: [f32; 3]) -> State {
        State {
            attitude: attitude_of(roll, pitch, yaw),
            velocity: Velocity::ned(velocity[0], velocity[1], velocity[2]),
            ..State::default()
        }
    }

    #[test]
    fn a_gnss_heading_innovates_by_the_yaw_error_at_any_tilt() {
        // For ZYX Euler angles the forward axis's heading is the Euler yaw at any roll and
        // pitch, `R e₁ = (cθ cψ, cθ sψ, −sθ)`, so the innovation is the yaw difference exactly.
        // The tilt is what makes this a test of the rotation into navigation axes rather than
        // of a heading read off body axes.
        let estimate = state(0.3, -0.2, 1.1, [0.0; 3]);
        let truth = state(0.3, -0.2, 1.25, [0.0; 3]);
        let measured = Radians::from_radians(forward_heading(&truth));
        let y = gnss_observation(&estimate, measured, HeadingNoise::from_sigma(0.1)).y[0];
        assert!((y - 0.15).abs() < 1e-5, "y {y}");
    }

    #[test]
    fn rotating_by_the_innovation_about_down_removes_it() {
        let estimate = state(0.3, -0.2, 1.1, [0.0; 3]);
        let measured = Radians::from_radians(2.9);
        let y = gnss_observation(&estimate, measured, HeadingNoise::from_sigma(0.1)).y[0];
        let turned = crate::math::exp_quat(Vector3::z() * y) * estimate.attitude.quaternion();
        let after = State {
            attitude: crate::units::Attitude::from_quaternion(turned),
            ..estimate
        };
        let left = gnss_observation(&after, measured, HeadingNoise::from_sigma(0.1)).y[0];
        assert!(left.abs() < 1e-5, "left {left}");
    }

    #[test]
    fn a_heading_across_the_wrap_is_a_small_innovation() {
        let estimate = state(0.0, 0.0, PI - 0.05, [0.0; 3]);
        let y = gnss_observation(
            &estimate,
            Radians::from_radians(-PI + 0.05),
            HeadingNoise::from_sigma(0.1),
        )
        .y[0];
        assert!((y - 0.1).abs() < 1e-5, "y {y}");
    }

    #[test]
    fn a_forward_axis_near_vertical_has_no_heading() {
        assert!(has_heading(&state(0.0, 0.9, 0.0, [0.0; 3])));
        // 65° of pitch: body x is 25° from vertical, inside PX4's 30°.
        assert!(!has_heading(&state(0.0, 1.134, 0.0, [0.0; 3])));
    }

    #[test]
    fn the_course_innovation_is_track_less_heading() {
        // Flying north-east, nose 0.1 rad left of track.
        let s = state(0.0, 0.0, PI / 4.0 - 0.1, [10.0, 10.0, 0.0]);
        let o = course_observation(&s, HeadingNoise::from_sigma(0.05));
        assert!((o.y[0] - 0.1).abs() < 1e-5, "y {}", o.y[0]);
        assert_eq!(o.r_m[0], HeadingNoise::from_sigma(0.05).variance());
    }

    #[test]
    fn the_course_jacobian_matches_a_finite_difference_in_velocity() {
        let s = state(0.1, 0.05, 0.4, [12.0, 5.0, -1.0]);
        let h = course_observation(&s, HeadingNoise::from_sigma(0.05)).h;
        for axis in 0..3 {
            let step = 1e-2;
            let mut moved = s.velocity.vector();
            moved[axis] += step;
            let bumped = State {
                velocity: Velocity::from_vector(moved),
                ..s
            };
            // y = z − h(x), so ∂y/∂v = −∂h/∂v = −H_v.
            let dy = course_observation(&bumped, HeadingNoise::from_sigma(0.05)).y[0]
                - course_observation(&s, HeadingNoise::from_sigma(0.05)).y[0];
            let expected = -h[(0, ErrorState::VelocityNorth.index() + axis)] * step;
            assert!(
                (dy - expected).abs() < 1e-5,
                "axis {axis}: {dy} vs {expected}"
            );
        }
    }

    #[test]
    fn a_course_needs_speed_against_the_velocity_uncertainty() {
        let mut sigmas = [0.1; STATES];
        let v = ErrorState::VelocityNorth.index();
        sigmas[v..v + 3].fill(0.5);
        let p = Covariance::from_sigmas(sigmas);
        // 0.5 / sin 15° = 1.93 m/s is the bar.
        assert!(course_variance(&state(0.0, 0.0, 0.0, [1.8, 0.0, 0.0]), &p).is_none());
        let fast = course_variance(&state(0.0, 0.0, 0.0, [2.1, 0.0, 0.0]), &p);
        assert!(fast.is_some_and(|var| (var - 0.25 / (2.1 * 2.1)).abs() < 1e-6));
        // Straight down is no horizontal speed at all, and no NaN.
        assert!(course_variance(&state(0.0, 0.0, 0.0, [0.0, 0.0, 5.0]), &p).is_none());
        let still = course_observation(
            &state(0.0, 0.0, 0.0, [0.0; 3]),
            HeadingNoise::from_sigma(0.1),
        );
        assert!(still.y[0].is_finite() && still.h.iter().all(|x| x.is_finite()));
    }
}
