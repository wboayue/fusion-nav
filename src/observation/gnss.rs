//! GNSS observation models. Equations (28) and (29), at the antenna by (28′) and (29′).

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::frames::{Body, Ned};
use crate::math::skew;
use crate::state::{ErrorState, STATES, State};
use crate::units::{AngularRate, Position, PositionNoise, Velocity, VelocityNoise};
use crate::update::Observation;

/// Where the antenna sits relative to the IMU in navigation axes, `R̂ r`, and how that moves
/// with the attitude error: `−R̂ [r]×`, since `R = R̂ (I + [δθ]×)` for the body-frame `δθ` of
/// (2). Equation (28′).
///
/// The Jacobian is the one PX4 leaves out: it corrects the measurement by `R̂ r` and keeps
/// `H = [I 0 0 0 0]` (`EKF/aid_sources/gnss/gps_control.cpp:351-354` at `c4e4ef98`), so a fix
/// there never observes attitude through the arm. Here it does, and a zero arm is exactly (28).
///
/// Measured against PX4's form on the four corpus logs whose antenna is off the IMU, as
/// agreement with each log's own EKF2, which applies the same arm. On `a299e722` (0.30 m
/// left) under PX4's `R` floors, `pos_e_rms` was 0.650 m with no arm, 0.448 in PX4's form
/// and 0.408 in this one; on `cd7e0001` (0.29 m aft) the median heading gap was 1.61° with no
/// arm, 0.85° and 0.81°. `2c42096b` and `eb799954` read the same either way to the third
/// digit. The simulator cannot choose: on `lever_arm`, a 1 m mast, both give `pos_h` 0.291 m,
/// and `yaw` reads 0.289° in PX4's form against 0.296° here, inside the noise of one seed.
fn arm(state: &State, antenna: Position<Body>) -> (Vector3<f32>, Matrix3<f32>) {
    let r = state
        .attitude
        .body_to_ned()
        .to_rotation_matrix()
        .into_inner();
    let arm = antenna.vector();
    (r * arm, -r * skew(arm))
}

/// The first two rows of `H = [I₃ 0 0 0 0]`, the Jacobian of a position fix: north and east.
/// Equation (28).
///
/// The fix measures the nominal position directly, so the position block is all it sees;
/// everything else it corrects, it corrects through the correlations `P` carries. (28) is
/// applied as two observations, this and [`height_jacobian`], because each half has its own
/// gate; see `GnssFusion`. With `R` diagonal, the two sequential updates are the one joint
/// update whenever both are accepted.
pub(crate) fn horizontal_jacobian() -> SMatrix<f32, 2, STATES> {
    let mut h = SMatrix::<f32, 2, STATES>::zeros();
    h[(0, ErrorState::PositionNorth.index())] = 1.0;
    h[(1, ErrorState::PositionEast.index())] = 1.0;
    h
}

/// The third row of (28)'s `H`: down.
pub(crate) fn height_jacobian() -> SMatrix<f32, 1, STATES> {
    let mut h = SMatrix::<f32, 1, STATES>::zeros();
    h[(0, ErrorState::PositionDown.index())] = 1.0;
    h
}

/// The horizontal half of a position fix as the update reads it: `y = z − (p̂ + R̂ r)` in
/// north and east, (23) with `h(x)` from (28′), and the fix's own variances as `R_m`.
///
/// The fix is in the filter's NED frame about its origin; `Eskf::fuse_gnss_geodetic` is what
/// converts one that is not. `antenna` is `r`, the antenna's offset from the IMU in body axes.
pub(crate) fn horizontal_observation(
    state: &State,
    fix: Position<Ned>,
    noise: PositionNoise<Ned>,
    antenna: Position<Body>,
) -> Observation<2> {
    let (offset, attitude) = arm(state, antenna);
    let y = fix.vector() - (state.position.vector() + offset);
    let r_m = noise.variance();
    let r_m = SVector::<f32, 2>::new(r_m[0], r_m[1]);
    let mut h = horizontal_jacobian();
    h.fixed_view_mut::<2, 3>(0, ErrorState::AttitudeX.index())
        .copy_from(&attitude.fixed_rows::<2>(0));
    Observation {
        y: SVector::<f32, 2>::new(y[0], y[1]),
        h,
        h_b: SVector::<f32, 2>::zeros(),
        r_m,
        r_gain: r_m,
    }
}

/// The vertical half of the same fix: `y = z_D − (p̂ + R̂ r)_D` and the fix's vertical
/// variance.
pub(crate) fn height_observation(
    state: &State,
    fix: Position<Ned>,
    noise: PositionNoise<Ned>,
    antenna: Position<Body>,
) -> Observation<1> {
    let (offset, attitude) = arm(state, antenna);
    let r_m = SVector::<f32, 1>::new(noise.variance()[2]);
    let mut h = height_jacobian();
    h.fixed_view_mut::<1, 3>(0, ErrorState::AttitudeX.index())
        .copy_from(&attitude.fixed_rows::<1>(2));
    Observation {
        y: SVector::<f32, 1>::new(fix.vector()[2] - (state.position.vector()[2] + offset[2])),
        h,
        h_b: SVector::<f32, 1>::zeros(),
        r_m,
        r_gain: r_m,
    }
}

/// `H = [0 I₃ 0 0 0]`, the Jacobian of a GNSS velocity solution. Equation (29).
///
/// The same shape as (28) one block along, and the block is what makes the two observations
/// different in effect rather than in form. Velocity error is where accelerometer bias and
/// tilt arrive first — a bias integrates into velocity once and into position twice — so the
/// correlations (20) builds carry this innovation into both, one integration nearer than a
/// position fix reaches them.
pub(crate) fn velocity_jacobian() -> SMatrix<f32, 3, STATES> {
    let mut h = SMatrix::<f32, 3, STATES>::zeros();
    h.fixed_view_mut::<3, 3>(0, ErrorState::VelocityNorth.index())
        .copy_from(&Matrix3::identity());
    h
}

/// A velocity solution as the update reads it: `y = z − (v̂ + R̂ (ω × r))`, (23) with `h(x)`
/// from (29′), and the receiver's own variances as `R_m`.
///
/// The navigation origin does not enter, unlike (28): a velocity is a rate in the NED axes
/// rather than a displacement from a point, so a solution needs no conversion and there is no
/// `fuse_gnss_velocity_geodetic` for there to be. `omega` is the bias-corrected body rate at
/// the solution's time, `ω` of (9), which the antenna's motion about the IMU is taken at.
///
/// The antenna's velocity reads the attitude, `−R̂ [ω × r]×`, and the gyroscope bias,
/// `R̂ [r]×`, since `ω = ω_m − β_g`: a vehicle turning at 1 rad/s under a 0.5 m mast sees its
/// antenna move at 0.5 m/s, and a bias error of 0.01 rad/s is 5 mm/s of it. PX4 applies the
/// same correction to the measurement (`gps_control.cpp:313-318` at `c4e4ef98`) and neither
/// term to `H`; a zero arm is exactly (29).
pub(crate) fn velocity_observation(
    state: &State,
    solution: Velocity<Ned>,
    noise: VelocityNoise<Ned>,
    antenna: Position<Body>,
    omega: AngularRate<Body>,
) -> Observation<3> {
    let r = state
        .attitude
        .body_to_ned()
        .to_rotation_matrix()
        .into_inner();
    let (arm, turning) = (antenna.vector(), omega.vector().cross(&antenna.vector()));
    let mut h = velocity_jacobian();
    h.fixed_view_mut::<3, 3>(0, ErrorState::AttitudeX.index())
        .copy_from(&(-r * skew(turning)));
    h.fixed_view_mut::<3, 3>(0, ErrorState::GyroBiasX.index())
        .copy_from(&(r * skew(arm)));
    Observation {
        y: solution.vector() - (state.velocity.vector() + r * turning),
        h,
        h_b: SVector::<f32, 3>::zeros(),
        r_m: noise.variance().into(),
        r_gain: noise.variance().into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector2, Vector3};

    #[test]
    fn the_two_halves_select_position_and_nothing_else() {
        let dx = SVector::<f32, STATES>::from_fn(|i, _| i as f32 + 1.0);
        assert_eq!(horizontal_jacobian() * dx, Vector2::new(1.0, 2.0));
        assert_eq!((height_jacobian() * dx)[0], 3.0);
    }

    #[test]
    fn the_velocity_jacobian_selects_velocity_and_nothing_else() {
        // Three rows along from (28)'s, which is the whole difference between them and the
        // one thing a transcription error would get wrong.
        let dx = SVector::<f32, STATES>::from_fn(|i, _| i as f32 + 1.0);
        assert_eq!(velocity_jacobian() * dx, Vector3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn the_innovation_is_the_solution_less_the_estimated_velocity() {
        let state = State {
            velocity: Velocity::ned(3.0, -1.0, 0.5),
            ..State::default()
        };
        let observation = velocity_observation(
            &state,
            Velocity::ned(3.5, -1.0, 0.0),
            // Sigmas whose squares are exact in binary, so the assertion below needs no
            // tolerance and stays the same shape as (28)'s above.
            VelocityNoise::from_sigma(0.5, 0.25, 2.0),
            Position::zero(),
            AngularRate::body(0.3, -0.2, 0.1),
        );
        assert_eq!(observation.y, Vector3::new(0.5, 0.0, -0.5));
        assert_eq!(observation.r_m, Vector3::new(0.25, 0.0625, 4.0));
    }

    #[test]
    fn the_innovation_is_the_fix_less_the_estimate() {
        let state = State {
            position: Position::ned(10.0, -5.0, 2.0),
            ..State::default()
        };
        let (fix, noise) = (
            Position::ned(11.0, -5.5, 2.5),
            PositionNoise::from_sigma(1.0, 2.0, 3.0),
        );
        let horizontal = horizontal_observation(&state, fix, noise, Position::zero());
        assert_eq!(horizontal.y, Vector2::new(1.0, -0.5));
        assert_eq!(horizontal.r_m, Vector2::new(1.0, 4.0));
        let height = height_observation(&state, fix, noise, Position::zero());
        assert_eq!(height.y[0], 0.5);
        assert_eq!(height.r_m[0], 9.0);
    }

    /// A state that is neither level nor axis-aligned, so no term of `R̂` cancels.
    fn tilted() -> State {
        State {
            attitude: crate::units::Attitude::from_body_to_ned(
                nalgebra::UnitQuaternion::from_euler_angles(0.3, -0.2, 1.1),
            ),
            gyro_bias: AngularRate::body(0.01, -0.02, 0.005),
            ..State::default()
        }
    }

    /// `h(x)` of (28′) and (29′) at `state`, written out as the physics rather than through
    /// the functions under test: the antenna's position and velocity in navigation axes.
    fn antenna_at(state: &State, arm: Vector3<f32>, omega: Vector3<f32>) -> [Vector3<f32>; 2] {
        let r = state
            .attitude
            .body_to_ned()
            .to_rotation_matrix()
            .into_inner();
        [
            state.position.vector() + r * arm,
            state.velocity.vector() + r * omega.cross(&arm),
        ]
    }

    /// Each Jacobian block against a central difference of `h`, perturbing the body-frame
    /// attitude error the way (2) defines it and the gyroscope bias the way (9) subtracts it.
    /// A sign error in either the `[r]×` or the transpose would fail here and nowhere else:
    /// every corpus antenna is centimetres, where the term is below the noise.
    #[test]
    fn the_lever_arm_jacobians_match_a_numerical_derivative() {
        let state = tilted();
        let (arm, measured) = (Vector3::new(0.4, -0.3, -0.5), Vector3::new(0.8, -0.5, 1.2));
        let omega = measured - state.gyro_bias.vector();
        let antenna = Position::<Body>::from_vector(arm);
        let noise = PositionNoise::from_sigma(1.0, 1.0, 1.0);
        let at = |state: &State| {
            let omega = measured - state.gyro_bias.vector();
            antenna_at(state, arm, omega)
        };
        let (horizontal, height) = (
            horizontal_observation(&state, Position::zero(), noise, antenna),
            height_observation(&state, Position::zero(), noise, antenna),
        );
        let velocity = velocity_observation(
            &state,
            Velocity::zero(),
            VelocityNoise::from_sigma(1.0, 1.0, 1.0),
            antenna,
            AngularRate::from_vector(omega),
        );

        let step = 1e-3;
        for axis in 0..3 {
            let turn = |sign: f32| {
                let mut e = Vector3::zeros();
                e[axis] = sign * step;
                State {
                    attitude: crate::units::Attitude::from_body_to_ned(
                        state.attitude.body_to_ned() * crate::math::exp_quat(e),
                    ),
                    ..state
                }
            };
            let bias = |sign: f32| {
                let mut e = Vector3::zeros();
                e[axis] = sign * step;
                State {
                    gyro_bias: AngularRate::from_vector(state.gyro_bias.vector() + e),
                    ..state
                }
            };
            let [p_plus, v_plus] = at(&turn(1.0));
            let [p_minus, v_minus] = at(&turn(-1.0));
            let d_position = (p_plus - p_minus) / (2.0 * step);
            let d_velocity = (v_plus - v_minus) / (2.0 * step);
            let [_, vb_plus] = at(&bias(1.0));
            let [_, vb_minus] = at(&bias(-1.0));
            let d_bias = (vb_plus - vb_minus) / (2.0 * step);

            let theta = ErrorState::AttitudeX.index() + axis;
            let beta = ErrorState::GyroBiasX.index() + axis;
            for row in 0..2 {
                assert!((horizontal.h[(row, theta)] - d_position[row]).abs() < 1e-3);
            }
            assert!((height.h[(0, theta)] - d_position[2]).abs() < 1e-3);
            for row in 0..3 {
                assert!((velocity.h[(row, theta)] - d_velocity[row]).abs() < 1e-3);
                assert!((velocity.h[(row, beta)] - d_bias[row]).abs() < 1e-3);
            }
        }

        // And `y` is the measurement less `h` at the estimate.
        let [p, v] = antenna_at(&state, arm, omega);
        assert!((horizontal.y - Vector2::new(-p[0], -p[1])).norm() < 1e-6);
        assert!((height.y[0] + p[2]).abs() < 1e-6);
        assert!((velocity.y + v).norm() < 1e-6);
    }

    #[test]
    fn a_zero_arm_is_equations_28_and_29_exactly() {
        let state = tilted();
        let noise = PositionNoise::from_sigma(1.0, 1.0, 1.0);
        let fix = Position::ned(1.0, 2.0, 3.0);
        assert_eq!(
            horizontal_observation(&state, fix, noise, Position::zero()).h,
            horizontal_jacobian()
        );
        assert_eq!(
            height_observation(&state, fix, noise, Position::zero()).h,
            height_jacobian()
        );
        let velocity = velocity_observation(
            &state,
            Velocity::ned(1.0, 0.0, 0.0),
            VelocityNoise::from_sigma(1.0, 1.0, 1.0),
            Position::zero(),
            AngularRate::body(1.0, 2.0, 3.0),
        );
        assert_eq!(velocity.h, velocity_jacobian());
        assert_eq!(velocity.y, Vector3::new(1.0, 0.0, 0.0));
    }
}
