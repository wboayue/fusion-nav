//! GNSS observation models. Equations (28) and (29).

use nalgebra::{Matrix3, SMatrix};

use crate::frames::Ned;
use crate::state::{ErrorState, STATES, State};
use crate::units::{Position, PositionNoise, Velocity, VelocityNoise};
use crate::update::Observation;

/// `H = [I₃ 0 0 0 0]`, the Jacobian of a position fix. Equation (28).
///
/// The fix measures the nominal position directly, so the position block is all it sees;
/// everything else it corrects, it corrects through the correlations `P` carries.
pub(crate) fn position_jacobian() -> SMatrix<f32, 3, STATES> {
    let mut h = SMatrix::<f32, 3, STATES>::zeros();
    h.fixed_view_mut::<3, 3>(0, ErrorState::PositionNorth.index())
        .copy_from(&Matrix3::identity());
    h
}

/// A position fix as the update reads it: `y = z − p̂`, (23) with `h(x) = p` from (28), and
/// the fix's own variances as `R_m`.
///
/// The fix is in the filter's NED frame about its origin; `Eskf::fuse_gnss_geodetic` is what
/// converts one that is not.
pub(crate) fn position_observation(
    state: &State,
    fix: Position<Ned>,
    noise: PositionNoise<Ned>,
) -> Observation<3> {
    Observation {
        y: fix.vector() - state.position.vector(),
        h: position_jacobian(),
        r_m: noise.variance(),
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

/// A velocity solution as the update reads it: `y = z − v̂`, (23) with `h(x) = v` from (29),
/// and the receiver's own variances as `R_m`.
///
/// The navigation origin does not enter, unlike (28): a velocity is a rate in the NED axes
/// rather than a displacement from a point, so a solution needs no conversion and there is no
/// `fuse_gnss_velocity_geodetic` for there to be.
pub(crate) fn velocity_observation(
    state: &State,
    solution: Velocity<Ned>,
    noise: VelocityNoise<Ned>,
) -> Observation<3> {
    Observation {
        y: solution.vector() - state.velocity.vector(),
        h: velocity_jacobian(),
        r_m: noise.variance(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{SVector, Vector3};

    #[test]
    fn the_jacobian_selects_position_and_nothing_else() {
        let dx = SVector::<f32, STATES>::from_fn(|i, _| i as f32 + 1.0);
        assert_eq!(position_jacobian() * dx, Vector3::new(1.0, 2.0, 3.0));
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
        let observation = position_observation(
            &state,
            Position::ned(11.0, -5.5, 2.0),
            PositionNoise::from_sigma(1.0, 2.0, 3.0),
        );
        assert_eq!(observation.y, Vector3::new(1.0, -0.5, 0.0));
        assert_eq!(observation.r_m, Vector3::new(1.0, 4.0, 9.0));
    }
}
