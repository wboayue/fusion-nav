//! GNSS observation models. Equation (28); (29) is unbuilt.

use nalgebra::{Matrix3, SMatrix};

use crate::frames::Ned;
use crate::state::{ErrorState, STATES, State};
use crate::units::{Position, PositionNoise};
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
