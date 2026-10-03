//! Constraints from an assumption rather than a sensor: the position hold of (28″), which keeps
//! an unaided filter's position where aiding left it, and the zero velocity of (29″), which a
//! caller asserts for a vehicle it knows is still.
//!
//! Neither adds an equation. Each is an existing observation, (28) or (29) with no antenna arm,
//! whose `z` comes from the assumption: the anchor, or zero. What they buy is tilt: without
//! horizontal aiding nothing else observes it, and bounding velocity is what lets the
//! accelerometer's specific force level the filter.

use nalgebra::{SVector, Vector3};

use crate::state::State;
use crate::update::Observation;

use super::gnss;

/// The position hold as the update reads it: `y = z − p̂` in north and east, (28)'s horizontal
/// rows with `z` the anchor and `R_m = σ²` on each axis. Equation (28″).
///
/// The IMU's own position, no arm: the anchor is where the estimate was, not where an antenna
/// was.
pub(crate) fn position_hold(state: &State, anchor: Vector3<f32>, variance: f32) -> Observation<2> {
    let y = anchor - state.position.vector();
    let r_m = SVector::<f32, 2>::repeat(variance);
    Observation {
        y: SVector::<f32, 2>::new(y[0], y[1]),
        h: gnss::horizontal_jacobian(),
        h_b: SVector::<f32, 2>::zeros(),
        r_m,
        r_gain: r_m,
    }
}

/// A claimed standstill as the update reads it: `y = 0 − v̂`, (29) with `z = 0` and the caller's
/// variances as `R_m`. Equation (29″).
pub(crate) fn zero_velocity(state: &State, variance: Vector3<f32>) -> Observation<3> {
    Observation {
        y: -state.velocity.vector(),
        h: gnss::velocity_jacobian(),
        h_b: SVector::<f32, 3>::zeros(),
        r_m: variance,
        r_gain: variance,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::STATES;
    use crate::units::{Position, Velocity};
    use nalgebra::Vector2;

    #[test]
    fn the_hold_is_the_anchor_less_the_estimate_on_the_horizontal_axes() {
        let state = State {
            position: Position::ned(3.0, -1.0, 7.0),
            ..State::default()
        };
        let observation = position_hold(&state, Vector3::new(4.0, -3.0, -50.0), 100.0);
        // Down is not held: a hold that moved height would fight the barometer.
        assert_eq!(observation.y, Vector2::new(1.0, -2.0));
        assert_eq!(observation.r_m, Vector2::new(100.0, 100.0));
        let dx = SVector::<f32, STATES>::from_fn(|i, _| i as f32 + 1.0);
        assert_eq!(observation.h * dx, Vector2::new(1.0, 2.0));
    }

    #[test]
    fn a_standstill_observes_the_whole_velocity_against_zero() {
        let state = State {
            velocity: Velocity::ned(0.5, -0.25, 2.0),
            ..State::default()
        };
        let observation = zero_velocity(&state, Vector3::new(0.01, 0.01, 0.04));
        assert_eq!(observation.y, Vector3::new(-0.5, 0.25, -2.0));
        let dx = SVector::<f32, STATES>::from_fn(|i, _| i as f32 + 1.0);
        assert_eq!(observation.h * dx, Vector3::new(4.0, 5.0, 6.0));
    }
}
