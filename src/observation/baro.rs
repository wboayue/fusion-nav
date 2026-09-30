//! Barometric altitude. Equation (30).

use nalgebra::{SMatrix, SVector};

use crate::state::{ErrorState, STATES, State};
use crate::units::{Altitude, AltitudeNoise};
use crate::update::Observation;

/// `H = [e₃ᵀ 0 0 0 0]`, the Jacobian of a barometric altitude. Equation (30).
///
/// One row, and the only observation in the crate that is not a whole block: the barometer
/// measures the third component of position and says nothing about the first two. Everything
/// else it corrects — vertical velocity first, then the accelerometer bias under it — it
/// corrects through the correlations (20) builds.
pub(crate) fn altitude_jacobian() -> SMatrix<f32, 1, STATES> {
    let mut h = SMatrix::<f32, 1, STATES>::zeros();
    h[(0, ErrorState::PositionDown.index())] = 1.0;
    h
}

/// An altitude as the update reads it: `y = z − p̂_D` with `z = −(α − α₀)`, (23) with
/// `h(x) = p_D` from (30), and the caller's variance as `R_m`.
///
/// The sign is the whole of the conversion. A barometer reports height above its own
/// reference, positive up; the navigation frame is down-positive, so the measurement enters
/// as its negation and `α₀` is what makes it relative to the origin rather than to whatever
/// pressure the sensor was built around.
///
/// # The curvature term of (30), not built
///
/// (30) treats `−p_D` as height, which is off by the tangent plane's rise above the surface,
/// `(p_N² + p_E²)/2R`: 8 cm at 1 km, 7.8 m at 10 km. A GNSS position converted by (43) carries that
/// rise and the barometer does not, so far enough out the two disagree about height by exactly that
/// amount. `h(x) = p_D − (p_N² + p_E²)/2R` would remove it, with `H` unchanged to first order. Not
/// built: #124.
pub(crate) fn altitude_observation(
    state: &State,
    altitude: Altitude,
    reference: Altitude,
    noise: AltitudeNoise,
) -> Observation<1> {
    let z = -(altitude.as_meters() - reference.as_meters());
    Observation {
        y: SVector::<f32, 1>::new(z - state.position.vector()[2]),
        h: altitude_jacobian(),
        h_b: SVector::<f32, 1>::new(1.0),
        r_m: SVector::<f32, 1>::new(noise.variance()),
        r_gain: SVector::<f32, 1>::new(noise.variance()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Position;
    use nalgebra::SVector;

    #[test]
    fn the_jacobian_selects_the_down_position_and_nothing_else() {
        let dx = SVector::<f32, STATES>::from_fn(|i, _| i as f32 + 1.0);
        assert_eq!((altitude_jacobian() * dx)[0], 3.0);
    }

    #[test]
    fn an_altitude_at_the_reference_is_no_news_to_a_filter_at_the_origin() {
        let observation = altitude_observation(
            &State::default(),
            Altitude::from_meters(112.0),
            Altitude::from_meters(112.0),
            AltitudeNoise::from_sigma(2.0),
        );
        assert_eq!(observation.y[0], 0.0);
        assert_eq!(observation.r_m[0], 4.0);
    }

    #[test]
    fn a_metre_above_the_reference_is_a_metre_up_and_therefore_minus_one_down() {
        // The sign is the whole test: positive-up measurement, down-positive frame. An
        // (α − α₀) left unnegated reads +1 here and drives the filter into the ground.
        let observation = altitude_observation(
            &State::default(),
            Altitude::from_meters(113.0),
            Altitude::from_meters(112.0),
            AltitudeNoise::from_sigma(2.0),
        );
        assert_eq!(observation.y[0], -1.0);
    }

    #[test]
    fn a_filter_already_at_that_height_has_nothing_to_correct() {
        // The same metre of climb, with the estimate already reporting it. Distinguishes
        // `z − p̂_D` from `z` alone, which the two tests above cannot: both hold p̂_D = 0.
        let state = State {
            position: Position::ned(20.0, -5.0, -1.0),
            ..State::default()
        };
        let observation = altitude_observation(
            &state,
            Altitude::from_meters(113.0),
            Altitude::from_meters(112.0),
            AltitudeNoise::from_sigma(2.0),
        );
        assert_eq!(observation.y[0], 0.0);
    }
}
