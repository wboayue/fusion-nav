//! GNSS observation models. Equations (28) and (29).

use nalgebra::{ComplexField, Matrix3, SMatrix, SVector};

use crate::config::GnssCorrelation;
use crate::frames::Ned;
use crate::state::{ErrorState, STATES, State};
use crate::units::{Position, PositionNoise, Seconds, Velocity, VelocityNoise};
use crate::update::Observation;

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

/// The horizontal half of a position fix as the update reads it: `y = z − p̂` in north and
/// east, (23) with `h(x) = p` from (28), the fix's own variances as `R_m` and `fused`'s,
/// (28′)'s, as the `R` of the gain.
///
/// The fix is in the filter's NED frame about its origin; `Eskf::fuse_gnss_geodetic` is what
/// converts one that is not.
pub(crate) fn horizontal_observation(
    state: &State,
    fix: Position<Ned>,
    noise: PositionNoise<Ned>,
    fused: PositionNoise<Ned>,
) -> Observation<2> {
    let (y, r_m, r_gain) = (
        fix.vector() - state.position.vector(),
        noise.variance(),
        fused.variance(),
    );
    Observation {
        y: SVector::<f32, 2>::new(y[0], y[1]),
        h: horizontal_jacobian(),
        h_b: SVector::<f32, 2>::zeros(),
        r_m: SVector::<f32, 2>::new(r_m[0], r_m[1]),
        r_gain: SVector::<f32, 2>::new(r_gain[0], r_gain[1]),
    }
}

/// The vertical half of the same fix: `y = z_D − p̂_D`, and the vertical variances.
pub(crate) fn height_observation(
    state: &State,
    fix: Position<Ned>,
    noise: PositionNoise<Ned>,
    fused: PositionNoise<Ned>,
) -> Observation<1> {
    Observation {
        y: SVector::<f32, 1>::new(fix.vector()[2] - state.position.vector()[2]),
        h: height_jacobian(),
        h_b: SVector::<f32, 1>::zeros(),
        r_m: SVector::<f32, 1>::new(noise.variance()[2]),
        r_gain: SVector::<f32, 1>::new(fused.variance()[2]),
    }
}

/// The noise a fix is fused with when its error persists from the last: each axis's variance
/// times `(1 + ρ) / (1 − ρ)`, `ρ = exp(−Δt / τ)`. Equation (28′).
///
/// `Δt` is the interval since the previous fix. With none to measure — the first fix, or two
/// in one epoch — the factor is 1, as it is for an axis whose `τ` is `None`, or not a finite
/// positive number. The factor is 1 in the limit of a long interval too, where the fixes are
/// independent again, and `2τ / Δt` in that of a short one, so a receiver reporting faster
/// than its error changes buys no more per second than one reporting at `τ`.
///
/// Only the update reads this. An adoption writes a single fix's error onto the covariance,
/// and one fix's error is its stationary variance however correlated the next one is.
pub(crate) fn decorrelated(
    noise: PositionNoise<Ned>,
    interval: Option<Seconds>,
    correlation: GnssCorrelation,
) -> PositionNoise<Ned> {
    let r = noise.variance();
    let horizontal = inflation(interval, correlation.horizontal);
    let vertical = inflation(interval, correlation.vertical);
    PositionNoise::from_variance(r[0] * horizontal, r[1] * horizontal, r[2] * vertical)
}

/// `(1 + ρ) / (1 − ρ)` of (28′), written as `(2 − x) / x` with `x = 1 − ρ = −expm1(−Δt / τ)`:
/// at `Δt ≪ τ`, `ρ` rounds to 1 in `f32` and `1 − ρ` to zero, where `expm1` keeps the digits.
fn inflation(interval: Option<Seconds>, tau: Option<Seconds>) -> f32 {
    let (Some(dt), Some(tau)) = (interval, tau) else {
        return 1.0;
    };
    let (dt, tau) = (dt.as_secs(), tau.as_secs());
    if !(dt > 0.0 && tau > 0.0 && tau.is_finite()) {
        return 1.0;
    }
    let x = -ComplexField::exp_m1(-dt / tau);
    (2.0 - x) / x
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
        h_b: SVector::<f32, 3>::zeros(),
        r_m: noise.variance(),
        r_gain: noise.variance(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector2, Vector3};

    fn fix(sigma: f32) -> PositionNoise<Ned> {
        PositionNoise::from_sigma(sigma, sigma, 2.0 * sigma)
    }

    fn correlated(horizontal: f32, vertical: f32) -> GnssCorrelation {
        GnssCorrelation {
            horizontal: Some(Seconds::from_secs(horizontal)),
            vertical: Some(Seconds::from_secs(vertical)),
        }
    }

    #[test]
    fn a_correlated_fix_is_fused_at_the_variance_of_28_prime() {
        // At Δt = τ, ρ = 1/e and the factor is (e + 1)/(e − 1) = 2.1639.
        let r = decorrelated(
            fix(1.0),
            Some(Seconds::from_secs(0.5)),
            correlated(0.5, 0.25),
        )
        .variance();
        let at_tau = (1.0 + (-1.0f32).exp()) / (1.0 - (-1.0f32).exp());
        let at_two_tau = (1.0 + (-2.0f32).exp()) / (1.0 - (-2.0f32).exp());
        assert!((r[0] - at_tau).abs() < 1e-5, "{}", r[0]);
        assert_eq!(r[0], r[1]);
        assert!((r[2] - 4.0 * at_two_tau).abs() < 1e-4, "{}", r[2]);
    }

    #[test]
    fn the_limits_are_white_and_two_tau_over_the_interval() {
        let long = decorrelated(
            fix(1.0),
            Some(Seconds::from_secs(1e3)),
            correlated(1.0, 1.0),
        );
        assert_eq!(long.variance()[0], 1.0);
        // Where `1 − exp` would round ρ to 1 and divide by zero.
        let short = decorrelated(
            fix(1.0),
            Some(Seconds::from_secs(1e-6)),
            correlated(14.0, 14.0),
        );
        let expected = 2.0 * 14.0 / 1e-6;
        assert!(
            (short.variance()[0] / expected - 1.0).abs() < 1e-3,
            "{}",
            short.variance()[0]
        );
    }

    #[test]
    fn with_nothing_to_measure_or_nothing_configured_the_fix_is_white() {
        let white = fix(1.0).variance();
        let dt = Some(Seconds::from_secs(0.2));
        for (interval, correlation) in [
            (None, correlated(4.2, 14.0)),
            (Some(Seconds::ZERO), correlated(4.2, 14.0)),
            (dt, GnssCorrelation::WHITE),
            (dt, correlated(0.0, -1.0)),
            (dt, correlated(f32::INFINITY, f32::NAN)),
        ] {
            assert_eq!(
                decorrelated(fix(1.0), interval, correlation).variance(),
                white
            );
        }
    }

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
        let fused = PositionNoise::from_sigma(4.0, 5.0, 6.0);
        let horizontal = horizontal_observation(&state, fix, noise, fused);
        assert_eq!(horizontal.y, Vector2::new(1.0, -0.5));
        assert_eq!(horizontal.r_m, Vector2::new(1.0, 4.0));
        assert_eq!(horizontal.r_gain, Vector2::new(16.0, 25.0));
        let height = height_observation(&state, fix, noise, fused);
        assert_eq!(height.y[0], 0.5);
        assert_eq!(height.r_m[0], 9.0);
        assert_eq!(height.r_gain[0], 36.0);
    }
}
