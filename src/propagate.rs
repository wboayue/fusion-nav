//! Propagation: the IMU sample, bias correction and the nominal kinematics.
//! Equations (9)–(15); see the equation-to-code table in `EQUATIONS.md`.
//!
//! **Stub.** The covariance does not propagate. Equations (16)–(22) land here next, and
//! until they do [`Eskf::predict`](crate::Eskf::predict) advances the nominal state while
//! its uncertainty stays exactly where initialization put it.

use nalgebra::Vector3;

use crate::config::GRAVITY;
use crate::frames::Body;
use crate::math::exp_quat;
use crate::state::State;
use crate::units::{Acceleration, AngularRate, Attitude, Position, Seconds, Velocity};

/// One IMU measurement, uncorrected. The filter subtracts its own bias estimates,
/// equation (9).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    /// Angular rate, body frame.
    pub gyro: AngularRate<Body>,
    /// Specific force, body frame. A level, stationary vehicle reads `(0, 0, -g)`.
    pub accel: Acceleration<Body>,
}

impl ImuSample {
    /// Whether every number in the sample is finite.
    ///
    /// Written once and called from both places a sample enters the filter, so that
    /// [`Eskf::initialize`](crate::Eskf::initialize) and
    /// [`Eskf::predict`](crate::Eskf::predict) refuse the same sample.
    pub(crate) fn is_finite(self) -> bool {
        self.gyro.is_finite() && self.accel.is_finite()
    }
}

/// An [`ImuSample`] with the filter's bias estimates removed: `ω` and `a_b` of equations
/// (9) and (10).
///
/// A type of its own rather than another [`ImuSample`], which carries the same two fields
/// in the same frames. What separates them is whether the bias has been taken off, and
/// that is the claim that causes the bug: subtracting twice removes a bias the sample no
/// longer carries, subtracting never hands (11) the raw measurement. Neither shows up in
/// the numbers — both are small, plausible accelerations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Corrected {
    /// `ω`, the measured body rate less [`State::gyro_bias`].
    pub(crate) omega: AngularRate<Body>,
    /// `a_b`, the measured specific force less [`State::accel_bias`].
    pub(crate) accel: Acceleration<Body>,
}

/// Subtract the filter's own bias estimates from a raw sample. Equations (9) and (10).
pub(crate) fn corrected_imu(imu: ImuSample, state: &State) -> Corrected {
    Corrected {
        omega: AngularRate::from_vector(imu.gyro.vector() - state.gyro_bias.vector()),
        accel: Acceleration::from_vector(imu.accel.vector() - state.accel_bias.vector()),
    }
}

/// Advance the nominal state over `dt` by dead reckoning. Equations (11) and (13)–(15).
///
/// Position is evaluated **before** velocity, the one ordering constraint in (13)–(14):
/// (13) reads the pre-update `v̂`, and applying (14) first adds a spurious `a_n Δt²` to
/// position on every step — a bias that integrates without bound rather than averaging
/// out, and one that no test of a single step can see.
///
/// The biases do not move. (12) models them as random walks, so propagation leaves the
/// estimates where they are and only the covariance of (16)–(22) grows around them.
///
/// The caller owes a finite sample and a finite `dt`, and owes the check on the result:
/// `a_n` is a product of finite numbers that can still overflow f32, and an infinity here
/// reaches the quaternion and never leaves. There is no channel to refuse through from
/// here, so [`Eskf::predict`](crate::Eskf::predict) is what declines to commit it, as
/// [`Propagation::StateNotFinite`](crate::Propagation::StateNotFinite).
pub(crate) fn propagate_nominal(state: State, imu: Corrected, dt: Seconds) -> State {
    let dt = dt.as_secs();
    let rotation = state.attitude.quaternion();

    // (11): specific force into the navigation frame, gravity added. A level vehicle at
    // rest measures (0, 0, -γ) and this is zero, which is the sign convention's own test —
    // down-positive gravity against a down-negative specific force.
    let a_n = rotation * imu.accel.vector() + gravity();

    let velocity = state.velocity.vector();
    let position = state.position.vector() + velocity * dt + 0.5 * a_n * dt * dt; // (13)
    let velocity = velocity + a_n * dt; // (14), from the pre-update velocity above

    // (15): the body-frame rotation increment composes on the right. `exp_quat` keeps the
    // first-order term where `UnitQuaternion::from_scaled_axis` substitutes the identity,
    // and the product of two unit quaternions drifts off the manifold in f32 over a
    // flight. Not `renormalize_fast`: its first-order approximation saves a square root on
    // a path that already evaluates a sine and a cosine per sample.
    let mut attitude = rotation * exp_quat(imu.omega.vector() * dt);
    attitude.renormalize();

    State {
        attitude: Attitude::body_to_ned(attitude),
        position: Position::from_vector(position),
        velocity: Velocity::from_vector(velocity),
        ..state
    }
}

/// `g = [0, 0, γ]ᵀ`, the navigation-frame gravity vector of (11).
///
/// γ is [`GRAVITY`], the WGS-84 standard value, and stays a constant. It varies by about
/// 0.5 % between the equator and the poles, but the origin that would derive it is placed
/// by the first GNSS fix, which can arrive after propagation has begun — deriving it there
/// would change a propagation constant mid-flight, which is the self-retuning
/// differentiator 7's boundary forbids. `GOALS.md` records the decision (#66); the
/// derivation belongs to the offline tool that prints a `Config` (#51).
fn gravity() -> Vector3<f32> {
    Vector3::new(0.0, 0.0, GRAVITY)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    const DT: Seconds = Seconds::from_secs(0.005);

    /// At the origin, level, at rest, unbiased.
    fn at_rest() -> State {
        State {
            attitude: Attitude::level(),
            ..State::default()
        }
    }

    /// What a level vehicle at rest measures: gravity alone, down-negative.
    fn holding_still() -> ImuSample {
        ImuSample {
            gyro: AngularRate::body(0.0, 0.0, 0.0),
            accel: Acceleration::body(0.0, 0.0, -GRAVITY),
        }
    }

    fn step(state: State, imu: ImuSample, dt: Seconds) -> State {
        propagate_nominal(state, corrected_imu(imu, &state), dt)
    }

    fn run(mut state: State, imu: ImuSample, steps: u32) -> State {
        for _ in 0..steps {
            state = step(state, imu, DT);
        }
        state
    }

    /// (11) with the signs right: a level vehicle at rest reads `(0, 0, -γ)`, so `a_n` is
    /// zero and nothing moves over a minute. One test, and it fails on a sign error in
    /// (11), on a gravity vector pointing up, and on a body/navigation frame mix-up.
    #[test]
    fn a_level_vehicle_at_rest_stays_where_it_is() {
        let state = run(at_rest(), holding_still(), 12_000);
        assert!(state.position.vector().norm() < 1e-6, "{state:?}");
        assert!(state.velocity.vector().norm() < 1e-6, "{state:?}");
    }

    /// (15) against the closed form: a constant rate about one axis for `T` seconds is a
    /// rotation of `ωT` about that axis, and the quaternion stays unit the whole way.
    #[test]
    fn a_constant_body_rate_integrates_to_the_closed_form_rotation() {
        let rate = 0.4;
        let steps = 1_000;
        let imu = ImuSample {
            gyro: AngularRate::body(0.0, 0.0, rate),
            ..holding_still()
        };

        let mut state = at_rest();
        for _ in 0..steps {
            state = step(state, imu, DT);
            let q = state.attitude.quaternion().into_inner();
            assert!((q.norm() - 1.0).abs() < 1e-6, "{q:?}");
        }

        let expected = rate * DT.as_secs() * steps as f32;
        let (_, _, yaw) = state.attitude.euler_angles();
        assert!((yaw - expected).abs() < 1e-3, "{yaw} vs {expected}");
    }

    /// The ordering hazard of (13)/(14), and the only test that sees it: a constant `a_n`
    /// over `N` steps must match `p = ½at²`. Evaluating (14) first adds `N a Δt²` — 0.05 m
    /// here against an answer of 25 m, which is inside any tolerance a single step would
    /// justify and grows with the length of the flight.
    #[test]
    fn position_uses_the_pre_update_velocity() {
        let accel = 2.0;
        let seconds = 5.0;
        let steps = (seconds / DT.as_secs()) as u32;
        let imu = ImuSample {
            accel: Acceleration::body(accel, 0.0, -GRAVITY),
            ..holding_still()
        };

        let state = run(at_rest(), imu, steps);
        let closed_form = 0.5 * accel * seconds * seconds;
        assert!(
            (state.position.vector().x - closed_form).abs() < 1e-2,
            "{} vs {closed_form}",
            state.position.vector().x
        );

        // What the fused order would have added, stated so the margin above is checkable
        // rather than chosen: N a Δt² = a Δt T.
        let fused_excess = accel * DT.as_secs() * seconds;
        assert!((fused_excess - 0.05).abs() < 1e-6, "{fused_excess}");
    }

    /// (9): a gyroscope bias equal to the rate the sensor reports leaves the attitude
    /// alone. Without the subtraction the vehicle turns at 0.4 rad/s sitting still.
    #[test]
    fn a_gyro_bias_equal_to_the_measured_rate_produces_no_rotation() {
        let rate = 0.4;
        let state = State {
            gyro_bias: AngularRate::body(0.0, 0.0, rate),
            ..at_rest()
        };
        let imu = ImuSample {
            gyro: AngularRate::body(0.0, 0.0, rate),
            ..holding_still()
        };

        let (_, _, yaw) = run(state, imu, 2_000).attitude.euler_angles();
        assert!(yaw.abs() < 1e-6, "{yaw}");
    }

    /// (10): the same for the accelerometer. A bias `b` against a measurement of `-γ + b`
    /// is a vehicle at rest, not one accelerating at `b`.
    #[test]
    fn an_accel_bias_equal_to_the_measured_offset_produces_no_velocity() {
        let bias = 0.3;
        let state = State {
            accel_bias: Acceleration::body(bias, 0.0, 0.0),
            ..at_rest()
        };
        let imu = ImuSample {
            accel: Acceleration::body(bias, 0.0, -GRAVITY),
            ..holding_still()
        };

        let velocity = run(state, imu, 2_000).velocity.vector().norm();
        assert!(velocity < 1e-6, "{velocity}");
    }

    /// The biases are random walks in (12): propagation moves neither.
    #[test]
    fn propagation_leaves_the_biases_alone() {
        let state = State {
            accel_bias: Acceleration::body(0.1, -0.2, 0.3),
            gyro_bias: AngularRate::body(0.01, 0.02, -0.03),
            ..at_rest()
        };
        let after = step(state, holding_still(), DT);
        assert_eq!(after.accel_bias, state.accel_bias);
        assert_eq!(after.gyro_bias, state.gyro_bias);
    }

    /// Rotating the specific force is what makes (11) a navigation-frame equation, and the
    /// same body-frame numbers mean something else once the attitude moves. Rolled 90°
    /// right, what the accelerometer reads along body `-z` points east, so the vehicle
    /// accelerates east at γ — and falls at γ, because nothing is opposing gravity any
    /// more. Reading the unrotated sample instead leaves it hovering.
    #[test]
    fn specific_force_is_rotated_out_of_the_body_frame() {
        let state = State {
            attitude: Attitude::body_to_ned(UnitQuaternion::from_euler_angles(
                core::f32::consts::FRAC_PI_2,
                0.0,
                0.0,
            )),
            ..at_rest()
        };

        let velocity = step(state, holding_still(), DT).velocity.vector();
        let free_fall = GRAVITY * DT.as_secs();
        assert!(velocity.x.abs() < 1e-4, "{velocity:?}");
        assert!((velocity.y - free_fall).abs() < 1e-4, "{velocity:?}");
        assert!((velocity.z - free_fall).abs() < 1e-4, "{velocity:?}");
    }

    /// A finite sample can still overflow f32 on the way through (11)–(14), and it takes
    /// accumulation rather than one absurd step: even `f32::MAX` of specific force is only
    /// `1.7e36` of velocity over 5 ms, so the range survives a single step and gives out
    /// after a couple of hundred. Propagation has no channel to refuse through, so it
    /// produces the infinity and the caller declines to commit it; `Eskf::predict` has the
    /// test that it does.
    #[test]
    fn a_finite_but_enormous_sample_produces_a_state_that_is_not_finite() {
        let imu = ImuSample {
            accel: Acceleration::body(f32::MAX, 0.0, -GRAVITY),
            ..holding_still()
        };
        assert!(imu.is_finite());

        let state = run(at_rest(), imu, 1_000);
        assert!(!state.is_finite(), "{state:?}");
    }
}
