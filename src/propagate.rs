//! Propagation: the IMU sample, bias correction, the nominal kinematics and the covariance.
//! Equations (9)–(22); see the equation-to-code table in `EQUATIONS.md`.
//!
//! The nominal state and `P` advance together in [`propagate`], and
//! [`Eskf::predict`](crate::Eskf::predict) commits both or neither. Nothing here shrinks a
//! covariance: (22) only adds, and the measurement update of (23)–(28) is what takes
//! uncertainty back out.

use nalgebra::{Matrix3, SMatrix, Vector3};

use crate::config::{GRAVITY, ImuNoise};
use crate::frames::Body;
use crate::math::{enforce_symmetry, exp_quat, skew};
use crate::state::{Covariance, ErrorState, STATES, State};
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

/// `F` of equation (20): 15 x 15, the same shape as the covariance it propagates.
pub(crate) type Transition = SMatrix<f32, STATES, STATES>;

/// The nominal state and the covariance after one step, before either is committed.
///
/// A pair rather than two return values because the commit is atomic: (11)–(14) and (22) can
/// both overflow f32 from finite inputs, and a state written beside a covariance that was
/// refused — or the reverse — is the poisoning that keeping the whole step out of the filter
/// avoids. [`Eskf::predict`](crate::Eskf::predict) commits both or neither.
pub(crate) struct Propagated {
    /// The nominal state of (13)–(15).
    pub(crate) state: State,
    /// The covariance of (22).
    pub(crate) covariance: Covariance,
}

impl Propagated {
    /// Whether every number in the step is finite.
    pub(crate) fn is_finite(&self) -> bool {
        self.state.is_finite() && self.covariance.is_finite()
    }
}

/// Advance the nominal state and its covariance over `dt`. Equations (9)–(22).
///
/// The bias correction of (9)–(10) is applied once here, and both halves read that one
/// `Corrected` sample and the one attitude it arrived with. `F` is built before the nominal
/// step runs, because (20) linearizes about the state at the **start** of the interval:
/// building it afterwards would linearize about the state (15) has already rotated, and no
/// test comparing `F` against the propagation would see it, since both sides would use the
/// same wrong attitude.
pub(crate) fn propagate(
    state: State,
    covariance: Covariance,
    imu: ImuSample,
    dt: Seconds,
    noise: &ImuNoise,
) -> Propagated {
    let corrected = corrected_imu(imu, &state);
    let transition = transition_matrix(&state, corrected, dt);

    Propagated {
        state: propagate_nominal(state, corrected, dt),
        covariance: propagate_covariance(covariance, &transition, process_noise(noise, dt)),
    }
}

/// `F`, the discrete state transition matrix of equation (20), linearized about the nominal
/// state at the start of the interval.
///
/// The continuous error dynamics (16)–(19) that (20) discretizes, with the local attitude
/// error `δθ` of equation (2), `q = q̂ ⊗ Exp(δθ)`:
///
/// ```text
/// δṗ = δv                                            (16)
/// δv̇ = −R(q̂)[a_b]ₓ δθ − R(q̂) δβa − R(q̂) w_a          (17)
/// δθ̇ = −[ω]ₓ δθ − δβg − w_g                          (18)
/// δβ̇a = w_βa,   δβ̇g = w_βg                           (19)
/// ```
///
/// (17) is the coupling that motivates the whole filter: a tilt error rotates the measured
/// specific force into the wrong navigation-frame direction, and the mistake integrates into
/// velocity and then into position. Read backwards it is also what makes tilt observable at
/// all — the reason a velocity innovation can correct an attitude no sensor measures
/// directly.
///
/// Every block is first order in `Δt` except the attitude block, which is the exact solution
/// of (18)'s homogeneous part, `R{ω Δt}ᵀ = exp(−[ω]ₓ Δt)`. (20) permits `I − [ω]ₓ Δt` where
/// the exact form is not justified, and here it is: the exact block costs one [`exp_quat`] --
/// a sine, a cosine, and a quaternion to matrix — against the roughly 6750 multiplications
/// the two 15 x 15 products of (22) spend on the same sample. The approximation buys nothing
/// measurable and gives up accuracy at high rotation rates, which is the regime a propagated
/// covariance exists for.
///
/// `imu` is the sample the nominal step reads, and `state` is that step's input rather than
/// its output; [`propagate`] holds both, which is why neither is looked up here.
fn transition_matrix(state: &State, imu: Corrected, dt: Seconds) -> Transition {
    let dt = dt.as_secs();
    let rotation = state.attitude.quaternion().to_rotation_matrix();
    let r = rotation.matrix();

    // The identity supplies (16)'s and (19)'s diagonal blocks, and the `I` on the velocity
    // row: a first-order discretization changes only the couplings written below.
    let mut f = Transition::identity();
    let (p, v, theta, beta_a, beta_g) = (
        ErrorState::PositionNorth.index(),
        ErrorState::VelocityNorth.index(),
        ErrorState::AttitudeX.index(),
        ErrorState::AccelBiasX.index(),
        ErrorState::GyroBiasX.index(),
    );

    // (16): δp gains δv over the step.
    f.fixed_view_mut::<3, 3>(p, v)
        .copy_from(&(Matrix3::identity() * dt));
    // (17): the two terms an accelerometer error enters velocity through.
    f.fixed_view_mut::<3, 3>(v, theta)
        .copy_from(&(-r * skew(imu.accel.vector()) * dt));
    f.fixed_view_mut::<3, 3>(v, beta_a).copy_from(&(-r * dt));
    // (18): exact in the attitude block, first order in the gyroscope bias. `R{ω Δt}ᵀ` is
    // the exact solution of `δθ̇ = −[ω]ₓ δθ`, which is why (15)'s own increment builds it.
    let increment = exp_quat(imu.omega.vector() * dt).to_rotation_matrix();
    f.fixed_view_mut::<3, 3>(theta, theta)
        .copy_from(&increment.matrix().transpose());
    f.fixed_view_mut::<3, 3>(theta, beta_g)
        .copy_from(&(-Matrix3::identity() * dt));

    f
}

/// `Q`, the discrete process noise of equation (21), as its diagonal.
///
/// A diagonal rather than a 15 x 15 matrix because (21) has twelve nonzero entries: the matrix
/// form would spend 900 bytes of stack and 225 additions to add twelve numbers, and "the
/// diagonal of `Q` lands on the diagonal of `P`" is closer to the equation than a dense sum.
/// What makes that legitimate is the isotropy below; a per-axis `Σ_a` is what would make this
/// a matrix again.
///
/// **`Δt`, not `Δt²`.** [`ImuNoise`]'s four fields are spectral densities, and a density's
/// contribution over a step is `σ² Δt` — for the white-noise blocks and the two random walks
/// alike. The `Δt²` that (21) carried on the white-noise blocks is PX4's and ArduPilot's form,
/// where the parameter is the σ of one sample's increment rather than a density: PX4
/// `sq(dt) * accel_var` with `accel_var = sq(ekf2_acc_noise)`
/// (`src/modules/ekf2/EKF/python/ekf_derivation/generated/predict_covariance.h:161-164`,
/// `EKF/covariance.cpp:119-133`, at `c4e4ef98e9`), ArduPilot `dvxVar = sq(dt * _accNoise)`
/// (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1177` at `368dc0c428`). That form ties `Q` to
/// the IMU rate — the variance it adds over `T` seconds is `σ² Δt T`, so the same airframe
/// logged at 400 Hz is given eight times less process noise than at 50 Hz — and the corpus in
/// `data/manifest.txt` spans 50 Hz to 250 Hz with `Config` shared across all of it. `Δt` is
/// rate-independent, and `EQUATIONS.md` (21) is amended to it.
///
/// Reading [`ImuNoise`] as densities is what `examples/simulate.rs` already does when it draws
/// per-sample noise as `white / √Δt`, so filter and simulator now agree on what the numbers
/// mean. That file also states the consequence, which is the check on this decision: against
/// its IMU table the filter's `Q` is two orders of magnitude conservative, so every scenario
/// must come out *under*-confident, and a `nees_*` above one is a finding rather than a pass.
///
/// The velocity block of (21) is the rotated accelerometer noise `R Σ_a Rᵀ`, and writing it
/// `σ_a² I` is exact only where `Σ_a` is isotropic, since `R (σ_a² I) Rᵀ = σ_a² I` for
/// orthogonal `R`. Real IMUs are noisier about z. One scalar per sensor is what [`ImuNoise`]
/// can express, so the conservatism is in the number rather than in the model: `σ_a` is the
/// worst axis.
fn process_noise(noise: &ImuNoise, dt: Seconds) -> [f32; STATES] {
    let dt = dt.as_secs();
    let velocity = noise.accel_white * noise.accel_white * dt;
    let attitude = noise.gyro_white * noise.gyro_white * dt;
    let accel_bias = noise.accel_bias_walk * noise.accel_bias_walk * dt;
    let gyro_bias = noise.gyro_bias_walk * noise.gyro_bias_walk * dt;

    // In the `ErrorState` ordering: `[δp δv δθ δβa δβg]`. Position takes none of its own --
    // (16) has no driving noise, and position error is what the velocity block integrates.
    #[rustfmt::skip]
    let diagonal = [
        0.0,        0.0,        0.0,
        velocity,   velocity,   velocity,
        attitude,   attitude,   attitude,
        accel_bias, accel_bias, accel_bias,
        gyro_bias,  gyro_bias,  gyro_bias,
    ];
    diagonal
}

/// `P ← F P Fᵀ + Q`, equation (22), with the symmetry enforcement (42) asks for afterwards.
///
/// Written as (22) reads, which is the most expensive thing the filter does: of order 6750
/// multiplications and three 900-byte temporaries per IMU sample, at up to 400 Hz. (20) is
/// sparse enough — two identity blocks, two zero rows — that a block-wise form would cut
/// both, at the cost of the one equation a reader of this crate is most likely to have come
/// for.
///
/// The measurement, since the trade was made here: (22) takes
/// [`Eskf::predict`](crate::Eskf::predict)'s stack frame from 160 bytes to 2008 on
/// `thumbv6m-none-eabi`, and from 120 to 2032 on `thumbv7em-none-eabihf`
/// (`-Zemit-stack-sizes`, `opt-level = 3`; the figure is `predict`'s because this function
/// inlines into it). That is the "few kilobytes rather than one" `DESIGN.md` predicts for the
/// working set, comfortable on the STM32H7 class it names and a quarter of the RAM on an 8 KB
/// Cortex-M0 part. The block-wise form is the lever if a target needs it, and #41 — stack
/// high-water measured on hardware — is what would say so.
///
/// `Q` arrives as a diagonal and is added as one, which keeps those temporaries to three
/// rather than four.
fn propagate_covariance(p: Covariance, f: &Transition, q: [f32; STATES]) -> Covariance {
    let mut next = f * p.as_matrix() * f.transpose();

    // Both indices are the same `i` below `STATES`, the dimension of `P`, so the indexing
    // cannot be out of range: `nalgebra` panics there, and nothing in `src/` may.
    for (i, variance) in q.iter().enumerate() {
        next[(i, i)] += variance;
    }

    // (42). `F P Fᵀ` is symmetric in exact arithmetic and drifts off it in f32, and a `P`
    // that is not symmetric is one whose innovation covariance can go indefinite later.
    enforce_symmetry(&mut next);
    Covariance::from_matrix(next)
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

    use crate::config::Initialization;
    use crate::init;
    use crate::state::{CovarianceMatrix, ErrorState};
    use crate::units::Radians;

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

    // --- (16)–(22): the covariance ---------------------------------------------------------

    /// Tilted, turning, translating, and biased: nothing in the state or the sample below is
    /// zero, so no block of (20) can be checked against an accidental zero.
    fn tilted_and_moving() -> State {
        State {
            attitude: Attitude::body_to_ned(UnitQuaternion::from_euler_angles(0.2, -0.3, 0.7)),
            position: Position::ned(0.5, -0.25, 0.1),
            velocity: Velocity::ned(3.0, -1.0, 0.5),
            accel_bias: Acceleration::body(0.05, -0.08, 0.03),
            gyro_bias: AngularRate::body(0.004, -0.007, 0.002),
            ..State::default()
        }
    }

    /// Manoeuvring hard enough that `[a_b]ₓ` and `[ω]ₓ` are both far from zero.
    fn manoeuvring() -> ImuSample {
        ImuSample {
            gyro: AngularRate::body(0.15, -0.23, 0.31),
            accel: Acceleration::body(0.8, -1.3, -9.2),
        }
    }

    /// `x̂ ⊞ δx`: the error state applied to a nominal state, with the attitude composed on
    /// the **right**, which is what the local `δθ` of equation (2) means. Composing on the
    /// left is a global perturbation, and it is the mistake the attitude columns of the
    /// Jacobian test below are there to catch.
    fn perturb(state: State, dx: &[f32; STATES]) -> State {
        let part = |i: usize| Vector3::new(dx[i], dx[i + 1], dx[i + 2]);
        State {
            position: Position::from_vector(state.position.vector() + part(0)),
            velocity: Velocity::from_vector(state.velocity.vector() + part(3)),
            attitude: Attitude::body_to_ned(state.attitude.quaternion() * exp_quat(part(6))),
            accel_bias: Acceleration::from_vector(state.accel_bias.vector() + part(9)),
            gyro_bias: AngularRate::from_vector(state.gyro_bias.vector() + part(12)),
            ..state
        }
    }

    /// `x ⊟ x̂`, the inverse of [`perturb`]: the error state between two nominal states.
    fn error_between(reference: &State, perturbed: &State) -> [f32; STATES] {
        let attitude = reference.attitude.quaternion().inverse() * perturbed.attitude.quaternion();
        let parts = [
            perturbed.position.vector() - reference.position.vector(),
            perturbed.velocity.vector() - reference.velocity.vector(),
            attitude.scaled_axis(),
            perturbed.accel_bias.vector() - reference.accel_bias.vector(),
            perturbed.gyro_bias.vector() - reference.gyro_bias.vector(),
        ];

        let mut dx = [0.0; STATES];
        for (block, part) in parts.iter().enumerate() {
            for axis in 0..3 {
                dx[3 * block + axis] = part[axis];
            }
        }
        dx
    }

    /// (20) against a numerical Jacobian of the propagation it linearizes: perturb the
    /// nominal state in each of the 15 directions, propagate the perturbed state and the
    /// reference with the same raw sample, and compare the error that comes out against
    /// `F δx`. It checks every block independently, needs no truth, and is the strongest
    /// check available before a measurement lands.
    ///
    /// The tolerance is what the discretization itself leaves behind rather than a fudge.
    /// (20) is first order in `Δt`, and the propagation is not: a tilt or accelerometer-bias
    /// error reaches position through the `½ a_n Δt²` of (13), which `F` omits, and at
    /// `|a_b| ≈ 9.3` that term is `½ |a_b| Δt² ≈ 4.6e-4` per unit of `δθ`. The second-order
    /// part of the quaternion composition contributes `O(δ)`, and f32 cancellation in the
    /// differences another `~2e-4` at this `δ`. 2e-3 clears all three and is still five times
    /// smaller than the smallest entry `F` carries here, `Δt = 0.01`, so a dropped or
    /// sign-flipped coupling cannot pass.
    #[test]
    fn the_transition_matrix_matches_a_numerical_jacobian_of_the_nominal_step() {
        const DELTA: f32 = 1.0e-3;
        const TOLERANCE: f32 = 2.0e-3;
        let dt = Seconds::from_secs(0.01);
        let (state, imu) = (tilted_and_moving(), manoeuvring());

        let f = transition_matrix(&state, corrected_imu(imu, &state), dt);
        let reference = propagate_nominal(state, corrected_imu(imu, &state), dt);

        for column in 0..STATES {
            let mut dx = [0.0; STATES];
            dx[column] = DELTA;
            let perturbed = perturb(state, &dx);

            // The perturbed biases change the corrected sample, which is how the bias
            // columns of (20) are exercised at all.
            let propagated = propagate_nominal(perturbed, corrected_imu(imu, &perturbed), dt);
            let numerical = error_between(&reference, &propagated);

            for row in 0..STATES {
                let analytic = f[(row, column)];
                let measured = numerical[row] / DELTA;
                assert!(
                    (measured - analytic).abs() < TOLERANCE,
                    "F[{row}][{column}]: {analytic} analytic vs {measured} numerical",
                );
            }
        }
    }

    /// (21) against the closed form its `Δt` is chosen for: with only accelerometer white
    /// noise, an unaided velocity variance grows as `σ_a² T` and position as `σ_a² T³ / 3`,
    /// the integrals of a density over the interval — independent of the rate the samples
    /// arrive at.
    ///
    /// This is the test that fails on the `Δt²` form of (21), and it fails by the whole
    /// factor `1/Δt`: at 200 Hz the velocity variance would come out 200 times too small.
    /// Run it at two rates and the same numbers come back, which is the property the
    /// rate-dependent form gives up and the corpus, at 50 Hz to 250 Hz on one `Config`,
    /// needs.
    #[test]
    fn unaided_growth_from_a_density_matches_the_analytic_integrals() {
        const SIGMA: f32 = 0.35;
        let noise = ImuNoise {
            accel_white: SIGMA,
            gyro_white: 0.0,
            accel_bias_walk: 0.0,
            gyro_bias_walk: 0.0,
        };
        let seconds = 10.0;

        for rate in [50.0, 200.0] {
            let dt = Seconds::from_secs(1.0 / rate);
            let steps = (seconds * rate) as u32;
            let mut covariance = Covariance::zero();
            let mut state = at_rest();

            for _ in 0..steps {
                let step = propagate(state, covariance, holding_still(), dt, &noise);
                (state, covariance) = (step.state, step.covariance);
            }

            let velocity = covariance.variance(ErrorState::VelocityNorth);
            let position = covariance.variance(ErrorState::PositionNorth);
            let expected_velocity = SIGMA * SIGMA * seconds;
            let expected_position = SIGMA * SIGMA * seconds * seconds * seconds / 3.0;

            assert!(
                (velocity - expected_velocity).abs() / expected_velocity < 1.0e-3,
                "{rate} Hz: {velocity} vs {expected_velocity}",
            );
            // The discrete sum approaches `T³/3` from below as `O(1/steps)`, so this one is
            // a percent rather than a tenth of one.
            assert!(
                (position - expected_position).abs() / expected_position < 1.0e-2,
                "{rate} Hz: {position} vs {expected_position}",
            );
        }
    }

    /// (42) after (22), and what a covariance owes over a long unaided run: `P` stays exactly
    /// symmetric, every variance stays positive, and the total uncertainty never falls, since
    /// nothing in propagation removes any.
    ///
    /// Total, not per axis, and the difference is the attitude block: `R{ω Δt}ᵀ` **rotates**
    /// it, so an anisotropic attitude prior — 0.02 rad of tilt against 0.35 of yaw — moves
    /// variance between body axes as the vehicle turns, and yaw's own variance falls while the
    /// block's trace does not. A per-axis monotonicity assertion here fails on correct code;
    /// [`the_attitude_block_rotates_with_the_body`] is what pins the rotation instead.
    #[test]
    fn an_unaided_covariance_stays_symmetric_and_never_loses_uncertainty() {
        let noise = ImuNoise::default();
        let dt = Seconds::from_secs(0.005);
        let mut state = tilted_and_moving();
        let mut covariance = init::initial_covariance(
            &Initialization::default(),
            Radians::from_radians(0.02),
            Radians::from_radians(0.35),
        );

        for _ in 0..12_000 {
            let before = covariance.as_matrix().trace();
            let step = propagate(state, covariance, manoeuvring(), dt, &noise);
            (state, covariance) = (step.state, step.covariance);

            let p = covariance.as_matrix();
            assert!(
                p.trace() >= before,
                "trace fell from {before} to {}",
                p.trace()
            );
            for i in 0..STATES {
                assert!(p[(i, i)] > 0.0, "variance {i} is {}", p[(i, i)]);
                for j in (i + 1)..STATES {
                    assert_eq!(p[(i, j)], p[(j, i)], "asymmetric at ({i}, {j})");
                }
            }
        }
    }

    /// The attitude block of (20) is a rotation, and that is a claim with a consequence: the
    /// covariance of a **body-frame** error rotates with the body. Ninety degrees about body x
    /// swaps what the y and z axes carry, and with no process noise the swap is exact and the
    /// block's trace is conserved.
    ///
    /// It is the same convention the Jacobian test checks from the other side, and the reason
    /// a yaw variance can fall while the filter learns nothing: after a quarter roll, the
    /// well-known tilt axis is where yaw's ignorance used to be.
    #[test]
    fn the_attitude_block_rotates_with_the_body() {
        let quiet = ImuNoise {
            gyro_white: 0.0,
            accel_white: 0.0,
            gyro_bias_walk: 0.0,
            accel_bias_walk: 0.0,
        };
        let (tilt, yaw) = (0.02, 0.35);
        // Every other prior zero, so the block below is `R P_θθ Rᵀ` and nothing else: the
        // gyroscope-bias prior would otherwise reach it through (20)'s `−I Δt`.
        #[rustfmt::skip]
        let sigmas = [
            0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,
            tilt, tilt, yaw,
            0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,
        ];

        // A quarter turn about body x in one step: `Exp(ω Δt)` with `ω Δt = π/2 x̂`.
        let dt = Seconds::from_secs(1.0);
        let imu = ImuSample {
            gyro: AngularRate::body(core::f32::consts::FRAC_PI_2, 0.0, 0.0),
            accel: Acceleration::body(0.0, 0.0, -GRAVITY),
        };

        let step = propagate(at_rest(), Covariance::from_sigmas(sigmas), imu, dt, &quiet);
        let after = step.covariance;

        assert!((after.variance(ErrorState::AttitudeX) - tilt * tilt).abs() < 1.0e-9);
        assert!((after.variance(ErrorState::AttitudeY) - yaw * yaw).abs() < 1.0e-6);
        assert!((after.variance(ErrorState::AttitudeZ) - tilt * tilt).abs() < 1.0e-6);
    }

    /// The ordering hazard of (20), one level up from (13)/(14)'s: `F` linearizes about the
    /// attitude at the **start** of the step, because `R(q̂)` enters the velocity row and (15)
    /// has moved `q̂` by the time the nominal step returns.
    ///
    /// Stated as the two candidate orderings rather than as a tolerance, the way
    /// [`position_uses_the_pre_update_velocity`] states its excess: over a step carrying a
    /// radian of yaw, an accelerometer-bias prior on body x reaches north velocity through
    /// `−R Δt`, and the two linearization points put it in different places. The margin below
    /// is what that costs — the wrong ordering is not a rounding difference, and it grows with
    /// the rotation rate, which is exactly when a covariance is being relied on.
    #[test]
    fn the_transition_linearizes_about_the_start_of_the_step() {
        let quiet = ImuNoise {
            gyro_white: 0.0,
            accel_white: 0.0,
            gyro_bias_walk: 0.0,
            accel_bias_walk: 0.0,
        };
        // A radian of yaw in one step, and an accelerometer-bias prior on body x alone: the
        // only thing that differs between the two orderings is which way that axis points.
        let dt = Seconds::from_secs(0.1);
        let imu = ImuSample {
            gyro: AngularRate::body(0.0, 0.0, 10.0),
            accel: Acceleration::body(0.0, 0.0, -GRAVITY),
        };
        #[rustfmt::skip]
        let sigmas = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.5, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ];
        let prior = Covariance::from_sigmas(sigmas);
        let state = at_rest();
        let corrected = corrected_imu(imu, &state);
        let q = process_noise(&quiet, dt);

        let before = propagate_covariance(prior, &transition_matrix(&state, corrected, dt), q);
        let after = propagate_covariance(
            prior,
            &transition_matrix(&propagate_nominal(state, corrected, dt), corrected, dt),
            q,
        );

        let step = propagate(state, prior, imu, dt, &quiet);
        assert_eq!(step.covariance, before);

        // What the other ordering would have claimed about north velocity: `cos²(0)` of the
        // prior against `cos²(1 rad)`, which is 0.0025 m² s⁻² against 0.00073.
        let north = |p: &Covariance| p.variance(ErrorState::VelocityNorth);
        assert!(
            (north(&before) - 0.002_5).abs() < 1.0e-6,
            "{}",
            north(&before)
        );
        assert!(
            (north(&after) - 0.000_73).abs() < 1.0e-5,
            "{}",
            north(&after)
        );
    }

    /// The covariance half of what (11)–(14) already owed: `F P Fᵀ` leaves f32's range from
    /// finite inputs, and the step reports it rather than committing an infinity that reaches
    /// every later `S` and never leaves.
    #[test]
    fn a_covariance_that_overflows_is_not_finite() {
        let enormous = Covariance::from_matrix(CovarianceMatrix::from_diagonal_element(f32::MAX));
        let step = propagate(
            tilted_and_moving(),
            enormous,
            manoeuvring(),
            Seconds::from_secs(0.005),
            &ImuNoise::default(),
        );

        assert!(step.state.is_finite(), "the state itself is fine");
        assert!(!step.covariance.is_finite());
        assert!(!step.is_finite());
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
