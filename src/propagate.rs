//! Propagation: the IMU sample, bias correction, the nominal kinematics and the covariance.
//! Equations (9)–(22); see the equation-to-code table in `EQUATIONS.md`.
//!
//! The nominal state and `P` advance together in [`propagate`], and
//! [`Eskf::predict`](crate::Eskf::predict) commits both or neither. Nothing here shrinks a
//! covariance: (22) only adds, and the measurement update of (23)–(28) is what takes
//! uncertainty back out.

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::config::{Coast, Config, ImuNoise};
use crate::frames::Body;
use crate::math::{enforce_symmetry, exp_quat, skew};
use crate::state::{Covariance, ErrorState, Offset, STATES, State};
use crate::units::{
    Acceleration, AngularRate, Attitude, DeltaAngle, DeltaVelocity, Position, Seconds, Timestamp,
    Velocity,
};

/// One IMU measurement, uncorrected: the rotation and the velocity increments over the
/// intervals each was integrated across, and when they end. The filter subtracts its own
/// bias estimates, equations (9) and (10).
///
/// PX4's `imuSample` (`src/modules/ekf2/EKF/common.h:182-189` at `c4e4ef98`), and
/// ArduPilot's `imu_elements` (`libraries/AP_NavEKF3/AP_NavEKF3_core.h:598-604` at
/// `368dc0c4`). Increments rather than rates, because an integrating driver produces
/// increments and (13)–(15) consume them: a rate between the two is a division by the
/// interval that the next line multiplies back. A rate source is the one that converts,
/// through [`from_rates`](Self::from_rates).
///
/// Three times, where a single `dt` would collapse them. The two intervals are what each
/// increment integrated, kept apart as both estimators keep them, since a driver may close
/// the gyroscope's and the accelerometer's integrals at different moments. `time` is when
/// the sample ends, and the step [`Eskf::predict`](crate::Eskf::predict) takes is the time
/// since the previous sample's, which is neither interval: a logger that drops samples
/// leaves an increment integrated over 2.5 ms arriving a second after the one before it, and
/// only the timestamps see the second. Such a sample integrates what it measured and the rest
/// of the step goes unintegrated: a gap longer than
/// [`Config::max_predict_dt`](crate::Config::max_predict_dt) is coasted whole, and a shorter
/// one is not priced, so a driver that drops samples should hand over the integral across the
/// drop, as an integrating one does.
///
/// `Default` is a placeholder for struct-update syntax, not a sample: its intervals are zero,
/// which [`Eskf::predict`](crate::Eskf::predict) and
/// [`Eskf::initialize`](crate::Eskf::initialize) both refuse as an invalid interval.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    /// When both integration intervals end, on the clock every measurement is timed on.
    pub time: Timestamp,
    /// Rotation over [`angle_interval`](Self::angle_interval), body frame.
    pub delta_angle: DeltaAngle<Body>,
    /// The interval `delta_angle` integrates.
    pub angle_interval: Seconds,
    /// Specific force integrated over [`velocity_interval`](Self::velocity_interval), body
    /// frame. A level, stationary vehicle gains `(0, 0, −γ Δt)`.
    pub delta_velocity: DeltaVelocity<Body>,
    /// The interval `delta_velocity` integrates.
    pub velocity_interval: Seconds,
}

impl ImuSample {
    /// From a rate gyroscope and an accelerometer read at `time`, each reading standing for
    /// the `interval` since the previous one.
    ///
    /// Multiplies, which is the conversion a rate source owes and the only one: ekf2 does the
    /// same with `sensor_combined` (`src/modules/ekf2/EKF2.cpp:703-707` at `c4e4ef98`).
    pub fn from_rates(
        time: Timestamp,
        gyro: AngularRate<Body>,
        accel: Acceleration<Body>,
        interval: Seconds,
    ) -> Self {
        let dt = interval.as_secs();
        Self {
            time,
            delta_angle: DeltaAngle::from_vector(gyro.vector() * dt),
            angle_interval: interval,
            delta_velocity: DeltaVelocity::from_vector(accel.vector() * dt),
            velocity_interval: interval,
        }
    }

    /// `self` and the `later` sample after it as one sample: increments and intervals summed,
    /// timed at `later`'s end. What a driver reading a FIFO in batches, or running the filter
    /// slower than its IMU, hands over.
    ///
    /// Summed rather than composed, which is exact for a vehicle that does not rotate across
    /// the pair and first order for one that does. PX4's `ImuDownSampler` composes the
    /// rotations as quaternions and rotates the velocity increment into the frame the batch
    /// ends in (`src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp:25-37` at
    /// `c4e4ef98e9`); summing leaves out `½ Δθ₁ × Δθ₂`, the coning term, and its sculling
    /// counterpart in velocity. It is nonzero only while the rotation axis moves, and at most
    /// 3e-6 rad at 1 rad/s across two 2.5 ms samples; an integrating driver that already
    /// corrects for it should hand its own sum over instead.
    ///
    /// The times are not checked: [`Eskf::predict`](crate::Eskf::predict) refuses a sample
    /// that is not after the last, and a pair out of order is that sample.
    pub fn accumulate(self, later: ImuSample) -> ImuSample {
        let sum = |a: Seconds, b: Seconds| Seconds::from_secs(a.as_secs() + b.as_secs());
        ImuSample {
            time: later.time,
            delta_angle: DeltaAngle::from_vector(
                self.delta_angle.vector() + later.delta_angle.vector(),
            ),
            angle_interval: sum(self.angle_interval, later.angle_interval),
            delta_velocity: DeltaVelocity::from_vector(
                self.delta_velocity.vector() + later.delta_velocity.vector(),
            ),
            velocity_interval: sum(self.velocity_interval, later.velocity_interval),
        }
    }

    /// The average angular rate over the sample, `Δθ / Δt`: what initialization's
    /// stationarity test and gyroscope bias read, both statements about a rate.
    pub(crate) fn angular_rate(self) -> AngularRate<Body> {
        AngularRate::from_vector(self.delta_angle.vector() / self.angle_interval.as_secs())
    }

    /// The average specific force over the sample, `Δv / Δt`; see
    /// [`angular_rate`](Self::angular_rate).
    pub(crate) fn specific_force(self) -> Acceleration<Body> {
        Acceleration::from_vector(self.delta_velocity.vector() / self.velocity_interval.as_secs())
    }

    /// Whether every number in the sample is finite, intervals included.
    ///
    /// Written once and called from both places a sample enters the filter, so that
    /// [`Eskf::initialize`](crate::Eskf::initialize) and
    /// [`Eskf::predict`](crate::Eskf::predict) refuse the same sample.
    pub(crate) fn is_finite(self) -> bool {
        self.delta_angle.is_finite()
            && self.delta_velocity.is_finite()
            && self.angle_interval.as_secs().is_finite()
            && self.velocity_interval.as_secs().is_finite()
    }

    /// The interval that is not a forward span of time of at least a microsecond, the
    /// resolution a [`Timestamp`] keeps, if either is not. A zero one divides into a rate, a
    /// negative one subtracts (21)'s process noise, and one of 1e-40 s divides an increment
    /// into an infinity.
    ///
    /// The interval rather than a verdict, and written once, so that
    /// [`Eskf::initialize`](crate::Eskf::initialize) and
    /// [`Eskf::predict`](crate::Eskf::predict) refuse the same sample and name the same number.
    pub(crate) fn unusable_interval(self) -> Option<Seconds> {
        const LEAST: f32 = 1.0e-6;
        [self.angle_interval, self.velocity_interval]
            .into_iter()
            .find(|interval| interval.as_secs().is_nan() || interval.as_secs() < LEAST)
    }

    /// The longer of the two intervals: the span the sample claims to describe.
    pub(crate) fn longest_interval(self) -> Seconds {
        if self.angle_interval > self.velocity_interval {
            self.angle_interval
        } else {
            self.velocity_interval
        }
    }
}

/// Tests build samples from rates, the form most fixtures state a motion in, and time them
/// where they are used.
#[cfg(test)]
impl ImuSample {
    /// A sample reading `gyro` and `accel`, over one second until [`timed`](Self::timed).
    pub(crate) fn reading(gyro: AngularRate<Body>, accel: Acceleration<Body>) -> Self {
        Self::from_rates(Timestamp::ZERO, gyro, accel, Seconds::from_secs(1.0))
    }

    /// This sample with the gyroscope reading `gyro` over the same interval.
    pub(crate) fn with_gyro(self, gyro: AngularRate<Body>) -> Self {
        Self {
            delta_angle: DeltaAngle::from_vector(gyro.vector() * self.angle_interval.as_secs()),
            ..self
        }
    }

    /// This sample with the accelerometer reading `accel` over the same interval.
    pub(crate) fn with_accel(self, accel: Acceleration<Body>) -> Self {
        Self {
            delta_velocity: DeltaVelocity::from_vector(
                accel.vector() * self.velocity_interval.as_secs(),
            ),
            ..self
        }
    }

    /// The same rates over `interval`, ending at `time`. A sample with no interval, the
    /// default, reads as zero rates.
    pub(crate) fn timed(self, time: Timestamp, interval: Seconds) -> Self {
        let rate = |increment: Vector3<f32>, over: Seconds| {
            if over.as_secs() > 0.0 {
                increment / over.as_secs()
            } else {
                Vector3::zeros()
            }
        };
        Self::from_rates(
            time,
            AngularRate::from_vector(rate(self.delta_angle.vector(), self.angle_interval)),
            Acceleration::from_vector(rate(self.delta_velocity.vector(), self.velocity_interval)),
            interval,
        )
    }
}

/// An [`ImuSample`] with the filter's bias estimates removed: `ω Δt` and `a_b Δt` of equations
/// (9) and (10), with the intervals they cover.
///
/// A type of its own rather than another [`ImuSample`], which carries the same fields in the
/// same frames. What separates them is whether the bias has been taken off, and that is the
/// claim that causes the bug: subtracting twice removes a bias the sample no longer carries,
/// subtracting never hands (11) the raw measurement. Neither shows up in the numbers, both
/// being small, plausible increments.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Corrected {
    /// `ω Δt_θ`, the rotation increment less `β̂_g Δt_θ`.
    pub(crate) delta_angle: DeltaAngle<Body>,
    /// `Δt_θ`.
    pub(crate) angle_interval: Seconds,
    /// `a_b Δt_v`, the velocity increment less `β̂_a Δt_v`.
    pub(crate) delta_velocity: DeltaVelocity<Body>,
    /// `Δt_v`.
    pub(crate) velocity_interval: Seconds,
}

impl Corrected {
    /// `ω` of (9), what [`Eskf::angular_rate`](crate::Eskf::angular_rate) reports.
    pub(crate) fn omega(self) -> AngularRate<Body> {
        AngularRate::from_vector(self.delta_angle.vector() / self.angle_interval.as_secs())
    }
}

/// Subtract the filter's own bias estimates from a raw sample, each over its own increment's
/// interval. Equations (9) and (10).
pub(crate) fn corrected_imu(imu: ImuSample, state: &State) -> Corrected {
    Corrected {
        delta_angle: DeltaAngle::from_vector(
            imu.delta_angle.vector() - state.gyro_bias.vector() * imu.angle_interval.as_secs(),
        ),
        angle_interval: imu.angle_interval,
        delta_velocity: DeltaVelocity::from_vector(
            imu.delta_velocity.vector()
                - state.accel_bias.vector() * imu.velocity_interval.as_secs(),
        ),
        velocity_interval: imu.velocity_interval,
    }
}

/// Advance the nominal state across one sample by dead reckoning. Equations (11) and
/// (13)–(15), in increments.
///
/// Translation runs over the velocity increment's interval and rotation over the angle
/// increment's, each the span its measurement covers: gravity is added over the same `Δt_v`
/// the accelerometer integrated, so a vehicle at rest gains nothing whatever the two
/// intervals are. PX4 integrates over the same two (`EKF/ekf.cpp:241-271` at `c4e4ef98`).
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
pub(crate) fn propagate_nominal(state: State, imu: Corrected, gravity: f32) -> State {
    let dt = imu.velocity_interval.as_secs();
    let rotation = state.attitude.quaternion();

    // (11), times `Δt`: the velocity increment into the navigation frame, gravity's added.
    // A level vehicle at rest gains (0, 0, -γ Δt) and this is zero, which is the sign
    // convention's own test — down-positive gravity against a down-negative specific force.
    let delta_v = rotation * imu.delta_velocity.vector() + gravity_vector(gravity) * dt;

    let velocity = state.velocity.vector();
    let position = state.position.vector() + velocity * dt + 0.5 * delta_v * dt; // (13)
    let velocity = velocity + delta_v; // (14), from the pre-update velocity above

    // (15): the body-frame rotation increment composes on the right. `exp_quat` keeps the
    // first-order term where `UnitQuaternion::from_scaled_axis` substitutes the identity,
    // and the product of two unit quaternions drifts off the manifold in f32 over a
    // flight. Not `renormalize_fast`: its first-order approximation saves a square root on
    // a path that already evaluates a sine and a cosine per sample.
    let mut attitude = rotation * exp_quat(imu.delta_angle.vector());
    attitude.renormalize();

    State {
        attitude: Attitude::from_quaternion(attitude),
        position: Position::from_vector(position),
        velocity: Velocity::from_vector(velocity),
        ..state
    }
}

/// `F` of equation (20): 15 x 15, the same shape as the covariance it propagates.
pub(crate) type Transition = SMatrix<f32, STATES, STATES>;

/// The nominal state and the covariance after one step, before either is committed, with the
/// rate the step integrated.
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
    /// The barometric offset's share of it, (30′).
    pub(crate) offset: Offset,
    /// `ω` of (9), what [`Eskf::angular_rate`](crate::Eskf::angular_rate) reports: `None`
    /// from a coast, which reads no sample.
    pub(crate) omega: Option<AngularRate<Body>>,
}

impl Propagated {
    /// Whether every number in the step is finite.
    pub(crate) fn is_finite(&self) -> bool {
        self.state.is_finite()
            && self.covariance.is_finite()
            && self.offset.is_finite()
            && self.omega.is_none_or(|omega| omega.is_finite())
    }
}

/// Advance the nominal state and its covariance across one sample. Equations (9)–(22).
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
    offset: Offset,
    imu: ImuSample,
    config: &Config,
) -> Propagated {
    let corrected = corrected_imu(imu, &state);
    let transition = transition_matrix(&state, corrected);
    let q = process_noise(&config.imu, corrected);

    Propagated {
        state: propagate_nominal(state, corrected, config.gravity),
        covariance: propagate_covariance(covariance, &transition, q),
        offset: propagate_offset(
            offset,
            &transition,
            config.baro_offset_walk,
            corrected.velocity_interval,
        ),
        omega: Some(corrected.omega()),
    }
}

/// `P_xb ← F P_xb` and `P_bb ← P_bb + q_b² Δt`. Equation (30′).
///
/// The offset is a random walk with no dynamics of its own, so its block of the augmented
/// transition is one and the cross-covariance moves only with the error state it is
/// correlated with: a 15 × 15 by 15 × 1 product, where the augmented `F P Fᵀ` would repeat
/// (22) at sixteen.
fn propagate_offset(offset: Offset, f: &Transition, walk: f32, dt: Seconds) -> Offset {
    Offset {
        cross: f * offset.cross,
        variance: offset.variance + walk * walk * dt.as_secs(),
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
/// of (18)'s homogeneous part, `R{ω Δt}ᵀ = exp(−[ω]ₓ Δt)`. Each `Δt` is the interval of the
/// increment the block reads: `Δt_v` on the translational blocks, `Δt_θ` on the rotational,
/// so `[a_b]ₓ Δt` is the corrected velocity increment's own skew and no rate is formed. (20) permits `I − [ω]ₓ Δt` where
/// the exact form is not justified, and here it is: the exact block costs one [`exp_quat`] --
/// a sine, a cosine, and a quaternion to matrix — against the roughly 6750 multiplications
/// the two 15 x 15 products of (22) spend on the same sample. The approximation buys nothing
/// measurable and gives up accuracy at high rotation rates, which is the regime a propagated
/// covariance exists for.
///
/// `imu` is the sample the nominal step reads, and `state` is that step's input rather than
/// its output; [`propagate`] holds both, which is why neither is looked up here.
fn transition_matrix(state: &State, imu: Corrected) -> Transition {
    let (dt_v, dt_theta) = (
        imu.velocity_interval.as_secs(),
        imu.angle_interval.as_secs(),
    );
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
        .copy_from(&(Matrix3::identity() * dt_v));
    // (17): the two terms an accelerometer error enters velocity through.
    f.fixed_view_mut::<3, 3>(v, theta)
        .copy_from(&(-r * skew(imu.delta_velocity.vector())));
    f.fixed_view_mut::<3, 3>(v, beta_a).copy_from(&(-r * dt_v));
    // (18): exact in the attitude block, first order in the gyroscope bias. `R{ω Δt}ᵀ` is
    // the exact solution of `δθ̇ = −[ω]ₓ δθ`, which is why (15)'s own increment builds it.
    let increment = exp_quat(imu.delta_angle.vector()).to_rotation_matrix();
    f.fixed_view_mut::<3, 3>(theta, theta)
        .copy_from(&increment.matrix().transpose());
    f.fixed_view_mut::<3, 3>(theta, beta_g)
        .copy_from(&(-Matrix3::identity() * dt_theta));

    f
}

/// `A`, the continuous error dynamics (16)–(19) as a matrix: `δẋ = A δx` with the noise left
/// out. What (20) discretizes, taken here at the rates `ω` and `a_b` rather than a sample.
pub(crate) fn error_dynamics(state: &State, omega: Vector3<f32>, a_b: Vector3<f32>) -> Transition {
    let rotation = state.attitude.quaternion().to_rotation_matrix();
    let r = rotation.matrix();
    let mut a = Transition::zeros();
    let (p, v, theta, beta_a, beta_g) = (
        ErrorState::PositionNorth.index(),
        ErrorState::VelocityNorth.index(),
        ErrorState::AttitudeX.index(),
        ErrorState::AccelBiasX.index(),
        ErrorState::GyroBiasX.index(),
    );
    a.fixed_view_mut::<3, 3>(p, v)
        .copy_from(&Matrix3::identity());
    a.fixed_view_mut::<3, 3>(v, theta)
        .copy_from(&(-r * skew(a_b)));
    a.fixed_view_mut::<3, 3>(v, beta_a).copy_from(&(-r));
    a.fixed_view_mut::<3, 3>(theta, theta)
        .copy_from(&(-skew(omega)));
    a.fixed_view_mut::<3, 3>(theta, beta_g)
        .copy_from(&(-Matrix3::identity()));
    a
}

/// `Q`, the discrete process noise of equation (21), as its diagonal, over the intervals the
/// sample integrated: the accelerometer's densities over `Δt_v`, the gyroscope's over `Δt_θ`.
///
/// A diagonal rather than a 15 x 15 matrix because (21) has twelve nonzero entries: the matrix form
/// would spend a 15 × 15 temporary and 225 additions to add twelve numbers, and "the diagonal of
/// `Q` lands on the diagonal of `P`" is closer to the equation than a dense sum. What makes that
/// legitimate is the isotropy below; a per-axis `Σ_a` is what would make this a matrix again.
///
/// **`Δt`, not `Δt²`.** [`ImuNoise`]'s four fields are spectral densities, and a density's
/// contribution over a step is `σ² Δt` — for the white-noise blocks and the two random walks
/// alike. A `Δt²` on the white-noise blocks is PX4's and ArduPilot's form,
/// where the parameter is the σ of one sample's increment rather than a density: PX4
/// `sq(dt) * accel_var` with `accel_var = sq(ekf2_acc_noise)`
/// (`src/modules/ekf2/EKF/python/ekf_derivation/generated/predict_covariance.h:161-164`,
/// `EKF/covariance.cpp:119-133`, at `c4e4ef98e9`), ArduPilot `dvxVar = sq(dt * _accNoise)`
/// (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1177` at `368dc0c428`). That form ties `Q` to
/// the IMU rate — the variance it adds over `T` seconds is `σ² Δt T`, so the same airframe
/// logged at 400 Hz is given eight times less process noise than at 50 Hz — and the corpus in
/// `data/manifest.txt` spans 50 Hz to 250 Hz with `Config` shared across all of it. `Δt` is
/// rate-independent, and `EQUATIONS.md` (21) states it.
///
/// Reading [`ImuNoise`] as densities is what `examples/simulate.rs` does when it draws
/// per-sample noise as `white / √Δt`, so filter and simulator agree on what the numbers
/// mean. That file also states the consequence, which is the check on this decision: against
/// its IMU table the filter's `Q` is two orders of magnitude conservative, so every scenario
/// must come out *under*-confident, and a `nees_*` above one is a finding rather than a pass.
///
/// The velocity block of (21) is the rotated accelerometer noise `R Σ_a Rᵀ`, and writing it
/// `σ_a² I` is exact only where `Σ_a` is isotropic, since `R (σ_a² I) Rᵀ = σ_a² I` for
/// orthogonal `R`. Real IMUs are noisier about z. One scalar per sensor is what [`ImuNoise`]
/// can express, so the conservatism is in the number rather than in the model: `σ_a` is the
/// worst axis.
fn process_noise(noise: &ImuNoise, imu: Corrected) -> [f32; STATES] {
    let (dt_v, dt_theta) = (
        imu.velocity_interval.as_secs(),
        imu.angle_interval.as_secs(),
    );
    let densities = Densities::of(noise, None);
    let velocity = densities.velocity * dt_v;
    let attitude = densities.attitude * dt_theta;
    let accel_bias = densities.accel_bias * dt_v;
    let gyro_bias = densities.gyro_bias * dt_theta;

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
/// Written as (22) reads, which is the most expensive thing the filter does: three 15 × 15
/// temporaries per IMU sample, at up to 400 Hz. (20) is sparse enough (two identity blocks, two
/// zero rows) that a block-wise form would cut them, at the cost of the one equation a reader of
/// this crate is most likely to have come for. Its stack frame is [published], and its
/// arithmetic [counted]. The block-wise form is not built: [timed] on a 400 MHz Cortex-M7, a step
/// sits well inside a 400 Hz period.
///
/// `Q` arrives as a diagonal and is added as one, which keeps those temporaries to three
/// rather than four.
///
/// Out of line by attribute: inlined into `propagate`, its one caller, the three temporaries
/// sit in `predict`'s own chain ([measured]).
///
/// [published]: https://github.com/wboayue/fusion-nav/blob/main/validation/cost.md#stack
/// [counted]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#arithmetic
/// [timed]: https://github.com/wboayue/fusion-nav/blob/main/validation/cost.md#time-on-a-target
/// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#stack-frames
#[inline(never)]
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

/// Grow `P` over `horizon` as if nothing were measured and the vehicle stayed put.
/// Equation (22′), without the unmeasured motion a gap allows.
///
/// This is the arming question's half of [`Eskf::predicted_validity`](crate::Eskf::predicted_validity):
/// not *is the estimate good now*, but *will it still be good in `horizon` seconds if I take off
/// and nothing aids it*. The covariance is the only thing that can answer that, and the answer
/// keeps the correlations (17) and (20) build, which is where most of the growth a few seconds
/// out comes from.
///
/// The IMU input is the one the question presumes: a vehicle on the ground, not rotating, so
/// `ω = 0` and the specific force is `−R(q̂)ᵀ g`, what a stationary accelerometer reads at the
/// current attitude. That is what makes the tilt-to-velocity coupling of (17) the gravity leak
/// it is in flight, rather than zero. It is [`coast`]'s assumption with neither of [`Coast`]'s
/// densities, since `F` does not read velocity and an unaccelerated vehicle and one standing
/// still grow it alike, so both are [`unaccelerated_growth`]: one exact step at any horizon. The
/// nominal state is untouched and no timer moves. A projection is not time passing.
pub(crate) fn project(
    state: &State,
    covariance: Covariance,
    horizon: Seconds,
    config: &Config,
) -> Covariance {
    // A horizon that is not a positive duration projects nothing rather than projecting
    // backwards: a negative `T` *subtracts* process noise and lands variances below zero, which
    // `Validity` reads as an estimate better than any the filter could have. `Config::validate`
    // refuses a negative or NaN `Accuracy::horizon`; zero, which it allows, arrives here.
    let seconds = horizon.as_secs();
    if seconds.is_nan() || seconds <= 0.0 {
        return covariance;
    }
    let densities = Densities::of(&config.imu, None);
    unaccelerated_growth(
        state,
        &covariance,
        seconds,
        config.gravity,
        &densities,
        None,
    )
}

/// Advance the state and its covariance across a gap no IMU sample describes. Equation (22′).
///
/// The input assumed is [`unaccelerated_sample`]'s: no rotation, and the specific force that
/// cancels gravity. Run through (13)–(15) it moves position by `v̂ T` and nothing else, and its
/// error dynamics are [`unaccelerated_growth`]'s, so the state and the covariance describe one
/// hypothesis. What the hypothesis leaves out, the vehicle's actual acceleration and rotation,
/// enters as the white densities of [`Coast`] on the velocity and attitude blocks, beside (21)'s
/// own noise.
///
/// One step at any gap, the nominal one because an unaccelerated vehicle's (13) is exact over
/// any interval, and the covariance's because `ω = 0` makes the error dynamics nilpotent: the
/// transition and the noise it integrates are polynomials in `T`, exact where (22) run in short
/// steps understates. So a gap costs one (22)'s arithmetic, where the steps cost a long gap
/// nearly two periods of a 400 Hz loop ([measured]).
///
/// A gap that is not a positive duration coasts nothing, for [`project`]'s reason.
/// [`Eskf::predict`](crate::Eskf::predict) coasts only past a positive limit, so the guard
/// holds a bound no caller in the crate crosses.
///
/// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#coast-and-projection
pub(crate) fn coast(
    state: State,
    covariance: Covariance,
    offset: Offset,
    gap: Seconds,
    config: &Config,
    unmeasured: &Coast,
) -> Propagated {
    let seconds = gap.as_secs();
    if seconds.is_nan() || seconds <= 0.0 {
        return Propagated {
            state,
            covariance,
            offset,
            omega: None,
        };
    }
    let densities = Densities::of(&config.imu, Some(unmeasured));
    // (30′) across the whole gap at once: the offset has no dynamics, so its cross-covariance
    // moves with the error state through `Φ`, and its variance walks by `q_b² T`.
    let mut cross = offset.cross;
    let covariance = unaccelerated_growth(
        &state,
        &covariance,
        seconds,
        config.gravity,
        &densities,
        Some(&mut cross),
    );
    let walk = config.baro_offset_walk;
    Propagated {
        state: propagate_nominal(
            state,
            unaccelerated_sample(&state, gap, config.gravity),
            config.gravity,
        ),
        covariance,
        offset: Offset {
            cross,
            variance: offset.variance + walk * walk * seconds,
        },
        omega: None,
    }
}

/// The white densities driving the error, per axis: (21)'s, with [`Coast`]'s unmeasured
/// acceleration and rotation added where a gap allows them.
struct Densities {
    velocity: f32,
    attitude: f32,
    accel_bias: f32,
    gyro_bias: f32,
}

impl Densities {
    fn of(noise: &ImuNoise, unmeasured: Option<&Coast>) -> Self {
        let (acceleration, rotation) =
            unmeasured.map_or((0.0, 0.0), |c| (c.acceleration, c.rotation));
        Self {
            velocity: noise.accel_white * noise.accel_white + acceleration * acceleration,
            attitude: noise.gyro_white * noise.gyro_white + rotation * rotation,
            accel_bias: noise.accel_bias_walk * noise.accel_bias_walk,
            gyro_bias: noise.gyro_bias_walk * noise.gyro_bias_walk,
        }
    }
}

/// `Φ`, the exact transition over `T` seconds of an unaccelerated, unrotating vehicle.
/// Equation (22′).
///
/// With `ω = 0` and `a_b = −R(q̂)ᵀ g`, the dynamics (16)–(19) are a chain, `δβg → δθ → δv → δp`,
/// with `δβa` entering velocity, so `A⁴ = 0` and `Φ = exp(A T) = I + A T + A² T²/2 + A³ T³/6`
/// has four terms; `EQUATIONS.md` (22′) writes it out. Built in blocks rather than as `A` and its
/// powers: it has eleven nonzero blocks, each a scalar times `I`, `R` or `G`, where `G` is (17)'s
/// `−R(q̂)[a_b]ₓ`, for this `a_b` the gravity leak `[g]ₓ R(q̂)`.
fn unaccelerated_transition(state: &State, t: f32, gravity: f32) -> Transition {
    let (r, g) = gravity_leak(state, gravity);
    let i = Matrix3::<f32>::identity();
    let t2 = t * t;
    let (p, v, theta, beta_a, beta_g) = (
        ErrorState::PositionNorth.index(),
        ErrorState::VelocityNorth.index(),
        ErrorState::AttitudeX.index(),
        ErrorState::AccelBiasX.index(),
        ErrorState::GyroBiasX.index(),
    );
    // Row by row: position, velocity, attitude; the biases' rows are the identity's.
    let mut phi = Transition::identity();
    phi.fixed_view_mut::<3, 3>(p, v).copy_from(&(i * t));
    phi.fixed_view_mut::<3, 3>(p, theta)
        .copy_from(&(g * (t2 / 2.0)));
    phi.fixed_view_mut::<3, 3>(p, beta_a)
        .copy_from(&(-r * (t2 / 2.0)));
    phi.fixed_view_mut::<3, 3>(p, beta_g)
        .copy_from(&(-g * (t2 * t / 6.0)));
    phi.fixed_view_mut::<3, 3>(v, theta).copy_from(&(g * t));
    phi.fixed_view_mut::<3, 3>(v, beta_a).copy_from(&(-r * t));
    phi.fixed_view_mut::<3, 3>(v, beta_g)
        .copy_from(&(-g * (t2 / 2.0)));
    phi.fixed_view_mut::<3, 3>(theta, beta_g)
        .copy_from(&(-i * t));
    phi
}

/// `R(q̂)` and (17)'s `G = −R(q̂)[a_b]ₓ` at the specific force of an unaccelerated vehicle.
fn gravity_leak(state: &State, gravity: f32) -> (Matrix3<f32>, Matrix3<f32>) {
    let r = *state.attitude.quaternion().to_rotation_matrix().matrix();
    let g = -r * skew(unaccelerated_force(state, gravity));
    (r, g)
}

/// `P ← Φ P Φᵀ + Q_d` over `T` seconds of an unaccelerated, unrotating vehicle, exactly.
/// Equation (22′), with `Φ` [`unaccelerated_transition`]'s.
///
/// `Q_d = ∫₀ᵀ Φ(s) Q_c Φ(s)ᵀ ds` integrates each density along the column of `Φ` it enters by, in
/// closed form: twenty-one blocks counting both halves, each a scalar times `I`, `R`, `G` or
/// `G Gᵀ`. `cross`, where given, is a column correlated with the error state, (30′)'s `P_xb`,
/// carried through the same `Φ` here rather than handing `Φ` back: returned, it sat in every
/// caller's frame, `predicted_validity`'s included ([measured]).
///
/// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#stack-frames
fn unaccelerated_growth(
    state: &State,
    covariance: &Covariance,
    t: f32,
    gravity: f32,
    q: &Densities,
    cross: Option<&mut SVector<f32, STATES>>,
) -> Covariance {
    let (r, g) = gravity_leak(state, gravity);
    let i = Matrix3::<f32>::identity();
    let (t2, t3) = (t * t, t * t * t);
    let (t4, t5) = (t3 * t, t3 * t2);
    let (p, v, theta, beta_a, beta_g) = (
        ErrorState::PositionNorth.index(),
        ErrorState::VelocityNorth.index(),
        ErrorState::AttitudeX.index(),
        ErrorState::AccelBiasX.index(),
        ErrorState::GyroBiasX.index(),
    );

    let phi = unaccelerated_transition(state, t, gravity);
    if let Some(cross) = cross {
        *cross = phi * *cross;
    }
    let mut next = phi * covariance.as_matrix() * phi.transpose();

    // Q_d, the upper blocks; the lower are their transposes, written beside them. Each line
    // sums the densities that reach that pair of states, with the integral of the product of
    // the two columns they enter by.
    let ggt = g * g.transpose();
    let (qv, qt, qa, qg) = (q.velocity, q.attitude, q.accel_bias, q.gyro_bias);
    // One statement per block, with the indices constants: a loop over `(row, column)` pairs
    // leaves `fixed_view_mut`'s bounds check in, a panic path nothing in `src/` may carry.
    macro_rules! add {
        ($row:expr, $column:expr, $block:expr) => {{
            let block: Matrix3<f32> = $block;
            let mut upper = next.fixed_view_mut::<3, 3>($row, $column);
            upper += block;
            if $row != $column {
                let mut lower = next.fixed_view_mut::<3, 3>($column, $row);
                lower += block.transpose();
            }
        }};
    }
    add!(
        p,
        p,
        i * (qv * t3 / 3.0 + qa * t5 / 20.0) + ggt * (qt * t5 / 20.0 + qg * t5 * t2 / 252.0)
    );
    add!(
        p,
        v,
        i * (qv * t2 / 2.0 + qa * t4 / 8.0) + ggt * (qt * t4 / 8.0 + qg * t5 * t / 72.0)
    );
    add!(p, theta, g * (qt * t3 / 6.0 + qg * t5 / 30.0));
    add!(p, beta_a, -r * (qa * t3 / 6.0));
    add!(p, beta_g, -g * (qg * t4 / 24.0));
    add!(
        v,
        v,
        i * (qv * t + qa * t3 / 3.0) + ggt * (qt * t3 / 3.0 + qg * t5 / 20.0)
    );
    add!(v, theta, g * (qt * t2 / 2.0 + qg * t4 / 8.0));
    add!(v, beta_a, -r * (qa * t2 / 2.0));
    add!(v, beta_g, -g * (qg * t3 / 6.0));
    add!(theta, theta, i * (qt * t + qg * t3 / 3.0));
    add!(theta, beta_g, -i * (qg * t2 / 2.0));
    add!(beta_a, beta_a, i * (qa * t));
    add!(beta_g, beta_g, i * (qg * t));

    // (42), for the rounding the products leave.
    enforce_symmetry(&mut next);
    Covariance::from_matrix(next)
}

/// What the IMU of an unaccelerated vehicle at `state`'s attitude reads over `dt`: no
/// rotation, and the specific force `−R(q̂)ᵀ g` that holds it up. At rest or at constant
/// velocity alike.
///
/// The inverse of the test (11) is written against — a level vehicle at rest reads
/// `(0, 0, −γ)`, evaluated at an attitude that need not be level. [`coast`] moves the nominal
/// state with it, and [`unaccelerated_growth`] linearizes about the same specific force.
fn unaccelerated_sample(state: &State, dt: Seconds, gravity: f32) -> Corrected {
    Corrected {
        delta_angle: DeltaAngle::from_vector(Vector3::zeros()),
        angle_interval: dt,
        delta_velocity: DeltaVelocity::from_vector(
            unaccelerated_force(state, gravity) * dt.as_secs(),
        ),
        velocity_interval: dt,
    }
}

/// `−R(q̂)ᵀ g`, the specific force that holds an unaccelerated vehicle up, in body axes: what
/// [`unaccelerated_sample`] reads and [`gravity_leak`] linearizes about.
fn unaccelerated_force(state: &State, gravity: f32) -> Vector3<f32> {
    state.attitude.quaternion().to_rotation_matrix().inverse() * -gravity_vector(gravity)
}

/// `g = [0, 0, γ]ᵀ`, the navigation-frame gravity vector of (11), for `γ` as
/// [`Config::gravity`] holds it.
pub(crate) fn gravity_vector(gravity: f32) -> Vector3<f32> {
    Vector3::new(0.0, 0.0, gravity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    use crate::config::GRAVITY;

    use crate::config::Initialization;
    use crate::init;
    use crate::state::{CovarianceMatrix, ErrorState};
    use crate::units::Radians;

    const DT: Seconds = Seconds::from_secs(0.005);

    /// The defaults, with `noise` and the barometric offset's `walk` as a test sets them.
    fn under(noise: ImuNoise, walk: f32) -> Config {
        Config {
            imu: noise,
            baro_offset_walk: walk,
            ..Config::default()
        }
    }

    /// At the origin, level, at rest, unbiased.
    fn at_rest() -> State {
        State {
            attitude: Attitude::level(),
            ..State::default()
        }
    }

    /// What a level vehicle at rest measures: gravity alone, down-negative.
    fn holding_still() -> ImuSample {
        ImuSample::reading(
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        )
    }

    fn step(state: State, imu: ImuSample, dt: Seconds) -> State {
        propagate_nominal(
            state,
            corrected_imu(imu.timed(Timestamp::ZERO, dt), &state),
            GRAVITY,
        )
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
        let imu = holding_still().with_gyro(AngularRate::body(0.0, 0.0, rate));

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
        let imu = holding_still().with_accel(Acceleration::body(accel, 0.0, -GRAVITY));

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
        let imu = holding_still().with_gyro(AngularRate::body(0.0, 0.0, rate));

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
        let imu = holding_still().with_accel(Acceleration::body(bias, 0.0, -GRAVITY));

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
            attitude: Attitude::from_quaternion(UnitQuaternion::from_euler_angles(
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
            attitude: Attitude::from_quaternion(UnitQuaternion::from_euler_angles(0.2, -0.3, 0.7)),
            position: Position::ned(0.5, -0.25, 0.1),
            velocity: Velocity::ned(3.0, -1.0, 0.5),
            accel_bias: Acceleration::body(0.05, -0.08, 0.03),
            gyro_bias: AngularRate::body(0.004, -0.007, 0.002),
            ..State::default()
        }
    }

    /// Maneuvering hard enough that `[a_b]ₓ` and `[ω]ₓ` are both far from zero.
    fn maneuvering() -> ImuSample {
        ImuSample::reading(
            AngularRate::body(0.15, -0.23, 0.31),
            Acceleration::body(0.8, -1.3, -9.2),
        )
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
            attitude: Attitude::from_quaternion(state.attitude.quaternion() * exp_quat(part(6))),
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
    /// Two samples accumulated are one over both intervals, timed at the second's end: a
    /// rate read over the sum is the rates' interval-weighted mean.
    #[test]
    fn two_samples_accumulate_into_one_over_both_intervals() {
        let first = maneuvering().timed(Timestamp::from_micros(2_500), Seconds::from_secs(0.0025));
        let second = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 1.0),
            Acceleration::body(0.0, 0.0, -9.8),
        )
        .timed(Timestamp::from_micros(10_000), Seconds::from_secs(0.0075));
        let both = first.accumulate(second);
        assert_eq!(both.time, second.time);
        assert!((both.angle_interval.as_secs() - 0.01).abs() < 1e-7);
        assert!((both.velocity_interval.as_secs() - 0.01).abs() < 1e-7);
        let expected = (0.31 * 0.0025 + 1.0 * 0.0075) / 0.01;
        assert!((both.angular_rate().vector().z - expected).abs() < 1e-5);
        let expected = (-9.2 * 0.0025 - 9.8 * 0.0075) / 0.01;
        assert!((both.specific_force().vector().z - expected).abs() < 1e-4);
    }

    /// `A` is written out beside `F` rather than derived from it, so the two are held together
    /// here: `F = I + A Δt` to first order, at the rates the same corrected sample carries. At
    /// 1 ms the second-order remainder is `|ω|² Δt² / 2`, under 1e-7 at this maneuver, so a
    /// block of `A` placed or signed differently from `F`'s, each at least `Δt`, cannot pass.
    #[test]
    fn the_error_dynamics_are_the_transition_matrix_per_unit_time() {
        let dt = Seconds::from_secs(0.001);
        let state = tilted_and_moving();
        let imu = corrected_imu(maneuvering().timed(Timestamp::ZERO, dt), &state);
        let omega = imu.omega().vector();
        let a_b = imu.delta_velocity.vector() / dt.as_secs();

        let f = transition_matrix(&state, imu);
        let first_order =
            Transition::identity() + error_dynamics(&state, omega, a_b) * dt.as_secs();
        for row in 0..STATES {
            for column in 0..STATES {
                let (exact, linear) = (f[(row, column)], first_order[(row, column)]);
                assert!(
                    (exact - linear).abs() < 1.0e-5,
                    "[{row}][{column}]: F {exact}, I + A dt {linear}"
                );
            }
        }
    }

    #[test]
    fn the_transition_matrix_matches_a_numerical_jacobian_of_the_nominal_step() {
        const DELTA: f32 = 1.0e-3;
        const TOLERANCE: f32 = 2.0e-3;
        let dt = Seconds::from_secs(0.01);
        let (state, imu) = (tilted_and_moving(), maneuvering());

        let f = transition_matrix(
            &state,
            corrected_imu(imu.timed(Timestamp::ZERO, dt), &state),
        );
        let reference = propagate_nominal(
            state,
            corrected_imu(imu.timed(Timestamp::ZERO, dt), &state),
            GRAVITY,
        );

        for column in 0..STATES {
            let mut dx = [0.0; STATES];
            dx[column] = DELTA;
            let perturbed = perturb(state, &dx);

            // The perturbed biases change the corrected sample, which is how the bias
            // columns of (20) are exercised at all.
            let propagated = propagate_nominal(
                perturbed,
                corrected_imu(imu.timed(Timestamp::ZERO, dt), &perturbed),
                GRAVITY,
            );
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
                let step = propagate(
                    state,
                    covariance,
                    Offset::default(),
                    holding_still().timed(Timestamp::ZERO, dt),
                    &under(noise, 0.0),
                );
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
            &state.attitude,
            Radians::from_radians(0.02),
            Radians::from_radians(0.35),
            Some(0.0),
            Vector3::repeat(0.01),
            GRAVITY,
        );

        for _ in 0..12_000 {
            let before = covariance.as_matrix().trace();
            let step = propagate(
                state,
                covariance,
                Offset::default(),
                maneuvering().timed(Timestamp::ZERO, dt),
                &under(noise, 0.0),
            );
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
        let imu = ImuSample::reading(
            AngularRate::body(core::f32::consts::FRAC_PI_2, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );

        let step = propagate(
            at_rest(),
            Covariance::from_sigmas(sigmas),
            Offset::default(),
            imu.timed(Timestamp::ZERO, dt),
            &under(quiet, 0.0),
        );
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
        let imu = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 10.0),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );
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
        let corrected = corrected_imu(imu.timed(Timestamp::ZERO, dt), &state);
        let q = process_noise(&quiet, corrected);

        let before = propagate_covariance(prior, &transition_matrix(&state, corrected), q);
        let after = propagate_covariance(
            prior,
            &transition_matrix(&propagate_nominal(state, corrected, GRAVITY), corrected),
            q,
        );

        let step = propagate(
            state,
            prior,
            Offset::default(),
            imu.timed(Timestamp::ZERO, dt),
            &under(quiet, 0.0),
        );
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
            Offset::default(),
            maneuvering().timed(Timestamp::ZERO, Seconds::from_secs(0.005)),
            &under(ImuNoise::default(), 0.0),
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
        let imu = holding_still().with_accel(Acceleration::body(f32::MAX, 0.0, -GRAVITY));
        assert!(imu.is_finite());

        let state = run(at_rest(), imu, 1_000);
        assert!(!state.is_finite(), "{state:?}");
    }

    #[test]
    fn the_offset_walks_and_its_correlations_move_with_the_state() {
        // `P_xb` correlated with vertical velocity only: one step of (20) carries it into
        // down position by `Δt`, and `P_bb` grows by `q_b² Δt` whatever the state did.
        let mut cross = nalgebra::SVector::<f32, STATES>::zeros();
        cross[ErrorState::VelocityDown.index()] = 1.0;
        let offset = Offset {
            cross,
            variance: 0.25,
        };
        let dt = Seconds::from_secs(0.01);
        let step = propagate(
            at_rest(),
            Covariance::from_sigmas([0.1; STATES]),
            offset,
            holding_still().timed(Timestamp::ZERO, dt),
            &under(ImuNoise::default(), 2.0),
        );
        assert!((step.offset.variance - (0.25 + 4.0 * 0.01)).abs() < 1e-6);
        assert!((step.offset.cross[ErrorState::PositionDown.index()] - 0.01).abs() < 1e-6);
        assert_eq!(step.offset.cross[ErrorState::VelocityDown.index()], 1.0);
    }
}

#[cfg(test)]
mod unaccelerated {
    use super::*;
    use crate::config::{GRAVITY, Initialization};
    use crate::init;
    use crate::units::Radians;
    use nalgebra::UnitQuaternion;

    type Matrix64 = SMatrix<f64, STATES, STATES>;

    /// Tilted and turned, so `R` and the gravity leak `G` are full matrices and a block written
    /// with the wrong one, or transposed, reads differently.
    fn tilted() -> State {
        State {
            attitude: Attitude::from_quaternion(UnitQuaternion::from_euler_angles(0.3, -0.2, 1.1)),
            ..State::default()
        }
    }

    fn densities() -> Densities {
        // Distinct, and large enough that each term clears f32's rounding of the others.
        Densities {
            velocity: 0.3,
            attitude: 0.02,
            accel_bias: 0.005,
            gyro_bias: 0.0007,
        }
    }

    fn widen(m: &Transition) -> Matrix64 {
        m.map(f64::from)
    }

    /// `A` of (16)–(19) at the unaccelerated input, from `error_dynamics`, the function (23′)
    /// reads: a second statement of the dynamics the closed form is checked against.
    fn a(state: &State) -> Matrix64 {
        let rotation = state.attitude.quaternion().to_rotation_matrix();
        let a_b = rotation.inverse() * -gravity_vector(GRAVITY);
        widen(&error_dynamics(state, Vector3::zeros(), a_b))
    }

    fn exp(a: &Matrix64, t: f64) -> Matrix64 {
        let a2 = a * a;
        Matrix64::identity() + a * t + a2 * (t * t / 2.0) + a2 * a * (t * t * t / 6.0)
    }

    #[test]
    fn the_dynamics_are_nilpotent_and_the_transition_is_their_exponential() {
        let state = tilted();
        let a = a(&state);
        let a4 = a * a * a * a;
        assert!(a4.amax() < 1e-9, "A⁴ is not zero: {}", a4.amax());
        assert!(
            (a * a * a).amax() > 1e-3,
            "A³ is zero, so the cubic term is untested"
        );
        for t in [0.01f32, 1.3, 6.4] {
            let phi = unaccelerated_transition(&state, t, GRAVITY);
            let difference = (widen(&phi) - exp(&a, f64::from(t))).amax();
            assert!(
                difference < 1e-4,
                "{t} s: Φ differs from exp(A T) by {difference}"
            );
        }
    }

    /// `Q_d` against `∫ Φ(s) Q_c Φ(s)ᵀ ds` by Simpson's rule in `f64`, from `A` rather than the
    /// blocks: each of the twenty-one blocks, its sign and its power of `T`.
    #[test]
    fn the_noise_is_the_integral_of_the_densities_along_the_transition() {
        let state = tilted();
        let a = a(&state);
        let q = densities();
        let mut qc = Matrix64::zeros();
        for (first, density) in [
            (ErrorState::VelocityNorth, q.velocity),
            (ErrorState::AttitudeX, q.attitude),
            (ErrorState::AccelBiasX, q.accel_bias),
            (ErrorState::GyroBiasX, q.gyro_bias),
        ] {
            for k in 0..3 {
                qc[(first.index() + k, first.index() + k)] = f64::from(density);
            }
        }
        for t in [0.5f32, 4.0] {
            let n = 400;
            let h = f64::from(t) / f64::from(n);
            let integrand = |s: f64| {
                let phi = exp(&a, s);
                phi * qc * phi.transpose()
            };
            let mut integral = integrand(0.0) + integrand(f64::from(t));
            for k in 1..n {
                let weight = if k % 2 == 1 { 4.0 } else { 2.0 };
                integral += integrand(f64::from(k) * h) * weight;
            }
            integral *= h / 3.0;

            let closed = unaccelerated_growth(&state, &Covariance::zero(), t, GRAVITY, &q, None);
            let closed = closed.as_matrix().map(f64::from);
            for row in 0..STATES {
                for column in 0..STATES {
                    let (got, want) = (closed[(row, column)], integral[(row, column)]);
                    let scale = (integral[(row, row)] * integral[(column, column)]).sqrt();
                    assert!(
                        (got - want).abs() <= 1e-4 * scale + 1e-9,
                        "{t} s, ({row}, {column}): {got} against {want}"
                    );
                }
            }
        }
    }

    /// (22) run in short steps approaches the one step from below: a first-order step
    /// understates, and the shorter the steps the less. The one step is what they converge to,
    /// not a different model of the gap.
    #[test]
    fn short_steps_of_the_first_order_form_approach_it_from_below() {
        let state = tilted();
        let from = init::initial_covariance(
            &Initialization::default(),
            &state.attitude,
            Radians::from_radians(0.02),
            Radians::from_radians(0.35),
            Some(0.0),
            Vector3::repeat(0.01),
            GRAVITY,
        );
        let noise = ImuNoise::default();
        let seconds = 5.0;
        let exact = unaccelerated_growth(
            &state,
            &from,
            seconds,
            GRAVITY,
            &Densities::of(&noise, None),
            None,
        );
        let stepped = |rate: f32| {
            let dt = Seconds::from_secs(1.0 / rate);
            let unaccelerated = unaccelerated_sample(&state, dt, GRAVITY);
            let f = transition_matrix(&state, unaccelerated);
            let q = process_noise(&noise, unaccelerated);
            let mut p = from;
            for _ in 0..(seconds * rate) as usize {
                p = propagate_covariance(p, &f, q);
            }
            p.variance(ErrorState::PositionNorth) / exact.variance(ErrorState::PositionNorth)
        };
        let (coarse, fine) = (stepped(10.0), stepped(400.0));
        assert!(
            coarse < fine && fine <= 1.0001,
            "10 Hz {coarse}, 400 Hz {fine}"
        );
        assert!(fine > 0.995, "400 Hz {fine} of the exact position variance");
    }

    /// The offset's correlation with the error state moves through the same `Φ` as `P` does: one
    /// with down velocity alone reaches down position by `T`, `Φ`'s `I T` block, and keeps its
    /// velocity share, since an unaccelerated vehicle's velocity row is the identity there.
    #[test]
    fn a_coast_carries_the_offsets_correlation_through_the_transition() {
        let mut cross = SVector::<f32, STATES>::zeros();
        cross[ErrorState::VelocityDown.index()] = 1.0;
        let offset = Offset {
            cross,
            variance: 0.25,
        };
        let coasted = coast(
            tilted(),
            Covariance::from_sigmas([0.5; STATES]),
            offset,
            Seconds::from_secs(2.5),
            &Config::default(),
            &Coast::default(),
        );
        let after = coasted.offset.cross;
        assert!(
            (after[ErrorState::PositionDown.index()] - 2.5).abs() < 1e-6,
            "{after}"
        );
        assert_eq!(after[ErrorState::VelocityDown.index()], 1.0);
    }

    /// A projection grows a covariance and never shrinks one, whatever the horizon. The zero and
    /// negative cases are the ones a caller reaches by configuring a horizon nobody checked.
    #[test]
    fn a_horizon_that_is_not_positive_leaves_the_covariance_where_it_was() {
        let state = State::default();
        let from = Covariance::from_sigmas([0.5; STATES]);
        for seconds in [0.0f32, -1.0, f32::NAN] {
            let projected = project(
                &state,
                from,
                Seconds::from_secs(seconds),
                &Config::default(),
            );
            assert_eq!(projected.to_rows(), from.to_rows(), "{seconds} s");
        }
    }
}
