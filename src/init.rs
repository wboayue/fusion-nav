//! Initialization: the attitude and biases the window yields, classifying the window, the
//! initial covariance, and the barometric reference. Equations (5)–(8), and `α₀` of (30).
//!
//! The entry points are methods on [`Eskf`](crate::Eskf) — `initialize`,
//! `initialize_coarse`, `initialize_from`, and `alignment_of` — which call into the pure
//! functions here and then commit the result to the filter.

use nalgebra::{ComplexField, RealField, Rotation3, UnitQuaternion, Vector3};

use crate::config::{GRAVITY, Initialization};
use crate::frames::{Body, Ned};
use crate::math::wrap_pi;
use crate::propagate::ImuSample;
use crate::state::{Covariance, State};
use crate::units::{
    Acceleration, Altitude, AngularRate, Attitude, MagField, MetersPerSecond2, Position, Radians,
    RadiansPerSecond, Seconds, Velocity,
};

/// One sample from the quasi-static initialization window.
///
/// The magnetometer is optional: without it nothing observes the rotation about gravity,
/// `ψ₀` of equation (6) stays zero, and the first accepted magnetic heading is what
/// establishes it. A window with none anywhere in it says so —
/// [`Validity::heading`](crate::Validity) stays false until
/// [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading) accepts one — because
/// [`sigma_yaw`](crate::Initialization::sigma_yaw) is a prior and would otherwise read as
/// an estimate of a quantity nothing measured.
///
/// The barometer is optional in the same way, but less forgivingly: its reference is a
/// constant rather than a state, so a window carrying none leaves nothing for a later
/// altitude to be relative to and barometric fusion is refused for the whole flight.
///
/// GNSS velocity is the one field a window taken **in motion** can use, and the only
/// reason this type is not simply an IMU sample plus what stillness needs: see
/// [`velocity`](Self::velocity).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaticSample {
    /// IMU measurement.
    pub imu: ImuSample,
    /// Magnetometer measurement, if the vehicle has one.
    ///
    /// Averaged over the window to fix `ψ₀`, the initial heading (equation (6)), and
    /// averaged rather than taken from one sample for the reason the barometer and the
    /// gyroscope bias are: a single reading carries the sensor's full noise.
    pub mag: Option<MagField<Body>>,
    /// Barometric altitude, if the vehicle has a barometer.
    ///
    /// Averaged over the window to fix `α₀`, the barometer's reference at the navigation
    /// origin (equation (30)). Averaged rather than taken from one sample for the same
    /// reason the gyroscope bias is: a single reading carries the sensor's full noise.
    ///
    /// Without it there is no reference and
    /// [`Eskf::fuse_baro_altitude`](crate::Eskf::fuse_baro_altitude) refuses with
    /// [`Fusion::NoReference`](crate::Fusion::NoReference). `α₀` is a constant rather
    /// than a state, so it is established here or not at all.
    pub baro: Option<Altitude>,
    /// GNSS velocity in the navigation frame, if the vehicle has a receiver.
    ///
    /// Differenced across the window for `ā_n`, the vehicle's own acceleration
    /// (equation (5′)). At rest that term is zero and equation (5) reads tilt straight
    /// off the accelerometer; in motion the accelerometer reads specific force, and
    /// `ā_n` is what separates the two. It is the whole of what a moving window has that
    /// a still one does not need.
    ///
    /// Attach it to the epoch it arrived on rather than holding it across the IMU epochs
    /// that follow, as `mag` and `baro` may be held: those are averaged, and an average
    /// survives a repeated value, while a difference is dated by the samples carrying it
    /// and a held reading dates from before the epoch it sits on. Holding costs up to one
    /// GNSS interval of span, in either direction.
    ///
    /// **Stub.** Measured and reported on
    /// [`Coarse::NotStationary`](Coarse::NotStationary); nothing levels with it yet,
    /// which needs the attitude of equations (5)–(7).
    pub velocity: Option<Velocity<Ned>>,
}

impl StaticSample {
    /// Whether every number in the sample is finite, the optional ones included.
    pub(crate) fn is_finite(&self) -> bool {
        self.imu.is_finite()
            && self.mag.is_none_or(|field| field.is_finite())
            && self.baro.is_none_or(|b| b.as_meters().is_finite())
            && self.velocity.is_none_or(|v| v.is_finite())
    }
}

/// What [`Eskf::initialize`](crate::Eskf::initialize) achieved.
///
/// A window that is short or moving is not a failure — it is a coarser start, and the
/// filter says which it got rather than refusing to run. See
/// [`Status::Aligning`](crate::Status::Aligning).
#[must_use = "whether the filter aligned or only started coarsely changes what the estimate is worth"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Alignment {
    /// The window was long enough and genuinely still: tilt from averaged gravity, gyro
    /// bias from the averaged rate, and the covariance
    /// [`Initialization`](crate::Initialization) describes. Equations (5)–(8).
    Static,
    /// The window was usable but not a static interval, so attitude starts coarse and
    /// the covariance is inflated to say so. The filter runs and reports
    /// [`Status::Aligning`](crate::Status::Aligning) until tilt and heading uncertainty
    /// are within [`Config::accuracy`](crate::Config::accuracy).
    Coarse(Coarse),
    /// The state came from [`Eskf::initialize_from`](crate::Eskf::initialize_from) rather
    /// than from a window. Whether it counts as aligned is a question for the covariance
    /// the caller supplied, not for this value.
    Seeded,
}

impl Alignment {
    /// Whether this was a full static alignment.
    pub const fn is_static(self) -> bool {
        matches!(self, Self::Static)
    }
}

/// Why alignment was coarse rather than static.
///
/// Carries what was measured, so "not stationary" is diagnosable rather than a bare
/// verdict: an integrator tuning
/// [`Initialization`](crate::Initialization) needs to know by how much.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Coarse {
    /// The window spans less than
    /// [`Initialization::min_duration`](crate::Initialization::min_duration).
    WindowTooShort {
        /// Duration the configuration requires.
        required: Seconds,
        /// Duration the window covers, `window.len() * dt`.
        provided: Seconds,
    },
    /// The vehicle was moving: angular rate or specific force left the tolerance
    /// [`Initialization`](crate::Initialization) allows.
    NotStationary {
        /// Largest angular rate magnitude in the window.
        peak_gyro: RadiansPerSecond,
        /// Largest departure of the specific-force magnitude from gravity.
        peak_accel_deviation: MetersPerSecond2,
        /// How long the window spanned. With `peak_gyro`, this bounds how far the
        /// vehicle turned while its gravity vector was being averaged, which is the
        /// other way a moving window spoils tilt.
        span: Seconds,
        /// Mean navigation-frame acceleration over the window, `ā_n` of equation (5′),
        /// differenced from [`StaticSample::velocity`]. `None` when no two samples
        /// carried one.
        ///
        /// What separates a vehicle that is accelerating from an accelerometer that is
        /// lying: `peak_accel_deviation` measures specific force that is not gravity,
        /// and this is the part of it GNSS can account for. A launch off a moving deck
        /// reads both, and only one of them spoils tilt.
        ///
        /// **Stub.** Reported, not yet subtracted: the correction of (5′) needs the
        /// attitude of (5)–(7) to rotate `ā_n` into body axes, so today it narrows
        /// nothing — [`attitude_sigmas`] still charges the whole deviation to tilt.
        inertial_accel: Option<Acceleration<Ned>>,
    },
}

/// Why initialization could not run at all.
///
/// Distinct from [`Coarse`]: these are inputs the filter can make nothing of, not starts
/// of lower quality. Every one of them is the caller handing over something broken, which
/// is why they are checked rather than trusted — a seed in particular crosses a boundary
/// the filter does not control, arriving from another estimator or from storage that may
/// be stale or corrupt.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InitError {
    /// The window held no samples, so there is nothing to align from.
    NoSamples,
    /// `dt` was zero, negative, or not a number, so the window covers no measurable span
    /// of time.
    InvalidStep {
        /// The `dt` offered.
        dt: Seconds,
    },
    /// A measurement, state, or covariance carried a value that is not finite.
    NotFinite,
    /// A seed covariance had a variance on its diagonal that no prior has: zero or
    /// negative.
    ///
    /// Zero is the one that arrives in practice, from a warm start deserialized out of
    /// storage that never populated the diagonal. It reads as a tight prior and is not
    /// one: it claims perfect knowledge, so the gain `K = P Hᵀ S⁻¹` of equation (25) is
    /// zero for that quantity and no measurement ever corrects it, while
    /// [`Validity`](crate::Validity) compares the zero variance against
    /// [`Config::accuracy`](crate::Config::accuracy) and reports the quantity good from
    /// the first read. A negative variance claims better than perfect. The bar is the one
    /// every `fuse_*` puts on `R`; see [`Fusion::InvalidNoise`](crate::Fusion::InvalidNoise).
    ///
    /// Symmetry and positive-definiteness are not checked: that is a factorization on the
    /// caller's data, not a guard.
    InvalidVariance,
}

impl core::fmt::Display for InitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoSamples => write!(f, "initialization window held no samples"),
            Self::InvalidStep { dt } => {
                write!(f, "initialization dt of {} s is not usable", dt.as_secs())
            }
            Self::NotFinite => write!(f, "initialization input was not finite"),
            Self::InvalidVariance => {
                write!(f, "seed covariance had a variance that was not positive")
            }
        }
    }
}

impl core::error::Error for InitError {}

/// Classify a window as a static or a coarse start, or refuse it as unusable.
///
/// The test behind [`Eskf::alignment_of`](crate::Eskf::alignment_of): long enough, then
/// still enough, against the tolerances in `init`.
pub(crate) fn classify(
    window: &[StaticSample],
    dt: Seconds,
    init: &Initialization,
) -> Result<Alignment, InitError> {
    if window.is_empty() {
        return Err(InitError::NoSamples);
    }
    if !dt.is_usable_step() {
        return Err(InitError::InvalidStep { dt });
    }
    if !window.iter().all(StaticSample::is_finite) {
        return Err(InitError::NotFinite);
    }

    let required = init.min_duration;
    let provided = Seconds::from_secs(window.len() as f32 * dt.as_secs());
    if provided < required {
        return Ok(Alignment::Coarse(Coarse::WindowTooShort {
            required,
            provided,
        }));
    }

    let (peak_gyro, peak_accel_deviation) = peak_motion(window);
    if !at_rest(peak_gyro, peak_accel_deviation, init) {
        return Ok(Alignment::Coarse(Coarse::NotStationary {
            peak_gyro,
            peak_accel_deviation,
            span: provided,
            inertial_accel: inertial_acceleration(window, dt),
        }));
    }
    Ok(Alignment::Static)
}

/// The nominal state a window yields. Equation (7), from the attitude of (5)–(6).
///
/// `at_rest` is [`at_rest`]'s verdict on the window, and it gates the gyroscope bias
/// alone. (7) takes `β̂_g,0 = ω̄` because stillness is what makes the bias observable; a
/// window that was moving offers the vehicle's own rotation under the same name, and
/// seeding that would subtract a turn rate from every later measurement as though it
/// were a sensor error. Neither production estimator averages at all — ArduPilot zeroes
/// the bias at bootstrap (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:546`, `368dc0c4`),
/// and PX4 refuses to initialize outside 0.8–1.2 g and 15°/s
/// (`src/modules/ekf2/EKF/ekf.cpp:213-227`, `c4e4ef98`) — so averaging is this crate's,
/// and it is worth taking only where their precondition holds.
///
/// Every value read here is finite and the window is non-empty: [`classify`] refuses
/// both before any of this is reached.
pub(crate) fn nominal_state(window: &[StaticSample], declination: Radians, at_rest: bool) -> State {
    let (roll, pitch) = level_from_accel(mean_specific_force(window));
    // Stillness observes tilt and never the rotation about it, so a window with no
    // magnetometer anywhere in it keeps ψ₀ = 0 — a stated direction rather than a
    // measured one, which is what `Unestablished::heading` records.
    let yaw = mean_field(window).map_or(Radians::ZERO, |field| {
        heading_from_mag(field, roll, pitch, declination)
    });

    State {
        // q_ZYX(ψ₀, θ₀, φ₀) of (7): `from_euler_angles` composes Rz(ψ) Ry(θ) Rx(φ), the
        // sequence (6) levels with and `Attitude::euler_angles` reads back.
        attitude: Attitude::body_to_ned(UnitQuaternion::from_euler_angles(
            roll.as_radians(),
            pitch.as_radians(),
            yaw.as_radians(),
        )),
        position: Position::zero(),
        velocity: Velocity::zero(),
        // β̂_a,0 = 0. At rest an accelerometer bias is indistinguishable from a tilt —
        // it leans the measured gravity vector and (5) has already read that lean as
        // attitude — so there is nothing left for this to hold.
        accel_bias: Acceleration::zero(),
        gyro_bias: if at_rest {
            mean_angular_rate(window)
        } else {
            AngularRate::zero()
        },
        // `status` and `validity` are inert in the stored state; `Eskf::state`
        // overwrites both on every read.
        ..State::default()
    }
}

/// Roll and pitch from the averaged specific force. Equation (5).
///
/// The signs are the down-positive convention's: a level, stationary accelerometer reads
/// `f = [0, 0, -γ]ᵀ`, so `-f_z` is `+γ`, `-f_y` is zero, and both angles come out zero.
///
/// Written with `atan2` rather than the `asin` on a normalized vector ArduPilot takes
/// (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:527-536`, `368dc0c4`). The two agree; this
/// form never divides by `‖f‖`, so it needs neither the normalization nor the
/// `length() > 0.001` guard standing in front of it there, and a stopped or
/// disconnected accelerometer reading zero levels to zero rather than to NaN. PX4 builds
/// the same tilt differently again, as the shortest rotation carrying `f` onto `-e₃`
/// (`src/modules/ekf2/EKF/ekf.cpp:224`, `c4e4ef98`), which is this attitude with the yaw
/// left at zero; (6) needs the angles themselves.
pub(crate) fn level_from_accel(specific_force: Acceleration<Body>) -> (Radians, Radians) {
    let f = specific_force.vector();
    let roll = RealField::atan2(-f.y, -f.z);
    let pitch = RealField::atan2(f.x, ComplexField::sqrt(f.y * f.y + f.z * f.z));
    (Radians::from_radians(roll), Radians::from_radians(pitch))
}

/// True heading from the averaged magnetic field, levelled by (5)'s roll and pitch.
/// Equation (6).
///
/// The levelling is what makes this a heading rather than a projection. `R₀` is (7)'s
/// attitude with the yaw left out, so `m̃` sits in a frame differing from NED by the yaw
/// alone. Reading `atan2(m_y, m_x)` off the body field instead is correct only for a
/// vehicle already level and wrong by about `tan(dip)` times the tilt everywhere else:
/// at the 1.107 rad of dip the corpus carries, that factor is 1.96, so a 10° roll is
/// worth 19° of heading. A test at zero tilt cannot tell the two apart.
///
/// `declination` is east-positive, and adding it is what turns magnetic heading into
/// true.
pub(crate) fn heading_from_mag(
    field: MagField<Body>,
    roll: Radians,
    pitch: Radians,
    declination: Radians,
) -> Radians {
    // R₀ = R_y(θ₀) R_x(φ₀), which is the ZYX composition of (7) with its yaw set to zero.
    let levelled =
        Rotation3::from_euler_angles(roll.as_radians(), pitch.as_radians(), 0.0) * field.vector();
    // Wrapped because `D_m − atan2(·)` reaches π + |D_m|, and (7) is read back as Euler
    // angles that a test compares against a heading in range.
    Radians::from_radians(wrap_pi(
        declination.as_radians() - RealField::atan2(levelled.y, levelled.x),
    ))
}

/// `f̄`, the averaged specific force (5) levels from. Every sample carries one.
fn mean_specific_force(window: &[StaticSample]) -> Acceleration<Body> {
    Acceleration::from_vector(mean(window, |sample| Some(sample.imu.accel.vector())))
}

/// `ω̄`, the averaged angular rate (7) takes as the gyroscope bias. Every sample carries
/// one.
fn mean_angular_rate(window: &[StaticSample]) -> AngularRate<Body> {
    AngularRate::from_vector(mean(window, |sample| Some(sample.imu.gyro.vector())))
}

/// `m̄`, the averaged magnetic field (6) takes a heading from, over the samples that
/// carry one. `None` if none do, which is a vehicle with no magnetometer.
fn mean_field(window: &[StaticSample]) -> Option<MagField<Body>> {
    mean_present(window, |sample| sample.mag.map(MagField::vector)).map(MagField::from_vector)
}

/// Mean of a vector every sample carries, and zero for an empty window — which
/// [`classify`] refuses before any caller here sees it.
fn mean(
    window: &[StaticSample],
    select: impl Fn(&StaticSample) -> Option<Vector3<f32>>,
) -> Vector3<f32> {
    mean_present(window, select).unwrap_or_else(Vector3::zeros)
}

/// Mean of a vector quantity over the samples that carry one. `None` if none do.
///
/// Accumulated in `f64`, which here is precaution rather than necessity: summing a few
/// thousand readings near `γ` in f32 costs on the order of 10⁻⁵ rad of tilt, against a
/// 0.02 rad prior. It is the choice [`baro_reference`] has to make for real — altitudes
/// are metres above mean sea level — and this sum is paid once per flight.
fn mean_present(
    window: &[StaticSample],
    select: impl Fn(&StaticSample) -> Option<Vector3<f32>>,
) -> Option<Vector3<f32>> {
    let mut sum = Vector3::<f64>::zeros();
    let mut count = 0u32;
    for sample in window {
        if let Some(value) = select(sample) {
            sum += Vector3::new(f64::from(value.x), f64::from(value.y), f64::from(value.z));
            count += 1;
        }
    }
    (count > 0).then(|| {
        let n = f64::from(count);
        Vector3::new((sum.x / n) as f32, (sum.y / n) as f32, (sum.z / n) as f32)
    })
}

/// Initial tilt and yaw standard deviations for an alignment.
///
/// A static start gets the configured figures. A coarse one gets the standard deviation
/// of a heading known only to be somewhere on the circle, and a tilt bound that is the
/// worst of three: the configured value, specific force away from gravity
/// (small-angle, `deviation / g`), and rotation during the window (`ω · span`).
pub(crate) fn attitude_sigmas(init: &Initialization, alignment: Alignment) -> (Radians, Radians) {
    match alignment {
        Alignment::Static | Alignment::Seeded => (init.sigma_tilt, init.sigma_yaw),
        Alignment::Coarse(Coarse::WindowTooShort { .. }) => {
            (init.sigma_tilt, UNKNOWN_HEADING_SIGMA)
        }
        // `inertial_accel` is measured but not spent: charging the whole deviation to
        // tilt is right until (5′) actually subtracts it, since the tilt error a
        // correction would remove is still in the answer.
        Alignment::Coarse(Coarse::NotStationary {
            peak_gyro,
            peak_accel_deviation,
            span,
            inertial_accel: _,
        }) => {
            // Two ways a moving window spoils tilt, and the worse one governs.
            // Specific force that is not gravity tilts the answer directly,
            // small-angle, by `deviation / g`. Rotation spoils it instead by turning
            // the vehicle while its gravity vector is being averaged, by at most
            // `ω · span`. A window can suffer either without the other: a vehicle
            // rotating about its own gravity vector reads a clean `g`.
            let from_force = peak_accel_deviation.as_m_per_s2() / GRAVITY;
            let from_rotation = peak_gyro.as_rad_per_s() * span.as_secs();
            let tilt = init
                .sigma_tilt
                .as_radians()
                .max(from_force)
                .max(from_rotation);
            (Radians::from_radians(tilt), UNKNOWN_HEADING_SIGMA)
        }
    }
}

/// The diagonal initial covariance `P₀`. Equation (8).
///
/// Position, velocity and bias sigmas come from `init`; the attitude sigmas from
/// [`attitude_sigmas`], because they depend on how good the alignment was.
pub(crate) fn initial_covariance(
    init: &Initialization,
    sigma_tilt: Radians,
    sigma_yaw: Radians,
) -> Covariance {
    let position = init.sigma_position.as_meters();
    let velocity = init.sigma_velocity.as_m_per_s();
    let (tilt, yaw) = (sigma_tilt.as_radians(), sigma_yaw.as_radians());
    let accel_bias = init.sigma_accel_bias.as_m_per_s2();
    let gyro_bias = init.sigma_gyro_bias.as_rad_per_s();
    // In the `ErrorState` ordering: `[δp δv δθ δβa δβg]`.
    #[rustfmt::skip]
    let sigmas = [
        position,   position,   position,
        velocity,   velocity,   velocity,
        tilt,       tilt,       yaw,
        accel_bias, accel_bias, accel_bias,
        gyro_bias,  gyro_bias,  gyro_bias,
    ];
    Covariance::from_sigmas(sigmas)
}

/// Standard deviation of a heading known only to lie somewhere on the circle: `π / √3`,
/// the standard deviation of a uniform distribution over `[-π, π]`.
///
/// Honest, and at the same time a number the error state cannot really carry — the
/// three-component attitude error of equation (2) is a small-angle quantity, and a yaw
/// error of a radian is not small. It stands in until a heading source arrives, and the
/// right response to that source is a yaw **reset** rather than a gradual correction.
/// This is why PX4 and ArduPilot align yaw with a bank of hypotheses rather than one wide
/// prior; see `GOALS.md`.
const UNKNOWN_HEADING_SIGMA: Radians = Radians::from_radians(1.813_799_4);

/// Whether measured peak motion is within the tolerances that make a window a still one.
///
/// Half of what [`classify`] asks, and all of what the barometric reference asks, which is
/// why it is separate: a window can be too short to align an attitude from and still be a
/// window of a vehicle sitting on the ground. `classify` reports the short one as
/// [`Coarse::WindowTooShort`] before it ever measures motion, so window length is not a
/// stand-in for this test in either direction.
///
/// GNSS velocity does not enter, however well it explains the specific force. The two
/// things that hang on this answer — [`Alignment::Static`] and the barometric reference —
/// both mean *the vehicle was on the ground*, and a deck accelerating under it is not
/// that however precisely the acceleration is known.
pub(crate) fn at_rest(
    peak_gyro: RadiansPerSecond,
    peak_accel_deviation: MetersPerSecond2,
    init: &Initialization,
) -> bool {
    peak_gyro <= init.max_gyro_rate && peak_accel_deviation <= init.max_accel_deviation
}

/// Largest angular rate magnitude, and largest departure of the specific-force magnitude
/// from gravity, over a window. The two measures [`at_rest`] judges.
pub(crate) fn peak_motion(window: &[StaticSample]) -> (RadiansPerSecond, MetersPerSecond2) {
    let mut peak_gyro = 0.0f32;
    let mut peak_deviation = 0.0f32;
    for sample in window {
        let gyro = sample.imu.gyro.vector().norm();
        let deviation = (sample.imu.accel.vector().norm() - GRAVITY).abs();
        peak_gyro = peak_gyro.max(gyro);
        peak_deviation = peak_deviation.max(deviation);
    }
    (
        RadiansPerSecond::from_rad_per_s(peak_gyro),
        MetersPerSecond2::from_m_per_s2(peak_deviation),
    )
}

/// Mean navigation-frame acceleration over the window: `ā_n` of equation (5′), the term
/// in-motion levelling subtracts from the averaged specific force. `None` unless two
/// samples separated in time carry a velocity.
///
/// Endpoints only; the velocities in between are not differenced at all. The mean of a
/// derivative *is* its endpoint difference over the span, and the mean is what is wanted,
/// because (5) levels the averaged specific force and the term to subtract is therefore
/// the averaged acceleration. Differencing consecutive samples and averaging those gives
/// the same number with the intermediate noise added back — which matters here, since
/// this is the noisiest part of in-motion levelling: a receiver's velocity error divided
/// by a span, and a 1 Hz receiver over a 2 s window divides it by very little.
///
/// The span is counted in samples, so it is only as honest as the dating of the window;
/// see [`StaticSample::velocity`].
pub(crate) fn inertial_acceleration(
    window: &[StaticSample],
    dt: Seconds,
) -> Option<Acceleration<Ned>> {
    let mut first: Option<(usize, Velocity<Ned>)> = None;
    let mut last: Option<(usize, Velocity<Ned>)> = None;
    for (index, sample) in window.iter().enumerate() {
        if let Some(velocity) = sample.velocity {
            first.get_or_insert((index, velocity));
            last = Some((index, velocity));
        }
    }
    let ((first_index, first), (last_index, last)) = (first?, last?);
    let span = (last_index - first_index) as f32 * dt.as_secs();
    // One velocity, or several on the same sample, spans no time. A difference over zero
    // seconds is an infinity, not an acceleration, and this is the only division here.
    (span > 0.0).then(|| Acceleration::from_vector((last.vector() - first.vector()) / span))
}

/// Whether every number in a seed state is finite. A quaternion is unit by construction,
/// so only its finiteness is in question here.
pub(crate) fn state_is_finite(state: &State) -> bool {
    let q = state.attitude.quaternion();
    [q.w, q.i, q.j, q.k].iter().all(|v| v.is_finite())
        && state.position.is_finite()
        && state.velocity.is_finite()
        && state.accel_bias.is_finite()
        && state.gyro_bias.is_finite()
}

/// Mean barometric altitude over the samples that carry one: `α₀` of equation (30).
/// `None` if none do.
///
/// Accumulated in `f64`: a window is up to a few thousand samples and an altitude is
/// metres above mean sea level, so an `f32` running sum of 800 readings near 1000 m has
/// already lost more precision than the reference is worth.
pub(crate) fn baro_reference(window: &[StaticSample]) -> Option<Altitude> {
    let mut sum = 0.0f64;
    let mut count = 0u32;
    for sample in window {
        if let Some(altitude) = sample.baro {
            sum += f64::from(altitude.as_meters());
            count += 1;
        }
    }
    (count > 0).then(|| Altitude::from_meters((sum / f64::from(count)) as f32))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A sample from a vehicle genuinely sitting still: no rotation, gravity the only
    /// specific force. `StaticSample::default()` is not this — its zero acceleration is
    /// a full `g` away from anything the world does — so the stationarity check reads it
    /// as motion, correctly.
    pub(crate) fn still() -> StaticSample {
        StaticSample {
            imu: ImuSample {
                gyro: AngularRate::body(0.0, 0.0, 0.0),
                accel: Acceleration::body(0.0, 0.0, -GRAVITY),
            },
            ..StaticSample::default()
        }
    }

    /// 8 samples at 4 Hz: exactly the default 2 s `min_duration`.
    const DT: Seconds = Seconds::from_secs(0.25);

    fn classify_default(window: &[StaticSample], dt: Seconds) -> Result<Alignment, InitError> {
        classify(window, dt, &Initialization::default())
    }

    /// The tilt sigma a window would start with, in radians.
    fn coarse_tilt(window: &[StaticSample]) -> f32 {
        let init = Initialization::default();
        let alignment = classify(window, DT, &init).expect("moving, not unusable");
        attitude_sigmas(&init, alignment).0.as_radians()
    }

    /// Magnetic inclination, rad, down-positive: the mid-latitude dip the corpus carries
    /// and `examples/simulate.rs` writes its field with.
    const INCLINATION: f32 = 1.107;

    /// Both signs on each axis, and the two together. A down-positive convention inverts
    /// one sign at a time, so a case list that leans one way passes with the sign flipped.
    const TILTS: [(f32, f32); 6] = [
        (0.0, 0.0),
        (0.3, 0.0),
        (-0.3, 0.0),
        (0.0, 0.2),
        (0.0, -0.2),
        (0.4, -0.25),
    ];

    /// `R = Rz(ψ) Ry(θ) Rx(φ)`, the attitude of equation (7).
    fn attitude_of(roll: f32, pitch: f32, yaw: f32) -> Rotation3<f32> {
        Rotation3::from_euler_angles(roll, pitch, yaw)
    }

    /// What a vehicle held at this attitude and sitting still reads on its
    /// accelerometer: equation (11) with `a_n = 0`, so `f = Rᵀ(−g)`.
    pub(crate) fn gravity_at(roll: f32, pitch: f32, yaw: f32) -> Acceleration<Body> {
        let f = attitude_of(roll, pitch, yaw).inverse() * Vector3::new(0.0, 0.0, -GRAVITY);
        Acceleration::from_vector(f)
    }

    /// What it reads on its magnetometer, for a site whose field dips by [`INCLINATION`]
    /// and points `declination` east of true north.
    fn field_at(roll: f32, pitch: f32, yaw: f32, declination: f32) -> MagField<Body> {
        let (sin_dip, cos_dip) = ComplexField::sin_cos(INCLINATION);
        let north = attitude_of(0.0, 0.0, declination) * Vector3::new(cos_dip, 0.0, sin_dip);
        MagField::from_vector(attitude_of(roll, pitch, yaw).inverse() * north)
    }

    /// A still window of `[still(); 8]` reading this specific force and this field.
    fn window_at(roll: f32, pitch: f32, yaw: f32, declination: f32) -> [StaticSample; 8] {
        [StaticSample {
            imu: ImuSample {
                accel: gravity_at(roll, pitch, yaw),
                ..still().imu
            },
            mag: Some(field_at(roll, pitch, yaw, declination)),
            ..still()
        }; 8]
    }

    /// The roll, pitch and yaw equation (7) committed, in radians.
    fn committed_angles(state: &State) -> (f32, f32, f32) {
        state.attitude.euler_angles()
    }

    #[test]
    fn levelling_recovers_the_tilt_gravity_was_rotated_by() {
        for (roll, pitch) in TILTS {
            // A yaw the answer must not depend on: gravity says nothing about rotation
            // about itself, so `Rz(ψ)ᵀ` leaves the specific force where it was.
            let (measured_roll, measured_pitch) = level_from_accel(gravity_at(roll, pitch, 0.9));
            assert!(
                (measured_roll.as_radians() - roll).abs() < 1e-5
                    && (measured_pitch.as_radians() - pitch).abs() < 1e-5,
                "({roll}, {pitch}) read back as ({:?}, {:?})",
                measured_roll,
                measured_pitch
            );
        }
    }

    #[test]
    fn the_committed_attitude_carries_the_specific_force_back_onto_gravity() {
        // (5) and (7) round-tripped: the attitude the filter commits must rotate the
        // vector it was derived from back onto `−g`, or the two disagree about which
        // sequence they mean.
        for (roll, pitch) in TILTS {
            let window = window_at(roll, pitch, 0.0, 0.0);
            let state = nominal_state(&window, Radians::ZERO, true);
            let navigation = state.attitude.quaternion() * window[0].imu.accel.vector();
            assert!(
                (navigation - Vector3::new(0.0, 0.0, -GRAVITY)).norm() < 1e-4,
                "({roll}, {pitch}) rotated back to {navigation:?}"
            );
        }
    }

    #[test]
    fn heading_survives_the_tilt_it_is_levelled_by() {
        // The test that fails if the levelling is dropped: a field synthesised for a
        // known yaw at a tilt that is not zero must still give that yaw back.
        const DECLINATION: f32 = -0.06;
        for (roll, pitch) in TILTS {
            for yaw in [0.0, 0.9, -2.5, 3.0] {
                let state = nominal_state(
                    &window_at(roll, pitch, yaw, DECLINATION),
                    Radians::from_radians(DECLINATION),
                    true,
                );
                let (_, _, committed) = committed_angles(&state);
                assert!(
                    wrap_pi(committed - yaw).abs() < 1e-4,
                    "yaw {yaw} at ({roll}, {pitch}) read back as {committed}"
                );
            }
        }
    }

    #[test]
    fn an_unlevelled_heading_is_wrong_by_the_dip() {
        // What the levelling is worth, and why a zero-tilt test proves nothing: at 10°
        // of roll the raw body field gives a heading 19° from the true one, because the
        // dip leans into the horizontal axes. `tan(1.107)` is 1.96.
        let roll = 10.0f32.to_radians();
        let field = field_at(roll, 0.0, 0.0, 0.0).vector();
        let unlevelled = RealField::atan2(field.y, field.x);
        assert!(
            (unlevelled.to_degrees().abs() - 19.1).abs() < 0.1,
            "expected about 19 deg of error, got {}",
            unlevelled.to_degrees()
        );

        let levelled = heading_from_mag(
            MagField::from_vector(field),
            Radians::from_radians(roll),
            Radians::ZERO,
            Radians::ZERO,
        );
        assert!(levelled.as_radians().abs() < 1e-6, "{levelled:?}");
    }

    #[test]
    fn declination_moves_the_heading_by_exactly_itself() {
        // The same field, read at two sites: true heading differs from magnetic by the
        // declination and by nothing else.
        let (roll, pitch) = (0.2, -0.1);
        let field = field_at(roll, pitch, 0.7, 0.0);
        let heading = |declination: f32| {
            heading_from_mag(
                field,
                Radians::from_radians(roll),
                Radians::from_radians(pitch),
                Radians::from_radians(declination),
            )
            .as_radians()
        };
        assert!((heading(0.35) - heading(0.0) - 0.35).abs() < 1e-6);
        assert!((heading(-0.06) - heading(0.0) + 0.06).abs() < 1e-6);
    }

    #[test]
    fn a_window_with_no_magnetometer_keeps_a_heading_of_zero() {
        // Stillness observes tilt and never the rotation about it. Zero is a stated
        // direction rather than a measured one, which `Unestablished::heading` records.
        let state = nominal_state(&[still(); 8], Radians::from_radians(0.35), true);
        assert_eq!(committed_angles(&state).2, 0.0);
    }

    #[test]
    fn a_still_window_takes_its_gyroscope_bias_from_the_average() {
        // A gyroscope reading a constant offset while the vehicle does not turn is
        // reading its own bias, and at rest that is the one place it is observable.
        let offset = AngularRate::body(0.01, -0.02, 0.003);
        let window = [StaticSample {
            imu: ImuSample {
                gyro: offset,
                ..still().imu
            },
            ..still()
        }; 8];
        let state = nominal_state(&window, Radians::ZERO, true);
        assert!(
            (state.gyro_bias.vector() - offset.vector()).norm() < 1e-7,
            "{:?}",
            state.gyro_bias
        );
    }

    #[test]
    fn a_window_taken_in_motion_takes_no_gyroscope_bias_at_all() {
        // The average is the vehicle turning, not the sensor lying, and seeding it would
        // subtract a turn rate from every later measurement as a sensor error.
        let window = [StaticSample {
            imu: ImuSample {
                gyro: AngularRate::body(0.0, 0.4, 0.0),
                ..still().imu
            },
            ..still()
        }; 8];
        let state = nominal_state(&window, Radians::ZERO, false);
        assert_eq!(state.gyro_bias, AngularRate::zero());
    }

    #[test]
    fn a_still_window_of_min_duration_is_static() {
        assert_eq!(classify_default(&[still(); 8], DT), Ok(Alignment::Static));
    }

    #[test]
    fn the_static_window_is_measured_in_seconds_not_samples() {
        // The same 8 samples, now spanning 0.8 s instead of 2 s.
        let alignment = classify_default(&[still(); 8], Seconds::from_secs(0.1));
        assert!(matches!(
            alignment,
            Ok(Alignment::Coarse(Coarse::WindowTooShort { .. }))
        ));
    }

    #[test]
    fn a_moving_window_is_coarse_and_reports_what_it_measured() {
        let mut window = [still(); 8];
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        let alignment = classify_default(&window, DT).expect("moving, not unusable");
        let Alignment::Coarse(Coarse::NotStationary { peak_gyro, .. }) = alignment else {
            panic!("0.4 rad/s is over the 0.262 default: {alignment:?}");
        };
        let peak_gyro = peak_gyro.as_rad_per_s();
        assert!((peak_gyro - 0.4).abs() < 1e-6, "got {peak_gyro}");
    }

    #[test]
    fn rotation_spoils_tilt_even_when_the_specific_force_reads_a_clean_g() {
        let mut window = [still(); 8];
        // Turning about the gravity vector: |a| stays exactly g, and the average of a
        // gravity vector taken while the vehicle turned 0.8 rad is worth that much less.
        window[3].imu.gyro = AngularRate::body(0.0, 0.0, 0.4);
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.8).abs() < 1e-4,
            "0.4 rad/s over a 2 s window is 0.8 rad of turn, got {tilt}"
        );
    }

    #[test]
    fn a_coarse_start_widens_tilt_in_proportion_to_the_motion_it_saw() {
        let mut window = [still(); 8];
        // 2.94 m/s^2 of unexplained specific force — over the stationarity tolerance,
        // and three tenths of a radian of tilt the filter cannot account for.
        window[0].imu.accel = Acceleration::body(0.0, 0.0, -GRAVITY - 2.941_995);
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.3).abs() < 1e-4,
            "tilt should be about 0.3 rad, got {tilt}"
        );
    }

    #[test]
    fn an_empty_window_or_an_unusable_step_is_still_an_error() {
        assert_eq!(classify_default(&[], DT), Err(InitError::NoSamples));
        let zero = Seconds::from_secs(0.0);
        assert_eq!(
            classify_default(&[still(); 8], zero),
            Err(InitError::InvalidStep { dt: zero })
        );
        let mut poisoned = [still(); 8];
        poisoned[2].imu.accel = Acceleration::body(f32::NAN, 0.0, 0.0);
        assert_eq!(classify_default(&poisoned, DT), Err(InitError::NotFinite));
    }

    #[test]
    fn the_baro_reference_is_the_mean_over_the_window() {
        let window =
            [99.0, 101.0, 100.0, 100.0, 99.5, 100.5, 100.0, 100.0].map(|altitude| StaticSample {
                baro: Some(Altitude::from_meters(altitude)),
                ..still()
            });
        let reference = baro_reference(&window).expect("the window carried barometer samples");
        assert!(
            (reference.as_meters() - 100.0).abs() < 1e-4,
            "mean of the window is 100 m, got {}",
            reference.as_meters()
        );
    }

    #[test]
    fn samples_without_a_barometer_stay_out_of_the_mean() {
        // A barometer runs slower than the IMU, so most samples in a real window carry
        // nothing. Counting those as zero would drag the reference to the ground.
        let mut window = [still(); 8];
        window[0].baro = Some(Altitude::from_meters(10.0));
        window[7].baro = Some(Altitude::from_meters(20.0));
        assert_eq!(baro_reference(&window), Some(Altitude::from_meters(15.0)));
    }

    /// A window whose GNSS reports the vehicle gaining 4 m/s of north velocity over the
    /// four samples — one second at `DT` — that separate the two fixes.
    fn accelerating_window() -> [StaticSample; 8] {
        let mut window = [still(); 8];
        window[1].velocity = Some(Velocity::ned(1.0, 0.0, 0.0));
        window[5].velocity = Some(Velocity::ned(5.0, 0.0, 0.0));
        window
    }

    #[test]
    fn the_window_acceleration_is_the_endpoint_difference_over_the_span() {
        let measured = inertial_acceleration(&accelerating_window(), DT)
            .expect("two samples a second apart carry a velocity");
        assert_eq!(measured, Acceleration::ned(4.0, 0.0, 0.0));
    }

    #[test]
    fn velocity_between_the_endpoints_does_not_reach_the_mean() {
        // The mean of a derivative is its endpoint difference, so a noisy fix in the
        // middle of the window is not averaged in — it is not read at all.
        let mut window = accelerating_window();
        window[3].velocity = Some(Velocity::ned(-40.0, 12.0, 7.0));
        assert_eq!(
            inertial_acceleration(&window, DT),
            Some(Acceleration::ned(4.0, 0.0, 0.0))
        );
    }

    #[test]
    fn a_window_with_no_two_dated_velocities_reports_no_acceleration() {
        // Nothing to difference, and a difference over zero seconds is an infinity
        // rather than an acceleration.
        assert_eq!(inertial_acceleration(&[still(); 8], DT), None);
        let mut one = [still(); 8];
        one[4].velocity = Some(Velocity::ned(9.0, 0.0, 0.0));
        assert_eq!(inertial_acceleration(&one, DT), None);
    }

    #[test]
    fn a_moving_window_reports_the_acceleration_gnss_accounts_for() {
        let mut window = accelerating_window();
        // Over the 0.262 rad/s default, so the window classifies as moving.
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        let alignment = classify_default(&window, DT).expect("moving, not unusable");
        let Alignment::Coarse(Coarse::NotStationary { inertial_accel, .. }) = alignment else {
            panic!("0.4 rad/s is over the default: {alignment:?}");
        };
        assert_eq!(inertial_accel, Some(Acceleration::ned(4.0, 0.0, 0.0)));
    }

    #[test]
    fn a_vehicle_gnss_explains_entirely_is_still_not_at_rest() {
        // A launch off a deck accelerating north at a steady 8 m/s^2, level and not
        // turning: |f| is sqrt(8^2 + g^2), 2.85 m/s^2 off gravity, and GNSS accounts for
        // every bit of it. The window is still not one taken on the ground, which is
        // what `Alignment::Static` and the barometric reference both mean.
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu.accel = Acceleration::body(8.0, 0.0, -GRAVITY);
        }
        window[1].velocity = Some(Velocity::ned(1.0, 0.0, 0.0));
        window[5].velocity = Some(Velocity::ned(9.0, 0.0, 0.0));

        let alignment = classify_default(&window, DT).expect("moving, not unusable");
        let Alignment::Coarse(Coarse::NotStationary { inertial_accel, .. }) = alignment else {
            panic!("2.85 m/s^2 is over the 1.961 default: {alignment:?}");
        };
        assert_eq!(inertial_accel, Some(Acceleration::ned(8.0, 0.0, 0.0)));
    }

    #[test]
    fn a_velocity_that_is_not_a_number_is_refused_with_the_rest() {
        let mut window = accelerating_window();
        window[5].velocity = Some(Velocity::ned(f32::NAN, 0.0, 0.0));
        assert_eq!(classify_default(&window, DT), Err(InitError::NotFinite));
    }
}
