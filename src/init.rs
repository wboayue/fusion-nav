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
    /// [`Coarse::NotStationary`](Coarse::NotStationary); nothing levels with it yet.
    /// That is equation (5′), and the attitude it needs to rotate `ā_n` into body axes
    /// now exists.
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

/// What one pass over the initialization window measures: the averages equations (5)–(7)
/// level from, the peaks [`at_rest`] judges, and the span [`classify`] measures.
///
/// One value rather than a function per quantity, because the state and its covariance
/// have to describe the *same* average. [`nominal_state`] levels from `force` and
/// [`attitude_sigmas`] bounds how well it levelled; two walks of the window would be two
/// definitions of `f̄`, free to drift apart while each still looked right on its own.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Measured {
    /// `f̄`, the averaged specific force (5) levels from. Every sample carries one.
    pub force: Acceleration<Body>,
    /// `ω̄`, the averaged angular rate (7) takes as the gyroscope bias. Every sample
    /// carries one.
    pub rate: AngularRate<Body>,
    /// `m̄`, the averaged magnetic field (6) takes a heading from, over the samples that
    /// carry one. `None` if none do, which is a vehicle with no magnetometer.
    pub field: Option<MagField<Body>>,
    /// `ā_n` of equation (5′); see [`inertial_acceleration`].
    pub inertial_accel: Option<Acceleration<Ned>>,
    /// Largest angular rate magnitude in the window.
    pub peak_gyro: RadiansPerSecond,
    /// Largest departure of the specific-force magnitude from gravity.
    pub peak_deviation: MetersPerSecond2,
    /// `window.len() * dt`.
    pub span: Seconds,
    /// The same averages over each half of the window, for [`window_drift`]. `None` for
    /// a window of one, which has no halves to disagree.
    pub halves: Option<Halves>,
}

/// The averages of equations (5)–(6) taken over each half of the window separately, so
/// that the attitude the first half yields can be compared with the second half's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Halves {
    /// `f̄` over the first and the second half.
    force: [Acceleration<Body>; 2],
    /// `m̄` over each half, and `None` unless *both* halves carried a field: a heading
    /// the two can be compared on is the whole point, and one half alone yields nothing
    /// to compare.
    field: Option<[MagField<Body>; 2]>,
}

impl Measured {
    /// Measure a window whose samples have already been checked.
    ///
    /// [`measure`] is the entry point that checks them. The one other caller is
    /// [`Eskf::initialize_coarse`](crate::Eskf::initialize_coarse), which holds a window
    /// of one, checks it itself, and spans no time.
    ///
    /// The averages accumulate in `f64`, which here is precaution rather than necessity:
    /// summing a few thousand readings near `γ` in f32 costs on the order of 10⁻⁵ rad of
    /// tilt, against a 0.02 rad prior. It is the choice [`baro_reference`] has to make for
    /// real — altitudes are metres above mean sea level — and this sum is paid once per
    /// flight.
    pub(crate) fn over(window: &[StaticSample], dt: Seconds) -> Self {
        let mut rate = Vector3::<f64>::zeros();
        let mut peak_gyro = 0.0f32;
        let mut peak_deviation = 0.0f32;
        // Accumulated per half, so that the whole-window averages below are sums of two
        // rather than a third running total kept alongside them.
        let mut force = [Vector3::<f64>::zeros(); 2];
        let mut field = [Vector3::<f64>::zeros(); 2];
        let mut samples = [0u32; 2];
        let mut fields = [0u32; 2];
        let split = window.len() / 2;
        for (index, sample) in window.iter().enumerate() {
            let half = usize::from(index >= split);
            let (accel, gyro) = (sample.imu.accel.vector(), sample.imu.gyro.vector());
            force[half] += widen(accel);
            samples[half] += 1;
            rate += widen(gyro);
            if let Some(measurement) = sample.mag {
                field[half] += widen(measurement.vector());
                fields[half] += 1;
            }
            peak_gyro = peak_gyro.max(gyro.norm());
            peak_deviation = peak_deviation.max((accel.norm() - GRAVITY).abs());
        }
        let total = samples[0] + samples[1];
        let carried = fields[0] + fields[1];
        Self {
            force: Acceleration::from_vector(mean(force[0] + force[1], total)),
            rate: AngularRate::from_vector(mean(rate, total)),
            field: (carried > 0).then(|| MagField::from_vector(mean(field[0] + field[1], carried))),
            inertial_accel: inertial_acceleration(window, dt),
            peak_gyro: RadiansPerSecond::from_rad_per_s(peak_gyro),
            peak_deviation: MetersPerSecond2::from_m_per_s2(peak_deviation),
            span: Seconds::from_secs(window.len() as f32 * dt.as_secs()),
            halves: (samples[0] > 0 && samples[1] > 0).then(|| Halves {
                force: [
                    Acceleration::from_vector(mean(force[0], samples[0])),
                    Acceleration::from_vector(mean(force[1], samples[1])),
                ],
                field: (fields[0] > 0 && fields[1] > 0).then(|| {
                    [
                        MagField::from_vector(mean(field[0], fields[0])),
                        MagField::from_vector(mean(field[1], fields[1])),
                    ]
                }),
            }),
        }
    }

    /// The direction equation (5) called down, `−f̄ / ‖f̄‖`: the axis the tilt bound and
    /// the dip are both measured about.
    ///
    /// `None` for an average of zero, which is a stopped or disconnected accelerometer:
    /// that window has no vertical to measure anything about. [`level_from_accel`] levels
    /// it to zero rather than to NaN and [`coarse_sigmas`] keeps the same promise, falling
    /// back to bounds that do not need a direction rather than dividing by the norm.
    fn down(&self) -> Option<Vector3<f32>> {
        self.force
            .vector()
            .try_normalize(f32::MIN_POSITIVE)
            .map(|f| -f)
    }
}

/// Widen a measurement for accumulation; see [`Measured::over`].
fn widen(measurement: Vector3<f32>) -> Vector3<f64> {
    Vector3::new(
        f64::from(measurement.x),
        f64::from(measurement.y),
        f64::from(measurement.z),
    )
}

/// An accumulated sum as its mean, and zero for nothing accumulated — which [`measure`]
/// refuses before any caller sees it.
fn mean(sum: Vector3<f64>, count: u32) -> Vector3<f32> {
    if count == 0 {
        return Vector3::zeros();
    }
    let n = f64::from(count);
    Vector3::new((sum.x / n) as f32, (sum.y / n) as f32, (sum.z / n) as f32)
}

/// Measure a window, refusing the input nothing can be made of.
///
/// Every check initialization makes on raw input is here, so that everything downstream —
/// [`classify`], [`nominal_state`], [`attitude_sigmas`] — takes a [`Measured`] and cannot
/// be handed an empty window, an unusable `dt`, or a value that is not a number.
///
/// # Errors
///
/// [`InitError::NoSamples`], [`InitError::InvalidStep`], [`InitError::NotFinite`].
pub(crate) fn measure(window: &[StaticSample], dt: Seconds) -> Result<Measured, InitError> {
    if window.is_empty() {
        return Err(InitError::NoSamples);
    }
    if !dt.is_usable_step() {
        return Err(InitError::InvalidStep { dt });
    }
    if !window.iter().all(StaticSample::is_finite) {
        return Err(InitError::NotFinite);
    }
    Ok(Measured::over(window, dt))
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
    /// The window was usable but not a static interval, so the covariance says what that
    /// window supports rather than what a still one would. The filter runs and reports
    /// [`Status::Aligning`](crate::Status::Aligning) until tilt and heading uncertainty
    /// first come within [`ALIGNED_TILT`](crate::ALIGNED_TILT) and
    /// [`ALIGNED_HEADING`](crate::ALIGNED_HEADING).
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
    ///
    /// Reported before motion is measured at all, so it says nothing about whether the
    /// vehicle was moving: a short window of a parked vehicle is this variant, and
    /// [`at_rest`] is what separates the two.
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
        /// **Stub.** Reported, not yet subtracted, so it narrows nothing today:
        /// [`attitude_sigmas`] bounds tilt by how far the window's *averaged* specific
        /// force is from gravity, and a real `ā_n` is part of what puts it there.
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

/// Classify a measured window as a static or a coarse start.
///
/// The test behind [`Eskf::alignment_of`](crate::Eskf::alignment_of): long enough, then
/// still enough, against the tolerances in `init`. Total, because [`measure`] has already
/// refused every window that can be refused.
pub(crate) fn classify(measured: &Measured, init: &Initialization) -> Alignment {
    let required = init.min_duration;
    if measured.span < required {
        return Alignment::Coarse(Coarse::WindowTooShort {
            required,
            provided: measured.span,
        });
    }
    if !at_rest(measured, init) {
        return Alignment::Coarse(Coarse::NotStationary {
            peak_gyro: measured.peak_gyro,
            peak_accel_deviation: measured.peak_deviation,
            span: measured.span,
            inertial_accel: measured.inertial_accel,
        });
    }
    Alignment::Static
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
/// Every value read here is finite and the window is non-empty: [`measure`] refuses
/// both before any of this is reached.
pub(crate) fn nominal_state(measured: &Measured, declination: Radians, at_rest: bool) -> State {
    let (roll, pitch) = level_from_accel(measured.force);
    // Stillness observes tilt and never the rotation about it, so a window with no
    // magnetometer anywhere in it keeps ψ₀ = 0 — a stated direction rather than a
    // measured one, which is what `Unestablished::heading` records.
    let yaw = measured.field.map_or(Radians::ZERO, |field| {
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
            measured.rate
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

/// What the window's two halves disagree about: the tilt between the attitudes they
/// yield, and the heading between them. The drift terms of equation (8′).
///
/// (5)–(6) commit one attitude for the whole window, and the filter propagates from the
/// window's **end**. A window whose halves disagree cannot vouch for either, and the
/// disagreement is how far the attitude moved while it was being averaged — measured on
/// the very vectors being averaged, by the very equations that average them.
///
/// For a turn at a steady rate it is the error exactly: the whole-window mean lands
/// mid-window, the state starts half a turn later at the end, and the two half-means sit
/// that same half-turn apart. For a vehicle that swings out and comes back it is the
/// excursion, which is what the net rotation `ω̄ · T` cannot see — that cancels to nothing
/// and charges nothing, while the attitude the window commits is the middle of an arc the
/// vehicle has already left.
///
/// Vibration averages out of each half, which is what keeps this off a vehicle sitting
/// with its props spinning: `2c42096b`'s window disagrees by 0.195° of tilt and 0.544° of
/// heading, both under the configured priors, where integrating `‖ω_⊥‖` over the same
/// window accumulates 5.50° — the peak's mistake in another coat, since a path length
/// does not average either.
///
/// The heading is `None` unless both halves carried a field. Declination cancels in the
/// difference, so none is applied.
fn window_drift(halves: &Halves) -> (f32, Option<f32>) {
    let [first, second] = halves.force;
    let tilt = angle_between(first.vector(), second.vector());
    let heading = halves.field.map(|[first_field, second_field]| {
        let of = |force: Acceleration<Body>, field| {
            let (roll, pitch) = level_from_accel(force);
            heading_from_mag(field, roll, pitch, Radians::ZERO)
        };
        wrap_pi(of(second, second_field).as_radians() - of(first, first_field).as_radians()).abs()
    });
    (tilt, heading)
}

/// Angle between two vectors, and zero where either has no direction — the same answer
/// [`Measured::down`] gives for an average of zero, and for the same reason.
fn angle_between(first: Vector3<f32>, second: Vector3<f32>) -> f32 {
    let norms = first.norm() * second.norm();
    if norms <= 0.0 {
        return 0.0;
    }
    // Clamped because a dot product of parallel vectors overshoots 1 in f32, and `acos`
    // of 1.000001 is NaN.
    ComplexField::acos((first.dot(&second) / norms).clamp(-1.0, 1.0))
}

/// How much a tilt error leaks into the heading of (6): `tan δ` of equation (8′), measured
/// off the window's own field rather than configured.
///
/// (6) levels `m̄` and takes the `atan2` of what is left horizontal, so an error in the
/// tilt it levelled by tips the field and turns that horizontal part. The leak is the
/// ratio of the field's vertical component to its horizontal one, which is `tan(dip)`:
/// 1.96 at the 1.107 rad of dip [`heading_from_mag`] cites, and 1.22 on the window
/// `2c42096b` starts from. Both components are taken about `down`, the direction (5)
/// levelled to, so no Euler angles enter and the ratio does not depend on the frame they
/// would be read in.
///
/// `None` where the horizontal part is zero: a field pointing straight down observes no
/// heading at any tilt, so there is no error to scale rather than an infinite one.
///
/// Shared with [`observation::mag`](crate::observation::mag), which needs the same `tan δ`
/// for the levelling variance of (36′). The ratio is between the field and a direction,
/// so it is the same number in any frame the two are expressed in together: the window
/// passes the body-frame gravity direction, and the update passes the body-frame
/// navigation down axis.
pub(crate) fn heading_sensitivity(field: MagField<Body>, down: Vector3<f32>) -> Option<f32> {
    let field = field.vector();
    let vertical = field.dot(&down);
    let horizontal = (field - down * vertical).norm();
    (horizontal > 0.0).then(|| vertical.abs() / horizontal)
}

/// Initial tilt and yaw standard deviations for an alignment: the attitude block of
/// equation (8).
///
/// A static start gets the configured figures. A coarse one gets what its own window
/// supports, equation (8′); see [`coarse_sigmas`].
///
/// `gyro_bias` is the one [`nominal_state`] committed from the same window, and the
/// coarse bound reads the window's rotation net of it. Passed rather than re-derived so
/// that the two cannot disagree about what the average held.
pub(crate) fn attitude_sigmas(
    init: &Initialization,
    alignment: Alignment,
    measured: &Measured,
    gyro_bias: AngularRate<Body>,
) -> (Radians, Radians) {
    match alignment {
        Alignment::Static | Alignment::Seeded => (init.sigma_tilt, init.sigma_yaw),
        Alignment::Coarse(_) => coarse_sigmas(init, measured, gyro_bias),
    }
}

/// What a window that is not a static interval supports. Equation (8′): the widest of the
/// configured tilt, how far the window's average is from gravity, how far the vehicle
/// turned across gravity, and how far its two halves disagree — with a heading the dip
/// scales that tilt into, plus the turn about gravity and the halves' own disagreement.
///
/// Equations (5)–(6) level the *averaged* specific force, so what bounds the attitude
/// they yield is how far that average is from what a still vehicle reads — not how far
/// the worst sample in the window was. The two differ by two orders of magnitude on
/// `2c42096b`, the corpus's one moving start: its peak `|f| − γ` is 5.46 m/s² (31.9°) on
/// a vehicle vibrating with its props spinning, while the mean vector sits 0.06 m/s²
/// (0.33°) off gravity and levels to the `roll0=0.42 pitch0=-0.89` that `data/manifest.txt`
/// pins, under a degree off plumb. Vibration averages out; a peak does not know that.
///
/// A prior that wide is not free. [`Status::Aligning`](crate::Status::Aligning) outranks
/// `Degraded`, so a start that cannot resolve masks every aiding transition behind it: on
/// that log the peak's 0.557 rad of tilt and full-circle yaw are worth 888 transitions
/// masked over 7127 s, where the averages resolve the start 0.20 s in.
///
/// Two measures of how far the attitude moved during the window, and both are needed
/// because each is blind to what the other catches. The net rotation `ω̄ · T` cancels for
/// a vehicle that swings out and comes back, and charges nothing where the attitude (5)
/// commits is the middle of an arc already left. The disagreement between the window's
/// halves never cancels — but it cannot see a *coordinated* turn, where the specific
/// force stays put in body axes while the vehicle banks, so every average in the window
/// agrees and the tilt is wrong by the bank angle. `moving_start` is that case, and it
/// is measured: on the drift alone the filter reads `nees_att=2.83` and 626 falsely valid
/// quantity-epochs — overconfident, the failure a bound exists to prevent — against 0.50
/// and 108 with both (`data/scenarios.txt`). A bound is not tighter for dropping the term
/// that was holding it up.
///
/// The rotation is `ω̄ − β̂_g`, not `ω̄`, because (7) has already taken part of that average
/// out of the measurement: a window at rest commits the whole of it as
/// [`State::gyro_bias`], so charging it again as motion bounds the attitude by an error
/// the state has just removed. A parked vehicle whose gyroscope reads 0.05 rad/s —
/// ordinary for a MEMS part, and well inside the 0.262 rad/s `max_gyro_rate` — over a
/// 1.6 s window that is coarse only for being short otherwise starts at 0.080 rad of
/// tilt against an [`Accuracy::tilt`](crate::Accuracy::tilt) of 0.052. That start never
/// resolves: the alignment latch only promotes, so
/// [`Status::Aligning`](crate::Status::Aligning) masks every aiding transition for the
/// flight — the same cost the averages above exist to avoid, from a vehicle that never
/// moved.
///
/// [`Coarse`]'s own payload is not read here. It reports why [`classify`] refused the
/// window as static, which is a question about peaks; this is a question about averages,
/// and the same window answers the two differently.
fn coarse_sigmas(
    init: &Initialization,
    measured: &Measured,
    gyro_bias: AngularRate<Body>,
) -> (Radians, Radians) {
    // Small-angle, as (5) reads it: an average that is not gravity leans the levelled
    // vertical by the fraction of `γ` it is out by.
    let from_force = (measured.force.vector().norm() - GRAVITY).abs() / GRAVITY;

    // What the gyroscope says the vehicle did, split about the vertical (5) levelled to:
    // rotation across gravity moves that vector and spoils the tilt, rotation about it
    // leaves the vector alone and spoils the heading of (6) instead.
    let net = (measured.rate.vector() - gyro_bias.vector()) * measured.span.as_secs();
    let down = measured.down();
    let across = down.map_or(net.norm(), |down| (net - down * net.dot(&down)).norm());
    let about = down.map_or(0.0, |down| net.dot(&down).abs());

    // And what the window's own halves say, which is a different question with the same
    // units; see `window_drift`.
    let (tilt_drift, heading_drift) = measured.halves.as_ref().map_or((0.0, None), window_drift);

    let tilt = init
        .sigma_tilt
        .as_radians()
        .max(from_force)
        .max(across)
        .max(tilt_drift);

    // (6) levels the field by that tilt, so the tilt error leaks into the heading scaled
    // by the dip — and the heading has its own drift besides. Both saturate at the circle,
    // since `π/√3` is the widest standard deviation a heading can have and a bound past it
    // claims a spread the quantity does not have. A window no magnetometer observed gets
    // the circle outright, as does one whose field observes no heading to be wrong about.
    let circle = UNKNOWN_HEADING_SIGMA.as_radians();
    let yaw = down
        .zip(measured.field)
        .and_then(|(down, field)| heading_sensitivity(field, down))
        .map_or(circle, |dip| {
            init.sigma_yaw
                .as_radians()
                .max(dip * tilt)
                .max(about)
                .max(heading_drift.unwrap_or(0.0))
                .min(circle)
        });

    (Radians::from_radians(tilt), Radians::from_radians(yaw))
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

/// Whether the window's peak motion is within the tolerances that make it a still one.
///
/// Half of what [`classify`] asks, and all of what the barometric reference and what the
/// start establishes ask, which is why it is separate: a window can be too short to align
/// an attitude from and still be a window of a vehicle sitting on the ground. `classify`
/// reports the short one as [`Coarse::WindowTooShort`] before it ever measures motion, so
/// window length is not a stand-in for this test in either direction.
///
/// GNSS velocity does not enter, however well it explains the specific force. The things
/// that hang on this answer — [`Alignment::Static`], the barometric reference, and
/// whether position and velocity were established at all — all mean *the vehicle was on
/// the ground*, and a deck accelerating under it is not that however precisely the
/// acceleration is known.
pub(crate) fn at_rest(measured: &Measured, init: &Initialization) -> bool {
    measured.peak_gyro <= init.max_gyro_rate && measured.peak_deviation <= init.max_accel_deviation
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
        Ok(classify(&measure(window, dt)?, &Initialization::default()))
    }

    /// The nominal state a window yields at the default 4 Hz.
    fn nominal(window: &[StaticSample], declination: Radians, at_rest: bool) -> State {
        nominal_state(
            &measure(window, DT).expect("a usable window"),
            declination,
            at_rest,
        )
    }

    /// The tilt and yaw sigmas a window would start with, in radians.
    ///
    /// The gyroscope bias comes from (7) on the same window, the way `apply_alignment`
    /// supplies it, so a test sees the pair as the filter commits them.
    fn sigmas(window: &[StaticSample], dt: Seconds) -> (f32, f32) {
        let init = Initialization::default();
        let measured = measure(window, dt).expect("a usable window");
        let state = nominal_state(&measured, Radians::ZERO, at_rest(&measured, &init));
        let (tilt, yaw) = attitude_sigmas(
            &init,
            classify(&measured, &init),
            &measured,
            state.gyro_bias,
        );
        (tilt.as_radians(), yaw.as_radians())
    }

    /// The tilt sigma alone, at the default 4 Hz, which most cases here are about.
    fn coarse_tilt(window: &[StaticSample]) -> f32 {
        sigmas(window, DT).0
    }

    /// `tan` of the dip [`INCLINATION`] writes, which is what a tilt error costs a
    /// heading. No `f32::tan` here: the crate is `no_std`.
    fn dip_gain() -> f32 {
        let (sin, cos) = ComplexField::sin_cos(INCLINATION);
        sin / cos
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

    /// Eight samples of a vehicle turning at a steady rate about one body axis, each
    /// reading the gravity and the field that attitude produces.
    ///
    /// Consistent by construction, which the drift terms of (8′) need: [`window_drift`]
    /// reads how far the attitude moved off the specific force and the field themselves,
    /// so a gyroscope sample set without moving the accelerometer under it describes a
    /// vehicle that is not turning — which is what it is.
    ///
    /// Over eight samples at [`DT`] the half-means sit at 0.375 and 1.375 sample periods,
    /// so a rate of `ω` drifts by exactly `ω · 1 s` between them, whatever the axis.
    pub(crate) fn turning(roll_rate: f32, pitch_rate: f32, yaw_rate: f32) -> [StaticSample; 8] {
        let mut window = [still(); 8];
        for (index, sample) in window.iter_mut().enumerate() {
            let t = index as f32 * DT.as_secs();
            let (roll, pitch, yaw) = (roll_rate * t, pitch_rate * t, yaw_rate * t);
            sample.imu.accel = gravity_at(roll, pitch, yaw);
            // Exact while one axis turns at a time, which is all this builds.
            sample.imu.gyro = AngularRate::body(roll_rate, pitch_rate, yaw_rate);
            sample.mag = Some(field_at(roll, pitch, yaw, 0.0));
        }
        window
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
            let state = nominal(&window, Radians::ZERO, true);
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
                let state = nominal(
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
        let state = nominal(&[still(); 8], Radians::from_radians(0.35), true);
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
        let state = nominal(&window, Radians::ZERO, true);
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
        let state = nominal(&window, Radians::ZERO, false);
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
    fn a_coarse_start_widens_tilt_in_proportion_to_the_motion_it_saw() {
        let mut window = [still(); 8];
        // 2.94 m/s^2 of unexplained specific force on one sample of eight: over the
        // stationarity tolerance, so the window is coarse, and a mean 0.368 m/s^2 out.
        window[0].imu.accel = Acceleration::body(0.0, 0.0, -GRAVITY - 2.941_995);
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.0375).abs() < 1e-4,
            "an eighth of the spike is 0.0375 rad; charging the peak reads 0.3, got {tilt}"
        );
    }

    #[test]
    fn vibration_averages_out_of_the_tilt_where_a_peak_charges_all_of_it() {
        // The shape `2c42096b` has: a vehicle sitting with its props spinning, whose
        // specific force is never gravity and whose average is. Alternating, so every
        // sample is over the 1.961 m/s^2 tolerance and the mean cancels exactly.
        let mut window = [still(); 8];
        for (index, sample) in window.iter_mut().enumerate() {
            let shake = if index % 2 == 0 { 3.0 } else { -3.0 };
            sample.imu.accel = Acceleration::body(0.0, 0.0, -GRAVITY + shake);
        }
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - Initialization::default().sigma_tilt.as_radians()).abs() < 1e-6,
            "nothing above the floor; the peak of 3 m/s^2 would charge 0.306, got {tilt}"
        );
    }

    #[test]
    fn a_turn_about_gravity_charges_the_heading_and_leaves_the_tilt_alone() {
        // Yawing: |a| stays exactly g and the gravity vector does not move in body axes
        // at all, so the tilt its average yields is as good as a still window's. The
        // heading is what moves, and it is charged the 0.8 rad the vehicle turned.
        let (tilt, yaw) = sigmas(&turning(0.0, 0.0, 0.4), DT);
        assert!(
            (tilt - Initialization::default().sigma_tilt.as_radians()).abs() < 1e-6,
            "a turn about gravity is worth no tilt; without the split about `down` this \
             reads 0.8, got {tilt}"
        );
        assert!(
            (yaw - 0.8).abs() < 1e-4,
            "0.4 rad/s over a 2 s window, all of it heading, got {yaw}"
        );
    }

    #[test]
    fn a_coordinated_turn_is_charged_although_its_averages_all_agree() {
        // The case the halves cannot see, and what `moving_start` is: the vehicle banks
        // into a turn, the specific force it measures stays put in body axes, and every
        // average in the window agrees while the tilt is wrong by the bank angle. Only
        // the gyroscope witnesses it.
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        }
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.8).abs() < 1e-4,
            "0.4 rad/s over a 2 s window is 0.8 rad of turn across gravity, and the \
             window's own averages say nothing at all, got {tilt}"
        );
    }

    #[test]
    fn a_window_that_swings_out_and_back_is_charged_the_excursion() {
        // The case the gyroscope cannot see: the vehicle ends where it started, `ω̄ · T`
        // cancels to nothing, and the attitude (5) commits is the middle of an arc it has
        // already left. The halves disagree by 0.4 rad and say so.
        const PITCH: [f32; 8] = [0.0, 0.2, 0.4, 0.2, 0.0, -0.2, -0.4, -0.2];
        // dθ/dt between samples, so the gyroscope agrees with the attitude above and its
        // mean really is zero. The last repeats the previous, having no successor.
        const RATE: [f32; 8] = [0.8, 0.8, -0.8, -0.8, -0.8, -0.8, 0.8, 0.8];
        let mut window = [still(); 8];
        for ((sample, pitch), rate) in window.iter_mut().zip(PITCH).zip(RATE) {
            sample.imu.accel = gravity_at(0.0, pitch, 0.0);
            sample.imu.gyro = AngularRate::body(0.0, rate, 0.0);
        }
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.4).abs() < 1e-4,
            "the halves mean +0.2 and -0.2 rad; on the net rotation alone this reads the \
             0.0297 rad the shortened average is worth, got {tilt}"
        );
    }

    #[test]
    fn a_still_short_window_keeps_the_heading_its_magnetometer_observed() {
        // The window of #85: still, carrying a field, and coarse only because it is
        // short. `classify` calls it short before it measures motion, so nothing about
        // that verdict says the heading is worth less than a 2 s window's.
        let window = window_at(0.0, 0.0, 0.9, 0.0);
        let (tilt, yaw) = sigmas(&window, Seconds::from_secs(0.1));
        let init = Initialization::default();
        assert!(
            (tilt - init.sigma_tilt.as_radians()).abs() < 1e-6,
            "got {tilt}"
        );
        assert!(
            (yaw - init.sigma_yaw.as_radians()).abs() < 1e-6,
            "the configured prior, not the 1.81 rad of a heading nothing observed, got {yaw}"
        );
    }

    #[test]
    fn a_parked_vehicles_gyroscope_bias_is_not_also_charged_as_motion() {
        // (7) commits ω̄ as the gyroscope bias of a window at rest, so the same rotation
        // cannot also bound the attitude — it is not motion the window failed to see, it
        // is a sensor offset the state now carries. 0.05 rad/s is ordinary for a MEMS
        // part and well inside the 0.262 rad/s tolerance; eight samples at 0.2 s is a
        // 1.6 s window, still, and coarse only for being short.
        let mut window = window_at(0.0, 0.0, 0.0, 0.0);
        for sample in &mut window {
            sample.imu.gyro = AngularRate::body(0.05, 0.0, 0.0);
        }
        let dt = Seconds::from_secs(0.2);
        let init = Initialization::default();
        let measured = measure(&window, dt).expect("a usable window");
        assert!(at_rest(&measured, &init), "0.05 rad/s is inside 0.262");
        let bias = nominal_state(&measured, Radians::ZERO, true)
            .gyro_bias
            .vector();
        assert!(
            (bias.x - 0.05).abs() < 1e-6,
            "(7) commits the average, got {bias:?}"
        );

        let (tilt, yaw) = sigmas(&window, dt);
        assert!(
            (tilt - init.sigma_tilt.as_radians()).abs() < 1e-6,
            "charging ω̄ rather than ω̄ − β̂_g reads 0.08 rad here, over the 0.0524 (3°) of \
             `ALIGNED_TILT`, and latches `Aligning` for the flight; got {tilt}"
        );
        assert!(
            (yaw - init.sigma_yaw.as_radians()).abs() < 1e-6,
            "the configured prior; a bias about gravity would charge the heading the \
             same way, got {yaw}"
        );
    }

    #[test]
    fn a_window_no_magnetometer_observed_gets_the_circle_however_still_it_was() {
        // Stillness observes tilt and never the rotation about it.
        let (tilt, yaw) = sigmas(&[still(); 8], Seconds::from_secs(0.1));
        assert!(
            (tilt - Initialization::default().sigma_tilt.as_radians()).abs() < 1e-6,
            "got {tilt}"
        );
        assert!(
            (yaw - UNKNOWN_HEADING_SIGMA.as_radians()).abs() < 1e-6,
            "got {yaw}"
        );
    }

    #[test]
    fn a_window_charges_its_heading_the_tilt_it_levelled_by() {
        // The dip couples them: (6) levels the field by the tilt of (5), so a tilt this
        // window cannot vouch for is a heading error scaled by `tan(dip)`. Steady
        // unexplained specific force rather than a turn, so that the halves agree and the
        // dip is the only thing charging the heading.
        let mut window = window_at(0.0, 0.0, 0.9, 0.0);
        for sample in &mut window {
            let leaning = sample.imu.accel.vector() - Vector3::new(0.0, 0.0, 2.941_995);
            sample.imu.accel = Acceleration::from_vector(leaning);
        }
        let (tilt, yaw) = sigmas(&window, DT);
        assert!((tilt - 0.3).abs() < 1e-4, "2.942 / g, got {tilt}");
        assert!(
            (yaw - dip_gain() * tilt).abs() < 1e-4,
            "tan(dip) is {}, so the yaw is {}, got {yaw}",
            dip_gain(),
            dip_gain() * tilt
        );
    }

    #[test]
    fn a_heading_bound_saturates_at_the_circle_it_cannot_be_wider_than() {
        // 2 rad/s of yaw drifts the halves 2 rad apart, which is more spread than a
        // heading on a circle can have.
        let (_, yaw) = sigmas(&turning(0.0, 0.0, 2.0), DT);
        assert!(
            (yaw - UNKNOWN_HEADING_SIGMA.as_radians()).abs() < 1e-6,
            "pi/sqrt(3) is the widest a heading gets, got {yaw}"
        );
    }

    #[test]
    fn the_heading_sensitivity_is_the_tangent_of_the_dip_at_any_tilt() {
        for (roll, pitch) in TILTS {
            let down = -gravity_at(roll, pitch, 0.4).vector().normalize();
            let measured = heading_sensitivity(field_at(roll, pitch, 0.4, 0.0), down)
                .expect("a field with a horizontal part");
            assert!(
                (measured - dip_gain()).abs() < 1e-4,
                "at ({roll}, {pitch}) got {measured}, want {}",
                dip_gain()
            );
        }
    }

    #[test]
    fn a_field_straight_down_observes_no_heading_at_any_tilt() {
        // `tan(dip)` is unbounded there, and a bound that divided by the horizontal part
        // would put that infinity into `P` through `initial_covariance`.
        let down = Vector3::new(0.0, 0.0, 1.0);
        assert_eq!(
            heading_sensitivity(MagField::body(0.0, 0.0, 0.5), down),
            None
        );
        assert_eq!(
            heading_sensitivity(MagField::body(0.0, 0.0, 0.0), down),
            None
        );
    }

    #[test]
    fn an_accelerometer_averaging_to_zero_is_bounded_and_not_made_a_nan() {
        // Stopped or disconnected: `Measured::down` reports no vertical, so the turn
        // cannot be split about one and the whole of it is charged to tilt.
        //
        // The rate is 0.6 rad/s for a reason. At 1.2 rad it is the widest of the bounds,
        // so that fallback has to carry it: with `normalize()` in place of
        // `try_normalize` this reads the 1.0 rad of `|0 − g| / g` instead, because the
        // NaN that makes is dropped by `f32::max` rather than surfacing. A slower turn
        // passes either way. The magnetometer is there so that the circle the heading
        // falls back to is the missing vertical's doing and not its own.
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu.accel = Acceleration::body(0.0, 0.0, 0.0);
            sample.imu.gyro = AngularRate::body(0.0, 0.6, 0.0);
            sample.mag = Some(MagField::body(0.22, 0.0, 0.44));
        }
        let (tilt, yaw) = sigmas(&window, DT);
        assert!(
            (tilt - 1.2).abs() < 1e-6,
            "0.6 rad/s over 2 s, above the 1.0 rad that |0 - g| / g is worth, got {tilt}"
        );
        assert!(
            (yaw - UNKNOWN_HEADING_SIGMA.as_radians()).abs() < 1e-6,
            "no down means no dip to scale by, got {yaw}"
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
