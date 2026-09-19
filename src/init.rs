//! Initialization: classifying the window, the initial covariance, and the barometric
//! reference. Equations (5)–(8), and `α₀` of (30).
//!
//! The entry points are methods on [`Eskf`](crate::Eskf) — `initialize`,
//! `initialize_coarse`, `initialize_from`, and `alignment_of` — which call into the pure
//! functions here and then commit the result to the filter.

use crate::config::{GRAVITY, Initialization};
use crate::eskf::{ImuSample, is_usable_step};
use crate::frames::Body;
use crate::state::{Covariance, State};
use crate::units::{Altitude, MagField, MetersPerSecond2, Radians, RadiansPerSecond, Seconds};

/// One sample from the quasi-static initialization window.
///
/// The magnetometer is optional: without it, heading is unobserved and is initialized to
/// zero with [`Initialization::sigma_yaw`](crate::Initialization::sigma_yaw) inflated,
/// leaving the first accepted magnetic heading to correct it.
///
/// The barometer is optional in the same way, but less forgivingly: its reference is a
/// constant rather than a state, so a window carrying none leaves nothing for a later
/// altitude to be relative to and barometric fusion is refused for the whole flight.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaticSample {
    /// IMU measurement.
    pub imu: ImuSample,
    /// Magnetometer measurement, if the vehicle has one.
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
    /// [`Status::Aligning`](crate::Status::Aligning) until attitude uncertainty comes
    /// down to what a static start would have given.
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
    /// A seed covariance had a negative variance on its diagonal, which no prior has.
    ///
    /// Symmetry and positive-definiteness are not checked: that is a factorization on the
    /// caller's data, not a guard.
    NegativeVariance,
}

impl core::fmt::Display for InitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoSamples => write!(f, "initialization window held no samples"),
            Self::InvalidStep { dt } => {
                write!(f, "initialization dt of {} s is not usable", dt.as_secs())
            }
            Self::NotFinite => write!(f, "initialization input was not finite"),
            Self::NegativeVariance => write!(f, "seed covariance had a negative variance"),
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
    if !is_usable_step(dt) {
        return Err(InitError::InvalidStep { dt });
    }
    if !window.iter().all(sample_is_finite) {
        return Err(InitError::NotFinite);
    }

    let required = init.min_duration;
    let provided = Seconds::from_secs(window.len() as f32 * dt.as_secs());
    if provided.as_secs() < required.as_secs() {
        return Ok(Alignment::Coarse(Coarse::WindowTooShort {
            required,
            provided,
        }));
    }

    let (peak_gyro, peak_accel_deviation) = peak_motion(window);
    if peak_gyro > init.max_gyro_rate || peak_accel_deviation > init.max_accel_deviation {
        return Ok(Alignment::Coarse(Coarse::NotStationary {
            peak_gyro,
            peak_accel_deviation,
            span: provided,
        }));
    }
    Ok(Alignment::Static)
}

/// Initial tilt and yaw standard deviations for an alignment.
///
/// A static start gets the configured figures. A coarse one gets a tilt bound derived
/// from how far the specific force was from gravity — small-angle, so
/// `σ ≈ deviation / g` — and never tighter than the configured value, together with
/// the standard deviation of a heading known only to be somewhere on the circle.
pub(crate) fn attitude_sigmas(init: &Initialization, alignment: Alignment) -> (Radians, Radians) {
    match alignment {
        Alignment::Static | Alignment::Seeded => (init.sigma_tilt, init.sigma_yaw),
        Alignment::Coarse(Coarse::WindowTooShort { .. }) => {
            (init.sigma_tilt, UNKNOWN_HEADING_SIGMA)
        }
        Alignment::Coarse(Coarse::NotStationary {
            peak_gyro,
            peak_accel_deviation,
            span,
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

/// Largest angular rate magnitude, and largest departure of the specific-force magnitude
/// from gravity, over a window.
pub(crate) fn peak_motion(window: &[StaticSample]) -> (RadiansPerSecond, MetersPerSecond2) {
    let mut peak_gyro = 0.0f32;
    let mut peak_deviation = 0.0f32;
    for sample in window {
        let gyro = sample.imu.gyro.as_rad_per_s().norm();
        let deviation = (sample.imu.accel.as_m_per_s2().norm() - GRAVITY).abs();
        peak_gyro = peak_gyro.max(gyro);
        peak_deviation = peak_deviation.max(deviation);
    }
    (
        RadiansPerSecond::from_rad_per_s(peak_gyro),
        MetersPerSecond2::from_m_per_s2(peak_deviation),
    )
}

/// Whether every number in a window sample is finite.
pub(crate) fn sample_is_finite(sample: &StaticSample) -> bool {
    let gyro = sample.imu.gyro.as_rad_per_s();
    let accel = sample.imu.accel.as_m_per_s2();
    gyro.iter().chain(accel.iter()).all(|v| v.is_finite())
        && sample
            .mag
            .is_none_or(|field| field.as_components().iter().all(|v| v.is_finite()))
        && sample.baro.is_none_or(|b| b.as_meters().is_finite())
}

/// Whether every number in a seed state is finite. A quaternion is unit by construction,
/// so only its finiteness is in question here.
pub(crate) fn state_is_finite(state: &State) -> bool {
    let q = state.attitude.quaternion();
    let vectors = [
        state.position.as_meters(),
        state.velocity.as_m_per_s(),
        state.accel_bias.as_m_per_s2(),
        state.gyro_bias.as_rad_per_s(),
    ];
    [q.w, q.i, q.j, q.k].iter().all(|v| v.is_finite())
        && vectors
            .iter()
            .all(|v| v.iter().all(|component| component.is_finite()))
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
