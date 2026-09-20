//! Initialization: classifying the window, the initial covariance, and the barometric
//! reference. Equations (5)–(8), and `α₀` of (30).
//!
//! The entry points are methods on [`Eskf`](crate::Eskf) — `initialize`,
//! `initialize_coarse`, `initialize_from`, and `alignment_of` — which call into the pure
//! functions here and then commit the result to the filter.

use crate::config::{GRAVITY, Initialization};
use crate::frames::Body;
use crate::propagate::ImuSample;
use crate::state::{Covariance, State};
use crate::units::{Altitude, MagField, MetersPerSecond2, Radians, RadiansPerSecond, Seconds};

/// One sample from the quasi-static initialization window.
///
/// The magnetometer is optional: without it, heading is unobserved and should start at
/// zero with its variance inflated, leaving the first accepted magnetic heading to correct
/// it. A window with none anywhere in it says so — [`Validity::heading`](crate::Validity)
/// stays false until
/// [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading) accepts one — because
/// [`sigma_yaw`](crate::Initialization::sigma_yaw) is a prior and would otherwise read as
/// an estimate of a quantity nothing measured.
///
/// **Stub.** No heading is computed yet, so a static window starts with
/// [`Initialization::sigma_yaw`](crate::Initialization::sigma_yaw) whether or not it
/// carried a magnetometer; what the magnetometer's presence changes today is the validity
/// flag, not the yaw.
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

impl StaticSample {
    /// Whether every number in the sample is finite, the optional ones included.
    pub(crate) fn is_finite(&self) -> bool {
        self.imu.is_finite()
            && self.mag.is_none_or(|field| field.is_finite())
            && self.baro.is_none_or(|b| b.as_meters().is_finite())
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
        }));
    }
    Ok(Alignment::Static)
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

/// Whether measured peak motion is within the tolerances that make a window a still one.
///
/// Half of what [`classify`] asks, and all of what the barometric reference asks, which is
/// why it is separate: a window can be too short to align an attitude from and still be a
/// window of a vehicle sitting on the ground. `classify` reports the short one as
/// [`Coarse::WindowTooShort`] before it ever measures motion, so window length is not a
/// stand-in for this test in either direction.
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
    use crate::units::{Acceleration, AngularRate};

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
}
