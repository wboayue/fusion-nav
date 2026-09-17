//! The filter itself.

use crate::config::Config;
use crate::frames::{Body, Ned};
use crate::health::{Diagnostics, Fusion, Propagation, Status};
use crate::state::{Covariance, State};
use crate::units::{
    Acceleration, Altitude, AltitudeVariance, AngularRate, HeadingVariance, MagField, Position,
    PositionVariance, Seconds, Velocity, VelocityVariance,
};

/// One IMU measurement, uncorrected. The filter subtracts its own bias estimates.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    /// Angular rate, body frame.
    pub gyro: AngularRate<Body>,
    /// Specific force, body frame. A level, stationary vehicle reads `(0, 0, -g)`.
    pub accel: Acceleration<Body>,
}

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
    /// Without it there is no reference and [`Eskf::fuse_baro_altitude`] refuses with
    /// [`Fusion::NoReference`](crate::Fusion::NoReference). `α₀` is a constant rather
    /// than a state, so it is established here or not at all.
    pub baro: Option<Altitude>,
}

/// Why [`Eskf::initialize`] refused.
///
/// The static interval must be genuine; the filter validates rather than assumes it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InitError {
    /// The window spans less than
    /// [`Initialization::min_duration`](crate::Initialization::min_duration).
    WindowTooShort {
        /// Duration the configuration requires.
        required: Seconds,
        /// Duration the window covers, `window.len() * dt`.
        provided: Seconds,
    },
    /// The window was not stationary: angular rate or specific force moved further than
    /// the configured tolerance allows.
    NotStationary,
}

impl core::fmt::Display for InitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::WindowTooShort { required, provided } => {
                write!(
                    f,
                    "static window too short: {:.2} s of {:.2} s required",
                    provided.as_secs(),
                    required.as_secs()
                )
            }
            Self::NotStationary => write!(f, "static window was not stationary"),
        }
    }
}

impl core::error::Error for InitError {}

/// A 15-state error-state Kalman filter.
///
/// **Stub.** Every method below has its intended signature and does its own bookkeeping,
/// but none of the estimation mathematics is implemented: the state never moves and the
/// covariance stays where [`initialize`](Self::initialize) put it. This type exists to
/// let the API shape be written against before the equations land.
#[derive(Clone, Debug)]
pub struct Eskf {
    config: Config,
    state: State,
    covariance: Covariance,
    diagnostics: Diagnostics,
    baro_reference: Option<Altitude>,
    initialized: bool,
}

impl Eskf {
    /// Create an uninitialized filter. Measurements are refused with
    /// [`Fusion::NotInitialized`] until [`initialize`](Self::initialize) succeeds.
    pub fn new(config: Config) -> Self {
        Self {
            config,
            state: State::default(),
            covariance: Covariance::zero(),
            diagnostics: Diagnostics::default(),
            baro_reference: None,
            initialized: false,
        }
    }

    /// The configuration this filter was built with.
    pub const fn config(&self) -> &Config {
        &self.config
    }

    /// Whether [`initialize`](Self::initialize) has succeeded.
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Initialize attitude, gyroscope bias, covariance, and the barometric reference
    /// from a window of stationary samples. Equations (5)–(8), and `α₀` of (30).
    ///
    /// Position and velocity are zero, and the navigation origin is wherever the vehicle
    /// was during the window.
    ///
    /// `dt` is the interval between consecutive samples in `window`, so that the
    /// configured [`min_duration`](crate::Initialization::min_duration) can be checked
    /// against a real span of time. As everywhere else, the filter never reads a clock.
    ///
    /// **Stub.** Validates the window length, derives the barometric reference, and sets
    /// the filter initialized; computes no attitude.
    pub fn initialize(&mut self, window: &[StaticSample], dt: Seconds) -> Result<(), InitError> {
        let required = self.config.init.min_duration;
        let provided = Seconds::from_secs(window.len() as f32 * dt.as_secs());
        if provided.as_secs() < required.as_secs() {
            return Err(InitError::WindowTooShort { required, provided });
        }

        let init = &self.config.init;
        self.covariance = Covariance::from_sigmas([
            init.sigma_position,
            init.sigma_position,
            init.sigma_position,
            init.sigma_velocity,
            init.sigma_velocity,
            init.sigma_velocity,
            init.sigma_tilt.as_radians(),
            init.sigma_tilt.as_radians(),
            init.sigma_yaw.as_radians(),
            init.sigma_accel_bias,
            init.sigma_accel_bias,
            init.sigma_accel_bias,
            init.sigma_gyro_bias,
            init.sigma_gyro_bias,
            init.sigma_gyro_bias,
        ]);
        self.state = State::default();
        self.diagnostics = Diagnostics::default();
        self.baro_reference = baro_reference(window);
        self.initialized = true;
        Ok(())
    }

    /// Propagate the nominal state and covariance over `dt`. Equations (9)–(22).
    ///
    /// The hot path, called at IMU rate. `dt` is explicit; the filter never reads a clock.
    ///
    /// A `dt` longer than [`Config::max_predict_dt`](crate::Config::max_predict_dt) is
    /// refused: one IMU sample cannot describe a long interval, and propagating it anyway
    /// would put a number in the state that looks like an estimate and is not. The timers
    /// still advance, so [`Status`] degrades on schedule.
    ///
    /// A `dt` that is zero, negative, or NaN is refused before the timers move at all.
    ///
    /// **Stub.** Advances the fusion timers and propagates nothing.
    pub fn predict(&mut self, imu: ImuSample, dt: Seconds) -> Propagation {
        let _ = imu;
        if !self.initialized {
            return Propagation::NotInitialized;
        }
        // Rejected ahead of the bookkeeping: a negative `dt` would wind the timers back
        // and a NaN would poison them. `is_nan` is spelled out because `<= 0.0` alone is
        // false for NaN.
        let seconds = dt.as_secs();
        if seconds <= 0.0 || seconds.is_nan() {
            return Propagation::InvalidStep { dt };
        }

        // Past here the time genuinely passed, so the health bookkeeping is real even
        // when the propagation itself is refused.
        self.diagnostics.advance(dt);

        let limit = self.config.max_predict_dt;
        if seconds > limit.as_secs() {
            return Propagation::StepTooLong { dt, limit };
        }
        Propagation::Propagated
    }

    /// Fuse a GNSS position fix. Equation (28).
    ///
    /// `variance` is the receiver's own accuracy where it reports one — `eph²`
    /// horizontally, `epv²` vertically — but **floor it first**. An accuracy estimate is
    /// the receiver's view of its own geometry and residuals, and under multipath it
    /// stays small while the fix is metres wrong. Neither production autopilot trusts it
    /// raw: PX4 fuses `max(eph, EKF2_GPS_P_NOISE)` and ArduPilot
    /// `constrain(eph, EK3_POSNE_M_NSE, 100 m)`, both from a 0.5 m floor, and PX4
    /// additionally caps it at `EKF2_NOAID_NOISE` (10 m) while GNSS is the only
    /// horizontal aiding source.
    ///
    /// The filter applies no floor of its own, because `R` describes the measurement and
    /// belongs with it rather than in [`Config`]. A caller handing over a raw `eph²` is
    /// therefore trusting the receiver further than either autopilot does.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_gnss_position(
        &mut self,
        position: Position<Ned>,
        variance: PositionVariance<Ned>,
    ) -> Fusion {
        let _ = (position, variance);
        self.stub_fuse(|d| &mut d.gnss_position)
    }

    /// Fuse a GNSS velocity solution. Equation (29).
    ///
    /// `variance` is the receiver's speed accuracy squared, `sacc²`, and wants the same
    /// floor as [`fuse_gnss_position`](Self::fuse_gnss_position): PX4 fuses
    /// `max(sacc, EKF2_GPS_V_NOISE)` and ArduPilot
    /// `constrain(sacc, EK3_VELNE_M_NSE, 50 m/s)`, both from a 0.5 m/s floor.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_gnss_velocity(
        &mut self,
        velocity: Velocity<Ned>,
        variance: VelocityVariance<Ned>,
    ) -> Fusion {
        let _ = (velocity, variance);
        self.stub_fuse(|d| &mut d.gnss_velocity)
    }

    /// Fuse a barometric altitude. Equation (30).
    ///
    /// The measurement is `z = -(α - α₀)`, so it needs the reference
    /// [`initialize`](Self::initialize) derived from the static window. Without one the
    /// measurement is refused rather than referred to an invented origin.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_baro_altitude(&mut self, altitude: Altitude, variance: AltitudeVariance) -> Fusion {
        let _ = (altitude, variance);
        if !self.initialized {
            return Fusion::NotInitialized;
        }
        if self.baro_reference.is_none() {
            return Fusion::NoReference;
        }
        self.stub_fuse(|d| &mut d.baro_altitude)
    }

    /// Fuse magnetic heading from a calibrated three-axis magnetometer.
    /// Equations (34)–(36).
    ///
    /// Heading only: the field is reduced to one scalar, so a magnetic disturbance can
    /// corrupt yaw but cannot reach roll or pitch. `variance` is on the resulting
    /// heading, not on the field components.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_mag_heading(&mut self, field: MagField<Body>, variance: HeadingVariance) -> Fusion {
        let _ = (field, variance);
        self.stub_fuse(|d| &mut d.mag_heading)
    }

    /// The current estimate, including its [`Status`].
    ///
    /// The status is derived here rather than cached: it is a pure function of
    /// [`diagnostics`](Self::diagnostics) and [`Timeouts`](crate::Timeouts), so computing
    /// it on read means there is no invariant for the mutating methods to maintain. The
    /// loop costs four comparisons.
    pub const fn state(&self) -> State {
        // `self.state.status` is inert; the stored estimate never carries a meaningful
        // one, and every read overwrites it.
        let mut state = self.state;
        state.status = self.derive_status();
        state
    }

    /// Per-source health. Off the hot path.
    pub const fn diagnostics(&self) -> Diagnostics {
        self.diagnostics
    }

    /// The 15 x 15 error covariance, in the error-state ordering.
    pub const fn covariance(&self) -> &Covariance {
        &self.covariance
    }

    /// The barometric reference `α₀` fixed at initialization, or `None` if the static
    /// window carried no barometer sample. Equation (30).
    ///
    /// Exposed because it is the one initialization output an application may need to
    /// keep: it is what the filter's zero altitude means, and re-establishing it on the
    /// ground is the documented remedy for reference drift.
    pub const fn baro_reference(&self) -> Option<Altitude> {
        self.baro_reference
    }

    /// Force position to an external fix and reset its covariance block.
    ///
    /// The filter never does this on its own: on sustained rejection it reports
    /// [`Status::DeadReckoning`] and leaves recovery policy to the application, which is
    /// the only layer that knows whether a step input to the controller is acceptable.
    ///
    /// **Stub.** Sets the state; does not touch the covariance.
    pub fn reset_position_to(&mut self, position: Position<Ned>, variance: PositionVariance<Ned>) {
        let _ = variance;
        self.state.position = position;
    }

    /// Force velocity to an external solution and reset its covariance block.
    ///
    /// See [`reset_position_to`](Self::reset_position_to).
    ///
    /// **Stub.** Sets the state; does not touch the covariance.
    pub fn reset_velocity_to(&mut self, velocity: Velocity<Ned>, variance: VelocityVariance<Ned>) {
        let _ = variance;
        self.state.velocity = velocity;
    }

    fn stub_fuse(&mut self, source: fn(&mut Diagnostics) -> &mut crate::SourceHealth) -> Fusion {
        if !self.initialized {
            return Fusion::NotInitialized;
        }
        source(&mut self.diagnostics).record_accepted(0.0);
        Fusion::Accepted { test_ratio: 0.0 }
    }

    /// Aggregate the per-source timers into one status.
    ///
    /// Only sources that have ever been accepted count: a vehicle with no magnetometer is
    /// not permanently `Degraded` for lacking one.
    const fn derive_status(&self) -> Status {
        let degraded_after = self.config.timeouts.degraded_after.as_secs();
        let dead_after = self.config.timeouts.dead_reckoning_after.as_secs();

        let sources = self.diagnostics.as_array();
        let mut used = 0;
        let mut fresh = 0;
        let mut aiding = 0;
        let mut i = 0;
        while i < sources.len() {
            let source = sources[i];
            i += 1;
            let Some(elapsed) = source.time_since_accepted else {
                continue;
            };
            used += 1;
            if elapsed.as_secs() <= degraded_after {
                fresh += 1;
            }
            if elapsed.as_secs() <= dead_after {
                aiding += 1;
            }
        }

        if used == 0 || aiding == 0 {
            Status::DeadReckoning
        } else if fresh == used {
            Status::Healthy
        } else {
            Status::Degraded
        }
    }
}

/// Mean barometric altitude over the samples that carry one. `None` if none do.
///
/// Accumulated in `f64`: a window is up to a few thousand samples and an altitude is
/// metres above mean sea level, so an `f32` running sum of 800 readings near 1000 m has
/// already lost more precision than the reference is worth.
fn baro_reference(window: &[StaticSample]) -> Option<Altitude> {
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
mod tests {
    use super::*;

    const DT: Seconds = Seconds::from_secs(0.01);

    /// A window of exactly `Initialization::min_duration`: 8 samples at 4 Hz is 2 s.
    /// No barometer, so no reference is established.
    fn initialized() -> Eskf {
        let mut filter = Eskf::new(Config::default());
        filter
            .initialize(&[StaticSample::default(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window satisfies the default min_duration");
        filter
    }

    /// The same window with a barometer reading on every sample.
    fn window_with_baro(altitudes: [f32; 8]) -> [StaticSample; 8] {
        altitudes.map(|altitude| StaticSample {
            baro: Some(Altitude::from_meters(altitude)),
            ..StaticSample::default()
        })
    }

    /// Timers only run for a source that has been accepted, so fuse one first. Baro
    /// fusion needs a reference, so the window carries one.
    fn aided() -> Eskf {
        let mut filter = Eskf::new(Config::default());
        filter
            .initialize(&window_with_baro([100.0; 8]), Seconds::from_secs(0.25))
            .expect("a 2 s window satisfies the default min_duration");
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(100.0), AltitudeVariance::from_m2(4.0))
                .is_accepted()
        );
        filter
    }

    fn elapsed(filter: &Eskf) -> f32 {
        filter
            .diagnostics()
            .baro_altitude
            .time_since_accepted
            .expect("baro has been accepted")
            .as_secs()
    }

    #[test]
    fn predict_before_initialize_is_refused() {
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.predict(ImuSample::default(), DT),
            Propagation::NotInitialized
        );
    }

    #[test]
    fn a_normal_step_propagates_and_advances_the_timers() {
        let mut filter = aided();
        assert_eq!(
            filter.predict(ImuSample::default(), DT),
            Propagation::Propagated
        );
        assert_eq!(elapsed(&filter), DT.as_secs());
    }

    #[test]
    fn zero_negative_and_nan_steps_are_refused_without_moving_the_timers() {
        let mut filter = aided();
        for bad in [0.0, -0.01, f32::NAN] {
            let dt = Seconds::from_secs(bad);
            assert!(
                matches!(
                    filter.predict(ImuSample::default(), dt),
                    Propagation::InvalidStep { .. }
                ),
                "dt of {bad} should be refused"
            );
            assert_eq!(elapsed(&filter), 0.0, "dt of {bad} moved the timers");
        }
    }

    #[test]
    fn a_step_over_the_limit_is_refused_but_the_time_still_passes() {
        let mut filter = aided();
        // The worst SD-card dropout in the bundled corpus.
        let dt = Seconds::from_secs(1.304);
        assert!(matches!(
            filter.predict(ImuSample::default(), dt),
            Propagation::StepTooLong { .. }
        ));
        assert_eq!(
            elapsed(&filter),
            1.304,
            "a refused step still happened in real time"
        );
    }

    #[test]
    fn the_baro_reference_is_the_mean_over_the_window() {
        let mut filter = Eskf::new(Config::default());
        let window = window_with_baro([99.0, 101.0, 100.0, 100.0, 99.5, 100.5, 100.0, 100.0]);
        filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("a 2 s window");
        let reference = filter
            .baro_reference()
            .expect("the window carried barometer samples");
        assert!(
            (reference.as_meters() - 100.0).abs() < 1e-4,
            "mean of the window is 100 m, got {}",
            reference.as_meters()
        );
    }

    #[test]
    fn samples_without_a_barometer_stay_out_of_the_mean() {
        let mut filter = Eskf::new(Config::default());
        // A barometer runs slower than the IMU, so most samples in a real window carry
        // nothing. Counting those as zero would drag the reference to the ground.
        let mut window = [StaticSample::default(); 8];
        window[0].baro = Some(Altitude::from_meters(10.0));
        window[7].baro = Some(Altitude::from_meters(20.0));
        filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("a 2 s window");
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(15.0)));
    }

    #[test]
    fn without_a_barometer_in_the_window_fusion_is_refused_not_referred_to_nothing() {
        let mut filter = initialized();
        assert_eq!(filter.baro_reference(), None);
        assert_eq!(
            filter.fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeVariance::from_m2(4.0)),
            Fusion::NoReference
        );
        assert_eq!(
            filter.diagnostics().baro_altitude.time_since_accepted,
            None,
            "a refused measurement is not aiding"
        );
    }

    #[test]
    fn reinitializing_replaces_the_reference() {
        let mut filter = aided();
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));
        filter
            .initialize(&window_with_baro([250.0; 8]), Seconds::from_secs(0.25))
            .expect("a 2 s window");
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(250.0)));
    }

    #[test]
    fn the_static_window_is_measured_in_seconds_not_samples() {
        let mut filter = Eskf::new(Config::default());
        // The same 8 samples, now spanning 0.8 s instead of 2 s.
        let error = filter
            .initialize(&[StaticSample::default(); 8], Seconds::from_secs(0.1))
            .expect_err("0.8 s is under the default min_duration");
        assert!(matches!(error, InitError::WindowTooShort { .. }));
        assert!(!filter.is_initialized());
    }
}
