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
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaticSample {
    /// IMU measurement.
    pub imu: ImuSample,
    /// Magnetometer measurement, if the vehicle has one.
    pub mag: Option<MagField<Body>>,
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

    /// Initialize attitude, gyroscope bias, and covariance from a window of stationary
    /// samples. Equations (5)–(8).
    ///
    /// Position and velocity are zero, and the navigation origin is wherever the vehicle
    /// was during the window.
    ///
    /// `dt` is the interval between consecutive samples in `window`, so that the
    /// configured [`min_duration`](crate::Initialization::min_duration) can be checked
    /// against a real span of time. As everywhere else, the filter never reads a clock.
    ///
    /// **Stub.** Validates the window length and sets the filter initialized; computes no
    /// attitude.
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
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_baro_altitude(&mut self, altitude: Altitude, variance: AltitudeVariance) -> Fusion {
        let _ = (altitude, variance);
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

#[cfg(test)]
mod tests {
    use super::*;

    const DT: Seconds = Seconds::from_secs(0.01);

    /// A window of exactly `Initialization::min_duration`: 8 samples at 4 Hz is 2 s.
    fn initialized() -> Eskf {
        let mut filter = Eskf::new(Config::default());
        filter
            .initialize(&[StaticSample::default(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window satisfies the default min_duration");
        filter
    }

    /// Timers only run for a source that has been accepted, so fuse one first.
    fn aided() -> Eskf {
        let mut filter = initialized();
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(0.0), AltitudeVariance::from_m2(4.0))
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
