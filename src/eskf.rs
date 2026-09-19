//! The filter itself.

use crate::config::{Config, GRAVITY};
use crate::frames::{Body, Ned};
use crate::health::{Diagnostics, Fusion, Propagation, SourceHealth, Status, Validity};
use crate::state::{Covariance, ErrorState, STATES, State};
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

/// What [`Eskf::initialize`] achieved.
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
    /// The state came from [`Eskf::initialize_from`] rather than from a window. Whether
    /// it counts as aligned is a question for the covariance the caller supplied, not for
    /// this value.
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
        /// Largest angular rate magnitude in the window, rad s⁻¹.
        peak_gyro: f32,
        /// Largest departure of the specific-force magnitude from gravity, m s⁻².
        peak_accel_deviation: f32,
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
    /// Quantities a coarse start left unknown, to be adopted from the first fix rather
    /// than fused against a prior the filter does not have.
    unknown: Unknown,
    initialized: bool,
}

/// What initialization could not establish.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Unknown {
    position: bool,
    velocity: bool,
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
            unknown: Unknown::default(),
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

    /// Align from a window of samples taken while the vehicle was, ideally, still.
    /// Equations (5)–(8), and `α₀` of (30).
    ///
    /// The window does not have to be a genuine static interval. If it is — long enough
    /// and within the tolerances [`Initialization`](crate::Initialization) sets — the
    /// result is [`Alignment::Static`] and the filter starts with the covariance that
    /// configuration describes. If it is not, the result is [`Alignment::Coarse`],
    /// carrying what was measured: the filter still runs, with attitude uncertainty
    /// inflated to match, and reports [`Status::Aligning`] until that uncertainty comes
    /// down. Refusing instead would be a launch restriction, and a filter that will not
    /// start is worth less than one that starts and says how much to trust it.
    ///
    /// Where no window exists at all, use [`initialize_coarse`](Self::initialize_coarse)
    /// or [`initialize_from`](Self::initialize_from).
    ///
    /// Position and velocity start at the configured priors either way, which assume the
    /// origin is here and the vehicle is at rest. A launch that knows better — off a
    /// moving deck, say — should say so through [`initialize_from`](Self::initialize_from)
    /// rather than let the first GNSS fix arrive as a large innovation.
    ///
    /// `dt` is the interval between consecutive samples, so that
    /// [`min_duration`](crate::Initialization::min_duration) can be checked against a real
    /// span of time. As everywhere else, the filter never reads a clock.
    ///
    /// **Stub.** Classifies the window, derives the barometric reference and the initial
    /// covariance, and sets the filter initialized; computes no attitude.
    ///
    /// # Errors
    ///
    /// [`InitError::NoSamples`] for an empty window, [`InitError::InvalidStep`] for a
    /// `dt` that is zero, negative or NaN, and [`InitError::NotFinite`] if a sample
    /// carries a value that is not a number.
    pub fn initialize(
        &mut self,
        window: &[StaticSample],
        dt: Seconds,
    ) -> Result<Alignment, InitError> {
        let alignment = self.alignment_of(window, dt)?;
        let (sigma_tilt, sigma_yaw) = self.attitude_sigmas(alignment);
        self.apply_alignment(Unknown::after(alignment), sigma_tilt, sigma_yaw);
        self.baro_reference = baro_reference(window);
        Ok(alignment)
    }

    /// What [`initialize`](Self::initialize) would make of this window, without touching
    /// the filter.
    ///
    /// For the application that would rather wait for stillness than start coarsely:
    /// slide the window forward until this reports [`Alignment::Static`], then commit.
    /// The filter cannot do that waiting itself — it does not know whether the vehicle is
    /// about to launch or has been sitting on the bench for an hour.
    ///
    /// # Errors
    ///
    /// As [`initialize`](Self::initialize).
    pub fn alignment_of(
        &self,
        window: &[StaticSample],
        dt: Seconds,
    ) -> Result<Alignment, InitError> {
        if window.is_empty() {
            return Err(InitError::NoSamples);
        }
        let seconds = dt.as_secs();
        if seconds <= 0.0 || seconds.is_nan() {
            return Err(InitError::InvalidStep { dt });
        }
        if !window.iter().all(sample_is_finite) {
            return Err(InitError::NotFinite);
        }

        let init = &self.config.init;
        let required = init.min_duration;
        let provided = Seconds::from_secs(window.len() as f32 * seconds);
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

    /// Start from a single IMU sample, with no window at all.
    ///
    /// For the launch that never offers one: a hand launch, a deck that is always moving,
    /// a restart at altitude. Tilt comes from one accelerometer reading, which carries the
    /// sensor's full noise and whatever the vehicle's own acceleration was at that
    /// instant, so the covariance is inflated accordingly and the filter reports
    /// [`Status::Aligning`].
    ///
    /// Prefer [`initialize_from`](Self::initialize_from) where the application has an
    /// attitude from somewhere — a companion AHRS, the last flight — since a real estimate
    /// beats one sample of gravity.
    ///
    /// **Stub.** Sets the covariance and initializes; computes no attitude.
    ///
    /// # Errors
    ///
    /// [`InitError::NotFinite`] if the sample carries a value that is not a number.
    pub fn initialize_coarse(&mut self, imu: ImuSample) -> Result<Alignment, InitError> {
        let sample = StaticSample {
            imu,
            ..StaticSample::default()
        };
        if !sample_is_finite(&sample) {
            return Err(InitError::NotFinite);
        }
        let (peak_gyro, peak_accel_deviation) = peak_motion(&[sample]);
        let alignment = Alignment::Coarse(Coarse::NotStationary {
            peak_gyro,
            peak_accel_deviation,
            // One sample is not an average, so no rotation smears it.
            span: Seconds::ZERO,
        });
        let (sigma_tilt, sigma_yaw) = self.attitude_sigmas(alignment);
        self.apply_alignment(Unknown::after(alignment), sigma_tilt, sigma_yaw);
        // The barometric reference is left alone: one sample does not establish one, and
        // a restart at altitude should keep the reference the flight began with.
        Ok(alignment)
    }

    /// Initialize from an estimate the application already holds, rather than from a
    /// static window.
    ///
    /// [`initialize`](Self::initialize) is the better path wherever stillness is
    /// available: it averages sensor noise away, and gyroscope bias is observable at
    /// rest. This one exists for the launches that never offer stillness — a moving deck,
    /// a hand launch, a restart at altitude — and for a warm start from the last flight,
    /// where a saved gyroscope bias is worth far more to a moving vehicle than zero is.
    ///
    /// The caller owns the seed's quality and `covariance` is how it says so: an attitude
    /// from a companion AHRS deserves the uncertainty that AHRS reports, not the
    /// static-window figures in [`Initialization`](crate::Initialization). Seeding a
    /// coarse attitude with a confident covariance is the one way to misuse this, and it
    /// produces a filter that gates out the measurements that would have corrected it.
    ///
    /// `state.status` is ignored: status is derived from aiding, never asserted.
    /// [`baro_reference`](Self::baro_reference) is left alone, so re-initializing in
    /// flight keeps the reference the flight began with; a filter that never had one
    /// needs [`set_baro_reference`](Self::set_baro_reference) before barometric fusion
    /// will be accepted.
    ///
    /// # Errors
    ///
    /// [`InitError::NotFinite`] if `state` or `covariance` carries a value that is not a
    /// number, and [`InitError::NegativeVariance`] for a covariance diagonal no prior
    /// could have. A rejected seed leaves the filter uninitialized rather than poisoned.
    ///
    /// Whether the seed counts as aligned is the covariance's answer, not this one's: a
    /// confident seed reports [`Status::Healthy`] straight away, a coarse one
    /// [`Status::Aligning`] until it converges.
    pub fn initialize_from(
        &mut self,
        state: State,
        covariance: Covariance,
    ) -> Result<Alignment, InitError> {
        if !state_is_finite(&state) || !covariance.as_matrix().iter().all(|e| e.is_finite()) {
            return Err(InitError::NotFinite);
        }
        if (0..STATES).any(|i| covariance.as_matrix()[(i, i)] < 0.0) {
            return Err(InitError::NegativeVariance);
        }
        self.state = state;
        self.covariance = covariance;
        self.diagnostics = Diagnostics::default();
        // A seed carries a position and velocity the caller vouched for, with a
        // covariance that says how far. Nothing here is unknown in the sense that would
        // justify overwriting it with the first fix.
        self.unknown = Unknown::default();
        self.initialized = true;
        Ok(Alignment::Seeded)
    }

    /// Set the barometric reference `α₀` directly. Equation (30).
    ///
    /// Two uses: completing an [`initialize_from`](Self::initialize_from) seed, which
    /// carries no reference of its own, and re-establishing the reference on the ground
    /// when drift in it has become the dominant vertical error. That remedy is the
    /// application's to apply, because the filter cannot tell a drifting reference from a
    /// genuine climb.
    pub const fn set_baro_reference(&mut self, reference: Altitude) {
        self.baro_reference = Some(reference);
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
    /// After a coarse start the first fix is **adopted, not fused** — [`Fusion::Reset`] —
    /// because a vehicle that initialized while moving has no position for a gate to
    /// judge a measurement against. The state becomes the fix and its covariance block
    /// becomes the fix's, which is the exact limit of fusing against an infinitely
    /// uncertain prior. It happens once; the next fix is fused normally.
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
        if !self.initialized {
            return Fusion::NotInitialized;
        }
        if self.unknown.position {
            self.reset_position_to(position, variance);
            self.diagnostics.gnss_position.record_accepted(0.0);
            return Fusion::Reset;
        }
        let _ = (position, variance);
        self.stub_fuse(|d| &mut d.gnss_position)
    }

    /// Fuse a GNSS velocity solution. Equation (29).
    ///
    /// As with [`fuse_gnss_position`](Self::fuse_gnss_position), the first solution after
    /// a coarse start is adopted rather than fused. Velocity is the sharper case: a
    /// static start knows the vehicle is at rest, a coarse start knows nothing, and
    /// claiming the configured 0.1 m s⁻¹ prior for a vehicle doing 18 m s⁻¹ would gate
    /// out the fix that would have corrected it.
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
        if !self.initialized {
            return Fusion::NotInitialized;
        }
        if self.unknown.velocity {
            self.reset_velocity_to(velocity, variance);
            self.diagnostics.gnss_velocity.record_accepted(0.0);
            return Fusion::Reset;
        }
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
    pub fn state(&self) -> State {
        // `self.state.status` is inert; the stored estimate never carries a meaningful
        // one, and every read overwrites it.
        let mut state = self.state;
        state.status = self.derive_status();
        state.validity = self.validity();
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

    /// Whether attitude is good enough to use — the test behind
    /// [`Status::Aligning`], and the same bar as
    /// [`Validity::attitude`](crate::Validity::attitude).
    ///
    /// Read from the covariance against [`Config::accuracy`](crate::Config::accuracy), so
    /// it is the filter's own estimate of its convergence rather than a timer.
    pub fn is_aligned(&self) -> bool {
        self.validity().attitude()
    }

    /// Which parts of the estimate are good enough to use, right now.
    ///
    /// Also carried on [`State::validity`](crate::State::validity), which is where most
    /// callers will meet it.
    pub fn validity(&self) -> Validity {
        if !self.initialized {
            return Validity::NONE;
        }
        let accuracy = &self.config.accuracy;
        let within = |state, sigma: f32| self.covariance.variance(state) <= sigma * sigma;
        let tilt = accuracy.sigma_tilt.as_radians();
        let position = accuracy.sigma_position;
        let velocity = accuracy.sigma_velocity;

        Validity {
            tilt: within(ErrorState::AttitudeX, tilt) && within(ErrorState::AttitudeY, tilt),
            heading: within(ErrorState::AttitudeZ, accuracy.sigma_heading.as_radians()),
            // A quantity a coarse start never established is not valid however tight the
            // prior on it looks: nobody set that number.
            horizontal_position: !self.unknown.position
                && within(ErrorState::PositionNorth, position)
                && within(ErrorState::PositionEast, position),
            vertical_position: !self.unknown.position && within(ErrorState::PositionDown, position),
            horizontal_velocity: !self.unknown.velocity
                && within(ErrorState::VelocityNorth, velocity)
                && within(ErrorState::VelocityEast, velocity),
            vertical_velocity: !self.unknown.velocity && within(ErrorState::VelocityDown, velocity),
        }
    }

    /// Which parts of the estimate the filter expects to be good **if the vehicle left
    /// the ground now** — the arming question, rather than the current one.
    ///
    /// Sitting still, some states are simply unobservable: heading without a
    /// magnetometer, horizontal position before the first fix is fused. Asking
    /// [`validity`](Self::validity) at that moment says no, and says it about a filter
    /// that would in fact be navigating a second after takeoff. A vehicle that refused to
    /// arm on that answer would never arm at all.
    ///
    /// So a quantity counts here if it is already valid, or if a source that constrains
    /// it is currently being accepted — the aiding is there, and using it is a matter of
    /// time. `pred_horiz_pos_rel` in ArduPilot's status word is the same idea; PX4 has no
    /// equivalent.
    ///
    /// Tilt is the exception with no aiding path: nothing but a static window brings it
    /// in today, so it predicts exactly what it is. That changes when in-motion leveling
    /// lands — see `GOALS.md`.
    ///
    /// **Stub.** With no covariance propagation, "expects" cannot mean a projection
    /// forward; it means aiding is arriving. A real implementation should propagate to a
    /// horizon and test that.
    pub fn predicted_validity(&self) -> Validity {
        let now = self.validity();
        if !self.initialized {
            return now;
        }
        let fresh = |source: SourceHealth| {
            source.time_since_accepted.is_some_and(|elapsed| {
                elapsed.as_secs() <= self.config.timeouts.degraded_after.as_secs()
            })
        };
        let d = &self.diagnostics;
        let (position, velocity) = (fresh(d.gnss_position), fresh(d.gnss_velocity));
        let height = position || fresh(d.baro_altitude);

        Validity {
            // Gravity is not an aiding source the filter tracks, so tilt speaks for
            // itself.
            tilt: now.tilt,
            heading: now.heading || fresh(d.mag_heading),
            horizontal_position: now.horizontal_position || position,
            vertical_position: now.vertical_position || height,
            horizontal_velocity: now.horizontal_velocity || velocity,
            vertical_velocity: now.vertical_velocity || velocity,
        }
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
    /// The position becomes the fix, its variances become the fix's, and its
    /// correlations with the rest of the state are dropped — the new error came from the
    /// measurement and has nothing to do with the errors that preceded it.
    ///
    /// The filter does not do this on its own to **recover**: on sustained rejection it
    /// reports [`Status::DeadReckoning`] and leaves the policy to the application, which
    /// is the only layer that knows whether a step input to the controller is acceptable.
    /// The one exception is a quantity that was never established at all — see
    /// [`fuse_gnss_position`](Self::fuse_gnss_position) — where there is no estimate to
    /// step away from.
    pub fn reset_position_to(&mut self, position: Position<Ned>, variance: PositionVariance<Ned>) {
        self.state.position = position;
        self.covariance.reset_block(
            [
                ErrorState::PositionNorth,
                ErrorState::PositionEast,
                ErrorState::PositionDown,
            ],
            variance.as_m2().into(),
        );
        self.unknown.position = false;
    }

    /// Force velocity to an external solution and reset its covariance block.
    ///
    /// See [`reset_position_to`](Self::reset_position_to).
    pub fn reset_velocity_to(&mut self, velocity: Velocity<Ned>, variance: VelocityVariance<Ned>) {
        self.state.velocity = velocity;
        self.covariance.reset_block(
            [
                ErrorState::VelocityNorth,
                ErrorState::VelocityEast,
                ErrorState::VelocityDown,
            ],
            variance.as_m2_per_s2().into(),
        );
        self.unknown.velocity = false;
    }

    /// Initial tilt and yaw standard deviations for an alignment.
    ///
    /// A static start gets the configured figures. A coarse one gets a tilt bound derived
    /// from how far the specific force was from gravity — small-angle, so
    /// `σ ≈ deviation / g` — and never tighter than the configured value, together with
    /// the standard deviation of a heading known only to be somewhere on the circle.
    fn attitude_sigmas(&self, alignment: Alignment) -> (f32, f32) {
        let init = &self.config.init;
        match alignment {
            Alignment::Static | Alignment::Seeded => {
                (init.sigma_tilt.as_radians(), init.sigma_yaw.as_radians())
            }
            Alignment::Coarse(Coarse::WindowTooShort { .. }) => {
                (init.sigma_tilt.as_radians(), UNKNOWN_HEADING_SIGMA)
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
                let from_force = peak_accel_deviation / GRAVITY;
                let from_rotation = peak_gyro * span.as_secs();
                let mut tilt = init.sigma_tilt.as_radians();
                if from_force > tilt {
                    tilt = from_force;
                }
                if from_rotation > tilt {
                    tilt = from_rotation;
                }
                (tilt, UNKNOWN_HEADING_SIGMA)
            }
        }
    }

    /// Reset state, covariance and health for a fresh start with these attitude sigmas.
    /// The barometric reference is the caller's to set, because only it knows whether
    /// this start establishes a new one.
    fn apply_alignment(&mut self, unknown: Unknown, sigma_tilt: f32, sigma_yaw: f32) {
        let init = &self.config.init;
        self.covariance = Covariance::from_sigmas([
            init.sigma_position,
            init.sigma_position,
            init.sigma_position,
            init.sigma_velocity,
            init.sigma_velocity,
            init.sigma_velocity,
            sigma_tilt,
            sigma_tilt,
            sigma_yaw,
            init.sigma_accel_bias,
            init.sigma_accel_bias,
            init.sigma_accel_bias,
            init.sigma_gyro_bias,
            init.sigma_gyro_bias,
            init.sigma_gyro_bias,
        ]);
        self.state = State::default();
        self.diagnostics = Diagnostics::default();
        self.unknown = unknown;
        self.initialized = true;
    }

    fn stub_fuse(&mut self, source: fn(&mut Diagnostics) -> &mut crate::SourceHealth) -> Fusion {
        if !self.initialized {
            return Fusion::NotInitialized;
        }
        source(&mut self.diagnostics).record_accepted(0.0);
        Fusion::Accepted { test_ratio: 0.0 }
    }

    /// Aggregate alignment and the per-source timers into one status, most severe first.
    ///
    /// Only sources that have ever been accepted count toward aiding: a vehicle with no
    /// magnetometer is not permanently `Degraded` for lacking one.
    fn derive_status(&self) -> Status {
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
            // Nothing is arriving that could align the filter either, so this outranks
            // `Aligning`.
            Status::DeadReckoning
        } else if !self.is_aligned() {
            Status::Aligning
        } else if fresh == used {
            Status::Healthy
        } else {
            Status::Degraded
        }
    }
}

impl Unknown {
    /// What an alignment leaves unestablished.
    ///
    /// A static start defines the origin as where the vehicle was and its velocity as
    /// zero, and both are true by construction. A coarse start can say neither: the
    /// vehicle was moving, through somewhere the filter cannot name. Those wait for the
    /// first fix.
    const fn after(alignment: Alignment) -> Self {
        let coarse = matches!(alignment, Alignment::Coarse(..));
        Self {
            position: coarse,
            velocity: coarse,
        }
    }
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
const UNKNOWN_HEADING_SIGMA: f32 = 1.813_799_4;

/// Largest angular rate magnitude, and largest departure of the specific-force magnitude
/// from gravity, over a window.
fn peak_motion(window: &[StaticSample]) -> (f32, f32) {
    let mut peak_gyro = 0.0f32;
    let mut peak_deviation = 0.0f32;
    for sample in window {
        let gyro = sample.imu.gyro.as_rad_per_s().norm();
        if gyro > peak_gyro {
            peak_gyro = gyro;
        }
        let deviation = sample.imu.accel.as_m_per_s2().norm() - GRAVITY;
        let deviation = if deviation < 0.0 {
            -deviation
        } else {
            deviation
        };
        if deviation > peak_deviation {
            peak_deviation = deviation;
        }
    }
    (peak_gyro, peak_deviation)
}

/// Whether every number in a window sample is finite.
fn sample_is_finite(sample: &StaticSample) -> bool {
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
fn state_is_finite(state: &State) -> bool {
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
    use crate::state::ErrorState;

    const DT: Seconds = Seconds::from_secs(0.01);

    /// A sample from a vehicle genuinely sitting still: no rotation, gravity the only
    /// specific force. `StaticSample::default()` is not this — its zero acceleration is
    /// a full `g` away from anything the world does — so the stationarity check reads it
    /// as motion, correctly.
    fn still() -> StaticSample {
        StaticSample {
            imu: ImuSample {
                gyro: AngularRate::from_rad_per_s(0.0, 0.0, 0.0),
                accel: Acceleration::from_m_per_s2(0.0, 0.0, -GRAVITY),
            },
            ..StaticSample::default()
        }
    }

    /// A window of exactly `Initialization::min_duration`: 8 samples at 4 Hz is 2 s.
    /// No barometer, so no reference is established.
    fn initialized() -> Eskf {
        let mut filter = Eskf::new(Config::default());
        let alignment = filter
            .initialize(&[still(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert_eq!(alignment, Alignment::Static);
        filter
    }

    /// The same window with a barometer reading on every sample.
    fn window_with_baro(altitudes: [f32; 8]) -> [StaticSample; 8] {
        altitudes.map(|altitude| StaticSample {
            baro: Some(Altitude::from_meters(altitude)),
            ..still()
        })
    }

    /// Timers only run for a source that has been accepted, so fuse one first. Baro
    /// fusion needs a reference, so the window carries one.
    fn aided() -> Eskf {
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize(&window_with_baro([100.0; 8]), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
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
        let _ = filter
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
        let mut window = [still(); 8];
        window[0].baro = Some(Altitude::from_meters(10.0));
        window[7].baro = Some(Altitude::from_meters(20.0));
        let _ = filter
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
        let _ = filter
            .initialize(&window_with_baro([250.0; 8]), Seconds::from_secs(0.25))
            .expect("a 2 s window");
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(250.0)));
    }

    /// An attitude and biases such as a companion AHRS would hand over, with the
    /// uncertainty that source reports rather than the static-window figures.
    fn seed() -> (State, Covariance) {
        let state = State {
            velocity: Velocity::from_m_per_s(18.0, 0.0, 0.0),
            gyro_bias: AngularRate::from_rad_per_s(0.001, -0.002, 0.0005),
            ..State::default()
        };
        let mut sigmas = [0.5f32; STATES];
        sigmas[ErrorState::AttitudeZ.index()] = 1.0; // a moving start knows yaw poorly
        (state, Covariance::from_sigmas(sigmas))
    }

    #[test]
    fn a_seed_becomes_the_state_and_the_covariance() {
        let mut filter = Eskf::new(Config::default());
        let (state, covariance) = seed();
        let _ = filter
            .initialize_from(state, covariance)
            .expect("a sane seed");
        assert!(filter.is_initialized());
        assert_eq!(filter.state().velocity, state.velocity);
        assert_eq!(filter.state().gyro_bias, state.gyro_bias);
        assert_eq!(filter.covariance(), &covariance);
    }

    #[test]
    fn a_seed_carries_no_baro_reference_of_its_own() {
        let mut filter = Eskf::new(Config::default());
        let (state, covariance) = seed();
        let _ = filter
            .initialize_from(state, covariance)
            .expect("a sane seed");
        assert_eq!(filter.baro_reference(), None);
        assert_eq!(
            filter.fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeVariance::from_m2(4.0)),
            Fusion::NoReference
        );

        filter.set_baro_reference(Altitude::from_meters(52.0));
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeVariance::from_m2(4.0))
                .is_accepted()
        );
    }

    #[test]
    fn reinitializing_in_flight_keeps_the_reference_the_flight_began_with() {
        let mut filter = aided();
        let (state, covariance) = seed();
        let _ = filter
            .initialize_from(state, covariance)
            .expect("a sane seed");
        assert_eq!(
            filter.baro_reference(),
            Some(Altitude::from_meters(100.0)),
            "the barometer did not change when the filter restarted"
        );
    }

    #[test]
    fn a_seed_that_is_not_finite_is_refused_and_nothing_is_initialized() {
        let (state, covariance) = seed();
        let poisoned = State {
            position: Position::from_meters(f32::NAN, 0.0, 0.0),
            ..state
        };
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.initialize_from(poisoned, covariance),
            Err(InitError::NotFinite)
        );
        assert!(!filter.is_initialized(), "a refused seed leaves no state");

        // The same check on the covariance, which is where a stale warm start off
        // storage tends to arrive broken.
        let mut matrix = *covariance.as_matrix();
        matrix[(
            ErrorState::VelocityNorth.index(),
            ErrorState::VelocityNorth.index(),
        )] = -1.0;
        assert_eq!(
            filter.initialize_from(state, Covariance::from_matrix(matrix)),
            Err(InitError::NegativeVariance)
        );
        matrix[(
            ErrorState::VelocityNorth.index(),
            ErrorState::VelocityNorth.index(),
        )] = f32::NAN;
        assert_eq!(
            filter.initialize_from(state, Covariance::from_matrix(matrix)),
            Err(InitError::NotFinite)
        );
        assert!(!filter.is_initialized());
    }

    #[test]
    fn the_static_window_is_measured_in_seconds_not_samples() {
        let mut filter = Eskf::new(Config::default());
        // The same 8 samples, now spanning 0.8 s instead of 2 s.
        let alignment = filter
            .initialize(&[still(); 8], Seconds::from_secs(0.1))
            .expect("a short window is a coarse start, not a refusal");
        assert!(matches!(
            alignment,
            Alignment::Coarse(Coarse::WindowTooShort { .. })
        ));
        assert!(
            filter.is_initialized(),
            "refusing to run is the old behavior"
        );
        assert!(!filter.is_aligned());
    }

    #[test]
    fn a_moving_window_is_coarse_and_reports_what_it_measured() {
        let mut filter = Eskf::new(Config::default());
        let mut window = [still(); 8];
        window[3].imu.gyro = AngularRate::from_rad_per_s(0.0, 0.4, 0.0);
        let alignment = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        let Alignment::Coarse(Coarse::NotStationary { peak_gyro, .. }) = alignment else {
            panic!("0.4 rad/s is over the 0.05 default: {alignment:?}");
        };
        assert!((peak_gyro - 0.4).abs() < 1e-6, "got {peak_gyro}");
        assert!(!filter.is_aligned());
    }

    #[test]
    fn rotation_spoils_tilt_even_when_the_specific_force_reads_a_clean_g() {
        let mut filter = Eskf::new(Config::default());
        let mut window = [still(); 8];
        // Turning about the gravity vector: |a| stays exactly g, and the average of a
        // gravity vector taken while the vehicle turned 0.8 rad is worth that much less.
        window[3].imu.gyro = AngularRate::from_rad_per_s(0.0, 0.0, 0.4);
        let _ = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        let tilt = filter.covariance().variance(ErrorState::AttitudeX);
        assert!(
            (tilt - 0.8 * 0.8).abs() < 1e-4,
            "0.4 rad/s over a 2 s window is 0.8 rad of turn, got sigma^2 {tilt}"
        );
    }

    #[test]
    fn a_coarse_start_widens_tilt_in_proportion_to_the_motion_it_saw() {
        let mut filter = Eskf::new(Config::default());
        let mut window = [still(); 8];
        // 2.94 m/s^2 of unexplained specific force — over the stationarity tolerance,
        // and three tenths of a radian of tilt the filter cannot account for.
        window[0].imu.accel = Acceleration::from_m_per_s2(0.0, 0.0, -GRAVITY - 2.941_995);
        let _ = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        let tilt = filter.covariance().variance(ErrorState::AttitudeX);
        assert!(
            (tilt - 0.09).abs() < 1e-4,
            "tilt variance should be about (0.3 rad)^2, got {tilt}"
        );
    }

    #[test]
    fn with_no_window_at_all_one_sample_still_starts_the_filter() {
        let mut filter = Eskf::new(Config::default());
        let alignment = filter
            .initialize_coarse(still().imu)
            .expect("a finite sample");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(filter.is_initialized());
        assert!(!filter.is_aligned(), "one sample cannot settle heading");
    }

    #[test]
    fn a_coarse_start_reports_aligning_once_something_is_aiding_it() {
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize(&window_with_baro([100.0; 8]), Seconds::from_secs(0.1))
            .expect("short, not unusable");
        assert_eq!(
            filter.state().status,
            Status::DeadReckoning,
            "nothing is arriving that could align it, which outranks Aligning"
        );

        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(100.0), AltitudeVariance::from_m2(4.0))
                .is_accepted()
        );
        assert_eq!(filter.state().status, Status::Aligning);
    }

    /// A filter that started while moving: it knows neither where it is nor how fast.
    fn coarse() -> Eskf {
        let mut filter = Eskf::new(Config::default());
        let mut window = [still(); 8];
        window[3].imu.gyro = AngularRate::from_rad_per_s(0.0, 0.4, 0.0);
        let alignment = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        filter
    }

    #[test]
    fn after_a_coarse_start_the_first_fix_is_adopted_not_fused() {
        let mut filter = coarse();
        let fix = Position::<Ned>::from_meters(120.0, -40.0, -75.0);
        let outcome = filter.fuse_gnss_position(fix, PositionVariance::isotropic(2.25));

        assert_eq!(outcome, Fusion::Reset);
        assert!(outcome.is_accepted(), "the measurement was used");
        assert!(outcome.is_reset(), "and it stepped the state");
        assert_eq!(filter.state().position, fix);
        assert!(
            (filter.covariance().variance(ErrorState::PositionNorth) - 2.25).abs() < 1e-6,
            "the fix's own variance, not the configured prior"
        );

        // Once is once: there is now an estimate for a gate to judge against.
        let outcome = filter.fuse_gnss_position(fix, PositionVariance::isotropic(2.25));
        assert!(!outcome.is_reset());
    }

    #[test]
    fn velocity_is_adopted_too_and_the_two_are_independent() {
        let mut filter = coarse();
        let velocity = Velocity::<Ned>::from_m_per_s(18.0, 1.0, -0.5);
        assert!(
            filter
                .fuse_gnss_velocity(velocity, VelocityVariance::isotropic(0.09))
                .is_reset()
        );
        assert_eq!(filter.state().velocity, velocity);

        // Adopting velocity says nothing about position, which is still unknown.
        assert!(
            filter
                .fuse_gnss_position(
                    Position::<Ned>::from_meters(1.0, 2.0, 3.0),
                    PositionVariance::isotropic(2.25)
                )
                .is_reset()
        );
    }

    #[test]
    fn a_static_start_knows_where_it_is_so_its_first_fix_is_fused() {
        let mut filter = initialized();
        let outcome = filter.fuse_gnss_position(
            Position::<Ned>::from_meters(0.2, -0.1, 0.0),
            PositionVariance::isotropic(2.25),
        );
        assert!(
            !outcome.is_reset(),
            "the origin is where it was, by definition"
        );
        assert!(outcome.is_accepted());
    }

    #[test]
    fn a_reset_drops_the_correlations_the_old_estimate_had() {
        let mut filter = coarse();
        // Give the covariance a correlation to destroy.
        let mut matrix = *filter.covariance().as_matrix();
        let (p_n, v_n) = (
            ErrorState::PositionNorth.index(),
            ErrorState::VelocityNorth.index(),
        );
        matrix[(p_n, v_n)] = 0.5;
        matrix[(v_n, p_n)] = 0.5;
        filter.covariance = Covariance::from_matrix(matrix);

        filter.reset_position_to(
            Position::<Ned>::from_meters(10.0, 0.0, 0.0),
            PositionVariance::isotropic(1.0),
        );

        let after = filter.covariance().as_matrix();
        assert_eq!(after[(p_n, v_n)], 0.0, "the new error came from the fix");
        assert_eq!(after[(v_n, p_n)], 0.0);
        assert_eq!(after[(p_n, p_n)], 1.0);
        assert!(
            after[(v_n, v_n)] > 0.0,
            "resetting position must not disturb velocity"
        );
    }

    #[test]
    fn a_seed_is_trusted_and_is_never_overwritten_by_a_fix() {
        let (state, covariance) = seed();
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize_from(state, covariance)
            .expect("a sane seed");
        let outcome = filter.fuse_gnss_velocity(
            Velocity::<Ned>::from_m_per_s(0.0, 0.0, 0.0),
            VelocityVariance::isotropic(0.09),
        );
        assert!(
            !outcome.is_reset(),
            "the caller vouched for this velocity; its covariance says how far"
        );
        assert_eq!(filter.state().velocity, state.velocity);
    }

    #[test]
    fn a_static_start_is_valid_in_every_part() {
        let filter = initialized();
        let validity = filter.state().validity;
        assert!(validity.all(), "{validity:?}");
        assert!(validity.attitude() && validity.navigation());
    }

    #[test]
    fn an_uninitialized_filter_claims_nothing() {
        let filter = Eskf::new(Config::default());
        assert_eq!(filter.validity(), Validity::NONE);
        assert_eq!(filter.predicted_validity(), Validity::NONE);
    }

    #[test]
    fn a_coarse_start_has_attitude_invalid_and_position_unset() {
        let filter = coarse();
        let validity = filter.validity();
        assert!(!validity.heading, "heading is somewhere on the circle");
        assert!(
            !validity.horizontal_position && !validity.horizontal_velocity,
            "a tight prior on a number nobody set is not validity"
        );
    }

    #[test]
    fn adopting_a_fix_makes_position_valid_without_touching_attitude() {
        let mut filter = coarse();
        let before = filter.validity();
        assert!(
            filter
                .fuse_gnss_position(
                    Position::<Ned>::from_meters(120.0, -40.0, -75.0),
                    PositionVariance::isotropic(2.25),
                )
                .is_reset()
        );

        let after = filter.validity();
        assert!(
            after.horizontal_position && after.vertical_position,
            "position is now as good as the receiver"
        );
        assert_eq!(
            (after.tilt, after.heading),
            (before.tilt, before.heading),
            "a position fix says nothing about attitude"
        );
        assert_eq!(
            filter.state().status,
            Status::Aligning,
            "and the summary still says the worst of it"
        );
    }

    #[test]
    fn takeoff_prediction_counts_aiding_that_has_not_been_used_yet() {
        let mut filter = coarse();
        assert!(!filter.predicted_validity().horizontal_position);

        // A fix arrives and is adopted, so position is valid outright...
        let _ = filter.fuse_gnss_position(
            Position::<Ned>::from_meters(0.0, 0.0, 0.0),
            PositionVariance::isotropic(2.25),
        );
        // ...and the magnetometer is being accepted, so heading will come in even though
        // it is worthless at this instant.
        assert!(
            filter
                .fuse_mag_heading(
                    MagField::<Body>::from_components(0.22, 0.0, 0.44),
                    HeadingVariance::from_rad2(0.05),
                )
                .is_accepted()
        );

        let predicted = filter.predicted_validity();
        assert!(!filter.validity().heading, "not yet");
        assert!(predicted.heading, "but it is arriving");
        assert!(
            !predicted.tilt,
            "tilt has no aiding path until in-motion leveling lands"
        );
    }

    #[test]
    fn a_vehicle_with_only_a_barometer_has_height_and_nothing_horizontal() {
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize(&window_with_baro([100.0; 8]), Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(100.0), AltitudeVariance::from_m2(4.0))
                .is_accepted()
        );

        let predicted = filter.predicted_validity();
        assert!(
            predicted.vertical_position,
            "the barometer constrains height"
        );
        assert!(
            !predicted.horizontal_position,
            "and says nothing about where it is"
        );
    }

    #[test]
    fn a_confident_seed_is_aligned_and_a_vague_one_is_not() {
        let (state, _) = seed();
        let mut filter = Eskf::new(Config::default());

        let _ = filter
            .initialize_from(state, Covariance::from_sigmas([0.001; STATES]))
            .expect("a sane seed");
        assert!(filter.is_aligned());

        let _ = filter
            .initialize_from(state, Covariance::from_sigmas([2.0; STATES]))
            .expect("a sane seed");
        assert!(!filter.is_aligned());
    }

    #[test]
    fn an_empty_window_or_an_unusable_step_is_still_an_error() {
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.initialize(&[], Seconds::from_secs(0.25)),
            Err(InitError::NoSamples)
        );
        assert_eq!(
            filter.initialize(&[still(); 8], Seconds::from_secs(0.0)),
            Err(InitError::InvalidStep {
                dt: Seconds::from_secs(0.0)
            })
        );
        let mut poisoned = [still(); 8];
        poisoned[2].imu.accel = Acceleration::from_m_per_s2(f32::NAN, 0.0, 0.0);
        assert_eq!(
            filter.initialize(&poisoned, Seconds::from_secs(0.25)),
            Err(InitError::NotFinite)
        );
        assert!(!filter.is_initialized());
    }
}
