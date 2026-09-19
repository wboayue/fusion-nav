//! The filter itself.

use crate::config::Config;
use crate::frames::{Body, Ned};
use crate::health::{Diagnostics, Fusion, Propagation, SourceHealth, Status, Validity};
use crate::init::{
    self, Alignment, Coarse, InitError, StaticSample, baro_reference, peak_motion,
    sample_is_finite, state_is_finite,
};
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
        let (sigma_tilt, sigma_yaw) = init::attitude_sigmas(&self.config.init, alignment);
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
        init::classify(window, dt, &self.config.init)
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
        let (sigma_tilt, sigma_yaw) = init::attitude_sigmas(&self.config.init, alignment);
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

    /// Reset state, covariance and health for a fresh start with these attitude sigmas.
    /// The barometric reference is the caller's to set, because only it knows whether
    /// this start establishes a new one.
    fn apply_alignment(&mut self, unknown: Unknown, sigma_tilt: f32, sigma_yaw: f32) {
        self.covariance = init::initial_covariance(&self.config.init, sigma_tilt, sigma_yaw);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::GRAVITY;
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
            panic!("0.4 rad/s is over the 0.262 default: {alignment:?}");
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
