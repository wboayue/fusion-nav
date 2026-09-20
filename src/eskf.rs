//! The filter itself.

use crate::config::Config;
use crate::frames::{Body, Ned};
use crate::geodetic::{Geodetic, LocalOrigin};
use crate::health::{Diagnostics, Fusion, Propagation, SourceHealth, Status, Validity};
use crate::init::{
    self, Alignment, Coarse, InitError, StaticSample, baro_reference, peak_motion,
    sample_is_finite, state_is_finite,
};
use crate::propagate::ImuSample;
use crate::state::{Covariance, ErrorState, STATES, State};
use crate::units::{
    Altitude, AltitudeNoise, HeadingNoise, MagField, Position, PositionNoise, Seconds, Velocity,
    VelocityNoise,
};

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
    origin: Option<LocalOrigin>,
    unestablished: Unestablished,
    initialized: bool,
}

/// Quantities the start never established, which wait for the first measurement that
/// observes them: position and velocity for the first GNSS fix after a coarse start (see
/// [`Fusion::Reset`]), heading for the first magnetic heading.
///
/// The covariance cannot carry this on its own. Every entry on its diagonal is a prior,
/// and a prior tight enough to pass [`Config::accuracy`](crate::Config::accuracy) reads as
/// an estimate whether or not anything ever measured the quantity. This is the flag that
/// tells the two apart.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Unestablished {
    position: bool,
    velocity: bool,
    heading: bool,
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
            origin: None,
            unestablished: Unestablished::default(),
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
    /// `α₀`, the barometric reference of equation (30), is taken from the window whenever
    /// the window was taken **at rest**, which is not the same test as the alignment: a
    /// window too short to align an attitude from is still a window of a vehicle sitting on
    /// the ground, and the altitude it read is what zero will mean. A window that was
    /// moving keeps the reference the flight already has, as
    /// [`initialize_coarse`](Self::initialize_coarse) does — a restart at 100 m must not
    /// call its own altitude the ground.
    ///
    /// Position and velocity start at the configured priors either way, which assume the
    /// origin is here and the vehicle is at rest. A launch that knows better — off a
    /// moving deck, say — should say so through [`initialize_from`](Self::initialize_from)
    /// rather than let the first GNSS fix arrive as a large innovation.
    ///
    /// Heading is the one thing stillness cannot supply. Gravity pins roll and pitch;
    /// nothing pins the rotation about it, so a window carrying no magnetometer leaves
    /// yaw unobserved however long and however still it was, and
    /// [`validity`](Self::validity) withholds `heading` until the first
    /// [`fuse_mag_heading`](Self::fuse_mag_heading) is accepted. The covariance alone
    /// cannot say that: [`sigma_yaw`](crate::Initialization::sigma_yaw) is a prior on a
    /// number nobody measured.
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
        self.apply_alignment(alignment);
        // Measured from the window rather than read off `alignment`: the test is whether
        // the vehicle was at rest, which a window too short to align from can still pass.
        let (peak_gyro, peak_accel_deviation) = peak_motion(window);
        if init::at_rest(peak_gyro, peak_accel_deviation, &self.config.init) {
            self.baro_reference = baro_reference(window);
        }
        // Stillness observes tilt and gyroscope bias; it does not observe yaw. A window
        // with no magnetometer anywhere in it leaves heading a prior rather than an
        // estimate, and says so, whatever the alignment was.
        if !window.iter().any(|sample| sample.mag.is_some()) {
            self.unestablished.heading = true;
        }
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
        // Treated as a window of one, so the same finiteness and motion measures apply.
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
        self.apply_alignment(alignment);
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
    /// will be accepted. [`origin`](Self::origin) is left alone for the same reason, and
    /// `state.position` is taken as relative to it — or, with no origin yet, to the one
    /// the first [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic) will place around it.
    ///
    /// # Errors
    ///
    /// [`InitError::NotFinite`] if `state` or `covariance` carries a value that is not a
    /// number, and [`InitError::InvalidVariance`] for a variance on the diagonal that is
    /// not strictly positive — the bar every `fuse_*` puts on `R`, applied here because a
    /// seed is the one path that writes a covariance in whole. A rejected seed leaves the
    /// filter uninitialized rather than poisoned.
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
        if (0..STATES).any(|i| covariance.as_matrix()[(i, i)] <= 0.0) {
            return Err(InitError::InvalidVariance);
        }
        self.state = state;
        self.covariance = covariance;
        self.diagnostics = Diagnostics::default();
        // Nothing a seed carries is unestablished: the caller vouched for every quantity,
        // heading included, so no first measurement overwrites one.
        self.unestablished = Unestablished::default();
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
    ///
    /// Returns `false`, changing nothing, for a reference that is not a number: `α₀`
    /// appears in every barometric measurement for the rest of the flight, so a NaN here
    /// is not one bad update but the end of barometric aiding.
    #[must_use = "a refused reference leaves barometric fusion returning NoReference"]
    pub const fn set_baro_reference(&mut self, reference: Altitude) -> bool {
        if !reference.as_meters().is_finite() {
            return false;
        }
        self.baro_reference = Some(reference);
        true
    }

    /// Put the navigation origin at a known point, such as a surveyed home or a landing
    /// pad. Equation (43).
    ///
    /// Without this, the first [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic) places
    /// the origin, which is right for most flights. Setting it is for an application whose
    /// positions mean something relative to a fixed point.
    ///
    /// The vehicle does not move. With an origin already held, the position estimate is
    /// re-expressed about the new one, so the geodetic position is unchanged and only the
    /// numbers describing it are; its covariance is untouched, because a change of origin
    /// adds no uncertainty. With none held, the current position is taken to be relative
    /// to this one, which is the caller's claim to make — true after a static start on
    /// the point being named, false anywhere else.
    ///
    /// Call it after initializing: a static start clears the origin, since it declares
    /// position zero to be wherever the vehicle is.
    ///
    /// Returns `false`, changing nothing, for an origin with a coordinate that is not a
    /// number or a latitude beyond ±90°.
    #[must_use = "a refused origin leaves the filter to place its own on the first fix"]
    pub fn set_origin(&mut self, origin: Geodetic) -> bool {
        let Some(new) = LocalOrigin::new(origin) else {
            return false;
        };
        if let Some(old) = self.origin {
            self.state.position = new.to_ned(old.to_geodetic(self.state.position));
        }
        self.origin = Some(new);
        true
    }

    /// The navigation origin: the point [`State::position`](crate::State::position) is
    /// relative to, and the tangent plane the filter converts geodetic fixes in. `None`
    /// until the first [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic) or
    /// [`set_origin`](Self::set_origin).
    ///
    /// Also the conversion an application wants for anything else it holds in latitude
    /// and longitude — a waypoint, a geofence — so that it lands in the same frame as the
    /// estimate.
    pub const fn origin(&self) -> Option<LocalOrigin> {
        self.origin
    }

    /// The position estimate as latitude, longitude and height, once the filter is
    /// initialized and has an [`origin`](Self::origin) to place it with.
    pub fn geodetic_position(&self) -> Option<Geodetic> {
        if !self.initialized {
            return None;
        }
        self.origin
            .map(|origin| origin.to_geodetic(self.state.position))
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
    /// A sample carrying a NaN or an infinity is refused too, as
    /// [`Propagation::NotFinite`]: propagating it would put the NaN in the quaternion and
    /// then in the covariance, where nothing reports it and it never leaves. The timers
    /// advance, since the `dt` was fine and only the sample was not.
    ///
    /// **Stub.** Advances the fusion timers and propagates nothing.
    pub fn predict(&mut self, imu: ImuSample, dt: Seconds) -> Propagation {
        if !self.initialized {
            return Propagation::NotInitialized;
        }
        // Rejected ahead of the bookkeeping: a negative `dt` would wind the timers back
        // and a NaN would poison them.
        if !dt.is_usable_step() {
            let outcome = Propagation::InvalidStep { dt };
            self.diagnostics.propagation.record(outcome);
            return outcome;
        }

        // Past here the time genuinely passed, so the health bookkeeping is real even
        // when the propagation itself is refused.
        self.diagnostics.advance(dt);

        let limit = self.config.max_predict_dt;
        if dt > limit {
            let outcome = Propagation::StepTooLong { dt, limit };
            self.diagnostics.propagation.record(outcome);
            return outcome;
        }

        // Tested after the gap rather than before it, so that the gap is still measured:
        // `longest_refused` is the only record of how far the interval ran, and it
        // describes the timing whatever the sample holds. A sensor producing NaN produces
        // it again on the next step, where the count picks it up.
        if !imu.is_finite() {
            let outcome = Propagation::NotFinite;
            self.diagnostics.propagation.record(outcome);
            return outcome;
        }
        Propagation::Propagated
    }

    /// Fuse a position fix already expressed in NED meters about the filter's origin.
    /// Equation (28).
    ///
    /// For a receiver reporting latitude and longitude, use
    /// [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic), which converts about the origin
    /// the filter holds. This one is for a caller that owns the conversion, or whose
    /// positions were never geodetic — a local RTK base, motion capture. Mixing the two is
    /// only right if the caller's origin is [`origin`](Self::origin).
    ///
    /// After a coarse start the first fix is adopted rather than fused; see
    /// [`Fusion::Reset`].
    ///
    /// `noise` is the receiver's own accuracy where it reports one —
    /// [`PositionNoise::horizontal_vertical`](crate::PositionNoise::horizontal_vertical)
    /// takes `eph` and `epv` as reported — but **floor it first**. An accuracy estimate is
    /// the receiver's view of its own geometry and residuals, and under multipath it
    /// stays small while the fix is metres wrong. Neither production autopilot trusts it
    /// raw: PX4 fuses `max(eph, EKF2_GPS_P_NOISE)` and ArduPilot
    /// `constrain(eph, EK3_POSNE_M_NSE, 100 m)`, both from a 0.5 m floor, and PX4
    /// additionally caps it at `EKF2_NOAID_NOISE` (10 m) while GNSS is the only
    /// horizontal aiding source.
    ///
    /// The filter applies no floor of its own, because `R` describes the measurement and
    /// belongs with it rather than in [`Config`]. A caller handing over a raw `eph` is
    /// therefore trusting the receiver further than either autopilot does.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_gnss_position(
        &mut self,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
    ) -> Fusion {
        if !self.initialized {
            return refuse(&mut self.diagnostics.gnss_position, Fusion::NotInitialized);
        }
        if !position.is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.gnss_position, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.gnss_position, Fusion::InvalidNoise);
        }
        if self.unestablished.position {
            let adopted = self.reset_position_to(position, noise);
            debug_assert!(adopted, "the fix cleared the same checks just above");
            self.diagnostics.gnss_position.record_adopted();
            return Fusion::Reset;
        }
        let _ = (position, noise);
        stub_accept(&mut self.diagnostics.gnss_position)
    }

    /// Fuse a GNSS fix given as latitude, longitude and height. Equations (43), (44),
    /// then (28).
    ///
    /// The filter holds the navigation origin and converts about it, so the fix and the
    /// estimate are relative to the same point by construction. The first fix places the
    /// origin, unless [`set_origin`](Self::set_origin) already has:
    ///
    /// * With a position estimate — a static start, a seed — the origin goes where that
    ///   estimate says the vehicle started, so the fix lands on the estimate and nothing
    ///   steps. Equation (44). The fix is not fused: it was spent placing the origin, and
    ///   fusing it as well would count it twice. What it does settle is the position
    ///   uncertainty. About the new origin the position error *is* the fix's error, so
    ///   the position covariance block becomes `noise` and its correlations are dropped,
    ///   as [`reset_position_to`](Self::reset_position_to) does, with the value unchanged.
    ///   Reported as accepted with a zero test ratio: nothing was inconsistent.
    /// * Without one — after a coarse start — the origin goes at the fix, and the fix is
    ///   adopted as position zero; see [`Fusion::Reset`].
    ///
    /// Check the receiver's fix type before calling. Many report latitude and longitude
    /// zero until they have a fix, and a finite zero is a usable origin: the first one
    /// would put the navigation frame in the Gulf of Guinea for the rest of the flight.
    ///
    /// `noise` is as for [`fuse_gnss_position`](Self::fuse_gnss_position), floor
    /// included.
    ///
    /// A fix or noise that is not a number is refused with [`Fusion::NotFinite`]. A fix
    /// with a latitude beyond ±90° cannot place an origin, nor can one near a pole that no
    /// origin puts at the estimate (see [`LocalOrigin::placing`]): with none held it is
    /// refused with [`Fusion::NoReference`], and the next usable fix places it instead.
    pub fn fuse_gnss_geodetic(&mut self, fix: Geodetic, noise: PositionNoise<Ned>) -> Fusion {
        if !self.initialized {
            return refuse(&mut self.diagnostics.gnss_position, Fusion::NotInitialized);
        }
        if !fix.is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.gnss_position, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.gnss_position, Fusion::InvalidNoise);
        }
        if let Some(origin) = self.origin {
            return self.fuse_gnss_position(origin.to_ned(fix), noise);
        }

        if self.unestablished.position {
            let Some(origin) = LocalOrigin::new(fix) else {
                return refuse(&mut self.diagnostics.gnss_position, Fusion::NoReference);
            };
            self.origin = Some(origin);
            return self.fuse_gnss_position(Position::zero(), noise);
        }

        // Equation (44): the origin under the estimate, and the fix's error as the
        // position's.
        let Some(origin) = LocalOrigin::placing(fix, self.state.position) else {
            return refuse(&mut self.diagnostics.gnss_position, Fusion::NoReference);
        };
        self.origin = Some(origin);
        let placed = self.reset_position_to(self.state.position, noise);
        debug_assert!(
            placed,
            "the estimate and the fix's noise are both already checked"
        );
        self.diagnostics.gnss_position.record_accepted(0.0);
        Fusion::Accepted { test_ratio: 0.0 }
    }

    /// Fuse a GNSS velocity solution. Equation (29).
    ///
    /// After a coarse start the first solution is adopted rather than fused; see
    /// [`Fusion::Reset`].
    ///
    /// `noise` is the receiver's speed accuracy, `sacc`
    /// ([`VelocityNoise::from_speed_accuracy`](crate::VelocityNoise::from_speed_accuracy)),
    /// and wants the same floor as [`fuse_gnss_position`](Self::fuse_gnss_position): PX4 fuses
    /// `max(sacc, EKF2_GPS_V_NOISE)` and ArduPilot
    /// `constrain(sacc, EK3_VELNE_M_NSE, 50 m/s)`, both from a 0.5 m/s floor.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_gnss_velocity(
        &mut self,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
    ) -> Fusion {
        if !self.initialized {
            return refuse(&mut self.diagnostics.gnss_velocity, Fusion::NotInitialized);
        }
        if !velocity.is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.gnss_velocity, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.gnss_velocity, Fusion::InvalidNoise);
        }
        if self.unestablished.velocity {
            let adopted = self.reset_velocity_to(velocity, noise);
            debug_assert!(adopted, "the solution cleared the same checks just above");
            self.diagnostics.gnss_velocity.record_adopted();
            return Fusion::Reset;
        }
        let _ = (velocity, noise);
        stub_accept(&mut self.diagnostics.gnss_velocity)
    }

    /// Fuse a barometric altitude. Equation (30).
    ///
    /// The measurement is `z = -(α - α₀)`, so it needs a reference: one
    /// [`initialize`](Self::initialize) derived from a window taken at rest, or one
    /// [`set_baro_reference`](Self::set_baro_reference) named. Without one the measurement
    /// is refused rather than referred to an invented origin — which is what a start in
    /// motion, or a window with no barometer in it, leaves behind.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing.
    pub fn fuse_baro_altitude(&mut self, altitude: Altitude, noise: AltitudeNoise) -> Fusion {
        if !self.initialized {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::NotInitialized);
        }
        if !altitude.as_meters().is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::InvalidNoise);
        }
        if self.baro_reference.is_none() {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::NoReference);
        }
        let _ = (altitude, noise);
        stub_accept(&mut self.diagnostics.baro_altitude)
    }

    /// Fuse magnetic heading from a calibrated three-axis magnetometer.
    /// Equations (34)–(36).
    ///
    /// Heading only: the field is reduced to one scalar, so a magnetic disturbance can
    /// corrupt yaw but cannot reach roll or pitch. `noise` is on the resulting
    /// heading, not on the field components.
    ///
    /// This is also where heading becomes an estimate. A static window with no
    /// magnetometer in it, and any coarse start, leave yaw unobserved — see
    /// [`initialize`](Self::initialize) — and [`validity`](Self::validity) reports
    /// `heading` false until the first heading is accepted here, however tight
    /// [`sigma_yaw`](crate::Initialization::sigma_yaw) was.
    ///
    /// That first heading is the point `GOALS.md` names for a yaw **reset** rather than an
    /// ordinary update: the error-state attitude of equation (2) is a small-angle
    /// quantity, so a yaw error of a radian is wrong in a way no variance expresses, and
    /// widening the prior does not fix it. The bookkeeping below is the same either way.
    ///
    /// **Stub.** Records an acceptance with a zero test ratio; corrects nothing, and
    /// computes no heading from `field`, so the reset above is not yet performed: the
    /// validity flag moves, the yaw it describes does not.
    pub fn fuse_mag_heading(&mut self, field: MagField<Body>, noise: HeadingNoise) -> Fusion {
        if !self.initialized {
            return refuse(&mut self.diagnostics.mag_heading, Fusion::NotInitialized);
        }
        if !field.is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.mag_heading, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.mag_heading, Fusion::InvalidNoise);
        }
        let _ = (field, noise);
        let outcome = stub_accept(&mut self.diagnostics.mag_heading);
        // A rejected heading establishes nothing: it is the measurement the filter chose
        // not to believe.
        if outcome.is_accepted() {
            self.unestablished.heading = false;
        }
        outcome
    }

    /// The current estimate, including its [`Status`].
    ///
    /// Status and validity are derived here rather than cached: they are pure functions
    /// of [`diagnostics`](Self::diagnostics), the covariance, and [`Config`], so computing
    /// them on read means there is no invariant for the mutating methods to maintain. The
    /// cost is nine covariance entries and four source timers, compared.
    pub fn state(&self) -> State {
        // `self.state.status` and `.validity` are inert; the stored estimate never
        // carries meaningful ones, and every read overwrites them.
        let validity = self.validity();
        let mut state = self.state;
        state.status = self.derive_status(validity);
        state.validity = validity;
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
    /// convergence is measured rather than timed. With one thing the covariance cannot
    /// say: a heading nothing ever observed is not aligned however tight
    /// [`sigma_yaw`](crate::Initialization::sigma_yaw) is, so this stays false on a
    /// vehicle with no magnetometer. See [`validity`](Self::validity).
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
        let tilt = accuracy.tilt.as_radians();
        let position = accuracy.position.as_meters();
        let velocity = accuracy.velocity.as_m_per_s();

        Validity {
            tilt: within(ErrorState::AttitudeX, tilt) && within(ErrorState::AttitudeY, tilt),
            // A quantity nothing ever established is not valid however tight the prior on
            // it looks: nobody set that number.
            heading: !self.unestablished.heading
                && within(ErrorState::AttitudeZ, accuracy.heading.as_radians()),
            horizontal_position: !self.unestablished.position
                && within(ErrorState::PositionNorth, position)
                && within(ErrorState::PositionEast, position),
            vertical_position: !self.unestablished.position
                && within(ErrorState::PositionDown, position),
            horizontal_velocity: !self.unestablished.velocity
                && within(ErrorState::VelocityNorth, velocity)
                && within(ErrorState::VelocityEast, velocity),
            vertical_velocity: !self.unestablished.velocity
                && within(ErrorState::VelocityDown, velocity),
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
        let fresh =
            |source: SourceHealth| source.accepted_within(self.config.timeouts.degraded_after);
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

    /// The barometric reference `α₀` fixed at initialization, or `None` if no
    /// initialization has established one — a window with no barometer sample in it, or a
    /// start in motion, which keeps whatever reference the flight already had rather than
    /// calling its own altitude the ground. Equation (30).
    ///
    /// Exposed because it is the one initialization output an application may need to
    /// keep: it is what the filter's zero altitude means, and re-establishing it on the
    /// ground is the documented remedy for reference drift.
    pub const fn baro_reference(&self) -> Option<Altitude> {
        self.baro_reference
    }

    /// Force position to an external fix and reset its covariance block.
    ///
    /// The position becomes the fix, its variances become the fix's noise, and its
    /// correlations with the rest of the state are dropped — the new error came from the
    /// measurement and has nothing to do with the errors that preceded it.
    ///
    /// The filter does not do this on its own to **recover**: on sustained rejection it
    /// reports [`Status::DeadReckoning`] and leaves the policy to the application, which
    /// is the only layer that knows whether a step input to the controller is acceptable.
    /// The one exception is a quantity that was never established at all; see
    /// [`Fusion::Reset`].
    ///
    /// Returns `false`, changing nothing, for a fix or a noise that is not a number, or a
    /// variance that is not positive — the bar every `fuse_*` applies, and it matters more
    /// here: this writes `noise` straight onto the covariance diagonal, with no gate and
    /// no innovation to dilute it. See [`Fusion::NotFinite`] and [`Fusion::InvalidNoise`].
    #[must_use = "a refused reset leaves the estimate where it was, still dead-reckoning"]
    pub fn reset_position_to(
        &mut self,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
    ) -> bool {
        if !position.is_finite() || !noise.is_finite() || !noise.is_positive() {
            return false;
        }
        self.state.position = position;
        self.covariance.reset_block(
            [
                ErrorState::PositionNorth,
                ErrorState::PositionEast,
                ErrorState::PositionDown,
            ],
            noise.variance().into(),
        );
        self.unestablished.position = false;
        true
    }

    /// Force velocity to an external solution and reset its covariance block.
    ///
    /// See [`reset_position_to`](Self::reset_position_to), including what is refused.
    #[must_use = "a refused reset leaves the estimate where it was, still dead-reckoning"]
    pub fn reset_velocity_to(
        &mut self,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
    ) -> bool {
        if !velocity.is_finite() || !noise.is_finite() || !noise.is_positive() {
            return false;
        }
        self.state.velocity = velocity;
        self.covariance.reset_block(
            [
                ErrorState::VelocityNorth,
                ErrorState::VelocityEast,
                ErrorState::VelocityDown,
            ],
            noise.variance().into(),
        );
        self.unestablished.velocity = false;
        true
    }

    /// Commit an alignment: reset state, covariance and health for a fresh start whose
    /// attitude uncertainty matches how good the alignment was. The barometric reference
    /// is the caller's to set, because only it knows whether this start establishes a new
    /// one.
    ///
    /// A static start clears the origin. It declares position zero to be where the
    /// vehicle is now, and an origin held from before says zero is somewhere else; the
    /// next geodetic fix places a new one. A coarse start keeps it, because its position
    /// is unestablished and the first fix is adopted about the origin the flight already
    /// has.
    fn apply_alignment(&mut self, alignment: Alignment) {
        let (sigma_tilt, sigma_yaw) = init::attitude_sigmas(&self.config.init, alignment);
        self.covariance = init::initial_covariance(&self.config.init, sigma_tilt, sigma_yaw);
        self.state = State::default();
        self.diagnostics = Diagnostics::default();
        self.unestablished = Unestablished::after(alignment);
        if alignment.is_static() {
            self.origin = None;
        }
        self.initialized = true;
    }

    /// Aggregate alignment and the per-source timers into one status, most severe first.
    ///
    /// Only sources that have ever been accepted count toward aiding: a vehicle with no
    /// magnetometer is not permanently `Degraded` for lacking one.
    fn derive_status(&self, validity: Validity) -> Status {
        let timeouts = &self.config.timeouts;
        let mut used = 0;
        let mut fresh = 0;
        let mut aiding = 0;
        for (_, source) in self.diagnostics.sources() {
            if !source.has_been_used() {
                continue;
            }
            used += 1;
            if source.accepted_within(timeouts.degraded_after) {
                fresh += 1;
            }
            if source.accepted_within(timeouts.dead_reckoning_after) {
                aiding += 1;
            }
        }

        if used == 0 || aiding == 0 {
            // Nothing is arriving that could align the filter either, so this outranks
            // `Aligning`.
            Status::DeadReckoning
        } else if !validity.attitude() {
            Status::Aligning
        } else if fresh == used {
            Status::Healthy
        } else {
            Status::Degraded
        }
    }
}

impl Unestablished {
    /// What an alignment leaves unestablished.
    ///
    /// A static start defines the origin as where the vehicle was and its velocity as
    /// zero, and both are true by construction. A coarse start can say neither: the
    /// vehicle was moving, through somewhere the filter cannot name. Those wait for the
    /// first fix.
    ///
    /// Heading is unestablished after a coarse start even when a magnetometer was there,
    /// because levelling its reading needs the tilt that start did not get; the still
    /// window that carried none is [`Eskf::initialize`]'s to add.
    const fn after(alignment: Alignment) -> Self {
        let coarse = matches!(alignment, Alignment::Coarse(..));
        Self {
            position: coarse,
            velocity: coarse,
            heading: coarse,
        }
    }
}

/// Record a refusal against the source that produced it, and hand the outcome back to the
/// caller.
///
/// One place maps an outcome to what `Diagnostics` stores, so a `fuse_*` that grows another
/// guard cannot forget to count it. A refusal moves no timer; see
/// [`SourceHealth::record_refused`].
fn refuse(source: &mut SourceHealth, outcome: Fusion) -> Fusion {
    if let Some(refusal) = outcome.refusal() {
        source.record_refused(refusal);
    }
    outcome
}

/// What every `fuse_*` stub does in place of an update: record an acceptance with a zero
/// test ratio.
fn stub_accept(source: &mut SourceHealth) -> Fusion {
    source.record_accepted(0.0);
    Fusion::Accepted { test_ratio: 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::GRAVITY;
    use crate::geodetic::LocalOrigin;
    use crate::health::Refusal;
    use crate::init::tests::still;
    use crate::state::ErrorState;
    use crate::units::{Acceleration, AngularRate};

    const DT: Seconds = Seconds::from_secs(0.01);

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

    /// A still window carrying a magnetometer on every sample, which is what makes
    /// heading an estimate rather than a prior. `initialized()`'s window has none.
    fn window_with_mag() -> [StaticSample; 8] {
        [StaticSample {
            mag: Some(MagField::body(0.22, 0.0, 0.44)),
            ..still()
        }; 8]
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
                .fuse_baro_altitude(Altitude::from_meters(100.0), AltitudeNoise::from_sigma(2.0))
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
    fn a_non_finite_sample_is_refused_but_the_time_still_passes() {
        let level = Acceleration::body(0.0, 0.0, -GRAVITY);
        let still = AngularRate::body(0.0, 0.0, 0.0);
        for (name, imu) in [
            (
                "NaN gyro",
                ImuSample {
                    gyro: AngularRate::body(f32::NAN, 0.0, 0.0),
                    accel: level,
                },
            ),
            (
                "NaN accel",
                ImuSample {
                    gyro: still,
                    accel: Acceleration::body(0.0, f32::NAN, -GRAVITY),
                },
            ),
            (
                "infinite accel",
                ImuSample {
                    gyro: still,
                    accel: Acceleration::body(0.0, 0.0, f32::INFINITY),
                },
            ),
        ] {
            let mut filter = aided();
            let before = filter.state();
            assert_eq!(filter.predict(imu, DT), Propagation::NotFinite, "{name}");
            assert_eq!(filter.state(), before, "{name} reached the state");
            assert_eq!(
                elapsed(&filter),
                DT.as_secs(),
                "{name}: a refused step still happened in real time"
            );
            assert_eq!(
                filter.diagnostics().propagation.refused_not_finite,
                1,
                "{name} went uncounted"
            );
        }
    }

    #[test]
    fn a_long_step_carrying_a_non_finite_sample_still_measures_the_gap() {
        // Both refusals apply; the gap is reported because nothing else records how far
        // the interval ran, while the sensor fault recurs on the next step.
        let mut filter = aided();
        let dt = Seconds::from_secs(1.304);
        let imu = ImuSample {
            gyro: AngularRate::body(f32::NAN, 0.0, 0.0),
            accel: Acceleration::body(0.0, 0.0, -GRAVITY),
        };
        assert!(matches!(
            filter.predict(imu, dt),
            Propagation::StepTooLong { .. }
        ));
        let propagation = filter.diagnostics().propagation;
        assert_eq!(propagation.longest_refused, Some(dt));
        assert_eq!(propagation.refused_not_finite, 0);
    }

    #[test]
    fn a_refused_measurement_is_visible_in_diagnostics_not_only_in_the_return_value() {
        // The failure this guards: a miswired sensor feeding NaN reads as `never accepted`,
        // exactly like one that was never connected, unless the refusal is counted.
        let mut filter = initialized();
        let nan = Velocity::ned(f32::NAN, 0.0, 0.0);
        let noise = VelocityNoise::from_speed_accuracy(0.3);

        assert_eq!(filter.fuse_gnss_velocity(nan, noise), Fusion::NotFinite);
        assert_eq!(
            filter.fuse_gnss_velocity(
                Velocity::ned(1.0, 0.0, 0.0),
                VelocityNoise::from_variance(0.0, 1.0, 1.0)
            ),
            Fusion::InvalidNoise
        );

        let health = filter.diagnostics().gnss_velocity;
        assert_eq!(health.refused, 2);
        assert_eq!(health.last_refusal, Some(Refusal::InvalidNoise));
        assert_eq!(health.accepted, 0);
        assert_eq!(health.rejected, 0, "neither reached the gate");
        assert_eq!(
            health.time_since_accepted, None,
            "a refusal is not aiding, so no timer starts"
        );
        assert!(!health.has_been_used());
    }

    #[test]
    fn a_refusal_names_which_kind_it_was() {
        let mut filter = initialized();
        assert_eq!(
            filter.fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeNoise::from_sigma(2.0)),
            Fusion::NoReference
        );
        assert_eq!(
            filter.diagnostics().baro_altitude.last_refusal,
            Some(Refusal::NoReference),
            "a missing reference is a different problem from a bad number"
        );

        let mut fresh = Eskf::new(Config::default());
        assert_eq!(
            fresh.fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeNoise::from_sigma(2.0)),
            Fusion::NotInitialized
        );
        assert_eq!(
            fresh.diagnostics().baro_altitude.last_refusal,
            Some(Refusal::NotInitialized)
        );
    }

    #[test]
    fn an_adopted_measurement_is_counted_apart_from_an_ordinary_acceptance() {
        let mut filter = coarse();
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        assert_eq!(
            filter.fuse_gnss_position(Position::ned(120.0, -40.0, -75.0), noise),
            Fusion::Reset
        );
        // The second fix has an estimate to be judged against, so it is fused, not adopted.
        assert!(
            filter
                .fuse_gnss_position(Position::ned(121.0, -40.0, -75.0), noise)
                .is_accepted()
        );

        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.adopted, 1, "adoption happens once per quantity");
        assert_eq!(health.accepted, 2, "and counts as an acceptance besides");
    }

    #[test]
    fn propagation_refusals_are_counted_and_the_worst_gap_kept() {
        let mut filter = aided();
        for bad in [0.0, f32::NAN] {
            assert!(
                !filter
                    .predict(ImuSample::default(), Seconds::from_secs(bad))
                    .is_propagated()
            );
        }
        for gap in [0.34, 1.304, 0.5] {
            assert!(
                !filter
                    .predict(ImuSample::default(), Seconds::from_secs(gap))
                    .is_propagated()
            );
        }

        let propagation = filter.diagnostics().propagation;
        assert_eq!(propagation.refused_invalid, 2);
        assert_eq!(propagation.refused_too_long, 3);
        assert_eq!(
            propagation.longest_refused.map(Seconds::as_secs),
            Some(1.304),
            "the worst gap, not the last"
        );
    }

    #[test]
    fn without_a_barometer_in_the_window_fusion_is_refused_not_referred_to_nothing() {
        let mut filter = initialized();
        assert_eq!(filter.baro_reference(), None);
        assert_eq!(
            filter.fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeNoise::from_sigma(2.0)),
            Fusion::NoReference
        );
        assert_eq!(
            filter.diagnostics().baro_altitude.time_since_accepted,
            None,
            "a refused measurement is not aiding"
        );
    }

    #[test]
    fn a_static_restart_replaces_the_reference() {
        let mut filter = aided();
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));
        let alignment = filter
            .initialize(&window_with_baro([250.0; 8]), Seconds::from_secs(0.25))
            .expect("a 2 s window");
        assert_eq!(alignment, Alignment::Static);
        assert_eq!(
            filter.baro_reference(),
            Some(Altitude::from_meters(250.0)),
            "a static start declares this altitude to be zero"
        );
    }

    /// A window taken while the vehicle was moving, reading 250 m: a restart in flight.
    fn moving_window_at(altitude: f32) -> [StaticSample; 8] {
        let mut window = window_with_baro([altitude; 8]);
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        window
    }

    #[test]
    fn a_restart_in_motion_keeps_the_reference_the_flight_began_with() {
        let mut filter = aided();

        // Taking 250 m as the reference would call the current altitude zero: the
        // barometer would then say z ~ 0 while GNSS about the flight's origin says
        // z ~ -150.
        let alignment = filter
            .initialize(&moving_window_at(250.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));

        // Short as well as moving, which is what a restart in flight usually offers. The
        // window is too short to measure motion for `classify`, but `α₀` measures it
        // anyway rather than reading window length as stillness.
        let _ = filter
            .initialize(&moving_window_at(250.0), Seconds::from_secs(0.1))
            .expect("short and moving");
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));

        // And a moving window with no barometer in it does not wipe the reference either.
        let mut window = [still(); 8];
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        let _ = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));
    }

    #[test]
    fn a_window_too_short_to_align_from_still_names_the_ground_it_sat_on() {
        // `classify` reports a short window as coarse before it ever measures motion, so
        // alignment cannot answer this: a vehicle sitting on the ground with 0.8 s of
        // samples has an honest reference, and reading "coarse" as "moving" would have
        // cost it barometric aiding for the whole flight with nothing saying so.
        let mut filter = Eskf::new(Config::default());
        let alignment = filter
            .initialize(&window_with_baro([112.0; 8]), Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(112.0)));
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(112.0), AltitudeNoise::from_sigma(2.0))
                .is_accepted()
        );
    }

    /// An attitude and biases such as a companion AHRS would hand over, with the
    /// uncertainty that source reports rather than the static-window figures.
    fn seed() -> (State, Covariance) {
        let state = State {
            velocity: Velocity::ned(18.0, 0.0, 0.0),
            gyro_bias: AngularRate::body(0.001, -0.002, 0.0005),
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
            filter.fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeNoise::from_sigma(2.0)),
            Fusion::NoReference
        );

        assert!(filter.set_baro_reference(Altitude::from_meters(52.0)));
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeNoise::from_sigma(2.0))
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
            position: Position::ned(f32::NAN, 0.0, 0.0),
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
            Err(InitError::InvalidVariance)
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
    fn a_seed_certain_of_a_quantity_it_cannot_be_certain_of_is_refused() {
        // The failure this catches is a warm start off storage whose diagonal was never
        // populated, so it arrives all zeros rather than obviously broken. A zero
        // variance is not a tight prior: the gain is zero for that quantity, so nothing
        // ever corrects it, and `validity`'s `variance <= sigma^2` calls it good on the
        // first read.
        let (state, _) = seed();
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.initialize_from(state, Covariance::zero()),
            Err(InitError::InvalidVariance)
        );
        assert!(!filter.is_initialized(), "a refused seed leaves no state");
        assert!(!filter.validity().horizontal_position);

        // One zero entry is enough, and it is enough alone: the rest of the diagonal is
        // a covariance a caller could have meant.
        let (_, covariance) = seed();
        let mut matrix = *covariance.as_matrix();
        matrix[(ErrorState::GyroBiasZ.index(), ErrorState::GyroBiasZ.index())] = 0.0;
        assert_eq!(
            filter.initialize_from(state, Covariance::from_matrix(matrix)),
            Err(InitError::InvalidVariance)
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
                .fuse_baro_altitude(Altitude::from_meters(100.0), AltitudeNoise::from_sigma(2.0))
                .is_accepted()
        );
        assert_eq!(filter.state().status, Status::Aligning);
    }

    #[test]
    fn a_coarse_window_still_starts_the_filter_with_its_tilt_widened() {
        // Classification itself is tested in `init`; this is what the filter does with it.
        let mut filter = Eskf::new(Config::default());
        let alignment = filter
            .initialize(&[still(); 8], Seconds::from_secs(0.1))
            .expect("a short window is a coarse start, not a refusal");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(
            filter.is_initialized(),
            "refusing to run is the old behavior"
        );
        assert!(!filter.is_aligned());

        let mut window = [still(); 8];
        window[3].imu.gyro = AngularRate::body(0.0, 0.0, 0.4);
        let _ = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        let tilt = filter.covariance().variance(ErrorState::AttitudeX);
        assert!(
            (tilt - 0.8 * 0.8).abs() < 1e-4,
            "the covariance carries the widened tilt, got sigma^2 {tilt}"
        );
    }

    #[test]
    fn a_refused_window_leaves_the_filter_uninitialized() {
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.initialize(&[], Seconds::from_secs(0.25)),
            Err(InitError::NoSamples)
        );
        let mut poisoned = [still(); 8];
        poisoned[2].imu.accel = Acceleration::body(f32::NAN, 0.0, 0.0);
        assert_eq!(
            filter.initialize(&poisoned, Seconds::from_secs(0.25)),
            Err(InitError::NotFinite)
        );
        assert!(!filter.is_initialized());
    }

    /// A filter that started while moving: it knows neither where it is nor how fast.
    fn coarse() -> Eskf {
        let mut filter = Eskf::new(Config::default());
        let mut window = [still(); 8];
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        let alignment = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        filter
    }

    #[test]
    fn after_a_coarse_start_the_first_fix_is_adopted_not_fused() {
        let mut filter = coarse();
        let fix = Position::ned(120.0, -40.0, -75.0);
        let outcome = filter.fuse_gnss_position(fix, PositionNoise::horizontal_vertical(1.5, 1.5));

        assert_eq!(outcome, Fusion::Reset);
        assert!(outcome.is_accepted(), "the measurement was used");
        assert!(outcome.is_reset(), "and it stepped the state");
        assert_eq!(filter.state().position, fix);
        assert!(
            (filter.covariance().variance(ErrorState::PositionNorth) - 2.25).abs() < 1e-6,
            "the fix's own variance, not the configured prior"
        );

        // Once is once: there is now an estimate for a gate to judge against.
        let outcome = filter.fuse_gnss_position(fix, PositionNoise::horizontal_vertical(1.5, 1.5));
        assert!(!outcome.is_reset());
    }

    #[test]
    fn velocity_is_adopted_too_and_the_two_are_independent() {
        let mut filter = coarse();
        let velocity = Velocity::ned(18.0, 1.0, -0.5);
        assert!(
            filter
                .fuse_gnss_velocity(velocity, VelocityNoise::from_speed_accuracy(0.3))
                .is_reset()
        );
        assert_eq!(filter.state().velocity, velocity);

        // Adopting velocity says nothing about position, which is still unknown.
        assert!(
            filter
                .fuse_gnss_position(
                    Position::ned(1.0, 2.0, 3.0),
                    PositionNoise::horizontal_vertical(1.5, 1.5)
                )
                .is_reset()
        );
    }

    #[test]
    fn a_static_start_knows_where_it_is_so_its_first_fix_is_fused() {
        let mut filter = initialized();
        let outcome = filter.fuse_gnss_position(
            Position::ned(0.2, -0.1, 0.0),
            PositionNoise::horizontal_vertical(1.5, 1.5),
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

        assert!(filter.reset_position_to(
            Position::ned(10.0, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.0, 1.0),
        ));

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
            Velocity::ned(0.0, 0.0, 0.0),
            VelocityNoise::from_speed_accuracy(0.3),
        );
        assert!(
            !outcome.is_reset(),
            "the caller vouched for this velocity; its covariance says how far"
        );
        assert_eq!(filter.state().velocity, state.velocity);
    }

    #[test]
    fn a_static_start_is_valid_in_every_part() {
        let mut filter = Eskf::new(Config::default());
        let alignment = filter
            .initialize(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert_eq!(alignment, Alignment::Static);

        let validity = filter.state().validity;
        assert!(validity.all(), "{validity:?}");
        assert!(validity.attitude() && validity.navigation());
    }

    #[test]
    fn a_static_window_without_a_magnetometer_leaves_heading_unestablished() {
        // The covariance on its own would say otherwise: `Initialization::sigma_yaw` and
        // `Accuracy::heading` are both 0.35 rad, so the prior passes the bar exactly. It
        // is a prior on a yaw nothing ever observed.
        let filter = initialized();
        let validity = filter.validity();
        assert!(validity.tilt, "gravity pins roll and pitch");
        assert!(!validity.heading, "nothing pins the rotation about gravity");
        assert!(!filter.is_aligned());
    }

    #[test]
    fn the_first_accepted_magnetic_heading_establishes_yaw() {
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    MagField::body(0.22, 0.0, 0.44),
                    HeadingNoise::from_sigma(0.1),
                )
                .is_accepted()
        );
        assert!(filter.validity().heading, "something observed it at last");
        assert!(filter.is_aligned());
    }

    #[test]
    fn a_magnetometer_in_the_window_establishes_heading_at_once() {
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert!(filter.validity().heading);
    }

    #[test]
    fn a_coarse_start_needs_more_than_a_magnetometer_to_call_heading_valid() {
        // Having observed the quantity is necessary, not sufficient: a coarse start
        // widened yaw far past `Accuracy::heading`, and only fusion brings it back down.
        let mut filter = Eskf::new(Config::default());
        let mut window = window_with_mag();
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        let alignment = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(!filter.validity().heading, "the window was moving");

        assert!(
            filter
                .fuse_mag_heading(
                    MagField::body(0.22, 0.0, 0.44),
                    HeadingNoise::from_sigma(0.1),
                )
                .is_accepted()
        );
        assert!(
            !filter.validity().heading,
            "the covariance still says yaw is somewhere on the circle"
        );
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
                    Position::ned(120.0, -40.0, -75.0),
                    PositionNoise::horizontal_vertical(1.5, 1.5),
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
            Position::ned(0.0, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.5, 1.5),
        );
        // ...and the magnetometer is being accepted, so heading will come in even though
        // it is worthless at this instant.
        assert!(
            filter
                .fuse_mag_heading(
                    MagField::body(0.22, 0.0, 0.44),
                    HeadingNoise::from_sigma(0.22),
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
                .fuse_baro_altitude(Altitude::from_meters(100.0), AltitudeNoise::from_sigma(2.0))
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

    fn zurich() -> Geodetic {
        Geodetic::from_degrees(47.3977, 8.5456, 488.0)
    }

    fn near(a: Position<Ned>, b: Position<Ned>) -> bool {
        (a.vector() - b.vector()).norm() < 1e-2
    }

    #[test]
    fn the_first_geodetic_fix_places_the_origin_under_the_estimate() {
        let mut filter = initialized();
        assert_eq!(filter.origin(), None);
        let outcome =
            filter.fuse_gnss_geodetic(zurich(), PositionNoise::horizontal_vertical(1.5, 1.5));
        assert!(outcome.is_accepted() && !outcome.is_reset(), "{outcome:?}");

        let origin = filter.origin().expect("placed by the first fix");
        assert!(
            near(origin.to_ned(zurich()), filter.state().position),
            "the fix lands on the estimate, so nothing steps"
        );
    }

    #[test]
    fn placing_the_origin_makes_the_fix_error_the_position_error() {
        let mut filter = initialized();
        let mut matrix = *filter.covariance().as_matrix();
        let (pn, vn) = (
            ErrorState::PositionNorth as usize,
            ErrorState::VelocityNorth as usize,
        );
        matrix[(pn, vn)] = 0.01;
        matrix[(vn, pn)] = 0.01;
        filter.covariance = Covariance::from_matrix(matrix);

        let _ = filter.fuse_gnss_geodetic(zurich(), PositionNoise::horizontal_vertical(1.5, 3.0));

        let p = filter.covariance();
        assert_eq!(p.variance(ErrorState::PositionNorth), 2.25);
        assert_eq!(p.variance(ErrorState::PositionDown), 9.0);
        assert_eq!(
            p.as_matrix()[(pn, vn)],
            0.0,
            "the fix's error owes nothing to velocity"
        );
    }

    #[test]
    fn a_fix_that_is_not_a_number_is_refused_even_with_an_origin_held() {
        // The case that matters: an origin held and position unestablished, where a NaN
        // would otherwise be adopted outright.
        let mut filter = initialized();
        assert!(filter.set_origin(zurich()));
        let _ = filter
            .initialize(&[still(); 8], Seconds::from_secs(0.1))
            .expect("short, so coarse");
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);

        let nonsense = Geodetic::from_degrees(f64::NAN, 8.5, 488.0);
        assert_eq!(
            filter.fuse_gnss_geodetic(nonsense, noise),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_geodetic(zurich(), PositionNoise::from_sigma(f32::NAN, 1.5, 1.5)),
            Fusion::NotFinite
        );
        assert!(filter.state().position.is_finite());
        assert!(filter.fuse_gnss_geodetic(zurich(), noise).is_reset());
    }

    #[test]
    fn every_source_refuses_a_measurement_that_is_not_a_number() {
        let mut filter = initialized();
        assert!(filter.set_baro_reference(Altitude::from_meters(52.0)));
        let nan = f32::NAN;
        assert_eq!(
            filter.fuse_gnss_position(
                Position::ned(nan, 0.0, 0.0),
                PositionNoise::horizontal_vertical(1.5, 1.5)
            ),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                Velocity::ned(0.0, 0.0, 0.0),
                VelocityNoise::from_speed_accuracy(nan)
            ),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_baro_altitude(Altitude::from_meters(nan), AltitudeNoise::from_sigma(2.0)),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_mag_heading(
                MagField::body(0.2, 0.0, 0.4),
                HeadingNoise::from_variance(f32::INFINITY)
            ),
            Fusion::NotFinite
        );
        assert_eq!(filter.diagnostics().gnss_position.accepted, 0);
    }

    #[test]
    fn every_source_refuses_a_variance_no_sensor_could_have() {
        let mut filter = initialized();
        assert!(filter.set_baro_reference(Altitude::from_meters(52.0)));
        assert_eq!(
            filter.fuse_gnss_position(
                Position::ned(1.0, 2.0, 3.0),
                PositionNoise::<Ned>::from_variance(0.0, 1.0, 1.0),
            ),
            Fusion::InvalidNoise,
            "zero variance claims a perfect measurement and makes S singular"
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                Velocity::ned(0.0, 0.0, 0.0),
                VelocityNoise::<Ned>::from_variance(1.0, -4.0, 1.0),
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_baro_altitude(
                Altitude::from_meters(60.0),
                AltitudeNoise::from_variance(0.0)
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_mag_heading(
                MagField::body(0.2, 0.0, 0.4),
                HeadingNoise::from_variance(-1.0)
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_gnss_geodetic(zurich(), PositionNoise::<Ned>::from_variance(1.0, 1.0, 0.0)),
            Fusion::InvalidNoise
        );
        assert_eq!(filter.origin(), None, "and no origin was placed on the way");

        for (name, source) in filter.diagnostics().sources() {
            assert_eq!(source.accepted, 0, "{name} counted a refused measurement");
            assert_eq!(source.time_since_accepted, None, "{name} started aiding");
        }
    }

    #[test]
    fn a_coarse_start_does_not_adopt_a_fix_whose_noise_is_impossible() {
        // Where the check matters most: adoption writes `noise` onto the covariance
        // diagonal with no gate in the way, and a negative variance there passes
        // `validity`'s `variance <= sigma^2` — position would be reported valid.
        let mut filter = coarse();
        let fix = Position::ned(120.0, -40.0, -75.0);
        assert_eq!(
            filter.fuse_gnss_position(fix, PositionNoise::<Ned>::from_variance(-1.0, -1.0, -1.0)),
            Fusion::InvalidNoise
        );
        assert_eq!(filter.state().position, Position::zero(), "nothing adopted");
        assert!(!filter.validity().horizontal_position);
        assert!(
            filter
                .fuse_gnss_position(fix, PositionNoise::horizontal_vertical(1.5, 1.5))
                .is_reset(),
            "the adoption is still owed to the first usable fix"
        );
    }

    #[test]
    fn an_external_reset_refuses_what_would_poison_the_state() {
        let mut filter = initialized();
        assert!(!filter.reset_position_to(
            Position::ned(f32::NAN, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.5, 1.5)
        ));
        assert!(!filter.reset_position_to(
            Position::ned(10.0, 0.0, 0.0),
            PositionNoise::<Ned>::from_variance(1.0, 0.0, 1.0)
        ));
        assert_eq!(
            filter.state().position,
            Position::zero(),
            "a refused reset changes nothing"
        );

        assert!(!filter.reset_velocity_to(
            Velocity::ned(1.0, 0.0, 0.0),
            VelocityNoise::<Ned>::from_variance(1.0, 1.0, -0.25)
        ));
        assert_eq!(filter.state().velocity, Velocity::zero());

        assert!(!filter.set_baro_reference(Altitude::from_meters(f32::NAN)));
        assert_eq!(
            filter.baro_reference(),
            None,
            "a NaN alpha_0 would end barometric aiding for the flight, not one update"
        );
        assert!(filter.set_baro_reference(Altitude::from_meters(52.0)));
    }

    #[test]
    fn an_origin_placed_after_moving_accounts_for_the_move() {
        let (mut state, covariance) = seed();
        state.position = Position::ned(40.0, -15.0, -3.0);
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize_from(state, covariance)
            .expect("a sane seed");
        let _ = filter.fuse_gnss_geodetic(zurich(), PositionNoise::horizontal_vertical(1.5, 1.5));

        let origin = filter.origin().expect("placed");
        assert!(near(origin.to_ned(zurich()), state.position));
        assert_ne!(
            origin.geodetic(),
            zurich(),
            "the origin is where it started"
        );
    }

    #[test]
    fn after_a_coarse_start_the_origin_is_the_first_fix_and_it_is_adopted() {
        let mut filter = coarse();
        let outcome =
            filter.fuse_gnss_geodetic(zurich(), PositionNoise::horizontal_vertical(1.5, 1.5));
        assert_eq!(outcome, Fusion::Reset);
        assert_eq!(filter.origin().map(|o| o.geodetic()), Some(zurich()));
        assert_eq!(filter.state().position, Position::zero());
    }

    #[test]
    fn later_fixes_are_converted_about_the_same_origin() {
        let mut filter = coarse();
        let _ = filter.fuse_gnss_geodetic(zurich(), PositionNoise::horizontal_vertical(1.5, 1.5));
        // Position is established now, so this one is fused, not adopted.
        let north = Geodetic::from_degrees(47.3987, 8.5456, 488.0);
        assert!(
            !filter
                .fuse_gnss_geodetic(north, PositionNoise::horizontal_vertical(1.5, 1.5))
                .is_reset()
        );
        let p = filter.origin().expect("held").to_ned(north).vector();
        assert!((p.x - 111.2).abs() < 0.1 && p.y.abs() < 1e-3, "{p:?}");
    }

    #[test]
    fn a_fix_that_cannot_place_an_origin_is_refused_and_the_next_one_places_it() {
        let mut filter = initialized();
        let off_the_earth = Geodetic::from_degrees(91.0, 0.0, 0.0);
        assert_eq!(
            filter.fuse_gnss_geodetic(off_the_earth, PositionNoise::horizontal_vertical(1.5, 1.5)),
            Fusion::NoReference
        );
        assert_eq!(filter.origin(), None);
        assert!(
            filter
                .fuse_gnss_geodetic(zurich(), PositionNoise::horizontal_vertical(1.5, 1.5))
                .is_accepted()
        );
        assert!(filter.origin().is_some());
    }

    #[test]
    fn moving_the_origin_does_not_move_the_vehicle() {
        let mut filter = initialized();
        let _ = filter.fuse_gnss_geodetic(zurich(), PositionNoise::horizontal_vertical(1.5, 1.5));
        let before = filter.geodetic_position().expect("an origin is held");
        let variance = filter.covariance().variance(ErrorState::PositionNorth);

        let home = Geodetic::from_degrees(47.3967, 8.5446, 480.0);
        assert!(filter.set_origin(home));
        let after = filter.geodetic_position().expect("still held");
        let moved = LocalOrigin::new(before)
            .expect("usable")
            .to_ned(after)
            .vector()
            .norm();
        assert!(moved < 1e-2, "the vehicle moved {moved} m");
        assert_ne!(filter.state().position, Position::zero(), "the numbers did");
        assert_eq!(
            filter.covariance().variance(ErrorState::PositionNorth),
            variance
        );
    }

    #[test]
    fn a_static_start_clears_the_origin_and_a_coarse_one_keeps_it() {
        let mut filter = initialized();
        assert!(filter.set_origin(zurich()));
        let _ = filter
            .initialize(&[still(); 8], Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert!(
            filter.origin().is_some(),
            "a coarse restart keeps the flight's origin"
        );
        let _ = filter
            .initialize(&[still(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window");
        assert_eq!(filter.origin(), None, "zero is here now, wherever here is");
    }

    #[test]
    fn an_origin_off_the_earth_is_refused() {
        let mut filter = initialized();
        assert!(!filter.set_origin(Geodetic::from_degrees(91.0, 0.0, 0.0)));
        assert_eq!(filter.origin(), None);
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
}
