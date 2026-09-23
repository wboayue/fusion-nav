//! The filter itself.

use crate::config::{ALIGNED_HEADING, ALIGNED_TILT, Config};
use crate::frames::{Body, Ned};
use crate::geodetic::{Geodetic, LocalOrigin};
use crate::health::{Diagnostics, Fusion, Propagation, SourceHealth, Status, Validity};
use crate::init::{self, Alignment, Coarse, InitError, Measured, StaticSample, baro_reference};
use crate::math::{below_floor, exp_quat, floor_diagonal};
use crate::observation::{baro, gnss, mag};
use crate::propagate::{ImuSample, project, propagate};
use crate::state::{Covariance, ErrorState, State};
use crate::units::{
    Altitude, AltitudeNoise, Attitude, HeadingNoise, MagField, Position, PositionNoise, Radians,
    Seconds, Velocity, VelocityNoise,
};
use crate::update::{self, Update, update};
use nalgebra::Vector3;

/// A 15-state error-state Kalman filter.
///
/// Every equation the filter needs is implemented: initialization, (5)–(8), propagation,
/// (9)–(22), and the update of (23)–(41) for every source. The state dead reckons from
/// where [`initialize`](Self::initialize) put it, the covariance grows around it, and
/// [`fuse_gnss_position`](Self::fuse_gnss_position),
/// [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic),
/// [`fuse_gnss_velocity`](Self::fuse_gnss_velocity),
/// [`fuse_baro_altitude`](Self::fuse_baro_altitude) and
/// [`fuse_mag_heading`](Self::fuse_mag_heading) each correct both, or are turned down by
/// the gate of (37)–(38).
///
/// # Example
///
/// A whole flight's worth of calls, in the order they happen, with what each one is allowed to
/// answer. The README's quick start is the same loop without the assertions.
///
/// ```
/// use fusion_nav::prelude::*;
///
/// let mut filter = Eskf::new(Config::default());
/// let dt = Seconds::from_secs(0.0025); // 400 Hz IMU
///
/// // A vehicle sitting still: no rotation, gravity the only specific force. 800 samples
/// // at 400 Hz is the 2 s `Initialization::min_duration` wants.
/// let still = StaticSample {
///     imu: ImuSample {
///         gyro: AngularRate::body(0.0, 0.0, 0.0),
///         accel: Acceleration::body(0.0, 0.0, -GRAVITY),
///     },
///     ..StaticSample::default()
/// };
///
/// // Initialization reports what it achieved rather than refusing what it dislikes. A
/// // window that is short or moving gives `Alignment::Coarse`, and the filter runs and
/// // says `Status::Aligning` until attitude converges.
/// assert_eq!(filter.initialize(&[still; 800], dt)?, Alignment::Static);
///
/// // Nothing in that window carried a barometer, so there is no reference altitude and
/// // `fuse_baro_altitude` would refuse. See `StaticSample::baro`.
///
/// assert!(filter.predict(ImuSample::default(), dt).is_propagated());
///
/// // GNSS in latitude and longitude. The filter holds the navigation origin: the first
/// // fix places it, under the estimate, so every later fix converts about the same point.
/// // `clamped` bounds the receiver's own `eph` and `epv` the way both autopilots do.
/// let outcome = filter.fuse_gnss_geodetic(
///     Geodetic::from_degrees(47.397_742, 8.545_594, 488.0),
///     PositionNoise::clamped(1.5, 3.0, 0.5, 100.0),
/// );
/// assert!(outcome.is_accepted());
/// assert!(filter.origin().is_some());
///
/// // A solution whose vertical velocity the receiver did not measure: the down axis
/// // carries a σ large enough that its gain is negligible, rather than a claim.
/// let outcome = filter.fuse_gnss_velocity(
///     Velocity::ned(0.0, 0.0, 0.0),
///     VelocityNoise::horizontal_vertical(0.3, 1000.0),
/// );
/// assert!(outcome.is_accepted());
///
/// // Nothing in that window carried a magnetometer either, so nothing observed the
/// // rotation about gravity: heading is not valid, and `Aligning` says the attitude has
/// // not converged rather than the aiding having failed.
/// let state = filter.state();
/// assert_eq!(state.status, Status::Aligning);
/// assert!(state.validity.tilt && !state.validity.heading);
///
/// // The first accepted magnetic heading is what establishes yaw.
/// let outcome = filter.fuse_mag_heading(
///     MagField::body(0.22, 0.0, 0.44),
///     HeadingNoise::from_sigma(0.1),
/// );
/// assert!(outcome.is_accepted());
/// assert_eq!(filter.state().status, Status::Healthy);
/// # Ok::<(), fusion_nav::InitError>(())
/// ```
#[derive(Clone, Debug)]
pub struct Eskf {
    config: Config,
    state: State,
    covariance: Covariance,
    diagnostics: Diagnostics,
    baro_reference: Option<Altitude>,
    origin: Option<LocalOrigin>,
    unestablished: Unestablished,
    /// Whether the attitude has ever met [`ALIGNED_TILT`] and [`ALIGNED_HEADING`] since
    /// initialization. Latched, and the test behind [`Status::Aligning`]; see
    /// [`is_aligned`](Self::is_aligned) for why it is not read live.
    aligned: bool,
    initialized: bool,
}

/// Quantities the start never established, which wait for the first measurement that
/// observes them: position and velocity for the first GNSS fix after a coarse start (see
/// [`Fusion::Reset`]), heading for the first magnetic heading.
///
/// The covariance cannot carry this on its own. Every entry on its diagonal is a prior,
/// and a prior tight enough to pass [`Config::accuracy`](crate::Config::accuracy) or
/// [`ALIGNED_HEADING`] reads as an estimate whether or not anything ever measured the
/// quantity. This is the flag that tells the two apart.
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
            aligned: false,
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
    /// Whether those priors count as *established* is the window's answer, not the
    /// alignment's: a vehicle that held still through it was where the origin says and
    /// was not moving, whether or not the window was long enough to align an attitude
    /// from. A window taken in motion establishes neither, and the first fix is adopted
    /// rather than fused — see [`Fusion::Reset`].
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
        let measured = init::measure(window, dt)?;
        let alignment = init::classify(&measured, &self.config.init);
        // One answer to "was the vehicle on the ground", read by the gyroscope bias of
        // (7), the barometric reference of (30), and what this start establishes.
        // Measured from the window rather than read off `alignment`, because a window too
        // short to align an attitude from can still be a window of a parked vehicle.
        let at_rest = init::at_rest(&measured, &self.config.init);
        let state = init::nominal_state(&measured, self.config.magnetic_declination, at_rest);
        self.apply_alignment(alignment, state, &measured, at_rest);
        if at_rest {
            self.baro_reference = baro_reference(window);
        }
        self.note_alignment();
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
        Ok(init::classify(
            &init::measure(window, dt)?,
            &self.config.init,
        ))
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
    /// The sample carries no magnetometer, so heading starts at zero and stays
    /// unestablished whatever the vehicle was doing: this entry point levels, and
    /// nothing more.
    ///
    /// # Errors
    ///
    /// [`InitError::NotFinite`] if the sample carries a value that is not a number.
    pub fn initialize_coarse(&mut self, imu: ImuSample) -> Result<Alignment, InitError> {
        // Treated as a window of one, so the same finiteness, motion and averaging
        // measures apply — an average of one sample being that sample.
        let window = [StaticSample {
            imu,
            ..StaticSample::default()
        }];
        if !window[0].is_finite() {
            return Err(InitError::NotFinite);
        }
        // A window of one, spanning no time: its own average, with no rotation to smear
        // it and no second velocity to difference against — a caller with GNSS in hand
        // has a window, not this entry point.
        let measured = Measured::over(&window, Seconds::ZERO);
        let alignment = Alignment::Coarse(Coarse::NotStationary {
            peak_gyro: measured.peak_gyro,
            peak_accel_deviation: measured.peak_deviation,
            span: measured.span,
            inertial_accel: measured.inertial_accel,
        });
        // The same rule `initialize` applies: the gyroscope bias is worth taking only
        // where the sample says the vehicle was on the ground, and one reading of a
        // stationary gyroscope is a noisier bias than a window's average but a better
        // one than zero.
        let at_rest = init::at_rest(&measured, &self.config.init);
        let state = init::nominal_state(&measured, self.config.magnetic_declination, at_rest);
        // That reading establishes nothing, which is why it is not passed on as one. A
        // window shows rest by holding still over a span of time and this one spans none:
        // an accelerometer reading `γ` for an instant is a hover as readily as a vehicle
        // on the ground, and this entry point exists for the launches that are moving.
        self.apply_alignment(alignment, state, &measured, false);
        // The barometric reference is left alone: one sample does not establish one, and
        // a restart at altitude should keep the reference the flight began with.
        self.note_alignment();
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
    /// `state.attitude` names its own convention, through whichever
    /// [`Attitude`](crate::Attitude) constructor built it — PX4 and ArduPilot quaternions
    /// through [`body_to_ned`](crate::Attitude::body_to_ned), a ROS one through
    /// [`flu_to_enu`](crate::Attitude::flu_to_enu), which is where the conventions and
    /// what a wrong one costs are written down. It is the one error on this path no check
    /// downstream can reach.
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
    /// The bar is the floor of (42′) rather than zero, because zero is the tidy member of the
    /// class it is there to catch. A diagonal that was never populated arrives as exactly
    /// zero from a `memset`; the same accident with an exponent left in it — dropped in
    /// deserialization, or a variance scaled by its own units twice — arrives at 1e-30 and
    /// does the identical damage, since `1e-30 / (1e-30 + R)` is zero in f32 and
    /// [`validity`](Self::validity) calls the quantity good on the first read. Refusing one
    /// and accepting the other would catch the legible failure and wave through the messy
    /// ones. The floor is three to five decades below anything the filter itself reaches, so
    /// no seed anybody meant is near it.
    ///
    /// Whether the seed counts as aligned is the covariance's answer, not this one's: a
    /// confident seed reports [`Status::Healthy`] straight away, a coarse one
    /// [`Status::Aligning`] until it converges.
    pub fn initialize_from(
        &mut self,
        state: State,
        covariance: Covariance,
    ) -> Result<Alignment, InitError> {
        if !state.is_finite() || !covariance.is_finite() {
            return Err(InitError::NotFinite);
        }
        if below_floor(covariance.as_matrix()) {
            return Err(InitError::InvalidVariance);
        }
        self.state = state;
        // Diagnostics first: `commit_covariance` counts into them, and a seed sitting on the
        // floor is a fact about this filter's life rather than the last one's.
        self.diagnostics = Diagnostics::default();
        self.commit_covariance(covariance);
        // Nothing a seed carries is unestablished: the caller vouched for every quantity,
        // heading included, so no first measurement overwrites one.
        self.unestablished = Unestablished::default();
        self.initialized = true;
        self.aligned = false;
        self.note_alignment();
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
    /// A step that comes out of (11)–(14) or (22) non-finite is discarded rather than
    /// stored, as [`Propagation::StateNotFinite`], and the state and the covariance are
    /// discarded together: a finite sample is not enough to guarantee a finite result, since
    /// f32 has a finite range and both `a_n Δt` and `F P Fᵀ` can leave it. Committing one
    /// half would leave the filter reporting an estimate whose uncertainty describes a
    /// different step.
    ///
    /// The covariance only grows here. (22) adds `Q` and (20) spreads what is already there;
    /// nothing in propagation takes uncertainty back out, which is the measurement update's
    /// job. So an unaided filter's [`Validity`] flags go false in the order their variances
    /// cross [`Config::accuracy`](crate::Config::accuracy), and [`Status`] does not follow:
    /// it reads the aiding timers and an alignment that has already latched.
    pub fn predict(&mut self, imu: ImuSample, dt: Seconds) -> Propagation {
        if !self.initialized {
            return Propagation::NotInitialized;
        }
        // Rejected ahead of the bookkeeping: a negative `dt` would wind the timers back
        // and a NaN would poison them.
        if !dt.is_usable_step() {
            return self.refuse_step(Propagation::InvalidStep { dt });
        }

        // Past here the time genuinely passed, so the health bookkeeping is real even
        // when the propagation itself is refused.
        self.diagnostics.advance(dt);

        let limit = self.config.max_predict_dt;
        if dt > limit {
            return self.refuse_step(Propagation::StepTooLong { dt, limit });
        }

        // Tested after the gap rather than before it, so that the gap is still measured:
        // `longest_refused` is the only record of how far the interval ran, and it
        // describes the timing whatever the sample holds. A sensor producing NaN produces
        // it again on the next step, where the count picks it up.
        if !imu.is_finite() {
            return self.refuse_step(Propagation::NotFinite);
        }

        // Propagated into a local first: (11)–(14) and (22) can both overflow f32 on a
        // finite sample, and a state written before it is checked is one the filter has
        // already published.
        let propagated = propagate(self.state, self.covariance, imu, dt, &self.config.imu);
        if !propagated.is_finite() {
            return self.refuse_step(Propagation::StateNotFinite);
        }
        self.state = propagated.state;
        self.commit_covariance(propagated.covariance);
        self.note_alignment();
        Propagation::Propagated
    }

    /// Store a covariance, applying the diagonal floor of equation (42′) and counting what it
    /// raised.
    ///
    /// One place, so that the invariant is a property of the filter rather than of each
    /// equation that produces a `P`: **every covariance the filter commits has been floored**,
    /// and therefore so has every covariance it fuses against, tests [`Validity`] against, or
    /// reads back into the `S` of (24). Committed, not held: an `Eskf` that has not
    /// initialized holds [`Covariance::zero`](crate::Covariance::zero) and
    /// [`covariance`](Self::covariance) will hand it over, which is the one place a caller can
    /// read a variance of zero off this filter. Nothing acts on it — every path that would is
    /// behind `initialized` — and initializing is itself a commit.
    ///
    /// A floor applied inside (22) and (27) instead would be two call sites protecting the
    /// two operations that shrink a variance, and would leave the ones that *write* one —
    /// the adoption of
    /// [`Fusion::Reset`](crate::Fusion::Reset), the `reset_*_to` methods, the initial
    /// covariance of (8) — unprotected, each carrying a variance that came from outside the
    /// filter.
    ///
    /// Symmetry stays where the algebra is, in `propagate_covariance` and `reparameterize`:
    /// (42)'s two halves answer different faults. `½(P + Pᵀ)` repairs drift a product
    /// introduces, so it belongs to the product; the floor bounds a value, so it belongs to
    /// the value.
    fn commit_covariance(&mut self, covariance: Covariance) {
        let mut matrix = *covariance.as_matrix();
        let raised = floor_diagonal(&mut matrix);
        self.covariance = Covariance::from_matrix(matrix);
        self.diagnostics.floored = self.diagnostics.floored.saturating_add(raised);
    }

    /// Record a refused step and hand the outcome back, as [`refuse`] does for a
    /// measurement.
    ///
    /// Same reason: one place maps an outcome to what
    /// [`PropagationHealth`](crate::PropagationHealth) counts, so a guard added to
    /// [`predict`](Self::predict) cannot forget to count it. Whether the timers moved is
    /// the guard's business, not this one's.
    fn refuse_step(&mut self, outcome: Propagation) -> Propagation {
        self.diagnostics.propagation.record(outcome);
        outcome
    }

    /// Commit what an update produced and record it against its source, handing the outcome
    /// back. Equations (23)–(41) are `update`'s; this is only the bookkeeping.
    ///
    /// One place for the reason [`refuse`] is one place: every `fuse_*` that runs an update
    /// ends here, so none can commit a state without the covariance that goes with it, record
    /// an acceptance without restarting the timer, or forget [`note_alignment`] — which an
    /// update owes as much as a reset does, since a position fix narrows the attitude block
    /// through the correlations (17) builds.
    ///
    /// [`note_alignment`]: Self::note_alignment
    fn apply(
        &mut self,
        outcome: Update,
        source: fn(&mut Diagnostics) -> &mut SourceHealth,
    ) -> Fusion {
        match outcome {
            Update::Accepted {
                state,
                covariance,
                ratio,
                innovation,
            } => {
                self.state = state;
                self.commit_covariance(covariance);
                source(&mut self.diagnostics).record_accepted(ratio, Some(innovation));
                self.note_alignment();
                Fusion::Accepted { test_ratio: ratio }
            }
            Update::Rejected { ratio, innovation } => {
                source(&mut self.diagnostics).record_rejected(ratio, innovation);
                Fusion::Rejected { test_ratio: ratio }
            }
            Update::Invalid => refuse(source(&mut self.diagnostics), Fusion::StateInvalid),
        }
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
    /// `noise` is the receiver's own accuracy where it reports one, bounded before it
    /// arrives: [`PositionNoise::clamped`](crate::PositionNoise::clamped) takes `eph` and
    /// `epv` and holds each between a floor and a cap, for the reasons recorded there. A
    /// two-dimensional fix instead goes through
    /// [`PositionNoise::horizontal_vertical`](crate::PositionNoise::horizontal_vertical),
    /// which leaves the vertical σ where the caller put it: `clamped` caps both axes, so
    /// it would turn a declined height back into a measurement.
    ///
    /// The filter applies no bound of its own, because `R` describes the measurement and
    /// belongs with it rather than in [`Config`]. A caller handing over a raw `eph` is
    /// therefore trusting the receiver further than either production autopilot does.
    ///
    /// A fix inconsistent with the estimate at [`Gates::gnss_position`](crate::Gates) is
    /// [`Fusion::Rejected`] and changes nothing but the source's health. The test is joint
    /// over all three axes; see [`Gates`](crate::Gates) for why.
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
        let observation = gnss::position_observation(&self.state, position, noise);
        let outcome = update(
            &self.state,
            &self.covariance,
            &observation,
            self.config.gates.gnss_position,
        );
        self.apply(outcome, |diagnostics| &mut diagnostics.gnss_position)
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
        self.diagnostics.gnss_position.record_accepted(0.0, None);
        Fusion::Accepted { test_ratio: 0.0 }
    }

    /// Fuse a GNSS velocity solution. Equation (29).
    ///
    /// After a coarse start the first solution is adopted rather than fused; see
    /// [`Fusion::Reset`].
    ///
    /// `noise` is the receiver's speed accuracy, `sacc`, bounded the way
    /// [`fuse_gnss_position`](Self::fuse_gnss_position)'s is:
    /// [`VelocityNoise::clamped`](crate::VelocityNoise::clamped). Where the solution
    /// carries no usable vertical velocity, or the receiver reports the axes separately,
    /// [`VelocityNoise::horizontal_vertical`](crate::VelocityNoise::horizontal_vertical)
    /// is the constructor to reach for: all three axes are fused or none, and a per-axis
    /// σ is what says which of them the receiver actually measured.
    ///
    /// A solution inconsistent with the estimate at
    /// [`Gates::gnss_velocity`](crate::Gates) is [`Fusion::Rejected`] and changes nothing
    /// but the source's health. One joint test over all three axes, as (28)'s is.
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
        let observation = gnss::velocity_observation(&self.state, velocity, noise);
        let outcome = update(
            &self.state,
            &self.covariance,
            &observation,
            self.config.gates.gnss_velocity,
        );
        self.apply(outcome, |diagnostics| &mut diagnostics.gnss_velocity)
    }

    /// Fuse a barometric altitude. Equation (30).
    ///
    /// The measurement is `z = -(α - α₀)`, so it needs a reference: one
    /// [`initialize`](Self::initialize) derived from a window taken at rest, or one
    /// [`set_baro_reference`](Self::set_baro_reference) named. Without one the measurement
    /// is refused rather than referred to an invented origin — which is what a start in
    /// motion, or a window with no barometer in it, leaves behind.
    ///
    /// `noise` being per call is what lets it carry a condition neither platform's
    /// parameter can: ArduPilot multiplies the barometer variance by 4 in ground effect
    /// (`gndEffectBaroScaler`, `AP_NavEKF3.h:511`, applied at
    /// `AP_NavEKF3_PosVelFusion.cpp:1419`), and PX4 runs a deadzone around the same
    /// condition. An application here inflates σ on the calls it applies to, and says in
    /// its own code when that is.
    ///
    /// An altitude inconsistent with the estimate at
    /// [`Gates::baro_altitude`](crate::Gates) is [`Fusion::Rejected`] and changes nothing
    /// but the source's health. One degree of freedom, so the threshold is a `Gate<1>` and
    /// a `Gate<3>` in that field does not compile.
    ///
    /// There is no adoption here, unlike the two GNSS observations: a height is not a
    /// quantity this source can establish. `α₀` is what relates the barometer to the
    /// navigation origin, and a start that left position unestablished is exactly a start
    /// that fixed no `α₀` either, so the measurement is refused above rather than adopted.
    /// A caller that names a reference with
    /// [`set_baro_reference`](Self::set_baro_reference) after a coarse start is claiming
    /// the two are related, and the altitude then corrects an unestablished position
    /// through the gate like any other measurement — against a covariance wide enough to
    /// accept it.
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
        let Some(reference) = self.baro_reference else {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::NoReference);
        };
        let observation = baro::altitude_observation(&self.state, altitude, reference, noise);
        let outcome = update(
            &self.state,
            &self.covariance,
            &observation,
            self.config.gates.baro_altitude,
        );
        self.apply(outcome, |diagnostics| &mut diagnostics.baro_altitude)
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
    /// widening the prior does not fix it. It is adopted rather than gated, which is
    /// [`Fusion::Reset`]'s documented shape — a quantity never established, with no
    /// estimate to step away from. Gating it instead would judge the measurement against
    /// a prior nothing ever measured: a static window with no magnetometer leaves
    /// [`sigma_yaw`](crate::Initialization::sigma_yaw), 0.35 rad, on a yaw that is a
    /// guess, and a correct heading more than about 64° from that guess would be
    /// rejected — locking the filter out of the one source that can ever establish yaw.
    /// Once per quantity, never for recovery.
    ///
    /// Ordinary updates thereafter, gated at [`Gates::mag_heading`](crate::Gates) with
    /// one degree of freedom. The field is reduced to a scalar heading before the gate
    /// sees it, so a disturbance is tested as the yaw error it is.
    ///
    /// The field must be calibrated: this crate corrects no hard- or soft-iron error and
    /// carries no magnetic-field states to absorb one, so a bias in `field` is a bias in
    /// heading. A field of exactly zero is finite, and reports the vehicle as `D_m` off
    /// rather than producing NaN — gateable once yaw is established, and adopted where it
    /// is not, which is the strongest reason to check a magnetometer before the first
    /// call rather than after it.
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
        let observation = mag::heading_observation(
            &self.state,
            &self.covariance,
            field,
            self.config.magnetic_declination,
            noise,
        );
        if self.unestablished.heading {
            // The adoption reads the same `y` and the same `R` an ordinary update would.
            // (36′) is what makes that worth saying: the levelling error is priced on the
            // path where the tilt it comes from is worst.
            self.reset_heading_by(observation.y[0], observation.r_m[0]);
            self.diagnostics.mag_heading.record_adopted();
            self.note_alignment();
            return Fusion::Reset;
        }
        let outcome = update(
            &self.state,
            &self.covariance,
            &observation,
            self.config.gates.mag_heading,
        );
        self.apply(outcome, |diagnostics| &mut diagnostics.mag_heading)
    }

    /// Turn the estimate by a yaw error and give the result the measurement's variance:
    /// the adoption behind [`Fusion::Reset`] for heading.
    ///
    /// `y` is the innovation of (35), so the corrected attitude is `Exp(y e₃) ⊗ q̂` —
    /// composed on the **left**, because `e₃` is the navigation down axis, where the
    /// `δθ` of (2) that `update` injects is a body-frame rotation composed on the right.
    /// Tilt is untouched: a rotation about navigation down moves the tilt axis and not
    /// the tilt angle, so the roll and pitch gravity established survive a heading the
    /// magnetometer supplies.
    ///
    /// `Exp` charges its caller with a finite argument, which (35)'s wrap discharges by
    /// construction: `y` is in `(-π, π]` whatever the field and the attitude were.
    ///
    /// `variance` is `R` from (36′) rather than the caller's `σ_ψ²` alone. The levelling
    /// of (34) is done with the estimated attitude on this path too — on a coarse start,
    /// with the worst tilt the filter ever holds — so an adoption that stored the
    /// magnetometer's own number would report a heading good to
    /// [`Accuracy::heading`](crate::Accuracy::heading) while carrying the window's
    /// levelling error times `tan δ`. That is the falsely-valid attitude (36′) exists to
    /// remove, and the adoption is where it is largest.
    ///
    /// The correlations go with it, which is what fusing against an infinitely uncertain
    /// prior converges to — the same limit [`reset_position_to`](Self::reset_position_to)
    /// takes, one component wide. The yaw axis is `AttitudeZ`, the body-frame error
    /// component, while the rotation above is about navigation down: the two agree at zero
    /// tilt and differ by the `1/cos θ` equation (36) already carries.
    ///
    /// What survives — the tilt block, the biases, and the correlations between them — is
    /// reparameterized first, by the (41) of `update::reparameterize`. `δθ` is referenced
    /// to the nominal's body axes, so `R(q̂⁺)ᵀ R(q̂)` is the exact change of frame this
    /// rotation makes, and it is not the small one (41) approximates: a first heading can
    /// turn the estimate by half a circle. The tilt block is near-isotropic and largely
    /// survives it, but the bias blocks are in physical body axes that do **not** turn, so
    /// their correlations with attitude transform on one side only — left unrotated, a
    /// roll-error/gyro-bias-x correlation is read afterwards as roll-error/gyro-bias-y and
    /// the next velocity update pushes the correction into the wrong axis.
    fn reset_heading_by(&mut self, y: f32, variance: f32) {
        let before = self.state.attitude.quaternion().to_rotation_matrix();
        let mut corrected = exp_quat(Vector3::z() * y) * self.state.attitude.quaternion();
        corrected.renormalize();
        self.state.attitude = Attitude::body_to_ned(corrected);

        let g_theta = corrected.to_rotation_matrix().inverse() * before;
        let mut covariance =
            update::reparameterize(self.covariance.as_matrix(), g_theta.into_inner());
        covariance.reset_state(ErrorState::AttitudeZ, variance);
        self.commit_covariance(covariance);
        self.unestablished.heading = false;
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
        state.status = self.derive_status();
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

    /// Whether the attitude has **ever** met [`ALIGNED_TILT`] and [`ALIGNED_HEADING`] since
    /// initialization — the test behind [`Status::Aligning`].
    ///
    /// Promotion is measured, not timed: it reads the covariance against those two bars, and
    /// holds heading to the rule [`Validity`] does, so it stays false on a vehicle with no
    /// magnetometer — a heading nothing ever observed is not aligned however tight
    /// [`sigma_yaw`](crate::Initialization::sigma_yaw) is. It does not read
    /// [`Config::accuracy`](crate::Config::accuracy): how good the attitude must be for the
    /// mission and whether the start has been resolved are two questions, and a bar serving
    /// both has to be wrong for one of them. What it does not do is fall back:
    /// alignment is an event, "the start has been resolved", where
    /// [`validity`](Self::validity) is the live question, "is tilt good enough right now".
    ///
    /// Conflating the two is what a corpus replay showed costs: with the bar read live, the
    /// handled log `7592c9b2` flaps `Healthy`/`Aligning` four times in four seconds and every
    /// static log ends `Aligning`. Not because anything degraded — because a body-frame
    /// attitude covariance **rotates** with the body (equation (20)), so a 20° yaw prior
    /// becomes partly a roll-and-pitch prior as the vehicle turns, and back again. That is
    /// honest about tilt right now, which is [`Validity`]'s job, and useless as a report that
    /// the filter has not finished starting up. PX4 and ArduPilot both latch it for the same
    /// reason: `tilt_align` and `tiltAlignComplete` are only ever tested while false
    /// (`src/modules/ekf2/EKF/control.cpp:73-78` at `c4e4ef98e9`,
    /// `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp:520-525` at `368dc0c428`).
    pub const fn is_aligned(&self) -> bool {
        self.aligned
    }

    /// Latch [`is_aligned`](Self::is_aligned) if the attitude now meets the bar.
    ///
    /// Called from every path that can move the attitude covariance or establish heading.
    /// A latch is state, so it cannot be derived on read the way [`Status`] is — and reading
    /// it lazily would make the answer depend on whether anyone asked.
    fn note_alignment(&mut self) {
        if self.initialized && !self.aligned {
            self.aligned = self.tilt_within(&self.covariance, ALIGNED_TILT)
                && self.heading_within(&self.covariance, ALIGNED_HEADING);
        }
    }

    /// Which parts of the estimate are good enough to use, right now.
    ///
    /// Also carried on [`State::validity`](crate::State::validity), which is where most
    /// callers will meet it.
    pub fn validity(&self) -> Validity {
        self.validity_of(&self.covariance)
    }

    /// [`validity`](Self::validity)'s question asked of any covariance, not only the one the
    /// filter is holding.
    ///
    /// One definition, two covariances: the current one, and the one
    /// [`predicted_validity`](Self::predicted_validity) projects to its horizon. Reading the
    /// verdict off one and re-deriving the geometry for the other would be two
    /// implementations of a single claim — the mistake `false_valid` made in the replay
    /// harness, where a 2-D norm stood in for the per-axis test this actually performs.
    fn validity_of(&self, p: &Covariance) -> Validity {
        if !self.initialized {
            return Validity::NONE;
        }
        let accuracy = &self.config.accuracy;
        let position = accuracy.position.as_meters();
        let velocity = accuracy.velocity.as_m_per_s();

        Validity {
            tilt: self.tilt_within(p, accuracy.tilt),
            heading: self.heading_within(p, accuracy.heading),
            // A quantity nothing ever established is not valid however tight the prior on
            // it looks: nobody set that number.
            horizontal_position: !self.unestablished.position
                && within(p, ErrorState::PositionNorth, position)
                && within(p, ErrorState::PositionEast, position),
            vertical_position: !self.unestablished.position
                && within(p, ErrorState::PositionDown, position),
            horizontal_velocity: !self.unestablished.velocity
                && within(p, ErrorState::VelocityNorth, velocity)
                && within(p, ErrorState::VelocityEast, velocity),
            vertical_velocity: !self.unestablished.velocity
                && within(p, ErrorState::VelocityDown, velocity),
        }
    }

    /// Whether roll and pitch are both within `bar`, one standard deviation per axis.
    ///
    /// Shared by [`validity`](Self::validity) and the alignment latch, which ask it against
    /// different bars; one definition is what keeps the two claims the same shape.
    fn tilt_within(&self, p: &Covariance, bar: Radians) -> bool {
        let sigma = bar.as_radians();
        within(p, ErrorState::AttitudeX, sigma) && within(p, ErrorState::AttitudeY, sigma)
    }

    /// Whether heading has been established and is within `bar`.
    ///
    /// A heading nothing observed fails whatever its variance: stillness never observes yaw,
    /// and a prior on a yaw nobody measured is not an estimate of one.
    fn heading_within(&self, p: &Covariance, bar: Radians) -> bool {
        !self.unestablished.heading && within(p, ErrorState::AttitudeZ, bar.as_radians())
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
    /// So a quantity counts here if it survives the horizon, **or** if a source that
    /// constrains it is currently being accepted. The two halves answer the two ways an
    /// arming check can be wrong. The projection is the pessimistic half: `P` is propagated
    /// [`Accuracy::horizon`](crate::Accuracy::horizon) forward with nothing fusing, by the
    /// (16)–(22) `predict` itself runs, and each quantity is tested at the far end — so a
    /// tilt that is inside its bar now and will not be in a second reads false here and
    /// true from [`validity`](Self::validity). The aiding clause is the optimistic half,
    /// and it is what a projection cannot supply: before the first fix, horizontal position
    /// has no estimate to propagate and the fact that fixes are arriving is the whole
    /// answer. `pred_horiz_pos_rel` in ArduPilot's status word is that clause; PX4 has no
    /// equivalent, and neither publishes the projection.
    ///
    /// Tilt is where the projection earns its place, because nothing aids it: a static
    /// window brings it in and (20)'s gyroscope-bias term takes it back out, on a schedule
    /// the covariance knows and no acceptance timer does. At [`ImuNoise`](crate::ImuNoise)'s
    /// defaults an unaided start holds tilt for 3.82 s, so a horizon under that arms and one
    /// over it does not — an answer, where before there was only the current value repeated.
    ///
    /// The projection reads slightly optimistic and the amount is measured: a first-order
    /// step understates growth, and `propagate.rs`'s `PROJECTION_STEP` holds that within
    /// 2.5 % of the sigma out to a 5 s horizon. It costs one `F` and up to 64 covariance
    /// propagations, which is an arming-rate query and not something to poll at IMU rate.
    pub fn predicted_validity(&self) -> Validity {
        if !self.initialized {
            return Validity::NONE;
        }
        let horizon = project(
            &self.state,
            self.covariance,
            self.config.accuracy.horizon,
            &self.config.imu,
        );
        let ahead = self.validity_of(&horizon);

        let fresh =
            |source: SourceHealth| source.accepted_within(self.config.timeouts.degraded_after);
        let d = &self.diagnostics;
        let (position, velocity) = (fresh(d.gnss_position), fresh(d.gnss_velocity));
        let height = position || fresh(d.baro_altitude);

        Validity {
            // Gravity is not an aiding source the filter tracks, so tilt has only the
            // projection to speak for it -- which is the one quantity where that is the
            // whole answer rather than half of it.
            tilt: ahead.tilt,
            heading: ahead.heading || fresh(d.mag_heading),
            horizontal_position: ahead.horizontal_position || position,
            vertical_position: ahead.vertical_position || height,
            horizontal_velocity: ahead.horizontal_velocity || velocity,
            vertical_velocity: ahead.vertical_velocity || velocity,
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
        let mut covariance = self.covariance;
        covariance.reset_block(
            [
                ErrorState::PositionNorth,
                ErrorState::PositionEast,
                ErrorState::PositionDown,
            ],
            noise.variance().into(),
        );
        self.commit_covariance(covariance);
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
        let mut covariance = self.covariance;
        covariance.reset_block(
            [
                ErrorState::VelocityNorth,
                ErrorState::VelocityEast,
                ErrorState::VelocityDown,
            ],
            noise.variance().into(),
        );
        self.commit_covariance(covariance);
        self.unestablished.velocity = false;
        true
    }

    /// Commit an alignment: take the nominal state equation (7) built, and reset the
    /// covariance and health for a fresh start whose attitude uncertainty matches how good
    /// the alignment was. The barometric reference is the caller's to set, because only it
    /// knows whether this start establishes a new one.
    ///
    /// A start the window showed at rest clears the origin, on the same evidence that
    /// establishes its position: both are the claim *zero is here*, and an origin held
    /// from before says zero is somewhere else. The next geodetic fix places a new one.
    /// A start taken in motion keeps it, because its position is unestablished and the
    /// first fix is adopted about the origin the flight already has. The two move
    /// together or a still short window would report an established position of `(0,0,0)`
    /// about an origin nothing put under it.
    fn apply_alignment(
        &mut self,
        alignment: Alignment,
        state: State,
        measured: &Measured,
        settled: bool,
    ) {
        // The bias of (7) as committed, so that what it absorbed is not charged a second
        // time as motion the window could not vouch for; see `init::coarse_sigmas`.
        let (sigma_tilt, sigma_yaw) =
            init::attitude_sigmas(&self.config.init, alignment, measured, state.gyro_bias);
        let covariance = init::initial_covariance(&self.config.init, sigma_tilt, sigma_yaw);
        self.state = state;
        // Before the commit, which counts into them; see `initialize_from`.
        self.diagnostics = Diagnostics::default();
        self.commit_covariance(covariance);
        self.unestablished = Unestablished::after(settled, measured.field.is_some());
        if settled {
            self.origin = None;
        }
        self.initialized = true;
        // A fresh start is unaligned until its own covariance says otherwise, which
        // `note_alignment` reads at the end of each entry point.
        self.aligned = false;
    }

    /// Aggregate alignment and the per-source timers into one status, most severe first.
    ///
    /// Only sources that have ever been accepted count toward aiding: a vehicle with no
    /// magnetometer is not permanently `Degraded` for lacking one.
    ///
    /// Alignment enters as the latch of [`is_aligned`](Self::is_aligned) rather than as the
    /// live [`Validity`], which is the one thing here that is not derived on read.
    fn derive_status(&self) -> Status {
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
        } else if !self.aligned {
            Status::Aligning
        } else if fresh == used {
            Status::Healthy
        } else {
            Status::Degraded
        }
    }
}

impl Unestablished {
    /// What a start leaves unestablished, read off what its window showed rather than off
    /// which [`Alignment`] it earned.
    ///
    /// `settled` is [`init::at_rest`]'s verdict. A vehicle that held still through the
    /// window is where the origin says it is and is not moving, which is the whole of
    /// what a static start ever claimed about position and velocity — and a window too
    /// short to align an attitude from claims it just as honestly, since
    /// [`init::classify`] reports a short window as [`Coarse::WindowTooShort`] before it
    /// ever measures motion. A window taken in motion establishes neither: the vehicle
    /// passed through somewhere the filter cannot name. Those wait for the first fix.
    ///
    /// Heading needs a magnetometer on top of stillness. Gravity pins tilt and nothing
    /// pins the rotation about it, so a window carrying no field leaves yaw a prior
    /// however long and however still it was —
    /// [`Initialization::sigma_yaw`](crate::Initialization::sigma_yaw) is 0.35 rad
    /// against an [`Accuracy::heading`](crate::Accuracy::heading) of 0.52, so the
    /// covariance alone would report a yaw nobody measured as good.
    const fn after(settled: bool, observed_field: bool) -> Self {
        Self {
            position: !settled,
            velocity: !settled,
            heading: !(settled && observed_field),
        }
    }
}

/// Whether one error state's variance is within `sigma`, one standard deviation on that axis.
///
/// A free function rather than a method, because the covariance it reads is an argument: the
/// filter asks this of the one it holds and of the one
/// [`Eskf::predicted_validity`] projects, and a method taking `&self` would quietly answer
/// for the wrong one.
fn within(p: &Covariance, state: ErrorState, sigma: f32) -> bool {
    p.variance(state) <= sigma * sigma
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Accuracy, GRAVITY};
    use crate::geodetic::LocalOrigin;
    use crate::health::Refusal;
    use crate::init::tests::{gravity_at, still, turning};
    use crate::observation::mag::tests::{attitude_of, measured};
    use crate::state::ErrorState;
    use crate::state::STATES;
    use crate::units::{Acceleration, AngularRate, Radians};

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

    /// The floor of (42′) exists to be unreachable, and this is the only place CI asserts
    /// it: `data/fetch.sh --check` pins `floored=0` on all five corpus logs, and it needs a
    /// network and PX4 tooling, so it runs locally. The margin between the floor and
    /// anything a filter that is propagating and fusing reaches is measured in `math.rs`'s
    /// `FLOOR` — a count here means the floor is masking a collapse rather than preventing
    /// one, and the `sigma_*` columns of `examples/replay.rs` say which state.
    #[test]
    fn an_ordinary_run_never_reaches_the_floor() {
        let mut filter = aided();
        for step in 0..400 {
            assert_eq!(filter.predict(still().imu, DT), Propagation::Propagated);
            if step % 25 == 0 {
                assert!(
                    filter
                        .fuse_gnss_position(
                            Position::ned(0.0, 0.0, 0.0),
                            PositionNoise::horizontal_vertical(1.5, 1.5),
                        )
                        .is_accepted()
                );
                assert!(
                    filter
                        .fuse_baro_altitude(
                            Altitude::from_meters(100.0),
                            AltitudeNoise::from_sigma(2.0),
                        )
                        .is_accepted()
                );
            }
        }
        assert_eq!(filter.diagnostics().floored, 0);
    }

    fn elapsed(filter: &Eskf) -> f32 {
        filter
            .diagnostics()
            .baro_altitude
            .time_since_accepted
            .expect("baro has been accepted")
            .as_secs()
    }

    /// The margin [`Accuracy`]'s defaults were chosen for, measured rather than derived: a
    /// static start with a magnetometer in the window holds its tilt for 3.82 s of unaided
    /// propagation and its heading for 35.8 s, at [`ImuNoise`](crate::ImuNoise)'s defaults.
    ///
    /// Not the `σ_g² t` the white-noise density alone would give — that is 10.4 s and 674 s.
    /// The gyroscope-bias prior reaches attitude through (20)'s `−I Δt` and accumulates as
    /// `σ_βg² t²`, which overtakes the white-noise term within two seconds and is what actually
    /// sets both figures. A test rather than a comment because [`Accuracy`] cites the numbers:
    /// change a bar or a density and this says by how much the margin moved.
    #[test]
    fn an_unaided_start_holds_its_attitude_for_the_margin_the_defaults_buy() {
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.initialize(&window_with_mag(), Seconds::from_secs(0.25)),
            Ok(Alignment::Static)
        );

        let dt = Seconds::from_secs(0.005);
        let holding_still = ImuSample {
            accel: Acceleration::body(0.0, 0.0, -GRAVITY),
            ..ImuSample::default()
        };
        let (mut tilt_held, mut heading_held) = (None, None);

        for step in 1..8_000 {
            assert_eq!(filter.predict(holding_still, dt), Propagation::Propagated);
            let elapsed = step as f32 * dt.as_secs();
            let validity = filter.validity();
            if tilt_held.is_none() && !validity.tilt {
                tilt_held = Some(elapsed);
            }
            if heading_held.is_none() && !validity.heading {
                heading_held = Some(elapsed);
                break;
            }
        }

        let tilt = tilt_held.expect("tilt leaves the bar inside 40 s");
        let heading = heading_held.expect("heading leaves the bar inside 40 s");
        assert!((tilt - 3.82).abs() < 0.05, "tilt held {tilt} s");
        assert!((heading - 35.8).abs() < 0.2, "heading held {heading} s");
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

    /// The `Eskf`-level check that (16)–(22) are wired at all: a step grows the uncertainty it
    /// was initialized with. Before this stage a static start held `Initialization`'s sigmas
    /// for the whole flight.
    #[test]
    fn a_step_grows_the_covariance() {
        let mut filter = initialized();
        let before = *filter.covariance();

        assert_eq!(
            filter.predict(ImuSample::default(), DT),
            Propagation::Propagated
        );

        let after = filter.covariance();
        for state in [
            ErrorState::PositionNorth,
            ErrorState::VelocityNorth,
            ErrorState::AttitudeX,
            ErrorState::GyroBiasX,
        ] {
            assert!(
                after.variance(state) > before.variance(state),
                "{state:?}: {} did not grow from {}",
                after.variance(state),
                before.variance(state),
            );
        }
    }

    /// A refused step commits neither half. The state and the covariance advance together or
    /// not at all: a state stored beside the covariance of a different step reports an estimate
    /// whose uncertainty describes something else, which is worse than the refusal it replaces.
    ///
    /// The seed is finite and its diagonal positive, so `initialize_from` accepts it, and one
    /// step of `F P Fᵀ` then leaves f32's range — the covariance's version of the overflow
    /// (11)–(14) already had.
    #[test]
    fn an_overflowing_covariance_commits_neither_half() {
        let mut filter = Eskf::new(Config::default());
        let seed = State {
            velocity: Velocity::ned(1.0, 2.0, 3.0),
            ..State::default()
        };
        let enormous = Covariance::from_matrix(
            crate::state::CovarianceMatrix::from_diagonal_element(f32::MAX),
        );
        assert_eq!(
            filter.initialize_from(seed, enormous),
            Ok(Alignment::Seeded)
        );

        assert_eq!(
            filter.predict(ImuSample::default(), DT),
            Propagation::StateNotFinite
        );
        assert_eq!(filter.state().velocity, seed.velocity);
        assert_eq!(*filter.covariance(), enormous);
        assert_eq!(filter.diagnostics().propagation.refused_state_not_finite, 1);
    }

    /// The `Eskf`-level check that (11) is wired at all: a sample reporting *no* specific
    /// force is a vehicle in free fall, whatever its attitude, because `a_n = R(q̂) 0 + g`
    /// is gravity in any frame. One step and the estimate is falling at `γ Δt`.
    ///
    /// Written on the default sample rather than a plausible one for that reason — the
    /// answer does not depend on what `aided()` happened to level to.
    #[test]
    fn a_step_with_no_specific_force_leaves_the_estimate_falling() {
        let mut filter = aided();
        assert_eq!(
            filter.predict(ImuSample::default(), DT),
            Propagation::Propagated
        );
        let velocity = filter.state().velocity.vector();
        let free_fall = GRAVITY * DT.as_secs();
        assert!((velocity.z - free_fall).abs() < 1e-6, "{velocity:?}");
        assert!(
            velocity.x.abs() < 1e-6 && velocity.y.abs() < 1e-6,
            "{velocity:?}"
        );
    }

    /// A finite sample whose propagation overflows f32 is refused, and the estimate the
    /// filter keeps is the last one that was a number.
    ///
    /// It takes accumulation rather than one step — `f32::MAX` of specific force is
    /// `1.7e36` of velocity over 5 ms — so the loop runs until the refusal rather than
    /// asserting on a step count, and the comparison is against the state immediately
    /// before it.
    #[test]
    fn a_propagation_that_overflows_is_refused_and_the_estimate_is_left_alone() {
        let mut filter = aided();
        let imu = ImuSample {
            gyro: AngularRate::body(0.0, 0.0, 0.0),
            accel: Acceleration::body(f32::MAX, 0.0, -GRAVITY),
        };
        assert!(imu.is_finite());

        let mut before = filter.state();
        let mut steps = 0;
        loop {
            match filter.predict(imu, DT) {
                Propagation::Propagated => {
                    before = filter.state();
                    steps += 1;
                    assert!(steps < 10_000, "never overflowed");
                }
                Propagation::StateNotFinite => break,
                other => panic!("unexpected outcome {other:?}"),
            }
        }

        assert_eq!(
            filter.state(),
            before,
            "a poisoned state reached the filter"
        );
        assert!(filter.state().is_finite());
        assert_eq!(
            filter.diagnostics().propagation.refused_state_not_finite,
            1,
            "the refusal went uncounted"
        );
        // The refused step counts too: the time passed. Compared with a tolerance because
        // the timer accumulates 0.005 a couple of hundred times in f32 while the
        // right-hand side multiplies once.
        let expected = DT.as_secs() * (steps + 1) as f32;
        assert!(
            (elapsed(&filter) - expected).abs() < 1e-3,
            "{} vs {expected}: a refused step still happened in real time",
            elapsed(&filter)
        );
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
    fn an_altitude_above_the_reference_pulls_the_estimate_up_and_moves_nothing_sideways() {
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize(&window_with_baro([100.0; 8]), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");

        // Two metres above the reference the window fixed, on a sensor claiming 0.5 m.
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(102.0), AltitudeNoise::from_sigma(0.5))
                .is_accepted()
        );

        let position = filter.state().position.vector();
        assert!(
            position[2] < -1.0,
            "up is negative down: {} should be near -2 m",
            position[2]
        );
        assert_eq!(
            (position[0], position[1]),
            (0.0, 0.0),
            "(30) observes p_D alone, and a still start correlates it with nothing"
        );
    }

    #[test]
    fn an_altitude_the_gate_turns_down_leaves_the_estimate_where_it_was() {
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize(&window_with_baro([100.0; 8]), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let before = filter.state().position;

        // A hundred metres of climb the instant the window closed, on a 0.5 m sensor: the
        // shape of a pressure transient, and what `Gates::baro_altitude` is there for.
        let outcome =
            filter.fuse_baro_altitude(Altitude::from_meters(200.0), AltitudeNoise::from_sigma(0.5));
        assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        assert_eq!(filter.state().position, before);
        assert_eq!(filter.diagnostics().baro_altitude.rejected, 1);
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

        // At the reference, so this asserts that fusion resumed rather than what the gate
        // of (37) makes of an altitude. The seed holds p_D = 0 at σ = 0.5 m, so the 60 m
        // asked for above would be 8 m of innovation on a 2 m sensor — ε = 15.1 against
        // the 10.83 of `Gate::<1>::at(P999)` — and is turned down on its merits.
        assert!(filter.set_baro_reference(Altitude::from_meters(52.0)));
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(52.0), AltitudeNoise::from_sigma(2.0))
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
    fn a_seed_below_the_floor_of_42_is_refused_like_a_zero() {
        // 1e-30 (rad/s)² is a gyroscope bias the seed claims to know to 1e-15 rad/s. It
        // clears a `> 0` test and does exactly what a zero does — the gain is zero in f32
        // and `validity` calls the quantity good on the first read — so the bar is the
        // floor, not zero, and the two are one check rather than two 24 decades apart.
        let (state, covariance) = seed();
        let mut matrix = *covariance.as_matrix();
        matrix[(ErrorState::GyroBiasZ.index(), ErrorState::GyroBiasZ.index())] = 1e-30;

        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.initialize_from(state, Covariance::from_matrix(matrix)),
            Err(InitError::InvalidVariance)
        );
        assert!(!filter.is_initialized(), "a refused seed leaves no state");

        // Just above its own floor is accepted, and untouched: the bar is per state group,
        // so a gyroscope bias at 1e-8 passes a floor of 1e-9 while a position at 1e-8 would
        // not clear its own 1e-6.
        let mut matrix = *covariance.as_matrix();
        matrix[(ErrorState::GyroBiasZ.index(), ErrorState::GyroBiasZ.index())] = 1e-8;
        let _ = filter
            .initialize_from(state, Covariance::from_matrix(matrix))
            .expect("above the floor");
        assert_eq!(filter.covariance().variance(ErrorState::GyroBiasZ), 1e-8);
        assert_eq!(filter.diagnostics().floored, 0);

        let mut matrix = *covariance.as_matrix();
        matrix[(
            ErrorState::PositionNorth.index(),
            ErrorState::PositionNorth.index(),
        )] = 1e-8;
        assert_eq!(
            filter.initialize_from(state, Covariance::from_matrix(matrix)),
            Err(InitError::InvalidVariance)
        );
    }

    #[test]
    fn a_reset_below_the_floor_is_floored_and_counted_rather_than_refused() {
        // The seed is refused because it writes the covariance in whole; a reset writes one
        // block against an estimate that exists, so the floor repairs it instead. This is
        // what keeps `floored` reachable from the public API at all now that
        // `initialize_from` turns the other path away.
        let mut filter = aided();
        assert_eq!(filter.diagnostics().floored, 0);

        assert!(filter.reset_position_to(
            Position::ned(10.0, 20.0, -5.0),
            PositionNoise::from_sigma(1e-15, 1e-15, 1e-15),
        ));

        assert_eq!(filter.diagnostics().floored, 3);
        assert!(filter.covariance().variance(ErrorState::PositionNorth) >= 1e-6);
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

        let _ = filter
            .initialize(&turning(0.0, 0.4, 0.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        let tilt = filter.covariance().variance(ErrorState::AttitudeX);
        assert!(
            (tilt - 0.8 * 0.8).abs() < 1e-4,
            "the covariance carries the widened tilt, got sigma^2 {tilt}"
        );
    }

    /// A still window of a vehicle parked at this attitude, reading nothing but gravity.
    fn window_tilted(roll: f32, pitch: f32) -> [StaticSample; 8] {
        [StaticSample {
            imu: ImuSample {
                accel: gravity_at(roll, pitch, 0.0),
                ..still().imu
            },
            ..still()
        }; 8]
    }

    #[test]
    fn a_static_window_commits_the_attitude_it_levelled() {
        // Equations (5)–(7) are `init`'s to test; this is that the filter starts at the
        // attitude they computed rather than level.
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter
                .initialize(&window_tilted(0.25, -0.1), Seconds::from_secs(0.25))
                .expect("a parked vehicle reads exactly g, however it is standing"),
            Alignment::Static
        );

        let (roll, pitch, yaw) = filter.state().attitude.euler_angles();
        assert!(
            (roll - 0.25).abs() < 1e-5 && (pitch + 0.1).abs() < 1e-5,
            "levelled to ({roll}, {pitch})"
        );
        assert_eq!(yaw, 0.0, "no magnetometer observed the rotation about it");
    }

    #[test]
    fn the_configured_declination_reaches_the_heading_it_commits() {
        // The wiring no test in `init` can see. A level vehicle reading a field with no
        // east component is pointing at magnetic north, so its true heading is the
        // declination and nothing else.
        let mut filter = Eskf::new(Config {
            magnetic_declination: Radians::from_radians(-0.06),
            ..Config::default()
        });
        let _ = filter
            .initialize(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");

        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw + 0.06).abs() < 1e-6, "heading committed as {yaw}");
    }

    #[test]
    fn a_still_window_commits_its_gyroscope_bias_and_a_moving_one_does_not() {
        let offset = AngularRate::body(0.01, -0.02, 0.003);
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu.gyro = offset;
        }

        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter
                .initialize(&window, Seconds::from_secs(0.25))
                .expect("0.022 rad/s is well inside the tolerance"),
            Alignment::Static
        );
        assert!(
            (filter.state().gyro_bias.vector() - offset.vector()).norm() < 1e-7,
            "{:?}",
            filter.state().gyro_bias
        );

        // Over the stationarity tolerance, so the same average is the vehicle turning
        // rather than the sensor lying, and taking it would subtract a turn rate from
        // every later measurement as though it were a sensor error.
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        let _ = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert_eq!(filter.state().gyro_bias, AngularRate::zero());
    }

    #[test]
    fn one_sample_is_levelled_like_a_window_of_one() {
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize_coarse(ImuSample {
                accel: gravity_at(0.0, 0.35, 0.0),
                ..still().imu
            })
            .expect("a finite sample");

        let (roll, pitch, _) = filter.state().attitude.euler_angles();
        assert!(
            roll.abs() < 1e-6 && (pitch - 0.35).abs() < 1e-5,
            "levelled to ({roll}, {pitch})"
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
        let mut window = turning(0.0, 0.4, 0.0);
        // No magnetometer, so heading stays a prior whatever the covariance says.
        for sample in &mut window {
            sample.mag = None;
        }
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

        // Once is once: there is now an estimate for a gate to judge against, and the same
        // fix again is fused against it rather than adopted a second time.
        let outcome = filter.fuse_gnss_position(fix, PositionNoise::horizontal_vertical(1.5, 1.5));
        assert!(matches!(outcome, Fusion::Accepted { .. }), "{outcome:?}");
        assert_eq!(filter.diagnostics().gnss_position.adopted, 1);
    }

    #[test]
    fn a_fix_that_agrees_with_the_estimate_narrows_its_uncertainty() {
        let mut filter = initialized();
        let before = filter.covariance().variance(ErrorState::PositionNorth);
        let outcome = filter.fuse_gnss_position(
            Position::ned(0.5, -0.5, 0.2),
            PositionNoise::horizontal_vertical(1.0, 1.0),
        );

        let Fusion::Accepted { test_ratio } = outcome else {
            panic!("expected an acceptance, got {outcome:?}");
        };
        assert!(test_ratio > 0.0 && test_ratio <= 1.0);
        assert!(filter.covariance().variance(ErrorState::PositionNorth) < before);
        assert!(filter.state().position.x() > 0.0, "moved toward the fix");
        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.test_ratio, Some(test_ratio));
        assert!(health.innovation.is_some());
    }

    #[test]
    fn a_rejected_fix_changes_nothing_but_the_sources_record_of_it() {
        let mut filter = initialized();
        let noise = PositionNoise::horizontal_vertical(1.0, 1.0);
        assert!(
            filter
                .fuse_gnss_position(Position::ned(0.1, 0.0, 0.0), noise)
                .is_accepted()
        );
        assert!(filter.predict(still().imu, DT).is_propagated());
        let (state, covariance) = (filter.state(), *filter.covariance());
        let timer = filter.diagnostics().gnss_position.time_since_accepted;

        let outcome = filter.fuse_gnss_position(Position::ned(1000.0, 0.0, 0.0), noise);
        assert!(matches!(outcome, Fusion::Rejected { test_ratio } if test_ratio > 1.0));
        assert_eq!(filter.state(), state);
        assert_eq!(filter.covariance(), &covariance);
        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.time_since_accepted, timer);
        assert_eq!((health.rejected, health.consecutive_rejections), (1, 1));
        assert_eq!(health.test_ratio, outcome.test_ratio());
    }

    #[test]
    fn a_covariance_that_is_no_longer_one_refuses_the_update() {
        let mut filter = initialized();
        let mut p = *filter.covariance().as_matrix();
        p[(0, 0)] = -10.0;
        filter.covariance = Covariance::from_matrix(p);
        let state = filter.state();

        let outcome = filter.fuse_gnss_position(
            Position::ned(0.1, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.0, 1.0),
        );
        assert_eq!(outcome, Fusion::StateInvalid);
        assert_eq!(filter.state(), state);
        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.last_refusal, Some(Refusal::StateInvalid));
        assert_eq!((health.accepted, health.rejected), (0, 0));
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

    /// Alignment does not fall back: once the attitude has met the bar, a covariance that
    /// grows past it takes `Validity::tilt` away and leaves [`Status`] alone.
    ///
    /// Both halves matter, and the second is the one a latch could get wrong by reporting a
    /// filter as started-up while its outputs are unusable. The live flag is what a
    /// controller branches on, and it goes false here.
    #[test]
    fn a_covariance_growing_past_the_bar_ends_validity_but_not_alignment() {
        // Aided *and* aligned: the baro gives a source timer to keep the status out of
        // `DeadReckoning`, and the magnetometer is what makes heading an estimate.
        let window = [StaticSample {
            baro: Some(Altitude::from_meters(100.0)),
            mag: Some(MagField::body(0.22, 0.0, 0.44)),
            ..still()
        }; 8];
        let mut filter = Eskf::new(Config::default());
        assert_eq!(
            filter.initialize(&window, Seconds::from_secs(0.25)),
            Ok(Alignment::Static)
        );
        assert!(
            filter
                .fuse_baro_altitude(Altitude::from_meters(100.0), AltitudeNoise::from_sigma(2.0))
                .is_accepted()
        );
        assert!(filter.is_aligned());
        assert!(filter.validity().tilt);

        // Past the 3.82 s the default bars buy, with the barometer still arriving at 2 Hz so
        // that aiding is never stale: otherwise `degraded_after` expires on the way and
        // `Degraded` would be what the status assertion below saw.
        for step in 1..=800 {
            assert_eq!(
                filter.predict(
                    ImuSample {
                        accel: Acceleration::body(0.0, 0.0, -GRAVITY),
                        ..ImuSample::default()
                    },
                    Seconds::from_secs(0.005),
                ),
                Propagation::Propagated
            );
            if step % 100 == 0 {
                assert!(
                    filter
                        .fuse_baro_altitude(
                            Altitude::from_meters(100.0),
                            AltitudeNoise::from_sigma(2.0)
                        )
                        .is_accepted()
                );
            }
        }

        assert!(
            !filter.validity().tilt,
            "tilt has grown past Accuracy::tilt"
        );
        assert!(
            filter.is_aligned(),
            "the start was resolved and stays resolved"
        );
        assert_eq!(filter.state().status, Status::Healthy);
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
        // The covariance on its own would say otherwise: `Initialization::sigma_yaw` is
        // 0.35 rad against an `Accuracy::heading` of 0.52, so the prior clears the bar
        // comfortably. It is a prior on a yaw nothing ever observed.
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
    fn the_first_magnetic_heading_is_adopted_and_every_one_after_it_is_fused() {
        // The exception is bounded at one, per quantity, which is the whole of what
        // separates it from the self-recovery GOALS.md rules out.
        let mut filter = initialized();
        let field = measured(attitude_of(0.0, 0.0, 1.1), 0.0);
        let noise = HeadingNoise::from_sigma(0.05);

        assert!(filter.fuse_mag_heading(field, noise).is_reset());
        assert_eq!(filter.diagnostics().mag_heading.adopted, 1);

        let second = filter.fuse_mag_heading(field, noise);
        assert!(!second.is_reset(), "adoption happens once: {second:?}");
        assert!(matches!(second, Fusion::Accepted { .. }), "{second:?}");
        assert_eq!(filter.diagnostics().mag_heading.adopted, 1);
    }

    #[test]
    fn the_adopted_yaw_is_the_heading_the_field_reports() {
        // The window levelled with no magnetometer, so yaw starts at zero against a
        // field that says 1.1 rad. A gradual correction would leave it somewhere between
        // the two; an adoption puts it at the measurement.
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    measured(attitude_of(0.0, 0.0, 1.1), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw - 1.1).abs() < 1e-4, "adopted {yaw} rather than 1.1");
    }

    #[test]
    fn the_adoption_turns_the_estimate_about_down_and_touches_nothing_else() {
        // A heading is yaw and nothing else: the tilt gravity established and the biases
        // the window measured are the same after it, and so are their variances. What
        // makes this worth asserting is that the rotation is applied to the whole
        // quaternion — about navigation down, where it moves the tilt axis and leaves
        // the tilt angle — rather than to a yaw component.
        let mut filter = initialized();
        let before = filter.state();
        let (roll, pitch, _) = before.attitude.euler_angles();
        let tilt_variance = filter.covariance().variance(ErrorState::AttitudeX);

        assert!(
            filter
                .fuse_mag_heading(
                    measured(attitude_of(0.0, 0.0, -2.4), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );

        let after = filter.state();
        let (roll_after, pitch_after, _) = after.attitude.euler_angles();
        assert!((roll_after - roll).abs() < 1e-5, "roll moved");
        assert!((pitch_after - pitch).abs() < 1e-5, "pitch moved");
        assert_eq!(after.gyro_bias, before.gyro_bias);
        assert_eq!(after.accel_bias, before.accel_bias);
        // Within an ulp rather than exactly: the adoption reparameterizes the attitude
        // block by (41), and a 2.4 rad rotation of an isotropic `v I` reconstructs `v`
        // through `v(c² + s²)`. A tolerance wide enough to hide a heading's variance
        // landing on the tilt would be four orders of magnitude wider than this.
        assert!(
            (filter.covariance().variance(ErrorState::AttitudeX) - tilt_variance).abs() < 1e-9,
            "the tilt's own uncertainty is not the heading's to replace"
        );
    }

    #[test]
    fn the_adoption_carries_the_attitude_correlations_into_the_new_body_axes() {
        // (41) on the adoption path. The error state of (2) is referenced to the
        // nominal's body axes, so turning the nominal turns the axes every attitude row
        // is written in — and the bias states it is correlated with are in physical body
        // axes that do not turn, so their cross-blocks transform on one side only.
        //
        // Propagation is what builds one: (20)'s `−I Δt` makes P_θ,βg ≈ −Δt·P_βg,βg, the
        // same correlation that lets a velocity update correct the gyroscope bias. A
        // quarter turn about a level vehicle's down axis maps the roll-error column onto
        // the pitch-error one, so the correlation the x bias had with roll error is the
        // one the y bias should hold afterwards. Skip the reparameterization and it stays
        // where it was, and the next velocity update pushes the bias correction into the
        // wrong axis.
        let mut filter = initialized();
        let holding_still = ImuSample {
            accel: Acceleration::body(0.0, 0.0, -GRAVITY),
            ..ImuSample::default()
        };
        for _ in 0..2_000 {
            assert_eq!(
                filter.predict(holding_still, Seconds::from_secs(0.005)),
                Propagation::Propagated
            );
        }

        let before = filter
            .covariance()
            .get(ErrorState::AttitudeX, ErrorState::GyroBiasX);
        assert!(
            before.abs() > 1e-9,
            "10 s of (20) should build one: {before}"
        );
        assert!(
            filter
                .covariance()
                .get(ErrorState::AttitudeX, ErrorState::GyroBiasY)
                .abs()
                < 1e-12,
            "and nothing across the axes yet"
        );

        assert!(
            filter
                .fuse_mag_heading(
                    measured(attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );

        let after = filter.covariance();
        assert!(
            (after.get(ErrorState::AttitudeX, ErrorState::GyroBiasY) - before).abs()
                < 1e-9 * before.abs().max(1.0),
            "the quarter turn should have moved it to the y bias, got {}",
            after.get(ErrorState::AttitudeX, ErrorState::GyroBiasY)
        );
        assert!(
            after
                .get(ErrorState::AttitudeX, ErrorState::GyroBiasX)
                .abs()
                < 1e-9,
            "and off the x bias, got {}",
            after.get(ErrorState::AttitudeX, ErrorState::GyroBiasX)
        );
    }

    #[test]
    fn a_refused_first_heading_establishes_nothing() {
        // A refusal is not a measurement the filter chose to believe, so the quantity is
        // still unestablished and the next usable field is still the one adopted.
        let mut filter = initialized();
        assert_eq!(
            filter.fuse_mag_heading(
                MagField::body(f32::NAN, 0.0, 0.44),
                HeadingNoise::from_sigma(0.05),
            ),
            Fusion::NotFinite
        );
        assert!(!filter.validity().heading);
        assert_eq!(filter.diagnostics().mag_heading.adopted, 0);

        assert!(
            filter
                .fuse_mag_heading(
                    measured(attitude_of(0.0, 0.0, 1.1), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset(),
            "the adoption was waiting for a usable field, not spent on a refused one"
        );
    }

    #[test]
    fn a_heading_far_from_an_established_yaw_is_rejected_and_changes_nothing() {
        // Once yaw is established the gate has a prior worth testing against, and this
        // is the disturbance case: a field turned a quarter circle against a σ of
        // 0.05 rad is hundreds of times the threshold of (37).
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );
        let before = filter.state().attitude;

        let outcome = filter.fuse_mag_heading(
            measured(attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2), 0.0),
            HeadingNoise::from_sigma(0.05),
        );
        assert!(
            matches!(outcome, Fusion::Rejected { test_ratio } if test_ratio > 1.0),
            "{outcome:?}"
        );
        assert_eq!(filter.state().attitude, before, "a rejection costs nothing");
        assert_eq!(filter.diagnostics().mag_heading.rejected, 1);
    }

    #[test]
    fn an_ordinary_heading_moves_yaw_part_of_the_way_and_shrinks_its_variance() {
        // The other side of the adoption: once established, a heading is fused rather
        // than taken. The distance it moves is (25)'s gain with (36′) in both `P` and
        // `R`, and the whole sum is checkable by hand — the window's tilt prior is
        // `sigma_tilt` = 0.02 and the dip is 2.0, so the levelling is worth
        // 2.0² · 0.02² = 0.0016 on every heading here. The adoption left yaw at
        // 0.05² + 0.0016 = 0.0041, the second field arrives with σ = 0.02, so
        // `R` = 0.02² + 0.0016 = 0.0020 and `K = 0.0041 / (0.0041 + 0.0020)` = 0.672:
        // a 0.05 rad disagreement moves 0.0336.
        //
        // The two σ differ on purpose. Equal ones make `P` and `R` equal, `K` exactly a
        // half, and the test blind to (36′) being dropped from either side — which is
        // what it is here to notice. Unpriced in `R` this moves 0.0456, unpriced in both
        // 0.0431.
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );

        assert!(
            filter
                .fuse_mag_heading(
                    measured(attitude_of(0.0, 0.0, 0.05), 0.0),
                    HeadingNoise::from_sigma(0.02),
                )
                .is_accepted()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!(
            (yaw - 0.0336).abs() < 1e-3,
            "expected 0.0336 by the arithmetic above, got {yaw}"
        );
        assert!(
            filter.covariance().variance(ErrorState::AttitudeZ) < 0.0041,
            "a fused heading leaves yaw better known than the adoption did"
        );
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
    fn a_moving_start_takes_its_heading_from_the_first_magnetometer() {
        // This window's halves level 0.4 rad apart, and the dip scales that into 0.78 rad
        // of yaw — half as much again as `Accuracy::heading`, so the start is worth no
        // heading at all. The first field is adopted rather than fused: a prior nothing
        // measured is replaced, not averaged with.
        let mut filter = Eskf::new(Config::default());
        let alignment = filter
            .initialize(&turning(0.0, 0.4, 0.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(!filter.validity().heading, "the window was moving");

        let field = MagField::body(0.22, 0.0, 0.44);
        let noise = HeadingNoise::from_sigma(0.1);
        assert!(filter.fuse_mag_heading(field, noise).is_reset());

        // (36′) on the adoption path, which is where the term is worth the most: (34)
        // levels with a tilt whose variance is 0.64 here, and the dip carries that into
        // the heading. 0.582 rather than the measurement's own 0.01 — σ = 0.76 rad,
        // outside `Accuracy::heading`. Storing the magnetometer's number alone is what
        // makes the filter claim an attitude it does not have, on the one path where the
        // tilt doing the levelling is worst.
        let yaw_variance = filter.covariance().variance(ErrorState::AttitudeZ);
        assert!(
            (yaw_variance - 0.582).abs() < 1e-2,
            "expected the levelling priced in, got {yaw_variance}"
        );
        assert!(
            !filter.validity().heading,
            "a heading levelled by this tilt is not good to Accuracy::heading"
        );
        assert!(!filter.validity().tilt, "the tilt is still the window's");

        // Established all the same, which is the other half of the claim: the quantity
        // has been observed, so the next field is fused rather than adopted. Established
        // and good are two questions and only the covariance answers the second.
        assert!(matches!(
            filter.fuse_mag_heading(field, noise),
            Fusion::Accepted { .. }
        ));
    }

    #[test]
    fn a_still_short_window_calls_its_heading_valid_at_once() {
        // #85. The same field, the same stillness, and the only thing wrong with the
        // window is its length — which `classify` reports before it measures motion, so
        // the coarse verdict says nothing about whether the vehicle moved. It did not.
        let mut filter = Eskf::new(Config::default());
        let alignment = filter
            .initialize(&window_with_mag(), Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert!(matches!(
            alignment,
            Alignment::Coarse(Coarse::WindowTooShort { .. })
        ));
        assert!(
            filter.validity().heading,
            "a still window observed a heading whatever its length"
        );
        assert!(filter.is_aligned(), "and nothing is left to resolve");
    }

    #[test]
    fn a_still_short_window_establishes_where_it_sat() {
        // The other half of #85, on the same evidence: a vehicle that held still was
        // where the origin says and was not moving, so there is an estimate for the first
        // fix to be gated against rather than adopted over.
        let mut filter = Eskf::new(Config::default());
        let _ = filter
            .initialize(&window_with_mag(), Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert!(filter.validity().horizontal_position);
        assert!(filter.validity().horizontal_velocity);
        let outcome = filter.fuse_gnss_position(
            Position::ned(0.2, -0.1, 0.0),
            PositionNoise::horizontal_vertical(1.5, 1.5),
        );
        assert!(
            !outcome.is_reset(),
            "adoption is for a start that established nothing"
        );
        assert!(outcome.is_accepted());
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
        // it is worthless at this instant. A σ of 0.6 rad is what makes it worthless: the
        // first heading is adopted and carries its own variance, so a measurement wider
        // than `Accuracy::heading` (0.5236) establishes the quantity without making it
        // good enough to fly on. That is the gap this method exists to report.
        assert!(
            filter
                .fuse_mag_heading(
                    MagField::body(0.22, 0.0, 0.44),
                    HeadingNoise::from_sigma(0.6),
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
        // Moving, so that nothing but the barometer has established anything: a window
        // taken at rest establishes its own position, short or not. A moving one
        // establishes no barometric reference either, so the application names it.
        let _ = filter
            .initialize(&moving_window_at(100.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(filter.set_baro_reference(Altitude::from_meters(100.0)));
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

    #[test]
    fn the_horizon_is_what_separates_predicted_validity_from_the_current_one() {
        // The projection's whole point, on the quantity nothing aids. A static start levels
        // tilt and holds it for 3.82 s unaided
        // (`an_unaided_start_holds_its_attitude_for_the_margin_the_defaults_buy`), so a
        // horizon inside that arms and one outside it does not -- while `validity` says the
        // same thing at both, because it is answering about now.
        let ask = |seconds: f32| {
            let config = Config {
                accuracy: Accuracy {
                    horizon: Seconds::from_secs(seconds),
                    ..Accuracy::default()
                },
                ..Config::default()
            };
            let mut filter = Eskf::new(config);
            let _ = filter
                .initialize(&window_with_mag(), Seconds::from_secs(0.25))
                .expect("a 2 s window of stillness");
            (filter.validity().tilt, filter.predicted_validity().tilt)
        };

        assert_eq!(ask(1.0), (true, true), "a second is inside the 3.82 s hold");
        assert_eq!(ask(6.0), (true, false), "six seconds is outside it");
    }

    #[test]
    fn a_horizon_of_zero_asks_what_validity_asks() {
        // The documented boundary: with nothing to project, the prediction is the current
        // answer widened only by the aiding clause. This is also what a caller who never
        // set a horizon would get if the default were zero, so it has to be honest rather
        // than accidental.
        let config = Config {
            accuracy: Accuracy {
                horizon: Seconds::from_secs(0.0),
                ..Accuracy::default()
            },
            ..Config::default()
        };
        let mut filter = Eskf::new(config);
        let _ = filter
            .initialize(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");

        let (now, predicted) = (filter.validity(), filter.predicted_validity());
        assert_eq!(now.tilt, predicted.tilt);
        assert_eq!(now.heading, predicted.heading);
        assert_eq!(now.vertical_position, predicted.vertical_position);
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
        let mut window = [still(); 8];
        window[3].imu.gyro = AngularRate::body(0.0, 0.4, 0.0);
        let _ = filter
            .initialize(&window, Seconds::from_secs(0.25))
            .expect("moving, so position is unestablished");
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
    fn the_origin_follows_stillness_and_not_the_alignment() {
        // Two halves of one claim, which have to move together: a window that held still
        // says zero is here, and that is the whole of what lets it report an established
        // position. A short one says it as honestly -- `classify` calls it coarse before
        // it measures motion -- so keying the origin on the verdict would leave such a
        // start reporting an established (0,0,0) about an origin nothing put under it.
        let mut filter = initialized();
        assert!(filter.set_origin(zurich()));
        let _ = filter
            .initialize(&[still(); 8], Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert_eq!(filter.origin(), None, "still, so zero is here now");
        assert!(filter.validity().horizontal_position);

        assert!(filter.set_origin(zurich()));
        let _ = filter
            .initialize(&turning(0.0, 0.4, 0.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(
            filter.origin().is_some(),
            "a restart in motion keeps the flight's origin"
        );
        assert!(
            !filter.validity().horizontal_position,
            "and says nothing about where it is until a fix is adopted about that origin"
        );
    }

    #[test]
    fn an_origin_off_the_earth_is_refused() {
        let mut filter = initialized();
        assert!(!filter.set_origin(Geodetic::from_degrees(91.0, 0.0, 0.0)));
        assert_eq!(filter.origin(), None);
    }

    #[test]
    fn a_mission_bar_tighter_than_the_prior_leaves_alignment_alone() {
        // `Accuracy` is the mission's question and `ALIGNED_*` the start's: a survey platform
        // asking for 1° of tilt and 10° of heading gets an attitude that is never valid
        // against a 20 mrad and 20° prior, and a filter that aligns anyway.
        let mut filter = Eskf::new(Config {
            accuracy: Accuracy {
                tilt: Radians::from_degrees(1.0),
                heading: Radians::from_degrees(10.0),
                ..Accuracy::default()
            },
            ..Config::default()
        });
        assert_eq!(
            filter.initialize(&window_with_mag(), Seconds::from_secs(0.25)),
            Ok(Alignment::Static)
        );

        assert!(filter.is_aligned(), "the start is resolved");
        let validity = filter.validity();
        assert!(
            !validity.tilt && !validity.heading,
            "and the mission bar is not met"
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
}
