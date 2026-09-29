//! The filter itself.

use crate::config::{ALIGNED_HEADING, ALIGNED_TILT, Config, ConfigError, Gate, LATENCY_HORIZON};
use crate::frames::{Body, Ned};
use crate::geodetic::{Geodetic, LocalOrigin};
use crate::health::{Diagnostics, Fusion, GnssFusion, Propagation, SourceHealth, Status, Validity};
use crate::history::History;
use crate::init::{self, Alignment, Coarse, InitError, Measured, StaticSample, StaticWindow};
use crate::math::{
    below_floor, correlation_inflation, exp_quat, floor_diagonal, floor_offset, wrap_pi,
};
use crate::observation::{baro, gnss, heading, mag};
use crate::propagate::{self, ImuSample, Propagated, project, propagate};
use crate::state::{AttitudeVariance, Covariance, ErrorState, Offset, State};
use crate::units::{
    Altitude, AltitudeNoise, AngularRate, Attitude, HeadingNoise, MagField, Position,
    PositionNoise, Radians, Seconds, Timestamp, Velocity, VelocityNoise,
};
use crate::update::{self, Observation, Update, update};
use nalgebra::Vector3;

/// A 15-state error-state Kalman filter.
///
/// Every equation the filter needs is implemented: initialization, (5)–(8), propagation,
/// (9)–(22), and the update of (23)–(41) for every source. The state dead reckons from
/// where [`initialize`](Self::initialize) put it, the covariance grows around it, and
/// [`fuse_gnss_position`](Self::fuse_gnss_position),
/// [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic),
/// [`fuse_gnss_velocity`](Self::fuse_gnss_velocity),
/// [`fuse_baro_altitude`](Self::fuse_baro_altitude),
/// [`fuse_mag_heading`](Self::fuse_mag_heading),
/// [`fuse_gnss_heading`](Self::fuse_gnss_heading) and
/// [`fuse_course`](Self::fuse_course) each correct both, or are turned down by the gate of
/// (37)–(38).
///
/// # Example
///
/// A whole flight's worth of calls, in the order they happen, with what each one is allowed to
/// answer. The README's quick start is the same loop without the assertions.
///
/// ```
/// use fusion_nav::prelude::*;
///
/// let mut filter = Eskf::default();
///
/// // A vehicle sitting still: no rotation, gravity the only specific force, read by a
/// // 400 Hz rate IMU whose clock counts microseconds.
/// let still = |sample: u64| {
///     ImuSample::from_rates(
///         Timestamp::from_micros(2_500 * sample),
///         AngularRate::body(0.0, 0.0, 0.0),
///         Acceleration::body(0.0, 0.0, -GRAVITY),
///         Seconds::from_secs(0.0025),
///     )
/// };
///
/// // Initialization reports what it achieved rather than refusing what it dislikes. A
/// // window that is short or moving gives `Alignment::Coarse`, and the filter runs and
/// // says `Status::Aligning` until attitude converges. 800 samples at 400 Hz is the 2 s
/// // `Initialization::min_duration` wants, folded in one at a time rather than buffered.
/// let mut window = StaticWindow::new();
/// for sample in 1..=800 {
///     window.push(StaticSample {
///         imu: still(sample),
///         ..StaticSample::default()
///     })?;
/// }
/// assert_eq!(filter.initialize(&window)?, Alignment::Static);
///
/// // Nothing in that window carried a barometer, so it fixes no reference altitude, and
/// // the first `fuse_baro_altitude` reads one from the estimate. See `StaticSample::baro`.
///
/// assert!(filter.predict(still(801)).is_propagated());
///
/// // Every measurement carries the time it was taken, on the IMU's clock: a receiver's fix
/// // describes where the vehicle was when it was computed, not when it arrived. These were
/// // all taken at the last sample.
/// let now = Timestamp::from_micros(2_500 * 801);
///
/// // GNSS in latitude and longitude. The filter holds the navigation origin: the first
/// // fix places it, under the estimate, so every later fix converts about the same point.
/// // `clamped` bounds the receiver's own `eph` and `epv` the way both autopilots do.
/// let outcome = filter.fuse_gnss_geodetic(
///     now,
///     Geodetic::from_degrees(47.397_742, 8.545_594, 488.0),
///     PositionNoise::clamped(
///         1.5,
///         3.0,
///         SigmaBounds::new(0.5, 100.0),
///         SigmaBounds::new(0.75, 100.0),
///     ), Position::zero(),
/// );
/// assert!(outcome.is_accepted());
/// assert!(filter.origin().is_some());
///
/// // A solution whose vertical velocity the receiver did not measure: the down axis
/// // carries a σ large enough that its gain is negligible, rather than a claim.
/// let outcome = filter.fuse_gnss_velocity(
///     now,
///     Velocity::ned(0.0, 0.0, 0.0),
///     VelocityNoise::horizontal_vertical(0.3, 1000.0), Position::zero(),
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
///     now,
///     MagField::body(0.22, 0.0, 0.44),
///     HeadingNoise::from_sigma(0.1),
/// );
/// assert!(outcome.is_accepted());
/// assert_eq!(filter.state().status, Status::Healthy);
///
/// // A vehicle without a magnetometer could have taken it from its course instead, once
/// // moving: one sitting still goes nowhere, and the direction of no velocity is refused.
/// let outcome = filter.fuse_course(now, HeadingNoise::from_sigma(0.05));
/// assert_eq!(outcome, Fusion::Unobservable);
/// # Ok::<(), fusion_nav::InitError>(())
/// ```
#[derive(Clone, Debug)]
pub struct Eskf {
    config: Config,
    state: State,
    covariance: Covariance,
    diagnostics: Diagnostics,
    baro_reference: Option<Altitude>,
    /// The covariance of the error in `baro_reference`, (30′). Zero while there is no
    /// reference; [`establish_reference`](Self::establish_reference) keeps the two together.
    offset: Offset,
    origin: Option<LocalOrigin>,
    /// `D_m` of equations (6) and (35); see
    /// [`set_magnetic_declination`](Self::set_magnetic_declination).
    declination: Radians,
    /// Whether the caller has set [`declination`](Self::declination), which the model the
    /// filter reads at each origin then never overrides.
    declination_set: bool,
    /// Whether the heading held is referred to true north through the declination alone:
    /// levelled from a window's magnetometer by (6) or adopted from `fuse_mag_heading`, with
    /// no true heading (a seed, a GNSS heading, a course) accepted since. While it is, a
    /// declination the filter learns turns the heading with it; see
    /// [`place_origin`](Self::place_origin).
    magnetic_north: bool,
    /// `ω` of equation (9) from the last step integrated from a sample; see
    /// [`angular_rate`](Self::angular_rate).
    angular_rate: Option<AngularRate<Body>>,
    /// The recent past of the state, for a measurement's age; see [`History`].
    history: History,
    /// The earliest time a measurement can be placed at: the start, or the epoch when the start
    /// was shown at rest and its state also describes the time before it. See
    /// [`admit`](Self::admit).
    earliest: Timestamp,
    unestablished: Unestablished,
    /// Whether the attitude has ever met [`ALIGNED_TILT`] and [`ALIGNED_HEADING`] since
    /// initialization. Latched, and the test behind [`Status::Aligning`]; see
    /// [`is_aligned`](Self::is_aligned) for why it is not read live.
    aligned: bool,
    initialized: bool,
    /// The clock; see [`time`](Self::time). Meaningless until `initialized`.
    time: Timestamp,
}

/// Quantities the start never established, which wait for the first measurement that
/// observes them: position and velocity for the first GNSS fix after a coarse start (see
/// [`Fusion::Reset`]), heading for the first heading any heading source offers.
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

impl Default for Eskf {
    /// A filter on [`Config::default`], which validates (a test holds it to that).
    fn default() -> Self {
        Self::with(Config::default())
    }
}

impl Eskf {
    /// Create an uninitialized filter, or refuse a [`Config`] with a value outside its bound
    /// ([`Config::validate`]). Measurements are refused with [`Fusion::NotInitialized`] until
    /// [`initialize`](Self::initialize) succeeds.
    pub fn new(config: Config) -> Result<Self, ConfigError> {
        config.validate()?;
        Ok(Self::with(config))
    }

    /// The filter [`new`](Self::new) returns, for a `config` already known to be valid.
    fn with(config: Config) -> Self {
        Self {
            config,
            state: State::default(),
            covariance: Covariance::zero(),
            diagnostics: Diagnostics::default(),
            baro_reference: None,
            offset: Offset::default(),
            origin: None,
            declination: Radians::ZERO,
            declination_set: false,
            magnetic_north: false,
            angular_rate: None,
            history: History::default(),
            earliest: Timestamp::ZERO,
            unestablished: Unestablished::default(),
            aligned: false,
            initialized: false,
            time: Timestamp::ZERO,
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

    /// The filter's clock: the [`time`](ImuSample::time) of the last sample
    /// [`predict`](Self::predict) was handed, or of the window or seed the filter started from.
    /// `None` until initialized.
    ///
    /// Every later sample is differenced against it and every measurement's age read from it.
    /// Only [`Propagation::InvalidStep`], a sample not after it, leaves it where it was; every
    /// other outcome moves it, integrated or not, since the time did pass. So it is when the
    /// estimate is valid only while the samples are being integrated: after a refused one the
    /// state is as old as the last that was.
    pub const fn time(&self) -> Option<Timestamp> {
        if self.initialized {
            Some(self.time)
        } else {
            None
        }
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
    /// [`validity`](Self::validity) withholds `heading` until the first heading from
    /// [`fuse_mag_heading`](Self::fuse_mag_heading),
    /// [`fuse_gnss_heading`](Self::fuse_gnss_heading) or [`fuse_course`](Self::fuse_course)
    /// is accepted. The covariance alone
    /// cannot say that: [`sigma_yaw`](crate::Initialization::sigma_yaw) is a prior on a
    /// number nobody measured.
    ///
    /// The window's span, which [`min_duration`](crate::Initialization::min_duration) is
    /// checked against, is the time its samples integrated; the filter's clock starts at the
    /// last sample's [`time`](ImuSample::time). As everywhere else, the filter never reads a
    /// clock.
    ///
    /// # Errors
    ///
    /// [`InitError::NoSamples`] for an empty window. A sample nothing can be made of is
    /// refused as it is [pushed](StaticWindow::push), so a window holds none.
    pub fn initialize(&mut self, window: &StaticWindow) -> Result<Alignment, InitError> {
        let measured = window.measured()?;
        let alignment = init::classify(&measured, &self.config.init);
        // One answer to "was the vehicle on the ground", read by the gyroscope bias of
        // (7), the barometric reference of (30), and what this start establishes.
        // Measured from the window rather than read off `alignment`, because a window too
        // short to align an attitude from can still be a window of a parked vehicle.
        let at_rest = init::at_rest(measured.peaks, &self.config.init);
        let state = init::nominal_state(&measured, self.declination, at_rest);
        self.apply_alignment(alignment, state, &measured, at_rest, measured.end)?;
        if at_rest {
            self.establish_reference(
                window
                    .alpha0()
                    .map(|(reference, variance)| (reference, Offset::independent(variance))),
            );
        }
        self.note_alignment();
        Ok(alignment)
    }

    /// What [`initialize`](Self::initialize) would make of this window, without touching
    /// the filter.
    ///
    /// For the application that would rather wait for stillness than start coarsely:
    /// restart the window, or slide a buffered one forward, until this reports
    /// [`Alignment::Static`], then commit; [`StaticWindow`] says which suits which.
    /// The filter cannot do that waiting itself — it does not know whether the vehicle is
    /// about to launch or has been sitting on the bench for an hour.
    ///
    /// # Errors
    ///
    /// As [`initialize`](Self::initialize).
    pub fn alignment_of(&self, window: &StaticWindow) -> Result<Alignment, InitError> {
        Ok(init::classify(&window.measured()?, &self.config.init))
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
    /// [`InitError::NotFinite`] if the sample carries a value that is not a number, and
    /// [`InitError::InvalidInterval`] for an integration interval under a microsecond.
    pub fn initialize_coarse(&mut self, imu: ImuSample) -> Result<Alignment, InitError> {
        // Treated as a window of one, so the same finiteness, motion and averaging
        // measures apply — an average of one sample being that sample. The window is most
        // of this frame (1984 bytes on `thumbv6m`), well under `update`'s.
        let mut window = StaticWindow::new();
        window.push(StaticSample {
            imu,
            ..StaticSample::default()
        })?;
        // A window of one: its own average, with no rotation to smear it and no second
        // velocity to difference against — a caller with GNSS in hand has a window, not
        // this entry point.
        let measured = window.measured()?;
        let alignment = Alignment::Coarse(Coarse::not_stationary(&measured));
        // The same rule `initialize` applies: the gyroscope bias is worth taking only
        // where the sample says the vehicle was on the ground, and one reading of a
        // stationary gyroscope is a noisier bias than a window's average but a better
        // one than zero.
        let at_rest = init::at_rest(measured.peaks, &self.config.init);
        let state = init::nominal_state(&measured, self.declination, at_rest);
        // That reading establishes nothing, which is why it is not passed on as one. A
        // window shows rest by holding still over a span of time and this one spans none:
        // an accelerometer reading `γ` for an instant is a hover as readily as a vehicle
        // on the ground, and this entry point exists for the launches that are moving.
        self.apply_alignment(alignment, state, &measured, false, imu.time)?;
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
    /// through [`body_to_ned`](crate::Attitude::from_body_to_ned), a ROS one through
    /// [`flu_to_enu`](crate::Attitude::from_flu_to_enu), which is where the conventions and
    /// what a wrong one costs are written down. It is the one error on this path no check
    /// downstream can reach.
    ///
    /// `state.status` is ignored: status is derived from aiding, never asserted.
    /// [`baro_reference`](Self::baro_reference) is left alone, so re-initializing in
    /// flight keeps the reference the flight began with; a filter that never had one takes
    /// it from the first altitude, read against the seeded position (see
    /// [`fuse_baro_altitude`](Self::fuse_baro_altitude)). [`origin`](Self::origin) is left
    /// alone for the same reason, and
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
    ///
    /// `time` is when the seed is valid, and starts the clock the next
    /// [`predict`](Self::predict) is differenced against.
    pub fn initialize_from(
        &mut self,
        state: State,
        covariance: Covariance,
        time: Timestamp,
    ) -> Result<Alignment, InitError> {
        if !state.is_finite() || !covariance.is_finite() {
            return Err(InitError::NotFinite);
        }
        if below_floor(covariance.as_matrix()) {
            return Err(InitError::InvalidVariance);
        }
        // Nothing a seed carries is unestablished: the caller vouched for every quantity,
        // heading included, so no first measurement overwrites one.
        self.start(state, covariance, Unestablished::default(), time, false);
        self.note_alignment();
        Ok(Alignment::Seeded)
    }

    /// Set the barometric reference `α₀` directly, with the σ it is known to. Equations (30)
    /// and (30′).
    ///
    /// For a reference known better than the estimate: a surveyed pad, or the ground
    /// re-established before takeoff. A filter with no reference needs none of this, since
    /// [`fuse_baro_altitude`](Self::fuse_baro_altitude) takes one from the estimate unless
    /// [`Config::baro_reference_from_estimate`](crate::Config::baro_reference_from_estimate)
    /// says otherwise. The filter estimates `α₀` from then on, starting from this σ and
    /// uncorrelated with the state, so `noise` is the claim that decides how far the first
    /// disagreement with GNSS height moves it.
    ///
    /// Returns `false`, changing nothing, for a reference that is not a number or a σ that is
    /// not positive: `α₀` appears in every barometric measurement for the rest of the flight,
    /// so a NaN here is not one bad update but the end of barometric aiding, and a σ of zero
    /// is a reference no disagreement could ever move.
    #[must_use = "a refused reference leaves barometric fusion returning NoReference"]
    pub fn set_baro_reference(&mut self, reference: Altitude, noise: AltitudeNoise) -> bool {
        if !reference.as_meters().is_finite() || !noise.is_finite() || !noise.is_positive() {
            return false;
        }
        self.establish_reference(Some((reference, Offset::independent(noise.variance()))));
        true
    }

    /// Set `α₀` together with the covariance of its error, or clear both. Equation (30′).
    ///
    /// One place, because the two describe one thing: a reference with no offset row claims
    /// to be exact, and an offset row with no reference correlates the estimate with nothing.
    fn establish_reference(&mut self, reference: Option<(Altitude, Offset)>) {
        match reference {
            Some((reference, offset)) => {
                self.baro_reference = Some(reference);
                self.commit_offset(offset);
            }
            None => {
                self.baro_reference = None;
                self.offset = Offset::default();
            }
        }
    }

    /// Store the offset's covariance, floored by (42′) as
    /// [`commit_covariance`](Self::commit_covariance) floors the rest — while there is a
    /// reference. Without one the offset is zero by definition and nothing reads it, so it is
    /// stored as zero rather than as what (30′)'s walk would have grown it to.
    fn commit_offset(&mut self, mut offset: Offset) {
        if self.baro_reference.is_none() {
            self.offset = Offset::default();
            return;
        }
        if floor_offset(&mut offset) {
            self.diagnostics.floored = self.diagnostics.floored.saturating_add(1);
        }
        self.offset = offset;
    }

    /// The offset a fresh covariance keeps: the reference's own variance, and no correlation
    /// with an error state that has just been replaced. A start that keeps the reference the
    /// flight had keeps its error too, since nothing about the reference changed.
    fn surviving_offset(&self) -> Offset {
        Offset::independent(self.offset.variance)
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
    /// position zero to be wherever the vehicle is. Called before, it still names the site,
    /// which is where the magnetic model is read (see
    /// [`set_magnetic_declination`](Self::set_magnetic_declination)): the window then levels
    /// its heading at the site's declination, and a heading only the magnetometer set is
    /// turned whenever a later origin changes it.
    ///
    /// Returns `false`, changing nothing, for an origin with a coordinate that is not a
    /// number or a latitude beyond ±90°, or one so far from the held position that it cannot
    /// be written in `f32` metres from the new origin.
    #[must_use = "a refused origin leaves the filter to place its own on the first fix"]
    pub fn set_origin(&mut self, origin: Geodetic) -> bool {
        let Some(new) = LocalOrigin::new(origin) else {
            return false;
        };
        if let Some(old) = self.origin {
            let position = new.to_ned(old.to_geodetic(self.state.position));
            // An origin further from the vehicle than `f32` reaches, such as a height of
            // 1e38 m, which `LocalOrigin::new` has no reason to refuse on its own.
            if !position.is_finite() {
                return false;
            }
            self.commit_state(State {
                position,
                ..self.state
            });
        }
        self.place_origin(new);
        true
    }

    /// Hold `origin` as the navigation origin, and read the site's declination from the
    /// magnetic model there unless the caller has set one. Every placement reads the model:
    /// [`set_origin`](Self::set_origin) and the coarse start's through here, and (44) through
    /// [`learn_declination`](Self::learn_declination) directly, at the fix.
    ///
    /// GOALS.md differentiator 7: the site is the one thing the model needs, and the origin is
    /// the moment the filter learns it, as PX4 learns it from its first valid fix
    /// (`updateWorldMagneticModel`, `EKF/aid_sources/magnetometer/mag_control.cpp:642-644` at
    /// `c4e4ef98`).
    ///
    /// A heading referred to north through the declination alone is turned by the change,
    /// about navigation down, since the heading (6) levelled or a magnetometer set was the
    /// magnetic heading plus the old value and is that plus the new one. A static start is
    /// the usual case: it levels before any fix names the site, and the first fix arrives a
    /// declination later. Left alone, that is a standing innovation of the whole change on
    /// every heading (13.8° at 56° N, 44° E), which the gate of (37) reads as a disturbed
    /// magnetometer and turns away until [`Config::recovery`](crate::Config::recovery)
    /// adopts one. The covariance is kept as it was: the turn composes on the left, so the
    /// body-frame error `δθ` of (2) is the same error before and after it, and the tilt a
    /// window levelled against its accelerometer bias by (8) keeps that correlation in the
    /// body axes it was built in. A heading any true source has vouched for is not turned,
    /// and the change arrives as an innovation, as a caller's does. GNSS position and velocity
    /// do not count as one, though they correct heading through the correlations while the
    /// vehicle accelerates: a first origin arrives before that matters, and a caller moving
    /// the origin mid-flight to a distant site sets the declination itself.
    fn place_origin(&mut self, origin: LocalOrigin) {
        self.learn_declination(origin.geodetic());
        self.origin = Some(origin);
    }

    /// What [`learn_declination`](Self::learn_declination) would do at `site`, committing
    /// nothing: the model's declination there, and the turn about navigation down it gives a
    /// heading referred to north through the declination alone (zero for any other). `None`
    /// where there is nothing to learn: a declination the caller set, or no model.
    fn declination_at(&self, site: Geodetic) -> Option<(Radians, f32)> {
        if self.declination_set {
            return None;
        }
        let declination = model_declination(site)?;
        let change = wrap_pi(declination.as_radians() - self.declination.as_radians());
        let turns = self.initialized && self.magnetic_north;
        Some((declination, if turns { change } else { 0.0 }))
    }

    /// The declination half of [`place_origin`](Self::place_origin), at `site`: for (44),
    /// which has to turn the heading before it reads the attitude it places the origin with.
    fn learn_declination(&mut self, site: Geodetic) {
        let Some((declination, turn)) = self.declination_at(site) else {
            return;
        };
        self.declination = declination;
        if turn != 0.0 {
            let mut turned = exp_quat(Vector3::z() * turn) * self.state.attitude.body_to_ned();
            turned.renormalize();
            self.commit_state(State {
                attitude: Attitude::from_body_to_ned(turned),
                ..self.state
            });
        }
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

    /// Set the magnetic declination at the operating site: `D_m` of equations (6) and (35),
    /// east-positive, the angle that turns a magnetic heading into a true one.
    ///
    /// Optional where the `magnetic-model` feature is on, as it is by default: the filter
    /// reads the declination from PX4's World Magnetic Model table wherever it places its
    /// [`origin`](Self::origin), and turns a heading only the magnetometer has referred to
    /// north along with it (see [`Geodetic::magnetic_declination`]). A value set here is the
    /// caller's and the model never overrides it; that is the call for a site the table
    /// describes badly or a date far from its epoch. Zero until one or the other.
    ///
    /// Held by the filter rather than [`Config`] because it is a property of where the vehicle
    /// is, like the [`origin`](Self::origin) and `α₀`, and a vehicle that powers on before a
    /// GNSS fix learns its site only once a fix arrives. PX4 tracks it at runtime from the last
    /// valid GNSS position (`EKF/estimator_interface.h:485` at `c4e4ef98`).
    ///
    /// Read wherever a magnetic heading becomes a true one: the heading a static window
    /// commits by (6), every [`fuse_mag_heading`](Self::fuse_mag_heading) by (35), and the
    /// heading adoption and recovery. So set it before [`initialize`](Self::initialize) when
    /// the site is known. It moves no state. A change once heading is established arrives as
    /// an innovation of exactly the change on the next heading; a large one against the
    /// heading's σ is [`Fusion::Rejected`] until
    /// [`Config::recovery`](crate::Config::recovery) adopts a heading at the new value.
    ///
    /// Returns `false`, changing nothing, for a value that is not a finite number: it enters
    /// every heading innovation for the rest of the flight, so a NaN here ends magnetic
    /// aiding rather than spoiling one update.
    #[must_use = "a refused declination leaves the previous one in every heading"]
    pub fn set_magnetic_declination(&mut self, declination: Radians) -> bool {
        if !declination.as_radians().is_finite() {
            return false;
        }
        self.declination = declination;
        self.declination_set = true;
        true
    }

    /// The magnetic declination in use; see
    /// [`set_magnetic_declination`](Self::set_magnetic_declination).
    pub const fn magnetic_declination(&self) -> Radians {
        self.declination
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

    /// Propagate the nominal state and covariance across one IMU sample. Equations (9)–(22).
    ///
    /// The hot path, called at IMU rate. The step is `Δt`, the time from the filter's
    /// [`time`](Self::time) to the sample's; the filter never reads a clock. The state
    /// integrates the sample's increments over their own intervals, and `Δt` is what
    /// everything that is about time passing reads: the health timers, and the test for a gap.
    ///
    /// A `Δt` longer than [`Config::max_predict_dt`](crate::Config::max_predict_dt) is not
    /// integrated: one IMU sample cannot describe a long interval, and propagating it anyway
    /// would put a number in the state that looks like an estimate and is not. The filter
    /// coasts across it instead, equation (22′), as [`Propagation::Coasted`]: position moves
    /// on the estimated velocity and the covariance grows by what
    /// [`Config::coast`](crate::Config::coast) allows, so the first fix after the gap is
    /// judged against an uncertainty that grew with it. With coasting off the step is refused,
    /// as [`Propagation::StepTooLong`]. Either way the timers advance, so [`Status`] degrades
    /// on schedule. It is `Δt` that is tested rather than an interval, because a gap is time
    /// no sample describes: a driver that integrated across a stall hands over an increment
    /// that does describe it, and a logger that dropped samples hands over one that does not.
    ///
    /// A sample no later than the filter's time is refused before the timers move at all,
    /// and leaves the clock where it was.
    ///
    /// A sample carrying a NaN or an infinity is refused too, as
    /// [`Propagation::NotFinite`]: propagating it would put the NaN in the quaternion and
    /// then in the covariance, where nothing reports it and it never leaves. So is one whose
    /// integration interval is under a microsecond or longer than `max_predict_dt`, as
    /// [`Propagation::InvalidInterval`]. The timers advance in both cases, since `Δt` was fine
    /// and only the sample was not.
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
    pub fn predict(&mut self, imu: ImuSample) -> Propagation {
        if !self.initialized {
            return Propagation::NotInitialized;
        }
        let dt = imu.time.since(self.time);
        // Rejected ahead of the bookkeeping: a negative `dt` would wind the timers back.
        if !dt.is_usable_step() {
            return self.refuse_step(Propagation::InvalidStep { dt });
        }

        // Past here the time genuinely passed, so the clock and the health bookkeeping are
        // real even when the propagation itself is refused.
        self.time = imu.time;
        self.diagnostics.advance(dt);

        let limit = self.config.max_predict_dt;
        if dt > limit {
            // Noted here, whatever becomes of the step: nothing else records how far the
            // interval ran, and a coast discarded as non-finite is a gap all the same.
            self.diagnostics.propagation.note_gap(dt);
            let Some(coast) = self.config.coast else {
                return self.refuse_step(Propagation::StepTooLong { dt, limit });
            };
            // The sample is not read, so a non-finite one does not stop a coast.
            let coasted = propagate::coast(
                self.state,
                self.covariance,
                self.offset,
                dt,
                &self.config.imu,
                self.config.baro_offset_walk,
                &coast,
            );
            return self.commit_step(coasted, Propagation::Coasted { dt });
        }

        // Tested after the gap rather than before it, so that a gap is still measured
        // whatever the sample holds. A sensor producing NaN produces it again on the next
        // step, where the count picks it up.
        if !imu.is_finite() {
            return self.refuse_step(Propagation::NotFinite);
        }
        if let Some(interval) = imu.unusable_interval() {
            return self.refuse_step(Propagation::InvalidInterval { interval });
        }
        // An interval past the limit is the gap test's case arriving as one increment, a
        // driver reporting milliseconds as seconds as readily as a stall: one sample does not
        // describe it, whichever of the two it was.
        if imu.longest_interval() > limit {
            let interval = imu.longest_interval();
            return self.refuse_step(Propagation::InvalidInterval { interval });
        }
        let propagated = propagate(
            self.state,
            self.covariance,
            self.offset,
            imu,
            &self.config.imu,
            self.config.baro_offset_walk,
        );
        self.commit_step(propagated, Propagation::Propagated)
    }

    /// Commit a propagated or coasted step, or discard it whole if it came out non-finite, and
    /// record the outcome.
    ///
    /// Propagated into a local first, a coast as much as a step: (11)–(14) and (22) can both
    /// overflow f32 on finite input, and a state written before it is checked is one the
    /// filter has already published. One tail for both, so neither can commit a state without
    /// its covariance or skip [`note_alignment`](Self::note_alignment). Taken by each branch
    /// rather than after them: carrying the step out of a branch as a value measured a
    /// 960-byte copy of `P` in `predict`'s frame.
    fn commit_step(&mut self, propagated: Propagated, outcome: Propagation) -> Propagation {
        if !propagated.is_finite() {
            return self.refuse_step(Propagation::StateNotFinite);
        }
        self.state = propagated.state;
        self.history.record(self.time, &self.state);
        self.angular_rate = propagated.omega;
        self.commit_covariance(propagated.covariance, propagated.offset);
        self.note_alignment();
        self.diagnostics.propagation.record(outcome);
        outcome
    }

    /// Store a corrected state, and correct the [`History`] by the same change.
    ///
    /// Every write of the state other than a propagation or a fresh start comes through here:
    /// an update, an adoption, a heading reset, a new origin. The past is only consistent with
    /// the present if each correction reaches both, and a writer that skipped it would leave
    /// the next old measurement to correct the same error a second time, which no test of the
    /// present state can see.
    fn commit_state(&mut self, state: State) {
        self.history.shift(&self.state, &state);
        self.state = state;
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
    fn commit_covariance(&mut self, covariance: Covariance, offset: Offset) {
        let mut matrix = *covariance.as_matrix();
        let raised = floor_diagonal(&mut matrix);
        self.covariance = Covariance::from_matrix(matrix);
        self.commit_offset(offset);
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
                offset,
                offset_correction,
                ratio,
                innovation,
            } => {
                self.commit_state(state);
                self.commit_covariance(covariance, offset);
                // (30′): `b` is the error in `α₀`, so its estimate comes off it.
                if let Some(reference) = self.baro_reference {
                    self.baro_reference = Some(Altitude::from_meters(
                        reference.as_meters() - offset_correction,
                    ));
                }
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

    /// [`apply`](Self::apply), unless the gate rejected a source locked out for `after`, in
    /// which case `adopt` takes the measurement instead: recovery from gate lockout, see
    /// [`Recovery`](crate::Recovery).
    ///
    /// One place, for the reason `apply` is one place: every source recovers through here, so
    /// none can adopt without counting it or forget [`note_alignment`](Self::note_alignment).
    ///
    /// Out of line for the reason [`observe`](Self::observe) is: inlined, the `Update` it takes
    /// sat in each `fuse_*` frame beneath `update::<3>`, `fuse_gnss_velocity` at 2384 bytes on
    /// `thumbv6m-none-eabi` against 1416. That frame into `update::<3>` is the crate's
    /// high-water mark, 9504.
    #[inline(never)]
    fn apply_or_recover(
        &mut self,
        outcome: Update,
        source: fn(&mut Diagnostics) -> &mut SourceHealth,
        after: Option<Seconds>,
        adopt: impl FnOnce(&mut Self),
    ) -> Fusion {
        let since_initialized = self.diagnostics.since_initialized;
        let locked_out = source(&mut self.diagnostics).locked_out(after, since_initialized);
        if !(matches!(outcome, Update::Rejected { .. }) && locked_out) {
            return self.apply(outcome, source);
        }
        adopt(self);
        source(&mut self.diagnostics).record_recovered();
        self.note_alignment();
        Fusion::Reset
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
    /// `epv` and holds each within its own floor and cap, for the reasons recorded there. A
    /// two-dimensional fix instead goes through
    /// [`PositionNoise::horizontal_vertical`](crate::PositionNoise::horizontal_vertical),
    /// which leaves the vertical σ where the caller put it: `clamped` caps both axes, so
    /// it would turn a declined height back into a measurement.
    ///
    /// The filter applies no bound of its own, because `R` describes the measurement and
    /// belongs with it rather than in [`Config`]. A caller handing over a raw `eph` is
    /// therefore trusting the receiver further than either production autopilot does.
    /// Nor does it smooth one: ArduPilot runs each accuracy through a decaying envelope with a
    /// 5 s time constant before bounding it (`AP_NavEKF3_Measurements.cpp:609-633` at
    /// `368dc0c4`), so a spike in `eph` deweights the fixes after it for seconds there and
    /// only its own fix here. The replay harness takes the same stance, for the reasons
    /// `data/README.md` gives under `r_policy=` (#105). What
    /// the filter does add is the receiver's rather than the fix's: a fix's error persists
    /// into the next one, and the update is computed at the variance that leaves, equation
    /// (24′), with the gate still reading `noise` itself. See
    /// [`Config::correlation`](crate::Config::correlation).
    ///
    /// The interval (24′) reads is the time since the previous fix fused through this method,
    /// so it assumes one receiver. Fixes from a second one, or from motion capture,
    /// interleaved with the first read as the same error arriving sooner and are deweighted
    /// though their errors are independent.
    ///
    /// The fix is two measurements, gated and reported apart: north and east at
    /// [`Gates::gnss_position`](crate::Gates), then down at
    /// [`Gates::gnss_height`](crate::Gates), against the state the first left. A half
    /// inconsistent with the estimate is [`Fusion::Rejected`] and changes nothing but its
    /// own health, so a height the estimate disagrees with costs no horizontal aiding; see
    /// [`GnssFusion`] for the measurement behind that. A half whose numbers are unusable is
    /// refused alone for the same reason — a 2D fix reporting `epv = 0` still fuses its
    /// horizontal position.
    ///
    /// The adoption after a coarse start is the exception, and takes the fix whole: it writes
    /// all three axes onto the covariance, so any unusable number refuses both halves.
    ///
    /// `antenna` is where the antenna sits relative to the IMU in body axes, forward, right,
    /// down ([`Position::body`](crate::Position)), and [`Position::zero`](crate::Position) for
    /// one on top of it. The fix is the antenna's, `p + R r` of (28′), and the estimate stays
    /// the IMU's. Unlike PX4, which subtracts `R̂ r` from the fix and keeps `H` as it was
    /// (`EKF/aid_sources/gnss/gps_control.cpp:351-354` at `c4e4ef98`), the update carries the
    /// arm's dependence on attitude, `−R̂[r]×`, so a fix observes heading through a long mast;
    /// see `observation/gnss.rs` for what that was measured against. An argument, as `noise`
    /// is and as PX4 carries it on each GNSS message (`antenna_offset_x/y/z`), because a
    /// second receiver has its own; from PX4's parameters it is `SENS_GPS0_OFF*` less
    /// `EKF2_IMU_POS*`.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_position(
        &mut self,
        time: Timestamp,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
        antenna: Position<Body>,
    ) -> GnssFusion {
        if let Err(refusal) = self.admit(time) {
            return self.refuse_gnss(refusal);
        }
        self.diagnostics.gnss_position.note_arrival(time);
        self.diagnostics.gnss_height.note_arrival(time);
        // An arm that is not a number spoils both halves, and an adoption would write it into
        // the state.
        if !antenna.is_finite() {
            return self.refuse_gnss(Fusion::NotFinite);
        }
        if self.unestablished.position {
            if !position.is_finite() || !noise.is_finite() {
                return self.refuse_gnss(Fusion::NotFinite);
            }
            if !noise.is_positive() {
                return self.refuse_gnss(Fusion::InvalidNoise);
            }
            let adopted = self.carried_position(position, antenna, time);
            self.adopt_position(adopted, noise, POSITION);
            self.unestablished.position = false;
            self.diagnostics.gnss_position.record_adopted();
            self.diagnostics.gnss_height.record_adopted();
            return GnssFusion::both(Fusion::Reset);
        }

        let (z, r) = (position.vector(), noise.variance());
        let horizontal = match screen(&[z[0], z[1]], &[r[0], r[1]]) {
            Some(refusal) => refuse(&mut self.diagnostics.gnss_position, refusal),
            None => {
                let observation = self
                    .observe(time, |past, _| {
                        gnss::horizontal_observation(past, position, noise, antenna)
                    })
                    .correlated(correlation_inflation(
                        self.diagnostics.gnss_position.since_measured,
                        self.config.correlation.gnss_position,
                    ));
                let outcome = update(
                    &self.state,
                    &self.covariance,
                    &self.offset,
                    &observation,
                    self.config.gates.gnss_position,
                );
                self.apply_or_recover(
                    outcome,
                    |diagnostics| &mut diagnostics.gnss_position,
                    self.config.recovery.gnss_position,
                    |filter| {
                        let adopted = filter.carried_position(position, antenna, time);
                        filter.adopt_position(adopted, noise, HORIZONTAL);
                    },
                )
            }
        };
        let height = match screen(&[z[2]], &[r[2]]) {
            Some(refusal) => refuse(&mut self.diagnostics.gnss_height, refusal),
            None => {
                let observation = self
                    .observe(time, |past, _| {
                        gnss::height_observation(past, position, noise, antenna)
                    })
                    .correlated(correlation_inflation(
                        self.diagnostics.gnss_height.since_measured,
                        self.config.correlation.gnss_height,
                    ));
                let outcome = update(
                    &self.state,
                    &self.covariance,
                    &self.offset,
                    &observation,
                    self.config.gates.gnss_height,
                );
                self.apply_or_recover(
                    outcome,
                    |diagnostics| &mut diagnostics.gnss_height,
                    self.config.recovery.gnss_height,
                    |filter| {
                        let adopted = filter.carried_position(position, antenna, time);
                        filter.adopt_height(adopted, noise);
                    },
                )
            }
        };
        GnssFusion { horizontal, height }
    }

    /// Whether a measurement taken at `time` can be fused, or the refusal that says why not:
    /// [`Fusion::NotInitialized`], or [`Fusion::OutOfHorizon`] for one older than
    /// [`LATENCY_HORIZON`], later than the state by more than
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt), or taken before a start
    /// that did not show the vehicle at rest.
    ///
    /// The history begins at the start, so a measurement from before it would be placed at
    /// the start's state. After a still window that is where the vehicle was; after a start in
    /// motion, a seed or a single sample, it is `v τ` from where the vehicle was, 1.6 m for a
    /// fix 110 ms old at 15 m/s, and the next measurement is a better use of the source.
    ///
    /// The filter's time is its last IMU sample's, so a measurement timed between that sample
    /// and the next arrives ahead of it, which is the ordinary case for a caller fusing a sensor
    /// the moment it reads; [`past`](Self::past) carries the state forward to it. The bound is
    /// the one step (22′) would coast rather than integrate, the longest the filter extrapolates
    /// on one sample.
    fn admit(&self, time: Timestamp) -> Result<(), Fusion> {
        if !self.initialized {
            return Err(Fusion::NotInitialized);
        }
        let age = self.time.since(time);
        if age > LATENCY_HORIZON
            || -age.as_secs() > self.config.max_predict_dt.as_secs()
            || time < self.earliest
        {
            return Err(Fusion::OutOfHorizon { age });
        }
        Ok(())
    }

    /// The state at `time`, from the [`History`], and how long before the present it was
    /// placed: no further back than the history reaches, so that a measurement older than the
    /// initialization is placed at the start rather than described by one time and linearized
    /// at another.
    fn past(&self, time: Timestamp) -> (State, Seconds) {
        let omega = self
            .angular_rate
            .unwrap_or_else(|| AngularRate::body(0.0, 0.0, 0.0));
        let (past, placed) = self.history.at(time, self.time, &self.state, omega);
        (past, self.time.since(placed))
    }

    /// Form an observation of the state at `time`, expressed in today's error.
    /// Equation (23′).
    ///
    /// `build` is the observation module's own function, handed the past state instead of the
    /// present: the innovation of (23) is then the measurement against where the vehicle was
    /// when it was taken, and a fix 150 ms old on a vehicle at 20 m/s is not 3 m of error
    /// handed to the gate as truth. The past is read from the [`History`] rather than
    /// extrapolated from the present, and its `H` is carried to today's error through (16)–(19)
    /// at the mean rates over the age, which the same history gives.
    ///
    /// Out of line, so that it sits beside `update` rather than beneath it: `observe::<3>` is
    /// 1464 bytes on `thumbv6m-none-eabi`, with `Observation::delayed` at 752 and
    /// `error_dynamics` at 400 below it. Inlined into `fuse_gnss_velocity`, it sat beneath
    /// `update::<3>` and put the crate's high-water mark at 10800 bytes against 9504.
    #[inline(never)]
    fn observe<const M: usize>(
        &self,
        time: Timestamp,
        build: impl FnOnce(&State, AngularRate<Body>) -> Observation<M>,
    ) -> Observation<M> {
        if time == self.time {
            return build(&self.state, self.mean_rate(&self.state, 0.0));
        }
        let (past, age) = self.past(time);
        let tau = age.as_secs();
        let omega = self.mean_rate(&past, tau);
        if tau == 0.0 {
            return build(&past, omega);
        }
        let now = self.state.attitude.body_to_ned();
        // Mean rates over the age, rather than the last sample's: one sample's specific force
        // carries the airframe's vibration, which the velocities either side of it average out.
        let a_n = (self.state.velocity.vector() - past.velocity.vector()) / tau;
        let a_b = now.inverse() * (a_n - propagate::gravity());
        let a = propagate::error_dynamics(&self.state, omega.vector(), a_b);
        build(&past, omega).delayed(age, &a)
    }

    /// The body rate between `past`, `tau` before now, and now: the rotation the attitude
    /// made, over the time it took. Bias-corrected by construction, since it is the estimate's
    /// own turn. With no interval, the last sample's `ω` of (9), or none after a coast.
    ///
    /// What (23′) carries `H` with, and the rate a GNSS velocity's antenna turned at, (29′):
    /// a solution 110 ms old is the antenna's motion then, not now.
    fn mean_rate(&self, past: &State, tau: f32) -> AngularRate<Body> {
        if tau == 0.0 {
            return self.angular_rate.unwrap_or_default();
        }
        let (now, then) = (
            self.state.attitude.body_to_ned(),
            past.attitude.body_to_ned(),
        );
        AngularRate::from_vector((then.inverse() * now).scaled_axis() / tau)
    }

    /// A fix of the antenna taken at `time`, as the IMU's position now: referred to the IMU
    /// by the attitude then, `z − R̂(t − τ) r` of (28′), and carried to now by the state's own
    /// motion since, `+ x̂ − x̂(t − τ)`. For an adoption, which writes the measurement as the
    /// state. A fix 110 ms old on a vehicle at 30 m/s, adopted as it stands, puts the estimate
    /// 3.3 m behind.
    fn carried_position(
        &self,
        taken: Position<Ned>,
        antenna: Position<Body>,
        time: Timestamp,
    ) -> Position<Ned> {
        let (past, _) = self.past(time);
        let arm = past.attitude.body_to_ned() * antenna.vector();
        let moved = self.state.position.vector() - past.position.vector();
        Position::from_vector(taken.vector() - arm + moved)
    }

    /// [`carried_position`](Self::carried_position) for a velocity, referred to the IMU by
    /// (29′) at the mean rate over the measurement's age, or the last sample's for one taken
    /// now.
    fn carried_velocity(
        &self,
        taken: Velocity<Ned>,
        antenna: Position<Body>,
        time: Timestamp,
    ) -> Velocity<Ned> {
        let (past, age) = self.past(time);
        let omega = self.mean_rate(&past, age.as_secs());
        let turning = past.attitude.body_to_ned() * omega.vector().cross(&antenna.vector());
        let moved = self.state.velocity.vector() - past.velocity.vector();
        Velocity::from_vector(taken.vector() - turning + moved)
    }

    /// Refuse both halves of a GNSS fix for one reason.
    fn refuse_gnss(&mut self, outcome: Fusion) -> GnssFusion {
        refuse(&mut self.diagnostics.gnss_position, outcome);
        GnssFusion::both(refuse(&mut self.diagnostics.gnss_height, outcome))
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
    /// `noise` and `antenna` are as for [`fuse_gnss_position`](Self::fuse_gnss_position), floor
    /// included, and so is the split into two gated halves. The origin (44) places goes under
    /// the estimate of the antenna, since the fix is the antenna's.
    ///
    /// A fix that is not a number is refused with [`Fusion::NotFinite`], both halves, since
    /// no conversion survives one; so is a noise that is not, where the fix would place the
    /// origin, which writes all three axes. A fix
    /// with a latitude beyond ±90° cannot place an origin, nor can one near a pole that no
    /// origin puts at the estimate (see [`LocalOrigin::placing`]): with none held it is
    /// refused with [`Fusion::NoReference`], and the next usable fix places it instead.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_geodetic(
        &mut self,
        time: Timestamp,
        fix: Geodetic,
        noise: PositionNoise<Ned>,
        antenna: Position<Body>,
    ) -> GnssFusion {
        if let Err(refusal) = self.admit(time) {
            return self.refuse_gnss(refusal);
        }
        // Noted again by `fuse_gnss_position` when it delegates, which the same time ignores.
        self.diagnostics.gnss_position.note_arrival(time);
        self.diagnostics.gnss_height.note_arrival(time);
        if !fix.is_finite() || !antenna.is_finite() {
            return self.refuse_gnss(Fusion::NotFinite);
        }
        if let Some(origin) = self.origin {
            return self.fuse_gnss_position(time, origin.to_ned(fix), noise, antenna);
        }
        if !noise.is_finite() {
            return self.refuse_gnss(Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return self.refuse_gnss(Fusion::InvalidNoise);
        }

        if self.unestablished.position {
            let Some(origin) = LocalOrigin::new(fix) else {
                return self.refuse_gnss(Fusion::NoReference);
            };
            self.place_origin(origin);
            return self.fuse_gnss_position(time, Position::zero(), noise, antenna);
        }

        // Equation (44): the origin under the estimate when the fix was taken, and the fix's
        // error as the position's. The estimate of the antenna's position, which is what the
        // fix measures, (28′), with the heading the site's declination turns it to first: read
        // before, a 1 m arm under a 13.8° turn misplaces the origin by 0.24 m. The turn is
        // committed only once the origin is found, so a fix refused here changes nothing.
        let turn = self.declination_at(fix).map_or(0.0, |(_, turn)| turn);
        let (past, _) = self.past(time);
        let attitude_then = exp_quat(Vector3::z() * turn) * past.attitude.body_to_ned();
        let antenna_then = past.position.vector() + attitude_then * antenna.vector();
        let Some(origin) = LocalOrigin::placing(fix, Position::from_vector(antenna_then)) else {
            return self.refuse_gnss(Fusion::NoReference);
        };
        self.learn_declination(fix);
        self.origin = Some(origin);
        let placed = self.reset_position_to(self.state.position, noise);
        debug_assert!(
            placed,
            "the estimate and the fix's noise are both already checked"
        );
        self.diagnostics.gnss_position.record_accepted(0.0, None);
        self.diagnostics.gnss_height.record_accepted(0.0, None);
        GnssFusion::both(Fusion::Accepted { test_ratio: 0.0 })
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
    ///
    /// Fused at the variance (24′) leaves for a solution whose error persists from the last,
    /// [`Config::correlation`](crate::Config::correlation)'s `gnss_velocity`;
    /// the gate reads `noise` itself.
    ///
    /// `antenna` is as for [`fuse_gnss_position`](Self::fuse_gnss_position): the solution is
    /// the antenna's velocity, `v + R(ω × r)` of (29′), with `ω` the rate the vehicle turned
    /// at over the solution's age, read off the state's own history rather than the last
    /// sample.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_velocity(
        &mut self,
        time: Timestamp,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
        antenna: Position<Body>,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.gnss_velocity, refusal);
        }
        self.diagnostics.gnss_velocity.note_arrival(time);
        if !velocity.is_finite() || !noise.is_finite() || !antenna.is_finite() {
            return refuse(&mut self.diagnostics.gnss_velocity, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.gnss_velocity, Fusion::InvalidNoise);
        }
        if self.unestablished.velocity {
            self.adopt_velocity(self.carried_velocity(velocity, antenna, time), noise);
            self.unestablished.velocity = false;
            self.diagnostics.gnss_velocity.record_adopted();
            return Fusion::Reset;
        }
        let observation = self
            .observe(time, |past, omega| {
                gnss::velocity_observation(past, velocity, noise, antenna, omega)
            })
            .correlated(correlation_inflation(
                self.diagnostics.gnss_velocity.since_measured,
                self.config.correlation.gnss_velocity,
            ));
        let outcome = update(
            &self.state,
            &self.covariance,
            &self.offset,
            &observation,
            self.config.gates.gnss_velocity,
        );
        self.apply_or_recover(
            outcome,
            |diagnostics| &mut diagnostics.gnss_velocity,
            self.config.recovery.gnss_velocity,
            |filter| {
                let adopted = filter.carried_velocity(velocity, antenna, time);
                filter.adopt_velocity(adopted, noise);
            },
        )
    }

    /// Fuse a barometric altitude. Equation (30).
    ///
    /// The measurement is `z = -(α - α₀)`, so it needs a reference: one
    /// [`initialize`](Self::initialize) derived from a window taken at rest, one
    /// [`set_baro_reference`](Self::set_baro_reference) named, or — failing both — one this
    /// call establishes from the estimate, below.
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
    /// With no reference held, the first altitude offered once position is established
    /// sets one, `α̂₀ = α + p̂_D`, so that it lands on the estimate — PX4's
    /// `baro_height_control.cpp:79` at `c4e4ef98`. That covers every start that leaves no
    /// reference: one in motion after its first GNSS fix, a window with no barometer in it,
    /// and a seed. Leaving it to the caller costs `cd7e0001`, a coarse start, all 3530 of its
    /// altitudes, with nothing but [`Fusion::NoReference`] on a source nobody reads to say
    /// so. [`Config::baro_reference_from_estimate`](crate::Config::baro_reference_from_estimate)
    /// turns it off for a caller that names its own.
    ///
    /// The reference inherits the height error of the estimate it was read against, so it is
    /// seeded correlated with it, `P_bb = P_DD + R_m` and `P_xb = −P[:, D]` of (30′), rather
    /// than as an independent σ. That is what keeps the error the first fix left in `α̂₀`
    /// from reading as a hundred barometer readings' worth of agreement: held constant and
    /// uncorrelated, it took `moving_start`'s `nees_pos` to 112.59.
    ///
    /// The altitude is spent placing the reference rather than fused, as a first fix is
    /// spent placing the origin in [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic), and is
    /// reported the same way: accepted with a zero test ratio, since nothing was
    /// inconsistent. Fusing it as well would count its noise twice, once in `P_bb`.
    ///
    /// Until position is established there is no estimate to read the reference against,
    /// and the altitude is refused with [`Fusion::NoReference`] rather than adopted: a
    /// height is not a quantity this source can establish, because `α₀` is what relates
    /// it to the origin.
    ///
    /// Fused at the variance (24′) leaves for an altitude whose error persists from the last,
    /// [`Config::correlation`](crate::Config::correlation)'s `baro_altitude`;
    /// the gate reads `noise` itself.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_baro_altitude(
        &mut self,
        time: Timestamp,
        altitude: Altitude,
        noise: AltitudeNoise,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.baro_altitude, refusal);
        }
        self.diagnostics.baro_altitude.note_arrival(time);
        if !altitude.as_meters().is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::InvalidNoise);
        }
        let Some(reference) = self.baro_reference else {
            if self.unestablished.position || !self.config.baro_reference_from_estimate {
                return refuse(&mut self.diagnostics.baro_altitude, Fusion::NoReference);
            }
            self.reference_from_estimate(altitude, noise, time);
            self.diagnostics.baro_altitude.record_accepted(0.0, None);
            return Fusion::Accepted { test_ratio: 0.0 };
        };
        let observation = self
            .observe(time, |past, _| {
                baro::altitude_observation(past, altitude, reference, noise)
            })
            .correlated(correlation_inflation(
                self.diagnostics.baro_altitude.since_measured,
                self.config.correlation.baro_altitude,
            ));
        let outcome = update(
            &self.state,
            &self.covariance,
            &self.offset,
            &observation,
            self.config.gates.baro_altitude,
        );
        // Read against the estimate, so only once there is an estimate to read it against, and
        // never over a reference the caller owns.
        let after = if self.unestablished.position || !self.config.baro_reference_from_estimate {
            None
        } else {
            self.config.recovery.baro_altitude
        };
        self.apply_or_recover(
            outcome,
            |diagnostics| &mut diagnostics.baro_altitude,
            after,
            |filter| filter.reference_from_estimate(altitude, noise, time),
        )
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
    /// `heading` false until the first heading is accepted, here or from
    /// [`fuse_gnss_heading`](Self::fuse_gnss_heading) or [`fuse_course`](Self::fuse_course),
    /// however tight [`sigma_yaw`](crate::Initialization::sigma_yaw) was.
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
    /// rejected — locking the filter out of what may be the one source that can ever
    /// establish yaw.
    ///
    /// Ordinary updates thereafter, gated at [`Gates::mag_heading`](crate::Gates) with
    /// one degree of freedom, until a run of rejections outlasts
    /// [`Recovery::mag_heading`](crate::Recovery::mag_heading) while no GNSS position,
    /// velocity or heading is arriving,
    /// when the next heading is adopted the same way. The field is reduced to a scalar heading before the gate
    /// sees it, so a disturbance is tested as the yaw error it is.
    ///
    /// The field must be calibrated: this crate corrects no hard- or soft-iron error and
    /// carries no magnetic-field states to absorb one, so a bias in `field` is a bias in
    /// heading. A field of exactly zero is finite, and reports the vehicle as `D_m` off
    /// rather than producing NaN — gateable once yaw is established, and adopted where it
    /// is not, which is the strongest reason to check a magnetometer before the first
    /// call rather than after it.
    ///
    /// Fused at the variance (24′) leaves for a heading whose error persists from the last,
    /// [`Config::correlation`](crate::Config::correlation)'s `mag_heading`;
    /// the gate reads `noise` itself.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_mag_heading(
        &mut self,
        time: Timestamp,
        field: MagField<Body>,
        noise: HeadingNoise,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.mag_heading, refusal);
        }
        self.diagnostics.mag_heading.note_arrival(time);
        if !field.is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.mag_heading, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.mag_heading, Fusion::InvalidNoise);
        }
        let declination = self.declination;
        // With GNSS arriving, a magnetometer that disagrees this long is more likely disturbed
        // than right: horizontal aiding is already correcting heading through (20), and a GNSS
        // heading measures it outright.
        let d = &self.diagnostics;
        let recovery = self.unless_accepted(
            self.config.recovery.mag_heading,
            &[&d.gnss_position, &d.gnss_velocity, &d.gnss_heading],
        );
        let source = HeadingSource {
            health: |diagnostics| &mut diagnostics.mag_heading,
            gate: self.config.gates.mag_heading,
            correlation: self.config.correlation.mag_heading,
            recovery,
            magnetic: true,
        };
        self.fuse_heading(time, source, 0.0, |past, covariance| {
            mag::heading_observation(past, covariance, field, declination, noise)
        })
    }

    /// Fuse a true heading from a dual-antenna (moving-baseline) GNSS receiver.
    /// Equations (35′) and (36).
    ///
    /// The heading of the line from the primary antenna to the secondary, which is taken to
    /// lie along body x: a receiver whose antennas are mounted otherwise has the mounting
    /// angle subtracted by the caller first, as PX4 and ArduPilot each do from a parameter.
    /// True rather than magnetic, so no [declination](Self::set_magnetic_declination) applies,
    /// and nothing is levelled with the estimated attitude, so the (36′) a magnetic heading
    /// carries has no counterpart; see `observation/heading.rs` for the model and why its
    /// Jacobian is (36) rather than the exact one PX4 differentiates.
    ///
    /// `noise` is the receiver's `heading_accuracy`, bounded before it arrives:
    /// [`HeadingNoise::clamped`](crate::HeadingNoise::clamped) holds PX4's and ArduPilot's
    /// floors, and both production estimators apply one because a moving-baseline solution is
    /// as optimistic about itself as a position fix is.
    ///
    /// The first heading after any start that observed no yaw is adopted rather than gated,
    /// as a magnetic one is — [`Fusion::Reset`], with this `noise` as the adopted variance —
    /// and both production estimators align on the first sample the same way (PX4
    /// `aid_sources/gnss/gnss_yaw_control.cpp:100-111` at `c4e4ef98`, ArduPilot
    /// `AP_NavEKF3_MagFusion.cpp:404-410` at `368dc0c4`). Ordinary updates thereafter, gated at
    /// [`Gates::gnss_heading`](crate::Gates), and recovered after
    /// [`Recovery::gnss_heading`](crate::Recovery::gnss_heading).
    ///
    /// Fused beside a magnetometer rather than instead of one, where PX4 stops fusing the
    /// magnetometer while GNSS yaw is active (`aid_sources/magnetometer/mag_control.cpp:189`):
    /// each is gated against an estimate the other has corrected, and a magnetometer that
    /// disagrees is not recovered while this is accepted. A body x within 30° of vertical has
    /// no heading to compare, and is refused as [`Fusion::Unobservable`].
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_heading(
        &mut self,
        time: Timestamp,
        heading: Radians,
        noise: HeadingNoise,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.gnss_heading, refusal);
        }
        self.diagnostics.gnss_heading.note_arrival(time);
        if !heading.as_radians().is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.gnss_heading, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.gnss_heading, Fusion::InvalidNoise);
        }
        if !heading::has_heading(&self.past(time).0) {
            return refuse(&mut self.diagnostics.gnss_heading, Fusion::Unobservable);
        }
        let source = HeadingSource {
            health: |diagnostics| &mut diagnostics.gnss_heading,
            gate: self.config.gates.gnss_heading,
            correlation: self.config.correlation.gnss_heading,
            recovery: self.config.recovery.gnss_heading,
            magnetic: false,
        };
        self.fuse_heading(time, source, 0.0, |past, _| {
            heading::gnss_observation(past, heading, noise)
        })
    }

    /// Constrain heading to the direction of travel: the vehicle points along its estimated
    /// velocity, to within `sideslip`. Equations (35″) and (36).
    ///
    /// Yaw from course over ground, for a vehicle with no magnetometer or one it cannot trust.
    /// It is a constraint rather than a measurement: nothing new is read, and what the call
    /// states is that the nose and the track agree, so it is only as true as the vehicle makes
    /// it. A fixed-wing in coordinated flight and a ground vehicle on its wheels hold it to a
    /// few degrees; a crosswind adds its crab angle to `sideslip`. **A multirotor holds no such
    /// relation** — it hovers, and flies in any direction facing any other — and must not call
    /// this.
    ///
    /// The velocity is the estimate's, not a GNSS velocity handed in: that velocity has already
    /// been fused through [`fuse_gnss_velocity`](Self::fuse_gnss_velocity), and taking a course
    /// from it again would count its cross-track error twice. Read off the state, the
    /// velocity's uncertainty reaches the update through `P`, and the update corrects velocity
    /// and heading together. So `time` is when the constraint is claimed to hold, and a caller
    /// fusing one per GNSS velocity passes that fix's time.
    ///
    /// Refused as [`Fusion::NoReference`] where no fresh GNSS velocity holds the estimated one
    /// ([`SourceHealth::is_fresh`](crate::SourceHealth::is_fresh)), which also covers one never
    /// established, and as [`Fusion::Unobservable`] where it names no direction: too slow
    /// against its own uncertainty, which is ArduPilot's 15° bar
    /// on the course read from `P` — see `observation/heading.rs` — and so a speed threshold
    /// the velocity's accuracy sets rather than a parameter. Refused the same way with body x
    /// within 30° of vertical. ArduPilot's plane realigns yaw from course above 5 m/s
    /// (`realignYawGPS`, `AP_NavEKF3_MagFusion.cpp:145-218` at `368dc0c4`); PX4 has no course
    /// source and reaches the same vehicle through its GSF yaw estimator.
    ///
    /// The first course after a start that observed no yaw is adopted, as a first heading is,
    /// at `sideslip`'s variance plus the course's own; ordinary updates thereafter, continuously
    /// rather than ArduPilot's once, gated at [`Gates::course`](crate::Gates) and recovered
    /// after [`Recovery::course`](crate::Recovery::course) while no other heading is accepted.
    ///
    /// The adoption keeps none of the heading's correlation with velocity, though the adopted
    /// heading error is the course error, `∇χᵀδv`, plus the sideslip. Dropping it overstates
    /// the next update's `S` by about `2σ_χ²`, which is conservative; what it costs is that a
    /// velocity fix cannot correct the part of the heading error the velocity lent it until
    /// propagation correlates the two again.
    ///
    /// Not a source [`Status`] counts: it reads the filter's own velocity, so it aids nothing
    /// the GNSS velocity it needs does not, and a vehicle slowing to a stop, where every course
    /// is refused, is not degraded for it.
    pub fn fuse_course(&mut self, time: Timestamp, sideslip: HeadingNoise) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.course, refusal);
        }
        self.diagnostics.course.note_arrival(time);
        if !sideslip.is_finite() {
            return refuse(&mut self.diagnostics.course, Fusion::NotFinite);
        }
        if !sideslip.is_positive() {
            return refuse(&mut self.diagnostics.course, Fusion::InvalidNoise);
        }
        // A velocity no GNSS velocity is holding is a dead-reckoned one, and a course along it
        // would read as aiding while it drifts. It also covers a velocity never established,
        // which only a GNSS velocity's acceptance or `reset_velocity_to` establishes.
        if !self.accepted_recently(&self.diagnostics.gnss_velocity) {
            return refuse(&mut self.diagnostics.course, Fusion::NoReference);
        }
        // Screened on the state the observation is built on, the one at `time`.
        let past = self.past(time).0;
        let spread = heading::course_variance(&past, &self.covariance)
            .filter(|_| heading::has_heading(&past));
        let Some(spread) = spread else {
            return refuse(&mut self.diagnostics.course, Fusion::Unobservable);
        };
        // With a heading source arriving, a course that disagrees this long is sideslip the
        // caller did not allow for, not a wrong heading.
        let d = &self.diagnostics;
        let recovery = self.unless_accepted(
            self.config.recovery.course,
            &[&d.mag_heading, &d.gnss_heading],
        );
        let source = HeadingSource {
            health: |diagnostics| &mut diagnostics.course,
            gate: self.config.gates.course,
            correlation: self.config.correlation.course,
            recovery,
            magnetic: false,
        };
        self.fuse_heading(time, source, spread, |past, _| {
            heading::course_observation(past, sideslip)
        })
    }

    /// The update every heading source shares, once its measurement has been screened:
    /// formed at `time` by (23′), fused at (24′)'s variance, adopted where heading was never
    /// established, and otherwise gated and recovered.
    ///
    /// `spread` is the variance an adoption carries beyond `R`: zero for a source whose `H`
    /// reads attitude alone, and the course's own for the constraint, whose `H` reads velocity
    /// too and whose adopted heading is only as good as the velocity it was taken along.
    ///
    /// Out of line for the reason [`observe`](Self::observe) is. On `thumbv6m-none-eabi` it is
    /// 1224 bytes over `update::<1>`'s 6368, and the deepest caller above it, `fuse_course`, 264
    /// with its screening on the past state: 7856 at the peak, against 7624 when
    /// `fuse_mag_heading` did all of this in its own 1240-byte frame, and under
    /// `fuse_gnss_velocity`'s 9504.
    #[inline(never)]
    fn fuse_heading(
        &mut self,
        time: Timestamp,
        source: HeadingSource,
        spread: f32,
        build: impl FnOnce(&State, &Covariance) -> Observation<1>,
    ) -> Fusion {
        let since_measured = (source.health)(&mut self.diagnostics).since_measured;
        let observation = self
            .observe(time, |past, _| build(past, &self.covariance))
            .correlated(correlation_inflation(since_measured, source.correlation));
        if self.unestablished.heading {
            self.adopt_heading(&observation, spread);
            (source.health)(&mut self.diagnostics).record_adopted();
            self.note_alignment();
            self.magnetic_north = source.magnetic;
            return Fusion::Reset;
        }
        let outcome = update(
            &self.state,
            &self.covariance,
            &self.offset,
            &observation,
            source.gate,
        );
        let fused = self.apply_or_recover(outcome, source.health, source.recovery, |filter| {
            filter.adopt_heading(&observation, spread)
        });
        // An adoption replaces the heading with this source's; a true heading accepted means
        // the estimate is no longer referred to north through the declination alone.
        match fused {
            Fusion::Reset => self.magnetic_north = source.magnetic,
            Fusion::Accepted { .. } if !source.magnetic => self.magnetic_north = false,
            _ => {}
        }
        fused
    }

    /// `recovery`, unless any of `arbiters` was accepted recently: a source that disagrees
    /// with better aiding that is arriving is the one at fault, and adopting it would step the
    /// estimate away from what the others say.
    fn unless_accepted(
        &self,
        recovery: Option<Seconds>,
        arbiters: &[&SourceHealth],
    ) -> Option<Seconds> {
        if arbiters
            .iter()
            .any(|arbiter| self.accepted_recently(arbiter))
        {
            None
        } else {
            recovery
        }
    }

    /// Adopt a heading: [`reset_heading_by`](Self::reset_heading_by) with the `y` and `R`
    /// an ordinary update would read. (36′) is what makes that worth saying: the levelling
    /// error is priced on the path where the tilt it comes from is worst.
    ///
    /// `spread` is added to `R`: see [`fuse_heading`](Self::fuse_heading).
    fn adopt_heading(&mut self, observation: &Observation<1>, spread: f32) {
        // Navigation down in body axes, `R(q̂)ᵀe₃`, of the present state: the axis the
        // adoption turns about. Not read off the observation's attitude row, which (23′) carries
        // through the error dynamics: a course's gains a tilt component from its velocity block,
        // `∇χᵀR[a_b]× τ`, about 0.06 at 18 m/s and 110 ms, and stops being the unit vector
        // `reset_attitude_direction` needs.
        let down = self.state.attitude.body_to_ned().inverse() * Vector3::z();
        self.reset_heading_by(observation.y[0], observation.r_m[0] + spread, down);
    }

    /// Turn the estimate by a yaw error and give the result the measurement's variance:
    /// the adoption behind [`Fusion::Reset`] for heading.
    ///
    /// `y` is the innovation of (35), (35′) or (35″), so the corrected attitude is `Exp(y e₃) ⊗ q̂` —
    /// composed on the **left**, because `e₃` is the navigation down axis, where the
    /// `δθ` of (2) that `update` injects is a body-frame rotation composed on the right.
    /// Tilt is untouched: a rotation about navigation down moves the tilt axis and not
    /// the tilt angle, so the roll and pitch gravity established survive a heading any
    /// heading source supplies.
    ///
    /// `Exp` charges its caller with a finite argument, which each innovation's wrap
    /// discharges by construction: `y` is in `(-π, π]` whatever was measured and the attitude
    /// were.
    ///
    /// `variance` is the `R` the source's update would read, plus what the heading inherits
    /// that `R` does not carry: for the course, the uncertainty of the velocity it was taken
    /// along ([`fuse_heading`](Self::fuse_heading)'s `spread`). For a magnetic heading it is
    /// `R` from (36′) rather than the caller's `σ_ψ²` alone. The levelling
    /// of (34) is done with the estimated attitude on this path too — on a coarse start,
    /// with the worst tilt the filter ever holds — so an adoption that stored the
    /// magnetometer's own number would report a heading good to
    /// [`Accuracy::heading`](crate::Accuracy::heading) while carrying the window's
    /// levelling error times `tan δ`. That is the falsely-valid attitude (36′) exists to
    /// remove, and the adoption is where it is largest.
    ///
    /// The correlations go with it, which is what fusing against an infinitely uncertain
    /// prior converges to — the same limit [`reset_position_to`](Self::reset_position_to)
    /// takes, one component wide. That component is the rotation about navigation down,
    /// `uᵀδθ` with `u = R(q̂)ᵀe₃` in body axes, not `δθ_z`: the two agree only while the
    /// vehicle is level, and at 90° of pitch `δθ_z` is a tilt. The rotation above leaves
    /// `e₃` where it was, so `u` is the same before the adoption and after it.
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
    fn reset_heading_by(&mut self, y: f32, variance: f32, down: Vector3<f32>) {
        let before = self.state.attitude.body_to_ned().to_rotation_matrix();
        let mut corrected = exp_quat(Vector3::z() * y) * self.state.attitude.body_to_ned();
        corrected.renormalize();
        self.commit_state(State {
            attitude: Attitude::from_body_to_ned(corrected),
            ..self.state
        });

        let g_theta = corrected.to_rotation_matrix().inverse() * before;
        let g_theta = g_theta.into_inner();
        let mut covariance = update::reparameterize(*self.covariance.as_matrix(), g_theta);
        let mut offset = update::reparameterize_offset(&self.offset, g_theta);
        covariance.reset_attitude_direction(down, variance);
        offset.decorrelate_attitude_direction(down);
        self.commit_covariance(covariance, offset);
        self.unestablished.heading = false;
    }

    /// The current estimate, including its [`Status`].
    ///
    /// Status and validity are derived here rather than cached: they are pure functions
    /// of [`diagnostics`](Self::diagnostics), the covariance, and [`Config`], so computing
    /// them on read means there is no invariant for the mutating methods to maintain. The
    /// cost is two rotations of the attitude block, one each for tilt and heading, six other
    /// covariance entries and six source timers, compared.
    pub fn state(&self) -> State {
        // `self.state.status` and `.validity` are inert; the stored estimate never
        // carries meaningful ones, and every read overwrites them.
        let validity = self.validity();
        let mut state = self.state;
        state.status = self.derive_status();
        state.validity = validity;
        state
    }

    /// The body angular rate with the estimated gyroscope bias removed, `ω` of equation (9),
    /// from the last step [`predict`](Self::predict) integrated from a sample.
    ///
    /// For moving the estimate to a point other than the IMU. The GNSS `fuse_*` take their
    /// antenna's offset and refer the fix to the IMU themselves, (28′) and (29′); the estimate
    /// they correct stays the IMU's, and a point at `r` in body axes (the centre of mass a
    /// controller wants, a payload, a sensor this crate does not fuse) is at the IMU's position
    /// plus `R r` and moves at its velocity plus `R (ω × r)`. PX4 reports its estimate at the
    /// IMU the same way, and corrects each aiding source with this bias-corrected rate
    /// (`EKF/aid_sources/gnss/gps_control.cpp:313-318` at `c4e4ef98`).
    ///
    /// ```
    /// use fusion_nav::prelude::*;
    /// # let mut filter = Eskf::default();
    /// # let dt = Seconds::from_secs(0.0025);
    /// # let (level, gravity) = (AngularRate::zero(), Acceleration::body(0.0, 0.0, -GRAVITY));
    /// # let at = |i: u64| Timestamp::from_micros(2_500 * i);
    /// # let mut window = StaticWindow::new();
    /// # for i in 1..=800 {
    /// #     window.push(StaticSample {
    /// #         imu: ImuSample::from_rates(at(i), level, gravity, dt),
    /// #         ..StaticSample::default()
    /// #     })?;
    /// # }
    /// # filter.initialize(&window)?;
    /// # let imu = ImuSample::from_rates(at(801), AngularRate::body(0.0, 0.0, 0.5), gravity, dt);
    /// # assert!(filter.predict(imu).is_propagated());
    /// // The centre of mass, 0.2 m behind the IMU: measured on the airframe.
    /// let r = Position::body(-0.2, 0.0, 0.0).vector();
    ///
    /// // None before the first step and across a gap, where no sample says how fast the
    /// // vehicle turned.
    /// if let Some(omega) = filter.angular_rate() {
    ///     let state = filter.state();
    ///     let rotation = state.attitude.body_to_ned();
    ///     let position = state.position.vector() + rotation * r;
    ///     let velocity = state.velocity.vector() + rotation * omega.vector().cross(&r);
    ///     // Yawing at 0.5 rad/s, a point 0.2 m aft swings sideways at 0.1 m/s.
    ///     assert!((velocity.norm() - 0.1).abs() < 1e-3);
    /// #   let _ = position;
    /// }
    /// # Ok::<(), InitError>(())
    /// ```
    ///
    /// Committed with the state, and only with it. `None` until a step is integrated, after
    /// any initialization, and after a gap coasted past
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt), until the next sample: a
    /// coast reads no sample, and the rate before the gap is not the rate after it. A refused
    /// step leaves the last rate in place, as it leaves the state.
    pub const fn angular_rate(&self) -> Option<AngularRate<Body>> {
        self.angular_rate
    }

    /// Per-source health. Off the hot path.
    pub const fn diagnostics(&self) -> &Diagnostics {
        &self.diagnostics
    }

    /// The 15 x 15 error covariance, in the error-state ordering.
    pub const fn covariance(&self) -> &Covariance {
        &self.covariance
    }

    /// How uncertain the attitude is about each navigation axis: tilt about north and east,
    /// heading about down.
    ///
    /// The numbers [`validity`](Self::validity) tests against
    /// [`Accuracy`](crate::Accuracy), and the ones to plot beside a tilt or a heading. The
    /// attitude rows of [`covariance`](Self::covariance) are in body axes and mean tilt and
    /// heading only while the vehicle is level; see [`AttitudeVariance`]. All zero before
    /// the filter is initialized, which is the covariance it then holds.
    pub fn attitude_variance(&self) -> AttitudeVariance {
        AttitudeVariance::of(&self.state.attitude, &self.covariance)
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
    /// Conflating the two is what a corpus replay showed costs. Read live, the bar is crossed
    /// 703 times on `2c42096b`, a grounded vehicle under a poor sky view whose tilt σ sits
    /// above [`ALIGNED_TILT`] for 79 % of two hours, and `7592c9b2` and `f16771dd` start
    /// aligned and end `Aligning`. The counts are the same with tilt read on body axes or on
    /// navigation ones. That is honest about tilt right now, which is [`Validity`]'s job, and
    /// useless as a report that the filter has not finished starting up. PX4 and ArduPilot both latch it for the same
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

    /// Whether tilt about north and about east are both within `bar`, one standard
    /// deviation per axis.
    ///
    /// Shared by [`validity`](Self::validity) and the alignment latch, which ask it against
    /// different bars; one definition is what keeps the two claims the same shape. Read on
    /// navigation axes through [`AttitudeVariance`], not off `δθ_x` and `δθ_y`, which are
    /// tilt only while the vehicle is level.
    fn tilt_within(&self, p: &Covariance, bar: Radians) -> bool {
        let sigma = bar.as_radians();
        let variance = AttitudeVariance::of(&self.state.attitude, p);
        variance.tilt_north <= sigma * sigma && variance.tilt_east <= sigma * sigma
    }

    /// Whether heading has been established and is within `bar`.
    ///
    /// A heading nothing observed fails whatever its variance: stillness never observes yaw,
    /// and a prior on a yaw nobody measured is not an estimate of one.
    fn heading_within(&self, p: &Covariance, bar: Radians) -> bool {
        let sigma = bar.as_radians();
        !self.unestablished.heading
            && AttitudeVariance::of(&self.state.attitude, p).heading <= sigma * sigma
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
    /// defaults an unaided start holds tilt for 3.83 s, so a horizon under that arms and one
    /// over it does not — an answer, where before there was only the current value repeated.
    ///
    /// The projection reads slightly optimistic and the amount is measured: a first-order
    /// step understates growth, and `propagate.rs`'s `PROJECTION_STEP` holds that within
    /// 3.2 % of the sigma out to a 5 s horizon. It costs one `F` and up to 64 covariance
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

        let fresh = |source: &SourceHealth| self.accepted_recently(source);
        let d = &self.diagnostics;
        let (position, velocity) = (fresh(&d.gnss_position), fresh(&d.gnss_velocity));
        let height = fresh(&d.gnss_height) || fresh(&d.baro_altitude);

        Validity {
            // Gravity is not an aiding source the filter tracks, so tilt has only the
            // projection to speak for it -- which is the one quantity where that is the
            // whole answer rather than half of it.
            tilt: ahead.tilt,
            heading: ahead.heading
                || fresh(&d.mag_heading)
                || fresh(&d.gnss_heading)
                || fresh(&d.course),
            horizontal_position: ahead.horizontal_position || position,
            vertical_position: ahead.vertical_position || height,
            horizontal_velocity: ahead.horizontal_velocity || velocity,
            vertical_velocity: ahead.vertical_velocity || velocity,
        }
    }

    /// Whether `source` was accepted within its own
    /// [`timeout`](crate::SourceHealth::timeout): aiding that is arriving, as
    /// [`Status`], [`predicted_validity`](Self::predicted_validity), the course and the
    /// recovery guards all ask.
    fn accepted_recently(&self, source: &SourceHealth) -> bool {
        source.is_fresh(&self.config.timeouts)
    }

    /// The barometric reference `α₀` as currently estimated, equations (30) and (30′):
    /// measured by a window taken at rest, named by
    /// [`set_baro_reference`](Self::set_baro_reference), or read against the estimate by the
    /// first altitude once position is established. `None` until one of those has happened —
    /// a start in motion keeps whatever reference the flight already had rather than calling
    /// its own altitude the ground.
    ///
    /// Exposed because it is what the filter's zero altitude means: the barometer reading
    /// the filter would call the origin's height. It moves as barometer and GNSS height
    /// disagree, at the rate [`Config::baro_offset_walk`](crate::Config::baro_offset_walk)
    /// allows.
    pub const fn baro_reference(&self) -> Option<Altitude> {
        self.baro_reference
    }

    /// Force position to an external fix and reset its covariance block.
    ///
    /// The position becomes the fix, its variances become the fix's noise, and its
    /// correlations with the rest of the state are dropped — the new error came from the
    /// measurement and has nothing to do with the errors that preceded it.
    ///
    /// For an application that owns recovery: the filter does the same on its own, per
    /// source, unless [`Config::recovery`](crate::Config::recovery) turns it off; see
    /// [`Recovery`](crate::Recovery).
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
        self.adopt_position(position, noise, POSITION);
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
        self.adopt_velocity(velocity, noise);
        self.unestablished.velocity = false;
        true
    }

    /// Take the `axes` of a checked fix as the position, with the fix's variances on them:
    /// every axis on a first adoption or a caller's reset, north and east or down alone on a
    /// recovery, since a fix is two sources gated apart.
    // Out of line, as `adopt_velocity`: inlined, its copy of `P` lands in the `fuse_*` frame
    // that `update` then stacks on, +976 bytes on `fuse_gnss_position` on `thumbv6m`. Called
    // after `update` returns, it adds nothing to the deepest path.
    #[inline(never)]
    fn adopt_position<const N: usize>(
        &mut self,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
        axes: [ErrorState; N],
    ) {
        let (z, r) = (position.vector(), noise.variance());
        let mut adopted = self.state.position.vector();
        let mut variances = [0.0; N];
        for (axis, variance) in axes.iter().zip(&mut variances) {
            // `get` rather than indexing: an out-of-range index is a panic, and every axis
            // passed here is a position state, so this never misses.
            let i = axis
                .index()
                .saturating_sub(ErrorState::PositionNorth.index());
            if let (Some(adopted), Some(z), Some(r)) = (adopted.get_mut(i), z.get(i), r.get(i)) {
                *adopted = *z;
                *variance = *r;
            }
        }
        self.commit_state(State {
            position: Position::ned(adopted[0], adopted[1], adopted[2]),
            ..self.state
        });
        self.reset_block(axes, variances);
    }

    /// Recover GNSS height: adopt the down axis, and let the barometer read its reference
    /// again against it.
    ///
    /// A lockout of GNSS height is a barometer holding the height somewhere the receiver
    /// disagrees with, so the reference that put it there is dropped with the height it
    /// described, and the next altitude reads one from the estimate as a start that left
    /// none does — where [`Config::baro_reference_from_estimate`](crate::Config::baro_reference_from_estimate)
    /// allows it. Where it does not, the caller owns the reference and it is kept, only
    /// decorrelated from the height `reset_block` replaced.
    fn adopt_height(&mut self, position: Position<Ned>, noise: PositionNoise<Ned>) {
        self.adopt_position(position, noise, [ErrorState::PositionDown]);
        if self.config.baro_reference_from_estimate {
            self.establish_reference(None);
        }
    }

    /// Take a checked velocity solution as the velocity, all three axes.
    // Out of line for the reason `adopt_position` is: +952 bytes on `fuse_gnss_velocity`.
    #[inline(never)]
    fn adopt_velocity(&mut self, velocity: Velocity<Ned>, noise: VelocityNoise<Ned>) {
        self.commit_state(State {
            velocity,
            ..self.state
        });
        self.reset_block(
            [
                ErrorState::VelocityNorth,
                ErrorState::VelocityEast,
                ErrorState::VelocityDown,
            ],
            noise.variance().into(),
        );
    }

    /// Read `α₀` from the estimate at one altitude, `α̂₀ = α + p̂_D` — (30) solved for α₀ with
    /// `z = p̂_D`, so that the altitude lands on the estimate — correlated with the height it
    /// was read against, `P_bb = P_DD + R_m` and `P_xb = −P[:, D]` of (30′). See
    /// [`fuse_baro_altitude`](Self::fuse_baro_altitude) for why.
    fn reference_from_estimate(
        &mut self,
        altitude: Altitude,
        noise: AltitudeNoise,
        time: Timestamp,
    ) {
        // Against the height when the altitude was read, as (23′) reads every measurement.
        let (past, _) = self.past(time);
        let reference = Altitude::from_meters(altitude.as_meters() + past.position.vector()[2]);
        let offset = Offset::from_estimate(&self.covariance, noise.variance());
        self.establish_reference(Some((reference, offset)));
    }

    /// Give three states the variances of a measurement adopted for them, dropping their
    /// correlations with everything else — the barometric offset of (30′) included, since the
    /// new error is the measurement's and has nothing to do with the reference's.
    fn reset_block<const N: usize>(&mut self, states: [ErrorState; N], variances: [f32; N]) {
        let mut covariance = self.covariance;
        covariance.reset_block(states, variances);
        let mut offset = self.offset;
        for state in states {
            offset.decorrelate(state);
        }
        self.commit_covariance(covariance, offset);
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
    ///
    /// Refused as [`InitError::NotFinite`], committing nothing, when the state or covariance
    /// is not finite. Every sample was finite, but their average can still overflow what (5)
    /// and (8) square: one accelerometer reading of `f32::MAX` levels to a NaN tilt variance.
    fn apply_alignment(
        &mut self,
        alignment: Alignment,
        state: State,
        measured: &Measured,
        settled: bool,
        time: Timestamp,
    ) -> Result<(), InitError> {
        // The bias of (7) as committed, so that what it absorbed is not charged a second
        // time as motion the window could not vouch for; see `init::coarse_sigmas`.
        let (sigma_tilt, sigma_yaw) =
            init::attitude_sigmas(&self.config.init, alignment, measured, state.gyro_bias);
        let covariance = init::initial_covariance(
            &self.config.init,
            &state.attitude,
            sigma_tilt,
            sigma_yaw,
            measured.level_variance,
        );
        if !(state.is_finite() && covariance.is_finite()) {
            return Err(InitError::NotFinite);
        }
        let unestablished = Unestablished::after(settled, measured.field.is_some());
        self.start(state, covariance, unestablished, time, settled);
        // (6) levelled the heading from the window's field with the declination it held.
        self.magnetic_north = measured.field.is_some();
        if settled {
            self.origin = None;
        }
        Ok(())
    }

    /// Begin a new life at `time`: the state and covariance a start committed, fresh health,
    /// and no past. What both entry points share; the barometric reference and the origin are
    /// each start's own decision.
    ///
    /// `at_rest` is whether the start showed the vehicle still, which is what lets a
    /// measurement taken before `time` be placed at all: see [`admit`](Self::admit).
    fn start(
        &mut self,
        state: State,
        covariance: Covariance,
        unestablished: Unestablished,
        time: Timestamp,
        at_rest: bool,
    ) {
        self.state = state;
        // Diagnostics first: `commit_covariance` counts into them, and a start sitting on the
        // floor is a fact about this filter's life rather than the last one's.
        self.diagnostics = Diagnostics::default();
        self.commit_covariance(covariance, self.surviving_offset());
        self.unestablished = unestablished;
        self.magnetic_north = false;
        self.angular_rate = None;
        self.initialized = true;
        self.time = time;
        self.history.clear();
        self.history.record(time, &self.state);
        self.earliest = if at_rest { Timestamp::ZERO } else { time };
        // A fresh start is unaligned until its own covariance says otherwise, which
        // `note_alignment` reads at the end of each entry point.
        self.aligned = false;
    }

    /// Aggregate alignment and the per-source timers into one status, most severe first.
    ///
    /// [`DeadReckoning`](Status::DeadReckoning) reads horizontal aiding alone, GNSS position
    /// or velocity within [`Timeouts::dead_reckoning_after`](crate::Timeouts): with GNSS gone
    /// the barometer still holds height, and position drifts all the same. The rest reads
    /// every source, each against its own period.
    ///
    /// Only sources that have ever been accepted count toward `Degraded`: a vehicle with no
    /// magnetometer is not permanently `Degraded` for lacking one. The course constraint
    /// never counts ([`Diagnostics::aiding`]): it aids nothing the GNSS velocity it needs does
    /// not.
    ///
    /// Alignment enters as the latch of [`is_aligned`](Self::is_aligned) rather than as the
    /// live [`Validity`], which is the one thing here that is not derived on read.
    fn derive_status(&self) -> Status {
        let d = &self.diagnostics;
        let horizontal = d
            .horizontal()
            .any(|source| source.accepted_within(self.config.timeouts.dead_reckoning_after));
        if !horizontal {
            // Position is unusable whatever the attitude is doing, so this outranks
            // `Aligning`.
            Status::DeadReckoning
        } else if !self.aligned {
            Status::Aligning
        } else if d
            .aiding()
            .filter(|source| source.has_been_used())
            .all(|source| self.accepted_recently(source))
        {
            Status::Healthy
        } else {
            Status::Degraded
        }
    }
}

/// The declination the magnetic model gives at `site`, or `None` without the
/// `magnetic-model` feature, which links no table.
fn model_declination(site: Geodetic) -> Option<Radians> {
    #[cfg(feature = "magnetic-model")]
    return site.magnetic_declination();
    #[cfg(not(feature = "magnetic-model"))]
    {
        let _ = site;
        None
    }
}

/// What distinguishes one heading source from another inside
/// [`Eskf::fuse_heading`]: where its health is kept, and the gate, `τ` and recovery
/// [`Config`] gives it.
struct HeadingSource {
    health: fn(&mut Diagnostics) -> &mut SourceHealth,
    gate: Gate<1>,
    correlation: Option<Seconds>,
    recovery: Option<Seconds>,
    /// Whether the heading is magnetic, referred to true north through the declination:
    /// what [`Eskf::place_origin`] reads to decide whether a learned declination turns it.
    magnetic: bool,
}

impl Unestablished {
    /// What a start leaves unestablished, read off what its window showed rather than off
    /// which [`Alignment`] it earned.
    ///
    /// `settled` is [`init::at_rest`]'s verdict. A vehicle that held still through the
    /// window is where the origin says it is and is not moving, which is the whole of
    /// what a static start ever claimed about position and velocity — and a window too
    /// short to align an attitude from claims it just as honestly, which is
    /// [`Coarse::WindowTooShort`]. A window taken in motion establishes neither: the vehicle
    /// passed through somewhere the filter cannot name. Those wait for the first fix.
    ///
    /// Heading needs a magnetometer in the window on top of stillness. Gravity pins tilt and nothing
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

/// The position axes, as a first adoption and a caller's reset take them.
const POSITION: [ErrorState; 3] = [
    ErrorState::PositionNorth,
    ErrorState::PositionEast,
    ErrorState::PositionDown,
];

/// The horizontal half of a GNSS fix, as its recovery adopts it.
const HORIZONTAL: [ErrorState; 2] = [ErrorState::PositionNorth, ErrorState::PositionEast];

/// Whether one error state's variance is within `sigma`, one standard deviation on that axis.
///
/// A free function rather than a method, because the covariance it reads is an argument: the
/// filter asks this of the one it holds and of the one
/// [`Eskf::predicted_validity`] projects, and a method taking `&self` would quietly answer
/// for the wrong one.
fn within(p: &Covariance, state: ErrorState, sigma: f32) -> bool {
    p.variance(state) <= sigma * sigma
}

/// Why one half of a GNSS fix cannot be judged, or `None` if it can: the checks every
/// `fuse_*` makes of a whole measurement, made of the components that half reads.
fn screen(values: &[f32], variances: &[f32]) -> Option<Fusion> {
    if !values.iter().chain(variances).all(|v| v.is_finite()) {
        return Some(Fusion::NotFinite);
    }
    if !variances.iter().all(|v| *v > 0.0) {
        return Some(Fusion::InvalidNoise);
    }
    None
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
mod adversarial;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Accuracy, Coast, Correlation, GRAVITY, Recovery};
    use crate::geodetic::LocalOrigin;
    use crate::health::Refusal;
    use crate::init::tests::{gravity_at, spaced, still, turning};
    use crate::observation::mag::tests::{attitude_of, measured};
    use crate::state::ErrorState;
    use crate::state::STATES;
    use crate::units::{Acceleration, AngularRate, Radians};

    const DT: Seconds = Seconds::from_secs(0.01);

    /// The fixtures are written as rates and a `dt`; these give them a clock. Only the method
    /// names differ from the filter's own, so a test reads as the call it makes.
    trait Clocked {
        /// [`Eskf::predict`] on `imu`'s rates over `dt`, timed `dt` after the filter's clock.
        fn step(&mut self, imu: ImuSample, dt: Seconds) -> Propagation;
        /// [`Eskf::initialize`] on `window`, each sample over `dt`.
        fn initialize_over(
            &mut self,
            window: &[StaticSample],
            dt: Seconds,
        ) -> Result<Alignment, InitError>;
        /// [`Eskf::initialize_from`] at the epoch.
        fn seed(&mut self, state: State, covariance: Covariance) -> Result<Alignment, InitError>;
        /// The filter's time, for a measurement taken now: the epoch before initialization.
        fn now(&self) -> Timestamp;
    }

    impl Clocked for Eskf {
        fn step(&mut self, imu: ImuSample, dt: Seconds) -> Propagation {
            let time = self.time().unwrap_or_default().after(dt);
            self.predict(imu.timed(time, dt))
        }

        fn initialize_over(
            &mut self,
            window: &[StaticSample],
            dt: Seconds,
        ) -> Result<Alignment, InitError> {
            self.initialize(&StaticWindow::try_from(spaced(window, dt).as_slice())?)
        }

        fn seed(&mut self, state: State, covariance: Covariance) -> Result<Alignment, InitError> {
            self.initialize_from(state, covariance, Timestamp::ZERO)
        }

        fn now(&self) -> Timestamp {
            self.time().unwrap_or_default()
        }
    }

    /// A window of exactly `Initialization::min_duration`: 8 samples at 4 Hz is 2 s.
    /// No barometer, so no reference is established.
    fn initialized() -> Eskf {
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert_eq!(alignment, Alignment::Static);
        filter
    }

    /// The same window with a barometer reading on every sample, scattered about `altitude`
    /// as a real one is. Eight identical readings are one reading held, which sets no
    /// reference. The offsets are exact in binary, so the mean is exactly `altitude`.
    fn window_at(altitude: f32) -> [StaticSample; 8] {
        window_with_baro([-0.5, 0.5, 0.0, -0.25, 0.25, 0.0, -0.125, 0.125].map(|d| altitude + d))
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
        aided_with(Config::default())
    }

    /// [`aided`] under another configuration.
    fn aided_with(config: Config) -> Eskf {
        let mut filter = Eskf::new(config).unwrap();
        let _ = filter
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(100.0),
                    AltitudeNoise::from_sigma(2.0)
                )
                .is_accepted()
        );
        filter
    }

    /// The floor of (42′) exists to be unreachable by an honest source, and this is the only
    /// place CI asserts it: `data/fetch.sh --check` pins `floored=` per corpus log — zero on
    /// seven, and 21 on `cd7e0001`, whose receiver claims 0.43 mm/s — and it needs a network
    /// and PX4 tooling, so it runs locally. The margin between the floor and
    /// anything a filter that is propagating and fusing reaches is measured in `math.rs`'s
    /// `FLOOR` — a count here means the floor is masking a collapse rather than preventing
    /// one, and the `sigma_*` columns of `examples/replay.rs` say which state.
    #[test]
    fn a_config_outside_its_bounds_builds_no_filter() {
        // A NaN timeout never compares true: built, this filter would never recover GNSS height.
        let config = Config {
            recovery: Recovery {
                gnss_height: Some(Seconds::from_secs(f32::NAN)),
                ..Recovery::default()
            },
            ..Config::default()
        };
        assert_eq!(
            Eskf::new(config).err(),
            Some(ConfigError {
                field: "recovery.gnss_height",
                bound: crate::config::ConfigBound::Positive,
            })
        );
        assert_eq!(Eskf::default().config(), &Config::default());
    }

    // Found by the adversarial suite (#44), `adversarial.rs`, and kept as literals so each
    // names the defect it guards.

    #[test]
    fn a_filter_with_no_barometric_reference_holds_no_offset() {
        let mut filter = initialized();
        assert_eq!(filter.baro_reference(), None);
        for _ in 0..10 {
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
        }
        assert_eq!(filter.offset, Offset::default());
    }

    #[test]
    fn a_start_that_overflows_is_refused_and_commits_nothing() {
        // Finite, so `push` takes it; (5) and (8) square it past `f32`.
        let mut filter = Eskf::default();
        let imu = ImuSample::from_rates(
            Timestamp::from_micros(1_000_000),
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, f32::MAX),
            DT,
        );
        assert_eq!(filter.initialize_coarse(imu), Err(InitError::NotFinite));
        assert!(!filter.is_initialized());
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn a_geodetic_fix_with_no_origin_under_it_turns_no_heading() {
        // A magnetometer-set heading is turned by the declination the first origin reads, so
        // a fix whose origin cannot be placed must not have turned it on the way to refusal.
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a still window");
        // Three metres out, so (44) must place an origin that puts a fix 3.4e38 m up three
        // metres away, which no `f64` resolves.
        let noise = PositionNoise::horizontal_vertical(1.0, 1.0);
        assert!(filter.reset_position_to(Position::ned(3.0, 0.0, 0.0), noise));
        let (attitude, declination) = (filter.state().attitude, filter.magnetic_declination());
        let outcome = filter.fuse_gnss_geodetic(
            filter.now(),
            Geodetic::from_degrees(56.0, 44.0, 3.4e38),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );
        assert_eq!(outcome.horizontal, Fusion::NoReference);
        assert_eq!(filter.state().attitude, attitude);
        assert_eq!(filter.magnetic_declination(), declination);
        assert_eq!(filter.origin(), None);
    }

    #[test]
    fn an_origin_the_position_cannot_be_written_about_is_refused() {
        let mut filter = initialized();
        assert!(filter.set_origin(Geodetic::from_degrees(47.4, 8.5, 488.0)));
        let held = (filter.origin(), filter.state().position);
        // A height `f64` holds and `f32` does not.
        assert!(!filter.set_origin(Geodetic::from_degrees(47.4, 8.5, -1.0e39)));
        assert_eq!((filter.origin(), filter.state().position), held);
    }

    #[test]
    fn an_ordinary_run_never_reaches_the_floor() {
        let mut filter = aided();
        for step in 0..400 {
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            if step % 25 == 0 {
                assert!(
                    filter
                        .fuse_gnss_position(
                            filter.now(),
                            Position::ned(0.0, 0.0, 0.0),
                            PositionNoise::horizontal_vertical(1.5, 1.5),
                            Position::zero(),
                        )
                        .is_accepted()
                );
                assert!(
                    filter
                        .fuse_baro_altitude(
                            filter.now(),
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
    /// static start with a magnetometer in the window holds its tilt for 3.83 s of unaided
    /// propagation and its heading for 37.8 s, at [`ImuNoise`](crate::ImuNoise)'s defaults.
    ///
    /// Not the `σ_g² t` the white-noise density alone would give — that is 10.4 s and 674 s.
    /// The gyroscope-bias prior reaches attitude through (20)'s `−I Δt` and accumulates as
    /// `σ_βg² t²`, which overtakes the white-noise term within two seconds and is what actually
    /// sets both figures. A test rather than a comment because [`Accuracy`] cites the numbers:
    /// change a bar or a density and this says by how much the margin moved.
    #[test]
    fn an_unaided_start_holds_its_attitude_for_the_margin_the_defaults_buy() {
        let mut filter = Eskf::default();
        assert_eq!(
            filter.initialize_over(&window_with_mag(), Seconds::from_secs(0.25)),
            Ok(Alignment::Static)
        );

        let dt = Seconds::from_secs(0.005);
        let holding_still = still().imu;
        let (mut tilt_held, mut heading_held) = (None, None);

        for step in 1..8_000 {
            assert_eq!(filter.step(holding_still, dt), Propagation::Propagated);
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
        assert!((tilt - 3.83).abs() < 0.05, "tilt held {tilt} s");
        assert!((heading - 37.8).abs() < 0.2, "heading held {heading} s");
    }

    #[test]
    fn predict_before_initialize_is_refused() {
        let mut filter = Eskf::default();
        assert_eq!(
            filter.step(ImuSample::default(), DT),
            Propagation::NotInitialized
        );
    }

    #[test]
    fn a_normal_step_propagates_and_advances_the_timers() {
        let mut filter = aided();
        assert_eq!(
            filter.step(ImuSample::default(), DT),
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
            filter.step(ImuSample::default(), DT),
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
        let mut filter = Eskf::default();
        let seed = State {
            velocity: Velocity::ned(1.0, 2.0, 3.0),
            ..State::default()
        };
        let enormous = Covariance::from_matrix(
            crate::state::CovarianceMatrix::from_diagonal_element(f32::MAX),
        );
        assert_eq!(filter.seed(seed, enormous), Ok(Alignment::Seeded));

        assert_eq!(
            filter.step(ImuSample::default(), DT),
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
            filter.step(ImuSample::default(), DT),
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
        let imu = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(f32::MAX, 0.0, -GRAVITY),
        );
        assert!(imu.is_finite());

        let mut before = filter.state();
        let mut steps = 0;
        loop {
            match filter.step(imu, DT) {
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
    fn a_sample_not_after_the_clock_is_refused_without_moving_the_timers_or_the_clock() {
        let mut filter = aided();
        let clock = filter.time();
        for bad in [0.0, -0.01] {
            let dt = Seconds::from_secs(bad);
            assert_eq!(
                filter.step(ImuSample::default(), dt),
                Propagation::InvalidStep { dt },
                "dt of {bad} should be refused"
            );
            assert_eq!(elapsed(&filter), 0.0, "dt of {bad} moved the timers");
            assert_eq!(filter.time(), clock, "dt of {bad} moved the clock");
        }
    }

    /// The horizon, both ends, at the barometer: a reading older than [`LATENCY_HORIZON`] or
    /// further ahead of the state than a step is refused, counted as a refusal and moves no
    /// timer, and one inside either bound reaches the gate.
    #[test]
    fn a_measurement_the_filter_cannot_place_in_time_is_refused() {
        let mut filter = aided();
        let now = filter.now();
        let altitude = Altitude::from_meters(100.0);
        let noise = AltitudeNoise::from_sigma(2.0);
        let before = filter.diagnostics().baro_altitude;

        let past = Seconds::from_secs(LATENCY_HORIZON.as_secs() + 0.001);
        let ahead = Seconds::from_secs(filter.config().max_predict_dt.as_secs() + 0.001);
        for (time, age) in [
            (now.before(past), past.as_secs()),
            (now.after(ahead), -ahead.as_secs()),
        ] {
            let Fusion::OutOfHorizon { age: refused } =
                filter.fuse_baro_altitude(time, altitude, noise)
            else {
                panic!("not refused");
            };
            assert!(
                (refused.as_secs() - age).abs() < 1e-6,
                "{refused:?} against {age}"
            );
        }
        let health = filter.diagnostics().baro_altitude;
        assert_eq!(health.refused, before.refused + 2);
        assert_eq!(health.last_refusal, Some(Refusal::OutOfHorizon));
        assert_eq!(health.time_since_accepted, before.time_since_accepted);

        let inside = Seconds::from_secs(LATENCY_HORIZON.as_secs() - 0.001);
        assert!(
            filter
                .fuse_baro_altitude(now.before(inside), altitude, noise)
                .is_accepted()
        );
        assert!(
            filter
                .fuse_baro_altitude(now.after(DT), altitude, noise)
                .is_accepted()
        );
    }

    /// An interval is the sample's, not time passing: refused, but the clock and the timers
    /// move as they do for a sample carrying a NaN.
    #[test]
    fn a_sample_with_an_unusable_interval_is_refused_after_the_time_passes() {
        let mut filter = aided();
        let before = filter.state();
        let time = filter.time().expect("initialized").after(DT);
        let backwards = Seconds::from_secs(-0.005);
        let imu = ImuSample {
            angle_interval: backwards,
            ..still().imu.timed(time, DT)
        };
        assert_eq!(
            filter.predict(imu),
            Propagation::InvalidInterval {
                interval: backwards
            }
        );
        assert_eq!(filter.state(), before);
        assert_eq!(filter.time(), Some(time));
        assert!((elapsed(&filter) - DT.as_secs()).abs() < 1e-6);
        assert_eq!(filter.diagnostics().propagation.refused_invalid, 1);
    }

    /// A gap is time no sample describes, and only the timestamps see it: a logger that
    /// dropped samples hands over an increment integrated over one IMU interval, a second
    /// after the last one.
    #[test]
    fn a_gap_is_read_off_the_timestamps_not_the_interval() {
        let mut filter = aided();
        let gap = Seconds::from_secs(1.0);
        let time = filter.time().expect("initialized").after(gap);
        assert_eq!(
            filter.predict(still().imu.timed(time, DT)),
            Propagation::Coasted { dt: gap }
        );
    }

    /// Coasting off, so a long step is refused.
    fn not_coasting() -> Config {
        Config {
            coast: None,
            ..Config::default()
        }
    }

    #[test]
    fn a_step_over_the_limit_is_refused_with_coasting_off_but_the_time_still_passes() {
        let mut filter = aided_with(not_coasting());
        let before = filter.state();
        // The worst SD-card dropout in the bundled corpus.
        let dt = Seconds::from_secs(1.304);
        assert!(matches!(
            filter.step(ImuSample::default(), dt),
            Propagation::StepTooLong { .. }
        ));
        assert_eq!(filter.state(), before, "a refused step moves nothing");
        assert_eq!(
            elapsed(&filter),
            1.304,
            "a refused step still happened in real time"
        );
    }

    /// An aided filter flying north at 20 m/s, as `logging_dropout` does into its gap.
    fn flying(config: Config) -> Eskf {
        let mut filter = aided_with(config);
        assert!(filter.reset_position_to(
            Position::ned(0.0, 0.0, -100.0),
            PositionNoise::horizontal_vertical(0.5, 0.5)
        ));
        assert!(filter.reset_velocity_to(
            Velocity::ned(20.0, 0.0, 0.0),
            VelocityNoise::from_speed_accuracy(0.1)
        ));
        filter
    }

    #[test]
    fn a_gap_is_coasted_on_the_estimated_velocity_and_the_time_still_passes() {
        let mut filter = flying(Config::default());
        let before = filter.state();
        let gap = Seconds::from_secs(1.2);
        assert_eq!(
            filter.step(still().imu, gap),
            Propagation::Coasted { dt: gap }
        );

        let after = filter.state();
        let moved = after.position.vector() - before.position.vector();
        assert!((moved.x - 24.0).abs() < 1e-3, "north {} m", moved.x);
        assert!(moved.y.abs() < 1e-6 && moved.z.abs() < 1e-6, "{moved:?}");
        assert_eq!(
            after.velocity, before.velocity,
            "nothing measured it change"
        );
        assert_eq!(after.attitude, before.attitude, "nothing measured it turn");
        assert_eq!(elapsed(&filter), 1.2, "the gap happened in real time");

        let propagation = filter.diagnostics().propagation;
        assert_eq!(propagation.coasted, 1);
        assert_eq!(propagation.refused_too_long, 0, "a coast is not a refusal");
        assert_eq!(propagation.longest_gap, Some(gap));
    }

    #[test]
    fn a_coast_discarded_as_non_finite_still_measures_the_gap() {
        // Finite, so `Config::validate` passes it, and squared into `P` it overflows.
        let mut filter = flying(Config {
            coast: Some(Coast {
                acceleration: f32::MAX,
                rotation: 0.1,
            }),
            ..Config::default()
        });
        let before = filter.state();
        let gap = Seconds::from_secs(1.2);
        assert_eq!(filter.step(still().imu, gap), Propagation::StateNotFinite);
        assert_eq!(filter.state(), before, "a discarded coast commits nothing");
        let propagation = filter.diagnostics().propagation;
        assert_eq!(propagation.longest_gap, Some(gap));
        assert_eq!(propagation.coasted, 0);
        assert_eq!(propagation.refused_state_not_finite, 1);
    }

    #[test]
    fn a_coast_grows_velocity_by_the_unmeasured_acceleration() {
        let acceleration = 3.0;
        let gap = Seconds::from_secs(1.2);
        let variances = |acceleration| {
            let mut filter = flying(Config {
                coast: Some(Coast {
                    acceleration,
                    rotation: 0.0,
                }),
                ..Config::default()
            });
            let _ = filter.step(still().imu, gap);
            let p = *filter.covariance().as_matrix();
            (
                p[(
                    ErrorState::VelocityNorth.index(),
                    ErrorState::VelocityNorth.index(),
                )],
                p[(
                    ErrorState::PositionNorth.index(),
                    ErrorState::PositionNorth.index(),
                )],
            )
        };
        let (velocity, position) = variances(acceleration);
        let (velocity_q, position_q) = variances(0.0);
        // Exactly the white-noise integral over the gap: `a² Δt` on velocity, `a² Δt³ / 3`
        // on position, whatever the step count.
        let expected = acceleration * acceleration * gap.as_secs();
        assert!(
            ((velocity - velocity_q) - expected).abs() < 1e-3 * expected,
            "velocity grew {} over (22) alone, expected {expected}",
            velocity - velocity_q
        );
        let integrated = expected * gap.as_secs() * gap.as_secs() / 3.0;
        let grown = position - position_q;
        assert!(
            (grown - integrated).abs() < 1e-3 * integrated,
            "position grew {grown}, expected {integrated}"
        );
    }

    #[test]
    fn a_coast_grows_attitude_by_the_unmeasured_rotation() {
        let rotation = 0.1;
        let gap = Seconds::from_secs(1.2);
        let variance = |rotation| {
            let mut filter = flying(Config {
                coast: Some(Coast {
                    acceleration: 0.0,
                    rotation,
                }),
                ..Config::default()
            });
            let _ = filter.step(still().imu, gap);
            let p = *filter.covariance().as_matrix();
            p[(ErrorState::AttitudeZ.index(), ErrorState::AttitudeZ.index())]
        };
        // The attitude block of `F` is the identity at `ω = 0`, so the density lands whole.
        let grown = variance(rotation) - variance(0.0);
        let expected = rotation * rotation * gap.as_secs();
        assert!(
            (grown - expected).abs() < 1e-3 * expected,
            "attitude grew {grown} over (22) alone, expected {expected}"
        );
    }

    #[test]
    fn a_coast_walks_the_barometric_offset_across_the_gap() {
        // (30′)'s offset drifts whether or not the IMU was logged.
        let mut filter = flying(Config::default());
        let before = filter.offset.variance;
        let gap = Seconds::from_secs(1.2);
        let _ = filter.step(still().imu, gap);
        let walk = filter.config.baro_offset_walk;
        let expected = walk * walk * gap.as_secs();
        let grown = filter.offset.variance - before;
        assert!(
            (grown - expected).abs() < 1e-3 * expected,
            "offset variance grew {grown}, expected {expected}"
        );
    }

    #[test]
    fn the_first_fix_after_a_coasted_gap_is_accepted_where_a_refused_one_is_rejected() {
        // Where the vehicle is after 1.2 s at 20 m/s north.
        let fix = Position::ned(24.0, 0.0, -100.0);
        let noise = PositionNoise::horizontal_vertical(1.0, 1.0);
        let gap = Seconds::from_secs(1.2);

        let mut coasting = flying(Config::default());
        let _ = coasting.step(still().imu, gap);
        assert!(
            coasting
                .fuse_gnss_position(coasting.now(), fix, noise, Position::zero())
                .horizontal
                .is_accepted()
        );

        let mut refusing = flying(not_coasting());
        let _ = refusing.step(still().imu, gap);
        assert!(matches!(
            refusing
                .fuse_gnss_position(refusing.now(), fix, noise, Position::zero())
                .horizontal,
            Fusion::Rejected { .. }
        ));
    }

    #[test]
    fn a_non_finite_sample_is_refused_but_the_time_still_passes() {
        let level = Acceleration::body(0.0, 0.0, -GRAVITY);
        let still = AngularRate::body(0.0, 0.0, 0.0);
        for (name, imu) in [
            (
                "NaN gyro",
                ImuSample::reading(AngularRate::body(f32::NAN, 0.0, 0.0), level),
            ),
            (
                "NaN accel",
                ImuSample::reading(still, Acceleration::body(0.0, f32::NAN, -GRAVITY)),
            ),
            (
                "infinite accel",
                ImuSample::reading(still, Acceleration::body(0.0, 0.0, f32::INFINITY)),
            ),
        ] {
            let mut filter = aided();
            let before = filter.state();
            assert_eq!(filter.step(imu, DT), Propagation::NotFinite, "{name}");
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
        // A coast reads no sample, so the NaN does not stop it either.
        let dt = Seconds::from_secs(1.304);
        let imu = ImuSample::reading(
            AngularRate::body(f32::NAN, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );
        for (config, coasted) in [(Config::default(), true), (not_coasting(), false)] {
            let mut filter = aided_with(config);
            let outcome = filter.step(imu, dt);
            assert_eq!(matches!(outcome, Propagation::Coasted { .. }), coasted);
            assert_eq!(matches!(outcome, Propagation::StepTooLong { .. }), !coasted);
            assert!(
                filter
                    .state()
                    .position
                    .vector()
                    .iter()
                    .all(|x| x.is_finite())
            );
            let propagation = filter.diagnostics().propagation;
            assert_eq!(propagation.longest_gap, Some(dt));
            assert_eq!(propagation.refused_not_finite, 0);
        }
    }

    #[test]
    fn a_refused_measurement_is_visible_in_diagnostics_not_only_in_the_return_value() {
        // The failure this guards: a miswired sensor feeding NaN reads as `never accepted`,
        // exactly like one that was never connected, unless the refusal is counted.
        let mut filter = initialized();
        let nan = Velocity::ned(f32::NAN, 0.0, 0.0);
        let noise = VelocityNoise::from_speed_accuracy(0.3);

        assert_eq!(
            filter.fuse_gnss_velocity(filter.now(), nan, noise, Position::zero()),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                filter.now(),
                Velocity::ned(1.0, 0.0, 0.0),
                VelocityNoise::from_variance(0.0, 1.0, 1.0),
                Position::zero()
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
        let mut filter = coarse();
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::NoReference
        );
        assert_eq!(
            filter.diagnostics().baro_altitude.last_refusal,
            Some(Refusal::NoReference),
            "a missing reference is a different problem from a bad number"
        );

        let mut fresh = Eskf::default();
        assert_eq!(
            fresh.fuse_baro_altitude(
                fresh.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
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
            filter.fuse_gnss_position(
                filter.now(),
                Position::ned(120.0, -40.0, -75.0),
                noise,
                Position::zero()
            ),
            GnssFusion::both(Fusion::Reset)
        );
        // The second fix has an estimate to be judged against, so it is fused, not adopted.
        assert!(
            filter
                .fuse_gnss_position(
                    filter.now(),
                    Position::ned(121.0, -40.0, -75.0),
                    noise,
                    Position::zero()
                )
                .is_accepted()
        );

        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.adopted, 1, "adoption happens once per quantity");
        assert_eq!(health.accepted, 2, "and counts as an acceptance besides");
    }

    #[test]
    fn propagation_refusals_are_counted_and_the_worst_gap_kept() {
        let mut filter = aided_with(not_coasting());
        for bad in [0.0, -0.01] {
            assert!(
                !filter
                    .step(ImuSample::default(), Seconds::from_secs(bad))
                    .is_propagated()
            );
        }
        for gap in [0.34, 1.304, 0.5] {
            assert!(
                !filter
                    .step(ImuSample::default(), Seconds::from_secs(gap))
                    .is_propagated()
            );
        }

        let propagation = filter.diagnostics().propagation;
        assert_eq!(propagation.refused_invalid, 2);
        assert_eq!(propagation.refused_too_long, 3);
        assert_eq!(
            propagation.longest_gap.map(Seconds::as_secs),
            Some(1.304),
            "the worst gap, not the last"
        );
    }

    #[test]
    fn before_position_is_established_an_altitude_is_refused_not_referred_to_nothing() {
        let mut filter = coarse();
        assert_eq!(filter.baro_reference(), None);
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::NoReference
        );
        assert_eq!(filter.baro_reference(), None);
        assert_eq!(
            filter.diagnostics().baro_altitude.time_since_accepted,
            None,
            "a refused measurement is not aiding"
        );
    }

    #[test]
    fn after_a_coarse_start_the_first_altitude_past_the_first_fix_sets_the_reference() {
        let mut filter = coarse();
        // Every reading here arrives with no step between it and the last, which (24′) takes
        // for the same error twice; this is (30′)'s arithmetic, on white sources.
        filter.config.correlation = Correlation::WHITE;
        assert_eq!(
            filter.fuse_gnss_position(
                filter.now(),
                Position::ned(10.0, -4.0, -30.0),
                PositionNoise::from_sigma(1.0, 1.0, 2.8),
                Position::zero()
            ),
            GnssFusion::both(Fusion::Reset)
        );

        // 30 m up by the fix, 130 m by the barometer: α₀ = α + p̂_D = 100 m, and the
        // altitude lands on the estimate rather than moving it.
        let before = filter.state().position;
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(130.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::Accepted { test_ratio: 0.0 }
        );
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));
        assert_eq!(filter.state().position, before, "spent on α₀, not fused");
        assert_eq!(filter.diagnostics().baro_altitude.accepted, 1);

        // Seeded correlated with the height it was read against, (30′): P_bb = P_DD + R_m,
        // P_xb = −P[:, D]. An independent σ here is the constant offset that took
        // `moving_start`'s `nees_pos` to 112.59.
        let down = ErrorState::PositionDown;
        let p_dd = filter.covariance().variance(down);
        assert!((filter.offset.variance - (p_dd + 4.0)).abs() < 1e-4);
        assert_eq!(
            filter.offset.cross,
            -filter.covariance().as_matrix().column(down.index())
        );

        // So the barometer alone says nothing about absolute height: h = p_D + b has no
        // covariance with p_D, and a second reading two metres higher moves α₀, not the
        // estimate. Only GNSS height, disagreeing with both, can.
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(132.0),
                    AltitudeNoise::from_sigma(2.0)
                )
                .is_accepted()
        );
        assert!((filter.state().position.vector()[2] + 30.0).abs() < 1e-3);
        let reference = filter.baro_reference().expect("held").as_meters();
        assert!(reference > 100.5, "{reference}");
    }

    #[test]
    fn with_the_seed_turned_off_an_altitude_waits_for_a_named_reference() {
        let mut filter = Eskf::new(Config {
            baro_reference_from_estimate: false,
            ..Config::default()
        })
        .unwrap();
        let _ = filter
            .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::NoReference
        );
        assert_eq!(filter.baro_reference(), None);
        assert!(
            filter.set_baro_reference(Altitude::from_meters(60.0), AltitudeNoise::from_sigma(0.5))
        );
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(60.0),
                    AltitudeNoise::from_sigma(2.0)
                )
                .is_accepted()
        );
    }

    #[test]
    fn a_static_window_with_no_barometer_takes_the_reference_from_the_first_altitude() {
        // A barometer that came online after the window closed. The static start put the
        // vehicle at the origin, so the first reading is the ground.
        let mut filter = initialized();
        assert_eq!(filter.baro_reference(), None);
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::Accepted { test_ratio: 0.0 }
        );
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(60.0)));
    }

    #[test]
    fn an_altitude_above_the_reference_pulls_the_estimate_up_and_moves_nothing_sideways() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");

        // Two metres above the reference the window fixed, on a sensor claiming 0.5 m.
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(102.0),
                    AltitudeNoise::from_sigma(0.5)
                )
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
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let before = filter.state().position;

        // A hundred metres of climb the instant the window closed, on a 0.5 m sensor: the
        // shape of a pressure transient, and what `Gates::baro_altitude` is there for.
        let outcome = filter.fuse_baro_altitude(
            filter.now(),
            Altitude::from_meters(200.0),
            AltitudeNoise::from_sigma(0.5),
        );
        assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        assert_eq!(filter.state().position, before);
        assert_eq!(filter.diagnostics().baro_altitude.rejected, 1);
    }

    #[test]
    fn a_static_restart_replaces_the_reference() {
        let mut filter = aided();
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));
        let alignment = filter
            .initialize_over(&window_at(250.0), Seconds::from_secs(0.25))
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
        let mut window = window_at(altitude);
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        window
    }

    #[test]
    fn a_restart_in_motion_keeps_the_reference_the_flight_began_with() {
        let mut filter = aided();

        // Taking 250 m as the reference would call the current altitude zero: the
        // barometer would then say z ~ 0 while GNSS about the flight's origin says
        // z ~ -150.
        let alignment = filter
            .initialize_over(&moving_window_at(250.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));

        // Short as well as moving, which is what a restart in flight usually offers. The
        // window is too short to measure motion for `classify`, but `α₀` measures it
        // anyway rather than reading window length as stillness.
        let _ = filter
            .initialize_over(&moving_window_at(250.0), Seconds::from_secs(0.1))
            .expect("short and moving");
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));

        // And a moving window with no barometer in it does not wipe the reference either.
        let mut window = [still(); 8];
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let _ = filter
            .initialize_over(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));
    }

    #[test]
    fn a_window_too_short_to_align_from_still_names_the_ground_it_sat_on() {
        // `classify` reports a short window as coarse before it ever measures motion, so
        // alignment cannot answer this: a vehicle sitting on the ground with 0.8 s of
        // samples has an honest reference, and reading "coarse" as "moving" would have
        // cost it barometric aiding for the whole flight with nothing saying so.
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(&window_at(112.0), Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(112.0)));
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(112.0),
                    AltitudeNoise::from_sigma(2.0)
                )
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

    /// Equation (23′): a fix 250 ms old on a vehicle at 20 m/s is where the vehicle was, 5 m
    /// back along its track. Taken at its own time it agrees with the estimate; taken as
    /// current it is 5 m of error the gate turns down.
    #[test]
    fn an_old_fix_is_judged_against_where_the_vehicle_was() {
        let flying = || {
            let mut filter = Eskf::default();
            let state = State {
                velocity: Velocity::ned(20.0, 0.0, 0.0),
                ..State::default()
            };
            let _ = filter
                .seed(state, Covariance::from_sigmas([0.5; STATES]))
                .expect("a sane seed");
            for _ in 0..50 {
                assert!(filter.step(still().imu, DT).is_propagated());
            }
            filter
        };
        let age = Seconds::from_secs(0.25);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);

        let mut filter = flying();
        let there = Position::ned(filter.state().position.x() - 5.0, 0.0, 0.0);
        let taken = filter.now().before(age);
        let aged = filter
            .fuse_gnss_position(taken, there, noise, Position::zero())
            .horizontal;
        assert!(
            aged.test_ratio().is_some_and(|ratio| ratio < 1.0e-3),
            "{aged:?}"
        );

        let mut filter = flying();
        let now = filter.now();
        let current = filter
            .fuse_gnss_position(now, there, noise, Position::zero())
            .horizontal;
        assert!(matches!(current, Fusion::Rejected { .. }), "{current:?}");
    }

    /// A start in motion says nothing about where the vehicle was before it, so a fix taken
    /// before a seed is refused; a still window says the vehicle was where it started, so one
    /// taken before that start is placed there and fused.
    #[test]
    fn a_fix_from_before_the_start_is_placed_only_after_a_start_at_rest() {
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let before = Seconds::from_secs(0.05);

        let mut seeded = Eskf::default();
        let (state, covariance) = seed();
        let start = Timestamp::from_micros(1_000_000);
        let _ = seeded
            .initialize_from(state, covariance, start)
            .expect("a sane seed");
        let taken = start.before(before);
        let refused =
            seeded.fuse_gnss_position(taken, Position::ned(0.0, 0.0, 0.0), noise, Position::zero());
        assert_eq!(
            refused,
            GnssFusion::both(Fusion::OutOfHorizon { age: before })
        );

        let mut still = initialized();
        let taken = still.now().before(before);
        let fused =
            still.fuse_gnss_position(taken, Position::ned(0.0, 0.0, 0.0), noise, Position::zero());
        assert!(fused.is_accepted(), "{fused:?}");
    }

    /// A correction reaches the past it was propagated from, so a second fix of the same moment
    /// is judged against the corrected past and agrees with it better than the first did. Were
    /// the history left behind, the second would innovate as the first did against a covariance
    /// the first had already narrowed: 1.33 against 0.12, where this reads 0.08.
    #[test]
    fn a_correction_reaches_the_history_the_next_old_fix_is_judged_against() {
        let mut filter = Eskf::default();
        let _ = filter
            .seed(State::default(), Covariance::from_sigmas([2.0; STATES]))
            .expect("a sane seed");
        for _ in 0..50 {
            assert!(filter.step(still().imu, DT).is_propagated());
        }
        let taken = filter.now().before(Seconds::from_secs(0.2));
        let off = Position::ned(3.0, 0.0, 0.0);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let first = filter
            .fuse_gnss_position(taken, off, noise, Position::zero())
            .horizontal;
        let second = filter
            .fuse_gnss_position(taken, off, noise, Position::zero())
            .horizontal;
        let (Some(first), Some(second)) = (first.test_ratio(), second.test_ratio()) else {
            panic!("both fixes reach the gate: {first:?}, {second:?}");
        };
        assert!(second < first, "{second} against {first}");
    }

    /// A fix timed ahead of the last IMU sample is where the vehicle will be: carried forward
    /// on the velocity rather than taken as current.
    #[test]
    fn a_fix_ahead_of_the_state_is_judged_against_where_the_vehicle_will_be() {
        let mut filter = Eskf::default();
        let state = State {
            velocity: Velocity::ned(20.0, 0.0, 0.0),
            ..State::default()
        };
        let _ = filter
            .seed(state, Covariance::from_sigmas([0.5; STATES]))
            .expect("a sane seed");
        for _ in 0..50 {
            assert!(filter.step(still().imu, DT).is_propagated());
        }
        let lead = Seconds::from_secs(0.05);
        let there = Position::ned(filter.state().position.x() + 1.0, 0.0, 0.0);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let taken = filter.now().after(lead);
        let outcome = filter
            .fuse_gnss_position(taken, there, noise, Position::zero())
            .horizontal;
        assert!(
            outcome.test_ratio().is_some_and(|ratio| ratio < 1.0e-3),
            "{outcome:?}"
        );
    }

    /// A heading timed ahead of the last IMU sample is judged against the attitude the turn
    /// will have reached, carried forward on the last sample's rate. Taken against the present
    /// instead, 90 ms into a 90°/s turn is 8° of error, which the gate turns down.
    #[test]
    fn a_heading_ahead_of_the_state_is_judged_against_the_turn() {
        let rate = core::f32::consts::FRAC_PI_2;
        let mut sigmas = [0.5; STATES];
        // Attitude and gyroscope bias tight, so that the covariance cannot absorb the turn.
        for axis in [
            ErrorState::AttitudeX,
            ErrorState::AttitudeY,
            ErrorState::AttitudeZ,
        ] {
            sigmas[axis.index()] = 0.005;
        }
        for axis in [
            ErrorState::GyroBiasX,
            ErrorState::GyroBiasY,
            ErrorState::GyroBiasZ,
        ] {
            sigmas[axis.index()] = 0.001;
        }
        let mut filter = Eskf::default();
        let _ = filter
            .seed(State::default(), Covariance::from_sigmas(sigmas))
            .expect("a sane seed");
        let turning = still().imu.with_gyro(AngularRate::body(0.0, 0.0, rate));
        for _ in 0..20 {
            assert!(filter.step(turning, DT).is_propagated());
        }
        let lead = Seconds::from_secs(0.09);
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        let field_at = |yaw: f32| MagField::body(0.22 * yaw.cos(), -0.22 * yaw.sin(), 0.44);
        let noise = HeadingNoise::from_sigma(0.02);

        let mut judged = filter.clone();
        let ahead = field_at(yaw + rate * lead.as_secs());
        let outcome = judged.fuse_mag_heading(filter.now().after(lead), ahead, noise);
        assert!(
            outcome.test_ratio().is_some_and(|ratio| ratio < 0.05),
            "{outcome:?}"
        );
        let current = filter.fuse_mag_heading(filter.now(), ahead, noise);
        assert!(matches!(current, Fusion::Rejected { .. }), "{current:?}");
    }

    /// An interval longer than any step the filter integrates is refused, whatever the
    /// timestamps say: one sample does not describe it.
    #[test]
    fn an_interval_longer_than_the_limit_is_refused() {
        let mut filter = aided();
        let before = filter.state();
        let limit = filter.config().max_predict_dt;
        let time = filter.now().after(DT);
        let long = Seconds::from_secs(limit.as_secs() * 25.0);
        let imu = ImuSample {
            velocity_interval: long,
            ..still().imu.timed(time, DT)
        };
        assert_eq!(
            filter.predict(imu),
            Propagation::InvalidInterval { interval: long }
        );
        assert_eq!(filter.state(), before);
    }

    #[test]
    fn a_seed_becomes_the_state_and_the_covariance() {
        let mut filter = Eskf::default();
        let (state, covariance) = seed();
        let _ = filter.seed(state, covariance).expect("a sane seed");
        assert!(filter.is_initialized());
        assert_eq!(filter.state().velocity, state.velocity);
        assert_eq!(filter.state().gyro_bias, state.gyro_bias);
        assert_eq!(filter.covariance(), &covariance);
    }

    #[test]
    fn a_seed_takes_its_baro_reference_from_the_first_altitude() {
        let mut filter = Eskf::default();
        let (state, covariance) = seed();
        let _ = filter.seed(state, covariance).expect("a sane seed");
        assert_eq!(filter.baro_reference(), None);

        // The seed holds p_D = 0, so the reading is its zero.
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::Accepted { test_ratio: 0.0 }
        );
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(60.0)));
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(60.5),
                    AltitudeNoise::from_sigma(2.0)
                )
                .is_accepted()
        );
    }

    #[test]
    fn reinitializing_in_flight_keeps_the_reference_the_flight_began_with() {
        let mut filter = aided();
        let (state, covariance) = seed();
        let _ = filter.seed(state, covariance).expect("a sane seed");
        assert_eq!(
            filter.baro_reference(),
            Some(Altitude::from_meters(100.0)),
            "the barometer did not change when the filter restarted"
        );
    }

    #[test]
    fn a_window_at_rest_measures_its_own_reference_variance() {
        let mut filter = Eskf::default();
        let window = window_at(100.0);
        let _ = filter
            .initialize_over(&window, Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert_eq!(filter.offset.cross.norm(), 0.0);
        // Squared deviations of 0.65625 m² over seven degrees of freedom, then over eight
        // readings: the standard error of their mean.
        let want = 0.65625 / 7.0 / 8.0;
        assert!(
            (filter.offset.variance - want).abs() < 2e-3 * want,
            "{}",
            filter.offset.variance
        );
    }

    #[test]
    fn a_barometer_the_receiver_disagrees_with_moves_its_reference() {
        // The barometer reads 3 m above where the receiver puts the vehicle, every time. A
        // constant `α₀` would split the difference in the height for ever; an estimated one
        // takes the disagreement into the reference, and the height follows the receiver.
        // The receiver is white, as a constant fix is, and says so: at (24′)'s default a
        // 10 Hz fix is worth 1/280 of one, and 30 s moves the reference 0.8 m of the 3.
        let mut filter = aided();
        filter.config.correlation = Correlation::WHITE;
        for step in 0..3000 {
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            if step % 10 == 0 {
                let _ = filter.fuse_gnss_position(
                    filter.now(),
                    Position::ned(0.0, 0.0, 0.0),
                    PositionNoise::from_sigma(1.0, 1.0, 1.0),
                    Position::zero(),
                );
                let _ = filter.fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(103.0),
                    AltitudeNoise::from_sigma(0.5),
                );
            }
        }
        let reference = filter
            .baro_reference()
            .expect("the window set one")
            .as_meters();
        assert!(reference > 102.0, "α₀ = {reference}");
        assert!(filter.state().position.vector()[2].abs() < 0.5);
    }

    #[test]
    fn a_run_of_correlated_fixes_leaves_more_uncertainty_than_a_run_of_white_ones() {
        // Twenty-five fixes at 5 Hz. As white, (24) averages them down; at (24′)'s default,
        // each is worth 1/42 of one horizontally and 1/140 in height, since the receiver's
        // error has barely moved between them. Worth less is not worth nothing: each half reads
        // its own clock, and one read off the other would see the horizontal half restart it
        // an instant before and take every height for the same error twice.
        let mut white = aided();
        white.config.correlation = Correlation::WHITE;
        let (mut correlated, mut unaided) = (aided(), aided());
        for step in 0..500 {
            for filter in [&mut white, &mut correlated, &mut unaided] {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
            if step % 20 == 0 {
                for filter in [&mut white, &mut correlated] {
                    let _ = filter.fuse_gnss_position(
                        filter.now(),
                        Position::ned(0.0, 0.0, 0.0),
                        PositionNoise::from_sigma(1.0, 1.0, 1.0),
                        Position::zero(),
                    );
                }
            }
        }
        for axis in [ErrorState::PositionNorth, ErrorState::PositionDown] {
            let variance = |filter: &Eskf| filter.covariance().variance(axis);
            let (w, c, u) = (variance(&white), variance(&correlated), variance(&unaided));
            assert!(c > 3.0 * w, "{axis:?}: {c} against {w} white");
            assert!(c < 0.95 * u, "{axis:?}: {c} against {u} with no fix at all");
        }
    }

    #[test]
    fn every_source_is_fused_at_its_own_correlation() {
        // Velocity at 5 Hz and heading at 20 Hz for 5 s: as white, (24) averages each down;
        // at their defaults each solution is worth about a fifth, each heading about a
        // thirty-fifth. Each source reads its own `τ` and its own clock.
        let mut white = initialized();
        white.config.correlation = Correlation::WHITE;
        let mut correlated = initialized();
        for step in 0..500 {
            for filter in [&mut white, &mut correlated] {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
                if step % 20 == 0 {
                    let _ = filter.fuse_gnss_velocity(
                        filter.now(),
                        Velocity::ned(0.0, 0.0, 0.0),
                        VelocityNoise::from_speed_accuracy(0.3),
                        Position::zero(),
                    );
                }
                if step % 5 == 0 {
                    let _ = filter.fuse_mag_heading(
                        filter.now(),
                        measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                        HeadingNoise::from_sigma(0.05),
                    );
                }
            }
        }
        for axis in [ErrorState::VelocityNorth, ErrorState::AttitudeZ] {
            let (w, c) = (
                white.covariance().variance(axis),
                correlated.covariance().variance(axis),
            );
            assert!(c > 2.0 * w, "{axis:?}: {c} against {w} white");
        }
    }

    #[test]
    fn a_refused_fix_does_not_restart_the_interval_of_24_prime() {
        // A receiver interleaving unusable fixes with good ones carries no error in the bad
        // ones, so the good ones are as far apart as they were.
        let good = |filter: &mut Eskf| {
            let _ = filter.fuse_gnss_position(
                filter.now(),
                Position::ned(0.0, 0.0, 0.0),
                PositionNoise::from_sigma(1.0, 1.0, 1.0),
                Position::zero(),
            );
        };
        let (mut interleaved, mut clean) = (aided(), aided());
        for filter in [&mut interleaved, &mut clean] {
            good(filter);
            for _ in 0..10 {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
        }
        let refused = interleaved.fuse_gnss_position(
            interleaved.now(),
            Position::ned(f32::NAN, f32::NAN, f32::NAN),
            PositionNoise::from_sigma(1.0, 1.0, 1.0),
            Position::zero(),
        );
        assert_eq!(refused, GnssFusion::both(Fusion::NotFinite));
        for filter in [&mut interleaved, &mut clean] {
            for _ in 0..10 {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
            good(filter);
        }
        let north = ErrorState::PositionNorth;
        assert_eq!(
            interleaved.covariance().variance(north),
            clean.covariance().variance(north)
        );
    }

    #[test]
    fn a_rejected_fix_or_a_refused_half_leaves_the_interval_alone() {
        // (24′) discounts a fix for the error it shares with those already fused, and neither a
        // rejected fix nor a refused half fused anything: the next is timed from the last fix
        // accepted, 110 ms before it, not from the one turned away 10 ms before. The half that
        // was accepted does restart its own clock, which the two halves keep apart.
        let fix = |filter: &mut Eskf, position: Position<Ned>, noise: PositionNoise<Ned>| {
            let _ = filter.fuse_gnss_position(filter.now(), position, noise, Position::zero());
        };
        let one = PositionNoise::from_sigma(1.0, 1.0, 1.0);
        let steps = |filter: &mut Eskf, n: usize| {
            for _ in 0..n {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
        };
        let (mut rejected, mut half, mut clean) = (aided(), aided(), aided());
        for filter in [&mut rejected, &mut half, &mut clean] {
            fix(filter, Position::ned(0.0, 0.0, 0.0), one);
            steps(filter, 10);
        }
        let outcome = rejected.fuse_gnss_position(
            rejected.now(),
            Position::ned(1000.0, 0.0, 1000.0),
            one,
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Rejected { .. }),
            "{outcome:?}"
        );
        let outcome = half.fuse_gnss_position(
            half.now(),
            Position::ned(0.0, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.0, f32::NAN),
            Position::zero(),
        );
        assert_eq!(outcome.height, Fusion::NotFinite);
        for filter in [&mut rejected, &mut half, &mut clean] {
            steps(filter, 1);
            fix(filter, Position::ned(0.0, 0.0, 0.0), one);
        }
        let (north, down) = (ErrorState::PositionNorth, ErrorState::PositionDown);
        let variance = |filter: &Eskf, axis| filter.covariance().variance(axis);
        assert_eq!(variance(&rejected, north), variance(&clean, north));
        assert_eq!(variance(&half, down), variance(&clean, down));
    }

    #[test]
    fn the_interval_of_24_prime_restarts_with_the_filter() {
        // The filter's clock restarts at initialization, so the fix before it is not the
        // previous one. Were it kept, the first fix after 0.2 s of the new clock would be
        // fused at 1/42 of its weight.
        let fix = |filter: &mut Eskf| {
            let _ = filter.fuse_gnss_position(
                filter.now(),
                Position::ned(0.0, 0.0, 0.0),
                PositionNoise::from_sigma(1.0, 1.0, 1.0),
                Position::zero(),
            );
        };
        let mut restarted = aided();
        fix(&mut restarted);
        let _ = restarted
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let mut fresh = aided();
        for filter in [&mut restarted, &mut fresh] {
            for _ in 0..20 {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
            fix(filter);
        }
        let north = ErrorState::PositionNorth;
        assert_eq!(
            restarted.covariance().variance(north),
            fresh.covariance().variance(north)
        );
    }

    #[test]
    fn an_adopted_position_carries_no_correlation_with_the_reference() {
        let mut filter = aided();
        for _ in 0..100 {
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
        }
        let _ = filter.fuse_baro_altitude(
            filter.now(),
            Altitude::from_meters(100.0),
            AltitudeNoise::from_sigma(0.5),
        );
        let down = ErrorState::PositionDown.index();
        assert!(
            filter.offset.cross[down] != 0.0,
            "the altitude correlated the two"
        );

        assert!(filter.reset_position_to(
            Position::ned(1.0, 2.0, -3.0),
            PositionNoise::from_sigma(1.0, 1.0, 1.0)
        ));
        assert_eq!(filter.offset.cross.fixed_rows::<3>(0).norm(), 0.0);
    }

    #[test]
    fn a_reference_with_no_uncertainty_is_refused() {
        let mut filter = initialized();
        assert!(
            !filter.set_baro_reference(Altitude::from_meters(52.0), AltitudeNoise::from_sigma(0.0))
        );
        assert_eq!(filter.baro_reference(), None);
    }

    #[test]
    fn a_seed_that_is_not_finite_is_refused_and_nothing_is_initialized() {
        let (state, covariance) = seed();
        let poisoned = State {
            position: Position::ned(f32::NAN, 0.0, 0.0),
            ..state
        };
        let mut filter = Eskf::default();
        assert_eq!(filter.seed(poisoned, covariance), Err(InitError::NotFinite));
        assert!(!filter.is_initialized(), "a refused seed leaves no state");

        // The same check on the covariance, which is where a stale warm start off
        // storage tends to arrive broken.
        let mut matrix = *covariance.as_matrix();
        matrix[(
            ErrorState::VelocityNorth.index(),
            ErrorState::VelocityNorth.index(),
        )] = -1.0;
        assert_eq!(
            filter.seed(state, Covariance::from_matrix(matrix)),
            Err(InitError::InvalidVariance)
        );
        matrix[(
            ErrorState::VelocityNorth.index(),
            ErrorState::VelocityNorth.index(),
        )] = f32::NAN;
        assert_eq!(
            filter.seed(state, Covariance::from_matrix(matrix)),
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
        let mut filter = Eskf::default();
        assert_eq!(
            filter.seed(state, Covariance::zero()),
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
            filter.seed(state, Covariance::from_matrix(matrix)),
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

        let mut filter = Eskf::default();
        assert_eq!(
            filter.seed(state, Covariance::from_matrix(matrix)),
            Err(InitError::InvalidVariance)
        );
        assert!(!filter.is_initialized(), "a refused seed leaves no state");

        // Just above its own floor is accepted, and untouched: the bar is per state group,
        // so a gyroscope bias at 1e-8 passes a floor of 1e-9 while a position at 1e-8 would
        // not clear its own 1e-6.
        let mut matrix = *covariance.as_matrix();
        matrix[(ErrorState::GyroBiasZ.index(), ErrorState::GyroBiasZ.index())] = 1e-8;
        let _ = filter
            .seed(state, Covariance::from_matrix(matrix))
            .expect("above the floor");
        assert_eq!(filter.covariance().variance(ErrorState::GyroBiasZ), 1e-8);
        assert_eq!(filter.diagnostics().floored, 0);

        let mut matrix = *covariance.as_matrix();
        matrix[(
            ErrorState::PositionNorth.index(),
            ErrorState::PositionNorth.index(),
        )] = 1e-8;
        assert_eq!(
            filter.seed(state, Covariance::from_matrix(matrix)),
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
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_coarse(still().imu)
            .expect("a finite sample");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(filter.is_initialized());
        assert!(!filter.is_aligned(), "one sample cannot settle heading");
    }

    #[test]
    fn a_coarse_start_reports_aligning_once_horizontal_aiding_arrives() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.1))
            .expect("short, not unusable");
        assert_eq!(
            filter.state().status,
            Status::DeadReckoning,
            "nothing is holding position, which outranks Aligning"
        );

        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(100.0),
                    AltitudeNoise::from_sigma(2.0)
                )
                .is_accepted()
        );
        assert_eq!(
            filter.state().status,
            Status::DeadReckoning,
            "a barometer holds height, and position drifts all the same"
        );

        let velocity = filter.fuse_gnss_velocity(
            filter.now(),
            Velocity::ned(0.0, 0.0, 0.0),
            VelocityNoise::from_speed_accuracy(0.3),
            Position::zero(),
        );
        assert!(velocity.is_accepted(), "{velocity:?}");
        assert_eq!(filter.state().status, Status::Aligning);
    }

    #[test]
    fn a_coarse_window_still_starts_the_filter_with_its_tilt_widened() {
        // Classification itself is tested in `init`; this is what the filter does with it.
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(&[still(); 8], Seconds::from_secs(0.1))
            .expect("a short window is a coarse start, not a refusal");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(
            filter.is_initialized(),
            "refusing to run is the old behavior"
        );
        assert!(!filter.is_aligned());

        let _ = filter
            .initialize_over(&turning(0.0, 0.4, 0.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        // On navigation axes: the window is tilted, so body x carries some of the yaw prior.
        let tilt = filter.attitude_variance().tilt_north;
        assert!(
            (tilt - 0.8 * 0.8).abs() < 1e-4,
            "the covariance carries the widened tilt, got sigma^2 {tilt}"
        );
    }

    /// A still window of a vehicle parked at this attitude, reading nothing but gravity.
    fn window_tilted(roll: f32, pitch: f32) -> [StaticSample; 8] {
        [StaticSample {
            imu: still().imu.with_accel(gravity_at(roll, pitch, 0.0)),
            ..still()
        }; 8]
    }

    #[test]
    fn a_static_window_commits_the_attitude_it_levelled() {
        // Equations (5)–(7) are `init`'s to test; this is that the filter starts at the
        // attitude they computed rather than level.
        let mut filter = Eskf::default();
        assert_eq!(
            filter
                .initialize_over(&window_tilted(0.25, -0.1), Seconds::from_secs(0.25))
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
    fn a_declination_set_before_initializing_reaches_the_heading_it_commits() {
        // The wiring no test in `init` can see. A level vehicle reading a field with no
        // east component is pointing at magnetic north, so its true heading is the
        // declination and nothing else.
        let mut filter = Eskf::default();
        assert!(filter.set_magnetic_declination(Radians::from_radians(-0.06)));
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");

        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw + 0.06).abs() < 1e-6, "heading committed as {yaw}");
    }

    /// [`seed`] at `attitude`, with gyroscope bias `bias`.
    fn seeded(attitude: Attitude, bias: AngularRate<Body>) -> Eskf {
        let mut filter = Eskf::default();
        let (state, covariance) = seed();
        let state = State {
            attitude,
            gyro_bias: bias,
            ..state
        };
        let _ = filter.seed(state, covariance).expect("a sane seed");
        filter
    }

    #[test]
    fn the_angular_rate_is_the_last_integrated_sample_less_the_bias() {
        let bias = AngularRate::body(0.01, -0.02, 0.005);
        let mut filter = seeded(Attitude::level(), bias);
        assert_eq!(filter.angular_rate(), None, "no step integrated yet");

        let gyro = AngularRate::body(0.3, 0.1, -0.2);
        let imu = ImuSample::reading(gyro, Acceleration::body(0.0, 0.0, -GRAVITY));
        assert!(filter.step(imu, DT).is_propagated());
        let omega = filter.angular_rate().expect("a step was integrated");
        // Corrected as an increment and divided back out, so equal to rounding.
        let expected = gyro.vector() - bias.vector();
        assert!((omega.vector() - expected).norm() < 1e-6, "{omega:?}");

        // A refused sample leaves the last rate, as it leaves the state.
        let broken = imu.with_gyro(AngularRate::body(f32::NAN, 0.0, 0.0));
        assert_eq!(filter.step(broken, DT), Propagation::NotFinite);
        assert_eq!(filter.angular_rate(), Some(omega));

        // A gap is coasted on no sample, so there is no rate until the next one.
        let gap = Seconds::from_secs(1.0);
        assert_eq!(filter.step(imu, gap), Propagation::Coasted { dt: gap });
        assert_eq!(filter.angular_rate(), None);
        assert!(filter.step(imu, DT).is_propagated());
        assert!(filter.angular_rate().is_some());

        // A fresh start forgets a rate measured against the last one's bias.
        let _ = filter
            .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert_eq!(filter.angular_rate(), None);
    }

    #[test]
    fn the_angular_rate_corrects_a_lever_arm_on_a_rolled_vehicle() {
        // Nose east and rolled right a quarter turn, so the right wing points down. Worked by
        // hand rather than through the rotation, so a transposed R cannot agree with itself:
        // an antenna 1 m forward sits 1 m east of the IMU; a rate about body right is a rate
        // about down, which turns east toward south, so that antenna moves south.
        use core::f32::consts::FRAC_PI_2;
        let mut filter = seeded(attitude_of(FRAC_PI_2, 0.0, FRAC_PI_2), AngularRate::zero());
        let imu = ImuSample::reading(
            AngularRate::body(0.0, 0.5, 0.0),
            Acceleration::body(0.0, -GRAVITY, 0.0),
        );
        assert!(filter.step(imu, DT).is_propagated());

        let r = Vector3::new(1.0, 0.0, 0.0);
        let rotation = filter.state().attitude.body_to_ned();
        let omega = filter.angular_rate().expect("a step was integrated");
        // To within the 5 mrad the one step turned it.
        let arm = rotation * r;
        assert!((arm - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-2, "{arm}");
        let lever = rotation * omega.vector().cross(&r);
        assert!(
            (lever - Vector3::new(-0.5, 0.0, 0.0)).norm() < 1e-2,
            "{lever}"
        );
    }

    #[test]
    fn a_declination_that_is_not_a_number_is_refused_and_changes_nothing() {
        let mut filter = initialized();
        assert!(filter.set_magnetic_declination(Radians::from_radians(0.1)));
        assert!(!filter.set_magnetic_declination(Radians::from_radians(f32::NAN)));
        assert!(!filter.set_magnetic_declination(Radians::from_radians(f32::INFINITY)));
        assert_eq!(filter.magnetic_declination(), Radians::from_radians(0.1));
    }

    #[test]
    fn a_declination_change_moves_the_next_heading_innovation_by_exactly_itself() {
        // Established heading, so both headings fuse rather than adopt, from the same state:
        // the declination is the only difference between the two filters.
        let mut charted = Eskf::default();
        let _ = charted
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let mut shifted = charted.clone();
        assert!(shifted.set_magnetic_declination(Radians::from_radians(0.02)));

        let field = MagField::body(0.22, 0.01, 0.44);
        let noise = HeadingNoise::from_sigma(0.05);
        let nu = |filter: &mut Eskf| {
            assert!(
                filter
                    .fuse_mag_heading(filter.now(), field, noise)
                    .is_accepted()
            );
            let innovation = filter.diagnostics().mag_heading.innovation;
            innovation
                .expect("an accepted heading publishes its ν")
                .values()[0]
        };
        let moved = nu(&mut shifted) - nu(&mut charted);
        assert!((moved - 0.02).abs() < 1e-6, "ν moved by {moved}");
    }

    #[test]
    fn a_heading_adopted_after_a_declination_change_is_true_heading_at_the_new_value() {
        // No magnetometer in the window, so the first heading is adopted. A field reading
        // magnetic north on the heading the filter holds is, at declination d, true heading
        // d: the adoption steps yaw onto exactly that.
        let mut filter = initialized();
        assert!(filter.set_magnetic_declination(Radians::from_radians(0.3)));
        let field = measured(filter.state().attitude, 0.0);
        assert!(
            filter
                .fuse_mag_heading(filter.now(), field, HeadingNoise::from_sigma(0.05))
                .is_reset()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw - 0.3).abs() < 1e-5, "adopted yaw {yaw}");
    }

    #[test]
    fn a_still_window_commits_its_gyroscope_bias_and_a_moving_one_does_not() {
        let offset = AngularRate::body(0.01, -0.02, 0.003);
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu = sample.imu.with_gyro(offset);
        }

        let mut filter = Eskf::default();
        assert_eq!(
            filter
                .initialize_over(&window, Seconds::from_secs(0.25))
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
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let _ = filter
            .initialize_over(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert_eq!(filter.state().gyro_bias, AngularRate::zero());
    }

    #[test]
    fn one_sample_is_levelled_like_a_window_of_one() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_coarse(still().imu.with_accel(gravity_at(0.0, 0.35, 0.0)))
            .expect("a finite sample");

        let (roll, pitch, _) = filter.state().attitude.euler_angles();
        assert!(
            roll.abs() < 1e-6 && (pitch - 0.35).abs() < 1e-5,
            "levelled to ({roll}, {pitch})"
        );
    }

    /// An empty window is the one `initialize` can refuse: a sample nothing can be made of
    /// never reaches the filter, because the window refuses it as it is pushed.
    #[test]
    fn an_empty_window_leaves_the_filter_uninitialized() {
        let mut filter = Eskf::default();
        let empty = StaticWindow::new();
        assert_eq!(filter.alignment_of(&empty), Err(InitError::NoSamples));
        assert_eq!(filter.initialize(&empty), Err(InitError::NoSamples));
        assert!(!filter.is_initialized());
    }

    /// A filter that started while moving: it knows neither where it is nor how fast.
    fn coarse() -> Eskf {
        let mut filter = Eskf::default();
        let mut window = turning(0.0, 0.4, 0.0);
        // No magnetometer, so heading stays a prior whatever the covariance says.
        for sample in &mut window {
            sample.mag = None;
        }
        let alignment = filter
            .initialize_over(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        filter
    }

    #[test]
    fn after_a_coarse_start_the_first_fix_is_adopted_not_fused() {
        let mut filter = coarse();
        let fix = Position::ned(120.0, -40.0, -75.0);
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            fix,
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );

        assert_eq!(outcome, GnssFusion::both(Fusion::Reset));
        assert!(outcome.is_accepted(), "the measurement was used");
        assert!(outcome.is_reset(), "and it stepped the state");
        assert_eq!(filter.state().position, fix);
        assert!(
            (filter.covariance().variance(ErrorState::PositionNorth) - 2.25).abs() < 1e-6,
            "the fix's own variance, not the configured prior"
        );

        // Once is once: there is now an estimate for a gate to judge against, and the same
        // fix again is fused against it rather than adopted a second time.
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            fix,
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Accepted { .. })
                && matches!(outcome.height, Fusion::Accepted { .. }),
            "{outcome:?}"
        );
        assert_eq!(filter.diagnostics().gnss_position.adopted, 1);
    }

    #[test]
    fn a_fix_that_agrees_with_the_estimate_narrows_its_uncertainty() {
        let mut filter = initialized();
        let before = filter.covariance().variance(ErrorState::PositionNorth);
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.5, -0.5, 0.2),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );

        let Fusion::Accepted { test_ratio } = outcome.horizontal else {
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
                .fuse_gnss_position(
                    filter.now(),
                    Position::ned(0.1, 0.0, 0.0),
                    noise,
                    Position::zero()
                )
                .is_accepted()
        );
        assert!(filter.step(still().imu, DT).is_propagated());
        let (state, covariance) = (filter.state(), *filter.covariance());
        let timer = filter.diagnostics().gnss_position.time_since_accepted;

        // A kilometre out in both halves, so neither changes anything.
        let both = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(1000.0, 0.0, 1000.0),
            noise,
            Position::zero(),
        );
        assert!(matches!(both.height, Fusion::Rejected { .. }), "{both:?}");
        let outcome = both.horizontal;
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
            filter.now(),
            Position::ned(0.1, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );
        assert_eq!(outcome.horizontal, Fusion::StateInvalid);
        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.last_refusal, Some(Refusal::StateInvalid));
        assert_eq!((health.accepted, health.rejected), (0, 0));
        // The broken variance is north's, which a height update never reads, so the height
        // half is judged against a block that is still a covariance.
        assert!(outcome.height.is_accepted(), "{outcome:?}");
        assert_eq!(
            filter.state().position.vector()[0],
            state.position.vector()[0]
        );
    }

    #[test]
    fn a_height_the_estimate_disagrees_with_costs_no_horizontal_aiding() {
        // The failure the split exists for: a receiver 50 m off in height on a 1 m σ,
        // with a horizontal fix that agrees.
        let mut filter = initialized();
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.2, -0.1, -50.0),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Accepted { .. }),
            "{outcome:?}"
        );
        assert!(
            matches!(outcome.height, Fusion::Rejected { .. }),
            "{outcome:?}"
        );
        let d = filter.diagnostics();
        assert_eq!((d.gnss_position.accepted, d.gnss_position.rejected), (1, 0));
        assert_eq!((d.gnss_height.accepted, d.gnss_height.rejected), (0, 1));
        assert!(
            filter.state().position.vector()[2].abs() < 1e-3,
            "height untouched"
        );
    }

    #[test]
    fn a_2d_fix_with_no_usable_height_still_fuses_its_horizontal_position() {
        let mut filter = initialized();
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.2, -0.1, 0.0),
            PositionNoise::<Ned>::from_variance(1.0, 1.0, 0.0),
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Accepted { .. }),
            "{outcome:?}"
        );
        assert_eq!(outcome.height, Fusion::InvalidNoise);
        assert_eq!(
            filter.diagnostics().gnss_height.last_refusal,
            Some(Refusal::InvalidNoise)
        );
    }

    #[test]
    fn velocity_is_adopted_too_and_the_two_are_independent() {
        let mut filter = coarse();
        let velocity = Velocity::ned(18.0, 1.0, -0.5);
        assert!(
            filter
                .fuse_gnss_velocity(
                    filter.now(),
                    velocity,
                    VelocityNoise::from_speed_accuracy(0.3),
                    Position::zero()
                )
                .is_reset()
        );
        assert_eq!(filter.state().velocity, velocity);

        // Adopting velocity says nothing about position, which is still unknown.
        assert!(
            filter
                .fuse_gnss_position(
                    filter.now(),
                    Position::ned(1.0, 2.0, 3.0),
                    PositionNoise::horizontal_vertical(1.5, 1.5),
                    Position::zero()
                )
                .is_reset()
        );
    }

    #[test]
    fn a_static_start_knows_where_it_is_so_its_first_fix_is_fused() {
        let mut filter = initialized();
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.2, -0.1, 0.0),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
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
        let mut filter = Eskf::default();
        let _ = filter.seed(state, covariance).expect("a sane seed");
        let outcome = filter.fuse_gnss_velocity(
            filter.now(),
            Velocity::ned(0.0, 0.0, 0.0),
            VelocityNoise::from_speed_accuracy(0.3),
            Position::zero(),
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
        // Aided *and* aligned: a GNSS velocity keeps the status out of `DeadReckoning`, and
        // the magnetometer is what makes heading an estimate.
        let window = window_at(100.0).map(|sample| StaticSample {
            mag: Some(MagField::body(0.22, 0.0, 0.44)),
            ..sample
        });
        let mut filter = Eskf::default();
        assert_eq!(
            filter.initialize_over(&window, Seconds::from_secs(0.25)),
            Ok(Alignment::Static)
        );
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(100.0),
                    AltitudeNoise::from_sigma(2.0)
                )
                .is_accepted()
        );
        assert!(filter.is_aligned());
        assert!(filter.validity().tilt);

        // Past the 3.83 s the default bars buy, with a GNSS fix arriving at 2 Hz so that
        // aiding is never stale. A 100 m one, which holds the status out of `DeadReckoning`
        // and tells tilt nothing: a velocity would, through the accelerometer bias (8)
        // correlates it with.
        for step in 1..=800 {
            assert_eq!(
                filter.step(still().imu, Seconds::from_secs(0.005),),
                Propagation::Propagated
            );
            if step % 100 == 0 {
                let fix = filter.fuse_gnss_position(
                    filter.now(),
                    Position::ned(0.0, 0.0, 0.0),
                    PositionNoise::from_sigma(100.0, 100.0, 100.0),
                    Position::zero(),
                );
                assert!(fix.horizontal.is_accepted(), "{fix:?}");
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
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
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
                    filter.now(),
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
        // A quantity never established is adopted once; after that only a lockout adopts
        // again, and a heading that agrees is no lockout.
        let mut filter = initialized();
        let field = measured(attitude_of(0.0, 0.0, 1.1), 0.0);
        let noise = HeadingNoise::from_sigma(0.05);

        assert!(
            filter
                .fuse_mag_heading(filter.now(), field, noise)
                .is_reset()
        );
        assert_eq!(filter.diagnostics().mag_heading.adopted, 1);

        let second = filter.fuse_mag_heading(filter.now(), field, noise);
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
                    filter.now(),
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
                    filter.now(),
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
        let holding_still = still().imu;
        for _ in 0..2_000 {
            assert_eq!(
                filter.step(holding_still, Seconds::from_secs(0.005)),
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
                    filter.now(),
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
                filter.now(),
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
                    filter.now(),
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
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );
        let before = filter.state().attitude;

        let outcome = filter.fuse_mag_heading(
            filter.now(),
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
        // 0.0431. Both headings arrive with no step between them, which (24′) takes for the
        // same error twice, so the sources are white here.
        let mut filter = initialized();
        filter.config.correlation = Correlation::WHITE;
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );

        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
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
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert!(filter.validity().heading);
    }

    #[test]
    fn a_moving_start_takes_its_heading_from_the_first_magnetometer() {
        // This window's halves level 0.4 rad apart, and the dip scales that into 0.78 rad
        // of yaw — half as much again as `Accuracy::heading`, so the start is worth no
        // heading at all. The first field is adopted rather than fused: a prior nothing
        // measured is replaced, not averaged with.
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(&turning(0.0, 0.4, 0.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(!filter.validity().heading, "the window was moving");

        let field = MagField::body(0.22, 0.0, 0.44);
        let noise = HeadingNoise::from_sigma(0.1);
        assert!(
            filter
                .fuse_mag_heading(filter.now(), field, noise)
                .is_reset()
        );

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
            filter.fuse_mag_heading(filter.now(), field, noise),
            Fusion::Accepted { .. }
        ));
    }

    #[test]
    fn a_still_short_window_calls_its_heading_valid_at_once() {
        // #85. The same field, the same stillness, and the only thing wrong with the
        // window is its length.
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.1))
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
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert!(filter.validity().horizontal_position);
        assert!(filter.validity().horizontal_velocity);
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.2, -0.1, 0.0),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );
        assert!(
            !outcome.is_reset(),
            "adoption is for a start that established nothing"
        );
        assert!(outcome.is_accepted());
    }

    #[test]
    fn an_uninitialized_filter_claims_nothing() {
        let filter = Eskf::default();
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
                    filter.now(),
                    Position::ned(120.0, -40.0, -75.0),
                    PositionNoise::horizontal_vertical(1.5, 1.5),
                    Position::zero(),
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
            filter.now(),
            Position::ned(0.0, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );
        // ...and the magnetometer is being accepted, so heading will come in even though
        // it is worthless at this instant. A σ of 0.6 rad is what makes it worthless: the
        // first heading is adopted and carries its own variance, so a measurement wider
        // than `Accuracy::heading` (0.5236) establishes the quantity without making it
        // good enough to fly on. That is the gap this method exists to report.
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
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
        let mut filter = Eskf::default();
        // Moving, so that nothing but the barometer has established anything: a window
        // taken at rest establishes its own position, short or not. A moving one
        // establishes no barometric reference either, so the application names it.
        let _ = filter
            .initialize_over(&moving_window_at(100.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(filter.set_baro_reference(
            Altitude::from_meters(100.0),
            AltitudeNoise::from_sigma(0.001)
        ));
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(100.0),
                    AltitudeNoise::from_sigma(2.0)
                )
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
        // tilt and holds it for 3.83 s unaided
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
            let mut filter = Eskf::new(config).unwrap();
            let _ = filter
                .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
                .expect("a 2 s window of stillness");
            (filter.validity().tilt, filter.predicted_validity().tilt)
        };

        assert_eq!(ask(1.0), (true, true), "a second is inside the 3.83 s hold");
        assert_eq!(ask(6.0), (true, false), "six seconds is outside it");
    }

    #[test]
    fn a_horizon_of_zero_projects_nothing_and_leaves_the_aiding_clause_alone() {
        // The documented boundary, and it is *not* "predicted_validity becomes validity":
        // the aiding clause is unconditional, so a zero horizon leaves the optimistic half
        // exactly where it was. Asserting the equality instead would pass on a start with no
        // fresh source and say nothing, which is the shape of test this one replaces.
        let zero_horizon = Config {
            accuracy: Accuracy {
                horizon: Seconds::from_secs(0.0),
                ..Accuracy::default()
            },
            ..Config::default()
        };

        // With nothing being accepted, the two do agree: there is no aiding to widen by.
        let mut quiet = Eskf::new(zero_horizon).unwrap();
        let _ = quiet
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let (now, predicted) = (quiet.validity(), quiet.predicted_validity());
        assert_eq!(now.tilt, predicted.tilt);
        assert_eq!(now.heading, predicted.heading);

        // With a source being accepted they do not, however short the horizon. A heading at
        // σ 0.6 rad is wider than `Accuracy::heading` (0.5236), so it establishes yaw without
        // making it good: invalid now, and predicted valid because the magnetometer is there.
        let mut aided = Eskf::new(zero_horizon).unwrap();
        let _ = aided
            .initialize_over(&moving_window_at(100.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(
            aided
                .fuse_mag_heading(
                    aided.now(),
                    MagField::body(0.22, 0.0, 0.44),
                    HeadingNoise::from_sigma(0.6),
                )
                .is_accepted()
        );
        assert!(!aided.validity().heading, "0.6 rad is outside the bar");
        assert!(
            aided.predicted_validity().heading,
            "a zero horizon must not take the aiding clause away"
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
        let outcome = filter.fuse_gnss_geodetic(
            filter.now(),
            zurich(),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );
        assert!(outcome.is_accepted() && !outcome.is_reset(), "{outcome:?}");

        let origin = filter.origin().expect("placed by the first fix");
        assert!(
            near(origin.to_ned(zurich()), filter.state().position),
            "the fix lands on the estimate, so nothing steps"
        );
    }

    /// A site whose declination is large, 13.8° east in PX4's table, so a heading that
    /// missed it is far outside any gate.
    fn east_of_moscow() -> Geodetic {
        Geodetic::from_degrees(56.41, 43.76, 150.0)
    }

    fn yaw_of(filter: &Eskf) -> f32 {
        filter.state().attitude.euler_angles().2
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn a_static_start_learns_its_declination_at_the_first_fix_and_turns_its_heading() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert_eq!(yaw_of(&filter), 0.0, "levelled at declination zero");
        let covariance = *filter.covariance();
        let model = east_of_moscow()
            .magnetic_declination()
            .expect("a finite site");

        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), east_of_moscow(), noise, Position::zero());
        assert_eq!(filter.magnetic_declination(), model);
        assert!((yaw_of(&filter) - model.as_radians()).abs() < 1e-6);
        let attitude = |p: &Covariance| p.as_matrix().fixed_view::<3, 3>(6, 6).into_owned();
        assert_eq!(
            attitude(filter.covariance()),
            attitude(&covariance),
            "the body-frame error is the same error after the turn"
        );

        // The next heading from the same field agrees with the turned estimate. Unturned, it
        // would carry the whole declination as its innovation.
        let field = MagField::body(0.22, 0.0, 0.44);
        let fusion = filter.fuse_mag_heading(filter.now(), field, HeadingNoise::from_sigma(0.05));
        assert!(fusion.is_accepted(), "{fusion:?}");
        let nu = filter
            .diagnostics()
            .mag_heading
            .innovation
            .expect("fused")
            .values()[0];
        assert!(nu.abs() < 1e-4, "ν {nu}");
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn a_declination_the_caller_set_is_never_replaced_by_the_model() {
        let mut filter = Eskf::default();
        assert!(filter.set_magnetic_declination(Radians::from_radians(0.1)));
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let yaw = yaw_of(&filter);
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), east_of_moscow(), noise, Position::zero());
        assert_eq!(filter.magnetic_declination(), Radians::from_radians(0.1));
        assert_eq!(yaw_of(&filter), yaw);
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn a_true_heading_is_not_turned_by_a_declination_learned_after_it() {
        // No magnetometer in the window; a dual-antenna heading establishes yaw, and it is
        // true heading, which no declination touches.
        let mut filter = initialized();
        let noise = HeadingNoise::from_sigma(0.02);
        let heading = filter.fuse_gnss_heading(filter.now(), Radians::from_radians(0.5), noise);
        assert!(heading.is_reset(), "{heading:?}");
        let yaw = yaw_of(&filter);

        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), east_of_moscow(), noise, Position::zero());
        assert!(
            filter.magnetic_declination().as_radians() > 0.2,
            "the model was read"
        );
        assert_eq!(yaw_of(&filter), yaw);
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn a_true_heading_fused_over_a_magnetic_one_stops_the_turn() {
        // Heading levelled from the window's magnetometer, then a dual-antenna heading
        // accepted over it: the estimate is no longer the magnetometer's alone.
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let noise = HeadingNoise::from_sigma(0.02);
        let heading = filter.fuse_gnss_heading(filter.now(), Radians::from_radians(0.0), noise);
        assert!(heading.is_accepted(), "{heading:?}");
        let yaw = yaw_of(&filter);

        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), east_of_moscow(), noise, Position::zero());
        assert!(
            filter.magnetic_declination().as_radians() > 0.2,
            "the model was read"
        );
        assert_eq!(yaw_of(&filter), yaw);
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn a_seed_vouches_for_its_heading_and_a_new_origin_rereads_the_site() {
        let (state, covariance) = seed();
        let mut filter = Eskf::default();
        let _ = filter.seed(state, covariance).expect("a sane seed");
        let yaw = yaw_of(&filter);
        assert!(filter.set_origin(east_of_moscow()));
        assert_eq!(
            yaw_of(&filter),
            yaw,
            "a seed's heading is the caller's claim"
        );
        let there = filter.magnetic_declination();

        assert!(filter.set_origin(zurich()));
        assert_ne!(
            filter.magnetic_declination(),
            there,
            "a new site, a new value"
        );
    }

    #[cfg(not(feature = "magnetic-model"))]
    #[test]
    fn without_the_model_declination_is_the_callers_alone() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), east_of_moscow(), noise, Position::zero());
        assert_eq!(filter.magnetic_declination(), Radians::ZERO);
        assert_eq!(yaw_of(&filter), 0.0);
    }

    /// Nose east and level, so body forward is navigation east: an antenna 1 m forward and
    /// 0.5 m up is 1 m east of the IMU and 0.5 m above it, worked by hand.
    fn nose_east() -> Eskf {
        seeded(
            attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2),
            AngularRate::zero(),
        )
    }

    fn mast() -> Position<Body> {
        Position::body(1.0, 0.0, -0.5)
    }

    #[test]
    fn a_fix_of_the_antenna_where_the_estimate_puts_it_moves_nothing() {
        let mut filter = nose_east();
        let before = filter.state().position;
        let at_antenna = Position::ned(0.0, 1.0, -0.5);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let fused = filter.fuse_gnss_position(filter.now(), at_antenna, noise, mast());
        assert!(
            fused
                .horizontal
                .test_ratio()
                .is_some_and(|ratio| ratio < 1e-9),
            "{fused:?}"
        );
        assert!(fused.height.test_ratio().is_some_and(|ratio| ratio < 1e-9));
        assert!(near(filter.state().position, before));

        // The same fix read as the IMU's is a metre east of the estimate.
        let mut unarmed = nose_east();
        let fused = unarmed.fuse_gnss_position(unarmed.now(), at_antenna, noise, Position::zero());
        assert!(
            fused
                .horizontal
                .test_ratio()
                .is_some_and(|ratio| ratio > 0.1)
        );
    }

    #[test]
    fn a_velocity_of_the_antenna_includes_its_swing_about_the_imu() {
        // Yawing at 0.5 rad/s, nose east: an antenna 1 m forward swings south at 0.5 m/s on
        // top of whatever the IMU is doing.
        let mut filter = nose_east();
        let imu = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 0.5),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );
        assert!(filter.step(imu, DT).is_propagated());
        let v = filter.state().velocity.vector();
        let swung = Velocity::ned(v.x - 0.5, v.y, v.z);
        let noise = VelocityNoise::from_speed_accuracy(0.1);
        let fused = filter.fuse_gnss_velocity(filter.now(), swung, noise, mast());
        let ratio = fused.test_ratio().expect("fused");
        assert!(ratio < 1e-3, "ratio {ratio}");
    }

    #[test]
    fn an_adopted_fix_is_referred_to_the_imu() {
        let mut filter = coarse();
        let yaw = filter.state().attitude.body_to_ned();
        let at_antenna = Position::ned(10.0, 20.0, -5.0);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let adopted = filter.fuse_gnss_position(filter.now(), at_antenna, noise, mast());
        assert!(adopted.is_reset(), "{adopted:?}");
        let imu = at_antenna.vector() - yaw * mast().vector();
        assert!(near(filter.state().position, Position::from_vector(imu)));
    }

    #[test]
    fn an_antenna_that_is_not_a_number_is_refused_before_it_reaches_the_state() {
        let broken = Position::body(f32::NAN, 0.0, 0.0);
        let mut filter = coarse();
        let before = filter.state();
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let fused = filter.fuse_gnss_position(filter.now(), Position::zero(), noise, broken);
        assert_eq!(fused, GnssFusion::both(Fusion::NotFinite));
        let noise = VelocityNoise::from_speed_accuracy(0.3);
        let fused = filter.fuse_gnss_velocity(filter.now(), Velocity::zero(), noise, broken);
        assert_eq!(fused, Fusion::NotFinite);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let fused = filter.fuse_gnss_geodetic(filter.now(), zurich(), noise, broken);
        assert_eq!(fused, GnssFusion::both(Fusion::NotFinite));
        assert_eq!(filter.state().position, before.position);
        assert_eq!(filter.state().velocity, before.velocity);
        assert_eq!(filter.origin(), None);
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn the_origin_goes_under_the_antenna_the_turned_heading_puts() {
        // A magnetometer-levelled heading turned 13.8° by the first fix: the arm has to be
        // read after the turn, or the origin sits 0.24 m from where the estimate puts the
        // antenna.
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), east_of_moscow(), noise, mast());
        let rotation = filter.state().attitude.body_to_ned();
        let antenna = Position::from_vector(rotation * mast().vector());
        let origin = filter.origin().expect("placed by the first fix");
        assert!(near(origin.to_ned(east_of_moscow()), antenna));
    }

    #[test]
    fn an_adopted_velocity_is_the_imus_not_the_antennas_swing() {
        let mut filter = coarse();
        let imu = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 0.5),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );
        assert!(filter.step(imu, DT).is_propagated());
        let rotation = filter.state().attitude.body_to_ned();
        let omega = filter
            .angular_rate()
            .expect("a step was integrated")
            .vector();
        let at_antenna = Velocity::ned(3.0, -2.0, 0.5);
        let noise = VelocityNoise::from_speed_accuracy(0.3);
        let adopted = filter.fuse_gnss_velocity(filter.now(), at_antenna, noise, mast());
        assert!(adopted.is_reset(), "{adopted:?}");
        let imu = at_antenna.vector() - rotation * omega.cross(&mast().vector());
        let got = filter.state().velocity.vector();
        assert!((got - imu).norm() < 1e-4, "{got} against {imu}");
    }

    #[test]
    fn the_origin_goes_under_the_antenna_the_first_fix_measured() {
        // Level and nose north after a static start: the antenna is 1 m north of the IMU,
        // so the fix places the origin 1 m south of it, under the IMU.
        let mut filter = initialized();
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), zurich(), noise, mast());
        let origin = filter.origin().expect("placed by the first fix");
        assert!(near(origin.to_ned(zurich()), Position::ned(1.0, 0.0, -0.5)));
        assert!(near(filter.state().position, Position::zero()));
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

        let _ = filter.fuse_gnss_geodetic(
            filter.now(),
            zurich(),
            PositionNoise::horizontal_vertical(1.5, 3.0),
            Position::zero(),
        );

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
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let _ = filter
            .initialize_over(&window, Seconds::from_secs(0.25))
            .expect("moving, so position is unestablished");
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);

        let nonsense = Geodetic::from_degrees(f64::NAN, 8.5, 488.0);
        assert_eq!(
            filter.fuse_gnss_geodetic(filter.now(), nonsense, noise, Position::zero()),
            GnssFusion::both(Fusion::NotFinite)
        );
        assert_eq!(
            filter.fuse_gnss_geodetic(
                filter.now(),
                zurich(),
                PositionNoise::from_sigma(f32::NAN, 1.5, 1.5),
                Position::zero()
            ),
            GnssFusion::both(Fusion::NotFinite)
        );
        assert!(filter.state().position.is_finite());
        assert!(
            filter
                .fuse_gnss_geodetic(filter.now(), zurich(), noise, Position::zero())
                .is_reset()
        );
    }

    #[test]
    fn every_source_refuses_a_measurement_that_is_not_a_number() {
        let mut filter = initialized();
        assert!(filter.set_baro_reference(
            Altitude::from_meters(52.0),
            AltitudeNoise::from_sigma(0.001)
        ));
        let nan = f32::NAN;
        assert_eq!(
            filter
                .fuse_gnss_position(
                    filter.now(),
                    Position::ned(nan, 0.0, 0.0),
                    PositionNoise::horizontal_vertical(1.5, 1.5),
                    Position::zero()
                )
                .horizontal,
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                filter.now(),
                Velocity::ned(0.0, 0.0, 0.0),
                VelocityNoise::from_speed_accuracy(nan),
                Position::zero()
            ),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(nan),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_mag_heading(
                filter.now(),
                MagField::body(0.2, 0.0, 0.4),
                HeadingNoise::from_variance(f32::INFINITY)
            ),
            Fusion::NotFinite
        );
        assert_eq!(filter.diagnostics().gnss_position.accepted, 0);
    }

    #[test]
    fn every_source_measures_its_period_from_what_it_offers() {
        // Refused, every one of them: a period is how often the sensor speaks, not how often
        // it is believed, so a source turning out NaN is still timed against its own rate.
        let mut filter = initialized();
        let nan = f32::NAN;
        hold(&mut filter, 4.0, 20, |filter| {
            let now = filter.now();
            let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
            let _ = filter.fuse_gnss_position(
                now,
                Position::ned(nan, nan, nan),
                noise,
                Position::zero(),
            );
            let _ = filter.fuse_gnss_velocity(
                now,
                Velocity::ned(nan, 0.0, 0.0),
                VelocityNoise::from_speed_accuracy(0.3),
                Position::zero(),
            );
            let _ = filter.fuse_baro_altitude(
                now,
                Altitude::from_meters(nan),
                AltitudeNoise::from_sigma(2.0),
            );
            let heading = HeadingNoise::from_variance(nan);
            let _ = filter.fuse_mag_heading(now, MagField::body(0.2, 0.0, 0.4), heading);
            let _ = filter.fuse_gnss_heading(now, Radians::from_radians(0.0), heading);
            let _ = filter.fuse_course(now, heading);
        });
        for (name, source) in filter.diagnostics().sources() {
            assert_eq!(source.accepted, 0, "{name}");
            let period = source.period().map(Seconds::as_secs);
            assert!(
                period.is_some_and(|period| (period - 0.2).abs() < 1e-4),
                "{name}: {period:?}"
            );
        }
    }

    #[test]
    fn every_source_refuses_a_variance_no_sensor_could_have() {
        let mut filter = initialized();
        assert!(filter.set_baro_reference(
            Altitude::from_meters(52.0),
            AltitudeNoise::from_sigma(0.001)
        ));
        assert_eq!(
            filter.fuse_gnss_position(
                filter.now(),
                Position::ned(1.0, 2.0, 3.0),
                PositionNoise::<Ned>::from_variance(0.0, 1.0, -1.0),
                Position::zero(),
            ),
            GnssFusion::both(Fusion::InvalidNoise),
            "zero variance claims a perfect measurement and makes S singular"
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                filter.now(),
                Velocity::ned(0.0, 0.0, 0.0),
                VelocityNoise::<Ned>::from_variance(1.0, -4.0, 1.0),
                Position::zero(),
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_variance(0.0)
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_mag_heading(
                filter.now(),
                MagField::body(0.2, 0.0, 0.4),
                HeadingNoise::from_variance(-1.0)
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_gnss_geodetic(
                filter.now(),
                zurich(),
                PositionNoise::<Ned>::from_variance(1.0, 1.0, 0.0),
                Position::zero()
            ),
            GnssFusion::both(Fusion::InvalidNoise)
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
            filter.fuse_gnss_position(
                filter.now(),
                fix,
                PositionNoise::<Ned>::from_variance(-1.0, -1.0, -1.0),
                Position::zero()
            ),
            GnssFusion::both(Fusion::InvalidNoise)
        );
        assert_eq!(filter.state().position, Position::zero(), "nothing adopted");
        assert!(!filter.validity().horizontal_position);
        assert!(
            filter
                .fuse_gnss_position(
                    filter.now(),
                    fix,
                    PositionNoise::horizontal_vertical(1.5, 1.5),
                    Position::zero()
                )
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

        assert!(!filter.set_baro_reference(
            Altitude::from_meters(f32::NAN),
            AltitudeNoise::from_sigma(0.001)
        ));
        assert_eq!(
            filter.baro_reference(),
            None,
            "a NaN alpha_0 would end barometric aiding for the flight, not one update"
        );
        assert!(filter.set_baro_reference(
            Altitude::from_meters(52.0),
            AltitudeNoise::from_sigma(0.001)
        ));
    }

    #[test]
    fn an_origin_placed_after_moving_accounts_for_the_move() {
        let (mut state, covariance) = seed();
        state.position = Position::ned(40.0, -15.0, -3.0);
        let mut filter = Eskf::default();
        let _ = filter.seed(state, covariance).expect("a sane seed");
        let _ = filter.fuse_gnss_geodetic(
            filter.now(),
            zurich(),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );

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
        let outcome = filter.fuse_gnss_geodetic(
            filter.now(),
            zurich(),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );
        assert_eq!(outcome, GnssFusion::both(Fusion::Reset));
        assert_eq!(filter.origin().map(|o| o.geodetic()), Some(zurich()));
        assert_eq!(filter.state().position, Position::zero());
    }

    #[test]
    fn later_fixes_are_converted_about_the_same_origin() {
        let mut filter = coarse();
        let _ = filter.fuse_gnss_geodetic(
            filter.now(),
            zurich(),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );
        // Position is established now, so this one is fused, not adopted.
        let north = Geodetic::from_degrees(47.3987, 8.5456, 488.0);
        assert!(
            !filter
                .fuse_gnss_geodetic(
                    filter.now(),
                    north,
                    PositionNoise::horizontal_vertical(1.5, 1.5),
                    Position::zero()
                )
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
            filter.fuse_gnss_geodetic(
                filter.now(),
                off_the_earth,
                PositionNoise::horizontal_vertical(1.5, 1.5),
                Position::zero()
            ),
            GnssFusion::both(Fusion::NoReference)
        );
        assert_eq!(filter.origin(), None);
        assert!(
            filter
                .fuse_gnss_geodetic(
                    filter.now(),
                    zurich(),
                    PositionNoise::horizontal_vertical(1.5, 1.5),
                    Position::zero()
                )
                .is_accepted()
        );
        assert!(filter.origin().is_some());
    }

    #[test]
    fn moving_the_origin_does_not_move_the_vehicle() {
        let mut filter = initialized();
        let _ = filter.fuse_gnss_geodetic(
            filter.now(),
            zurich(),
            PositionNoise::horizontal_vertical(1.5, 1.5),
            Position::zero(),
        );
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
            .initialize_over(&[still(); 8], Seconds::from_secs(0.1))
            .expect("short, so coarse");
        assert_eq!(filter.origin(), None, "still, so zero is here now");
        assert!(filter.validity().horizontal_position);

        assert!(filter.set_origin(zurich()));
        let _ = filter
            .initialize_over(&turning(0.0, 0.4, 0.0), Seconds::from_secs(0.25))
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
        })
        .unwrap();
        assert_eq!(
            filter.initialize_over(&window_with_mag(), Seconds::from_secs(0.25)),
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
        let mut filter = Eskf::default();

        let _ = filter
            .seed(state, Covariance::from_sigmas([0.001; STATES]))
            .expect("a sane seed");
        assert!(filter.is_aligned());

        let _ = filter
            .seed(state, Covariance::from_sigmas([2.0; STATES]))
            .expect("a sane seed");
        assert!(!filter.is_aligned());
    }

    /// A static window of a tailsitter standing on its tail, pitched 90° nose-up: body x
    /// points up, so the body diagonal of the attitude block is heading, one tilt and the
    /// other tilt, in that order.
    fn on_its_tail() -> Eskf {
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(
                &window_tilted(0.0, core::f32::consts::FRAC_PI_2),
                Seconds::from_secs(0.25),
            )
            .expect("a parked vehicle reads exactly g, however it is standing");
        assert_eq!(alignment, Alignment::Static);
        filter
    }

    #[test]
    fn validity_reads_tilt_and_heading_about_navigation_axes() {
        // Tight in heading and in tilt about east, loose in tilt about north. On its tail,
        // body x is heading and body z is tilt about north, so reading by body axis would
        // call tilt valid and heading not — the opposite of the attitude it holds.
        let attitude = attitude_of(0.0, core::f32::consts::FRAC_PI_2, 0.0);
        let (tight, loose) = (1e-4, 0.25);
        let block = AttitudeVariance {
            tilt_north: loose,
            tilt_east: tight,
            heading: tight,
        }
        .in_body(&attitude);
        let mut p = *Covariance::from_sigmas([0.1; STATES]).as_matrix();
        let theta = ErrorState::AttitudeX.index();
        p.fixed_view_mut::<3, 3>(theta, theta).copy_from(&block);

        let mut filter = Eskf::default();
        let _ = filter
            .seed(
                State {
                    attitude,
                    ..State::default()
                },
                Covariance::from_matrix(p),
            )
            .expect("a sane seed");

        let validity = filter.validity();
        assert!(validity.heading, "heading is known to 0.57°");
        assert!(!validity.tilt, "tilt about north is 29° uncertain");
    }

    #[test]
    fn on_its_tail_the_window_puts_the_yaw_prior_on_heading() {
        // Equation (8)'s prior is about navigation axes, so on a vehicle standing on its
        // tail it lands on body x, not z. Tilt is the wider of the configured figure and
        // what the accelerometer-bias prior levels in.
        let filter = on_its_tail();
        let init = Config::default().init;
        let variance = filter.attitude_variance();
        let tilt = init
            .sigma_tilt
            .as_radians()
            .max(init.sigma_accel_bias.as_m_per_s2() / GRAVITY);
        let yaw = init.sigma_yaw.as_radians();
        assert!((variance.heading - yaw * yaw).abs() < 1e-6, "{variance:?}");
        assert!(
            (variance.tilt_north - tilt * tilt).abs() < 1e-7,
            "{variance:?}"
        );
        assert!(
            (variance.tilt_east - tilt * tilt).abs() < 1e-7,
            "{variance:?}"
        );
    }

    #[test]
    fn on_its_tail_a_heading_adoption_resets_heading_and_leaves_tilt() {
        let mut filter = on_its_tail();
        let before = filter.attitude_variance();
        // The same vehicle turned 1.1 rad about down: a heading, and nothing else, differs.
        let truth = Attitude::from_body_to_ned(
            nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.1)
                * filter.state().attitude.body_to_ned(),
        );
        let field = measured(truth, 0.0);
        let noise = HeadingNoise::from_sigma(0.05);
        let adopted = mag::heading_observation(
            &filter.state,
            &filter.covariance,
            field,
            filter.declination,
            noise,
        )
        .r_m[0];

        assert!(
            filter
                .fuse_mag_heading(filter.now(), field, noise)
                .is_reset()
        );

        let after = filter.attitude_variance();
        assert!(
            (after.heading - adopted).abs() < 1e-6 * adopted,
            "heading carries the adopted variance: {after:?} against {adopted}"
        );
        assert!(
            (after.tilt_north - before.tilt_north).abs() < 1e-9,
            "{after:?}"
        );
        assert!(
            (after.tilt_east - before.tilt_east).abs() < 1e-9,
            "{after:?}"
        );
    }

    // ---- recovery from gate lockout ----

    /// Predict at rest for `seconds`, offering `offer` every `every` steps.
    fn hold(filter: &mut Eskf, seconds: f32, every: usize, mut offer: impl FnMut(&mut Eskf)) {
        let steps = (seconds / DT.as_secs()).round() as usize;
        for step in 1..=steps {
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            if step % every == 0 {
                offer(filter);
            }
        }
    }

    /// A kilometre north of a vehicle that has not moved.
    fn far() -> Position<Ned> {
        Position::ned(1000.0, 0.0, 0.0)
    }

    fn one_metre() -> PositionNoise<Ned> {
        PositionNoise::from_sigma(1.0, 1.0, 1.0)
    }

    #[test]
    fn a_fix_rejected_past_the_timeout_is_adopted_and_its_height_is_left_to_its_own_gate() {
        let mut filter = initialized();
        // Rejected for everything short of `Recovery::gnss_position`, counted from
        // initialization since nothing was ever accepted.
        hold(&mut filter, 6.9, 100, |filter| {
            let outcome =
                filter.fuse_gnss_position(filter.now(), far(), one_metre(), Position::zero());
            assert!(
                matches!(outcome.horizontal, Fusion::Rejected { .. }),
                "{outcome:?}"
            );
        });
        hold(&mut filter, 0.1, 10, |_| {});

        // Half a metre down: inside the height gate, and where an adoption of all three axes
        // would put the estimate exactly.
        let fix = Position::ned(1000.0, 0.0, 0.5);
        let outcome = filter.fuse_gnss_position(filter.now(), fix, one_metre(), Position::zero());
        assert_eq!(outcome.horizontal, Fusion::Reset);
        assert!(
            matches!(outcome.height, Fusion::Accepted { .. }),
            "the height agreed, so it is fused rather than adopted: {outcome:?}"
        );
        let position = filter.state().position.vector();
        assert_eq!((position[0], position[1]), (1000.0, 0.0));
        assert!(
            position[2] < 0.4,
            "fused part of the way, not adopted: {}",
            position[2]
        );
        assert!(
            (filter.covariance().variance(ErrorState::PositionNorth) - 1.0).abs() < 1e-6,
            "the fix's variance, which is what undoes the lockout"
        );
        let d = filter.diagnostics();
        assert_eq!((d.gnss_position.recovered, d.gnss_position.adopted), (1, 1));
        assert_eq!(d.gnss_height.recovered, 0);

        // Recovered, so the next fix is judged again rather than adopted.
        let outcome = filter.fuse_gnss_position(filter.now(), far(), one_metre(), Position::zero());
        assert!(
            matches!(outcome.horizontal, Fusion::Accepted { .. }),
            "{outcome:?}"
        );
    }

    #[test]
    fn a_fix_that_agrees_after_a_long_silence_is_fused_not_adopted() {
        // Past the timeout is not enough: only a measurement the gate rejects is a lockout.
        let mut filter = initialized();
        hold(&mut filter, 10.0, 1000, |_| {});
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::zero(),
            one_metre(),
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Accepted { .. })
                && matches!(outcome.height, Fusion::Accepted { .. }),
            "{outcome:?}"
        );
        assert_eq!(filter.diagnostics().gnss_position.adopted, 0);
    }

    #[test]
    fn a_named_reference_is_kept_through_a_barometer_lockout() {
        // `baro_reference_from_estimate` off says the caller owns `α₀` — a surveyed pad — so
        // a barometer that disagrees for longer than the timeout stays rejected.
        let mut filter = Eskf::new(Config {
            baro_reference_from_estimate: false,
            ..Config::default()
        })
        .unwrap();
        let _ = filter
            .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert!(
            filter.set_baro_reference(Altitude::from_meters(100.0), AltitudeNoise::from_sigma(0.1))
        );
        let mut step = 0;
        hold(&mut filter, 8.0, 10, |filter| {
            step += 1;
            if step % 10 == 0 {
                let _ = filter.fuse_gnss_position(
                    filter.now(),
                    Position::zero(),
                    one_metre(),
                    Position::zero(),
                );
            }
            let outcome = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(150.0),
                AltitudeNoise::from_sigma(0.5),
            );
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        let reference = filter.baro_reference().expect("named").as_meters();
        assert!((reference - 100.0).abs() < 0.5, "α₀ = {reference}");
        assert_eq!(filter.diagnostics().baro_altitude.recovered, 0);
    }

    #[test]
    fn a_barometer_is_not_re_referenced_before_position_is_established() {
        // A restart in motion keeps the flight's reference with no position to read a new
        // one against, so a barometer rejected past the timeout stays rejected.
        let mut filter = aided();
        let mut window = [still(); 8];
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let _ = filter
            .initialize_over(&window, Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        hold(&mut filter, 8.0, 10, |filter| {
            let outcome = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(5000.0),
                AltitudeNoise::from_sigma(0.5),
            );
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.baro_reference(), Some(Altitude::from_meters(100.0)));
    }

    #[test]
    fn with_recovery_off_a_locked_out_source_is_rejected_for_good() {
        let mut filter = Eskf::new(Config {
            recovery: Recovery::OFF,
            ..Config::default()
        })
        .unwrap();
        let _ = filter
            .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        // Twice the timeout, and short of where dead reckoning alone grows `P` enough to
        // take a kilometre back in, which is about 30 s at rest.
        hold(&mut filter, 15.0, 100, |filter| {
            let outcome =
                filter.fuse_gnss_position(filter.now(), far(), one_metre(), Position::zero());
            assert!(
                matches!(outcome.horizontal, Fusion::Rejected { .. }),
                "{outcome:?}"
            );
        });
        assert_eq!(filter.diagnostics().gnss_position.adopted, 0);
        assert!(filter.state().position.vector()[0].abs() < 1.0);
    }

    #[test]
    fn only_a_rejection_recovers_and_a_refusal_never_does() {
        let mut filter = initialized();
        let nan = PositionNoise::from_sigma(f32::NAN, f32::NAN, f32::NAN);
        hold(&mut filter, 10.0, 100, |filter| {
            assert_eq!(
                filter.fuse_gnss_position(filter.now(), far(), nan, Position::zero()),
                GnssFusion::both(Fusion::NotFinite)
            );
        });
        assert!(filter.state().position.vector()[0].abs() < 1.0);

        // Nothing was accepted through all of that either, so the first fix the gate can
        // judge and rejects is a lockout already: PX4 counts from `time_last_fuse` too.
        let outcome = filter.fuse_gnss_position(filter.now(), far(), one_metre(), Position::zero());
        assert_eq!(outcome.horizontal, Fusion::Reset);
    }

    #[test]
    fn a_locked_out_velocity_is_adopted() {
        let mut filter = initialized();
        let velocity = Velocity::ned(20.0, 0.0, 0.0);
        let noise = VelocityNoise::from_speed_accuracy(0.3);
        hold(&mut filter, 6.9, 100, |filter| {
            let outcome =
                filter.fuse_gnss_velocity(filter.now(), velocity, noise, Position::zero());
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        hold(&mut filter, 0.1, 10, |_| {});
        assert_eq!(
            filter.fuse_gnss_velocity(filter.now(), velocity, noise, Position::zero()),
            Fusion::Reset
        );
        assert_eq!(filter.state().velocity, velocity);
        assert_eq!(filter.diagnostics().gnss_velocity.recovered, 1);
    }

    #[test]
    fn an_adopted_velocity_is_established_and_the_next_solution_is_fused() {
        let mut filter = coarse();
        let velocity = Velocity::ned(18.0, 1.0, -0.5);
        let noise = VelocityNoise::from_speed_accuracy(0.3);
        assert!(
            filter
                .fuse_gnss_velocity(filter.now(), velocity, noise, Position::zero())
                .is_reset()
        );
        assert!(
            matches!(
                filter.fuse_gnss_velocity(filter.now(), velocity, noise, Position::zero()),
                Fusion::Accepted { .. }
            ),
            "once is once"
        );
        assert_eq!(filter.diagnostics().gnss_velocity.adopted, 1);
    }

    #[test]
    fn a_locked_out_height_is_adopted_and_the_barometer_reads_its_reference_again() {
        // The barometer holds the height at the window's while the receiver says 20 m up:
        // GNSS height is rejected until `Recovery::gnss_height`, then adopted.
        let mut filter = aided();
        let fix = Position::ned(0.0, 0.0, -20.0);
        let baro = AltitudeNoise::from_sigma(0.5);
        let mut adopted_at = None;
        let mut step = 0;
        hold(&mut filter, 6.0, 10, |filter| {
            step += 1;
            if adopted_at.is_some() {
                return;
            }
            if step % 10 == 0 {
                let outcome =
                    filter.fuse_gnss_position(filter.now(), fix, one_metre(), Position::zero());
                if outcome.height == Fusion::Reset {
                    adopted_at.get_or_insert(step);
                    assert_eq!(filter.state().position.vector()[2], -20.0);
                    assert_eq!(filter.baro_reference(), None, "dropped with the height");
                    // The next altitude reads it again, against the adopted height.
                    assert_eq!(
                        filter.fuse_baro_altitude(filter.now(), Altitude::from_meters(100.0), baro),
                        Fusion::Accepted { test_ratio: 0.0 }
                    );
                    let reference = filter.baro_reference().expect("read again").as_meters();
                    assert!((reference - 80.0).abs() < 1e-3, "α₀ = {reference}");
                    return;
                }
                assert!(
                    matches!(outcome.height, Fusion::Rejected { .. }),
                    "{outcome:?}"
                );
            }
            let _ = filter.fuse_baro_altitude(filter.now(), Altitude::from_meters(100.0), baro);
        });
        let at = adopted_at.expect("recovered within 6 s") as f32 * 0.1;
        assert!((5.0..=5.1).contains(&at), "at {at} s");
        assert_eq!(filter.diagnostics().gnss_height.recovered, 1);
        assert_eq!(filter.diagnostics().gnss_position.recovered, 0);
    }

    #[test]
    fn a_locked_out_barometer_reads_its_reference_again_and_moves_nothing() {
        let mut filter = aided();
        let baro = AltitudeNoise::from_sigma(0.5);
        let mut step = 0;
        let mut recovered = false;
        hold(&mut filter, 6.0, 10, |filter| {
            step += 1;
            if step % 10 == 0 {
                let _ = filter.fuse_gnss_position(
                    filter.now(),
                    Position::zero(),
                    one_metre(),
                    Position::zero(),
                );
            }
            if recovered {
                return;
            }
            let before = filter.state();
            match filter.fuse_baro_altitude(filter.now(), Altitude::from_meters(150.0), baro) {
                Fusion::Rejected { .. } => {}
                Fusion::Reset => {
                    recovered = true;
                    assert_eq!(filter.state(), before, "a reference, not a state");
                    let reference = filter.baro_reference().expect("read again").as_meters();
                    let down = before.position.vector()[2];
                    assert!(
                        (reference - (150.0 + down)).abs() < 1e-3,
                        "α₀ = {reference}"
                    );
                }
                outcome => panic!("{outcome:?}"),
            }
        });
        assert!(recovered);
        assert_eq!(filter.diagnostics().baro_altitude.recovered, 1);
    }

    /// An established yaw of zero, and a magnetometer turned a quarter circle from it.
    fn disturbed_heading() -> (Eskf, MagField<Body>) {
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );
        let turned = measured(attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2), 0.0);
        (filter, turned)
    }

    #[test]
    fn a_magnetometer_disagreeing_while_gnss_arrives_is_not_adopted() {
        // The guard PX4's `mag_control.cpp` keeps: with horizontal aiding arriving, a heading
        // that disagrees for this long is a disturbance, and adopting it would turn the
        // estimate by it.
        let (mut filter, turned) = disturbed_heading();
        let mut step = 0;
        hold(&mut filter, 20.0, 10, |filter| {
            step += 1;
            if step % 10 == 0 {
                let _ = filter.fuse_gnss_position(
                    filter.now(),
                    Position::zero(),
                    one_metre(),
                    Position::zero(),
                );
            }
            let outcome =
                filter.fuse_mag_heading(filter.now(), turned, HeadingNoise::from_sigma(0.05));
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().mag_heading.recovered, 0);
    }

    #[test]
    fn a_magnetometer_disagreeing_with_a_gnss_heading_is_not_adopted() {
        // The receiver measures heading outright, so the magnetometer is the one disturbed.
        let (mut filter, turned) = disturbed_heading();
        let north = Radians::from_radians(0.0);
        hold(&mut filter, 20.0, 10, |filter| {
            let _ = filter.fuse_gnss_heading(filter.now(), north, HeadingNoise::from_sigma(0.05));
            let outcome =
                filter.fuse_mag_heading(filter.now(), turned, HeadingNoise::from_sigma(0.05));
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().mag_heading.recovered, 0);
    }

    #[test]
    fn a_magnetometer_disagreeing_with_nothing_else_to_say_is_adopted() {
        let (mut filter, turned) = disturbed_heading();
        let mut recovered = false;
        hold(&mut filter, 8.0, 10, |filter| {
            if !recovered {
                recovered = filter
                    .fuse_mag_heading(filter.now(), turned, HeadingNoise::from_sigma(0.05))
                    .is_reset();
            }
        });
        assert!(recovered);
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!(
            (yaw - core::f32::consts::FRAC_PI_2).abs() < 1e-3,
            "yaw = {yaw}"
        );
        assert_eq!(filter.diagnostics().mag_heading.recovered, 1);
    }

    #[test]
    fn the_first_gnss_heading_is_adopted_and_every_one_after_it_is_fused() {
        let mut filter = initialized();
        let noise = HeadingNoise::from_sigma(0.05);
        let heading = Radians::from_radians(1.1);
        assert!(
            filter
                .fuse_gnss_heading(filter.now(), heading, noise)
                .is_reset()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw - 1.1).abs() < 1e-5, "yaw = {yaw}");
        assert!(filter.validity().heading);
        assert!(filter.is_aligned(), "a heading source is a heading source");
        assert_eq!(filter.diagnostics().gnss_heading.adopted, 1);

        let second = filter.fuse_gnss_heading(filter.now(), heading, noise);
        assert!(matches!(second, Fusion::Accepted { .. }), "{second:?}");
    }

    #[test]
    fn a_gnss_heading_is_screened_before_anything_else() {
        let mut filter = initialized();
        let at = |r| Radians::from_radians(r);
        assert_eq!(
            filter.fuse_gnss_heading(filter.now(), at(f32::NAN), HeadingNoise::from_sigma(0.1)),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_heading(filter.now(), at(0.3), HeadingNoise::from_variance(0.0)),
            Fusion::InvalidNoise
        );
        assert_eq!(filter.diagnostics().gnss_heading.adopted, 0);
        assert!(!filter.validity().heading, "a refusal establishes nothing");
    }

    #[test]
    fn a_gnss_heading_is_true_heading_whatever_the_declination() {
        let mut filter = initialized();
        assert!(filter.set_magnetic_declination(Radians::from_radians(0.3)));
        let _ = filter.fuse_gnss_heading(
            filter.now(),
            Radians::from_radians(1.1),
            HeadingNoise::from_sigma(0.05),
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw - 1.1).abs() < 1e-5, "yaw = {yaw}");
    }

    /// A static start, then flying `velocity` with its heading still unobserved, a GNSS
    /// velocity just accepted to hold it.
    fn cruising(velocity: Velocity<Ned>) -> Eskf {
        let mut filter = initialized();
        assert!(filter.reset_velocity_to(velocity, VelocityNoise::from_speed_accuracy(0.3)));
        hold_velocity(&mut filter, velocity);
        filter
    }

    fn hold_velocity(filter: &mut Eskf, velocity: Velocity<Ned>) {
        let noise = VelocityNoise::from_speed_accuracy(0.3);
        let outcome = filter.fuse_gnss_velocity(filter.now(), velocity, noise, Position::zero());
        assert!(outcome.is_accepted(), "{outcome:?}");
    }

    #[test]
    fn a_course_is_refused_without_speed_and_adopted_with_it() {
        let slow = Velocity::ned(0.5, 0.5, 0.0);
        let mut filter = cruising(slow);
        assert_eq!(
            filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05)),
            Fusion::Unobservable
        );
        assert_eq!(
            filter.diagnostics().course.last_refusal,
            Some(Refusal::Unobservable)
        );

        // North-east at 14 m/s: the course is π/4.
        let mut filter = cruising(Velocity::ned(10.0, 10.0, 0.0));
        assert!(
            filter
                .fuse_course(filter.now(), HeadingNoise::from_sigma(0.05))
                .is_reset()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!(
            (yaw - core::f32::consts::FRAC_PI_4).abs() < 1e-5,
            "yaw = {yaw}"
        );
        assert!(filter.is_aligned());
        let next = filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05));
        assert!(matches!(next, Fusion::Accepted { .. }), "{next:?}");
    }

    #[test]
    fn an_adopted_course_carries_the_velocity_s_uncertainty_as_well_as_the_sideslip() {
        // About 0.2 m/s across 14.1 m/s of track, once the velocity fix has been fused, on top
        // of 0.05 rad of sideslip.
        let mut filter = cruising(Velocity::ned(10.0, 10.0, 0.0));
        let course = heading::course_variance(&filter.state, &filter.covariance).expect("moving");
        assert!(course > 1e-5 && course < 0.09 / 200.0, "{course}");
        let sideslip = HeadingNoise::from_sigma(0.05);
        assert!(filter.fuse_course(filter.now(), sideslip).is_reset());
        let heading = AttitudeVariance::of(&filter.state().attitude, filter.covariance()).heading;
        let expected = sideslip.variance() + course;
        assert!(
            (heading - expected).abs() < 1e-6,
            "heading variance {heading}, expected {expected}"
        );
    }

    #[test]
    fn a_coarse_start_has_no_course_until_velocity_is_established() {
        let mut filter = coarse();
        assert_eq!(
            filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05)),
            Fusion::NoReference
        );
    }

    #[test]
    fn the_course_corrects_heading_and_velocity_together() {
        // Heading established at 0, then a course constraint along a velocity 0.2 rad east of
        // it: the update turns the nose toward the track and the track toward the nose, each
        // by its share of the uncertainty.
        let mut filter = cruising(Velocity::ned(20.0 * 0.2f32.cos(), 20.0 * 0.2f32.sin(), 0.0));
        let _ = filter.fuse_gnss_heading(
            filter.now(),
            Radians::from_radians(0.0),
            HeadingNoise::from_sigma(0.1),
        );
        let outcome = filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.1));
        assert!(matches!(outcome, Fusion::Accepted { .. }), "{outcome:?}");
        let state = filter.state();
        let (_, _, yaw) = state.attitude.euler_angles();
        let v = state.velocity.vector();
        let track = v.y.atan2(v.x);
        assert!(
            yaw > 0.0 && yaw < 0.2,
            "the nose turned toward the track: {yaw}"
        );
        assert!(
            track < 0.2 && track > yaw,
            "the track turned toward the nose: {track}"
        );
    }

    #[test]
    fn a_course_disagreeing_while_another_heading_arrives_is_not_adopted() {
        // Nose north on a GNSS heading, tracking east: a 90° crab no sideslip allowed for.
        let mut filter = cruising(Velocity::ned(0.0, 15.0, 0.0));
        let north = Radians::from_radians(0.0);
        let noise = HeadingNoise::from_sigma(0.02);
        assert!(
            filter
                .fuse_gnss_heading(filter.now(), north, noise)
                .is_reset()
        );
        // Velocity aided too, or its uncertainty outgrows the speed and the course is refused.
        let east = Velocity::ned(0.0, 15.0, 0.0);
        hold(&mut filter, 10.0, 10, |filter| {
            let _ = filter.fuse_gnss_heading(filter.now(), north, noise);
            let _ = filter.fuse_gnss_velocity(
                filter.now(),
                east,
                VelocityNoise::from_speed_accuracy(0.3),
                Position::zero(),
            );
            let outcome = filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.02));
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().course.recovered, 0);
    }

    #[test]
    fn a_course_disagreeing_with_nothing_else_to_say_is_adopted() {
        let mut filter = cruising(Velocity::ned(0.0, 15.0, 0.0));
        let noise = HeadingNoise::from_sigma(0.02);
        assert!(
            filter
                .fuse_gnss_heading(filter.now(), Radians::from_radians(0.0), noise)
                .is_reset()
        );
        let mut recovered = false;
        hold(&mut filter, 8.0, 10, |filter| {
            hold_velocity(filter, Velocity::ned(0.0, 15.0, 0.0));
            if !recovered {
                recovered = filter.fuse_course(filter.now(), noise).is_reset();
            }
        });
        assert!(recovered);
        assert_eq!(filter.diagnostics().course.recovered, 1);
    }

    #[test]
    fn a_course_along_a_dead_reckoned_velocity_is_refused() {
        // Moving fast enough, but a 10 Hz velocity has not held it for five of its periods,
        // twice its own timeout: a course along it would read as aiding while it drifts.
        let velocity = Velocity::ned(0.0, 15.0, 0.0);
        let mut filter = cruising(velocity);
        hold(&mut filter, 2.0, 10, |filter| {
            hold_velocity(filter, velocity)
        });
        hold(&mut filter, 0.5, 100, |_| {});
        assert_eq!(
            filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05)),
            Fusion::NoReference
        );
    }

    /// Started still with a barometer at 0 m and a magnetometer, so that `Status` can reach
    /// `Healthy`: without a heading the start stays `Aligning`.
    fn aligned() -> Eskf {
        let window = window_at(0.0).map(|sample| StaticSample {
            mag: Some(MagField::body(0.22, 0.0, 0.44)),
            ..sample
        });
        let mut filter = Eskf::default();
        assert_eq!(
            filter.initialize_over(&window, Seconds::from_secs(0.25)),
            Ok(Alignment::Static)
        );
        assert!(filter.is_aligned());
        filter
    }

    #[test]
    fn a_fast_source_that_stops_degrades_the_status_on_its_own_schedule() {
        // A 10 Hz barometer beside a 1 Hz GNSS: the barometer times out after 0.25 s of
        // silence, where one threshold sized for the receiver would take 2.5 s to notice.
        let mut filter = aligned();
        let baro = |filter: &mut Eskf| {
            let outcome = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(0.0),
                AltitudeNoise::from_sigma(2.0),
            );
            assert!(outcome.is_accepted(), "{outcome:?}");
        };
        let gnss = |filter: &mut Eskf| {
            hold_velocity(filter, Velocity::ned(0.0, 0.0, 0.0));
        };
        hold(&mut filter, 10.0, 10, |filter| {
            baro(filter);
            if filter.diagnostics().baro_altitude.accepted % 10 == 0 {
                gnss(filter);
            }
        });
        assert_eq!(filter.state().status, Status::Healthy);

        hold(&mut filter, 0.4, 100, |_| {});
        assert_eq!(filter.state().status, Status::Degraded);
    }

    #[test]
    fn a_status_reads_dead_reckoning_once_gnss_stops_whatever_else_arrives() {
        let mut filter = aligned();
        hold(&mut filter, 2.0, 10, |filter| {
            hold_velocity(filter, Velocity::ned(0.0, 0.0, 0.0));
            let _ = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(0.0),
                AltitudeNoise::from_sigma(2.0),
            );
        });
        assert_eq!(filter.state().status, Status::Healthy);

        // The barometer keeps arriving, and is still accepted, past `dead_reckoning_after`.
        hold(&mut filter, 5.5, 10, |filter| {
            let _ = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(0.0),
                AltitudeNoise::from_sigma(2.0),
            );
        });
        assert!(
            filter
                .diagnostics()
                .baro_altitude
                .is_fresh(&crate::Timeouts::default())
        );
        assert_eq!(filter.state().status, Status::DeadReckoning);
    }

    #[test]
    fn a_course_refused_at_a_stop_does_not_degrade_the_status() {
        // Two filters fed the same velocities and GNSS headings, one also fusing the course: when
        // the vehicle stops and every course is refused, `Status` must read the same for both,
        // and with every counted source fresh that is `Healthy`.
        let east = Velocity::ned(0.0, 15.0, 0.0);
        let stopped = Velocity::ned(0.0, 0.0, 0.0);
        let heading = Radians::from_radians(core::f32::consts::FRAC_PI_2);
        let noise = HeadingNoise::from_sigma(0.05);
        let mut with = cruising(east);
        let mut without = cruising(east);
        for filter in [&mut with, &mut without] {
            assert!(
                filter
                    .fuse_gnss_heading(filter.now(), heading, noise)
                    .is_reset()
            );
        }
        assert!(with.fuse_course(with.now(), noise).is_accepted());
        for filter in [&mut with, &mut without] {
            assert!(filter.reset_velocity_to(stopped, VelocityNoise::from_speed_accuracy(0.3)));
            hold(filter, 4.0, 10, |filter| {
                hold_velocity(filter, stopped);
                let _ = filter.fuse_gnss_heading(filter.now(), heading, noise);
                let _ = filter.fuse_course(filter.now(), noise);
            });
        }
        assert_eq!(
            with.diagnostics().course.last_refusal,
            Some(Refusal::Unobservable)
        );
        assert_eq!(without.state().status, Status::Healthy);
        assert_eq!(with.state().status, without.state().status);
    }

    #[test]
    fn a_course_adopted_in_the_past_turns_about_down_and_leaves_tilt_alone() {
        // (23′) carries the course's row back through the error dynamics, and its velocity block
        // lends the attitude row a tilt component; the adoption must still turn about down.
        let mut filter = cruising(Velocity::ned(10.0, 10.0, 0.0));
        hold(&mut filter, 0.2, 10, |filter| {
            hold_velocity(filter, Velocity::ned(10.0, 10.0, 0.0))
        });
        let tilt = |filter: &Eskf| {
            let v = AttitudeVariance::of(&filter.state().attitude, filter.covariance());
            v.tilt_north + v.tilt_east
        };
        let before = tilt(&filter);
        let taken = filter.now().before(Seconds::from_secs(0.11));
        assert!(
            filter
                .fuse_course(taken, HeadingNoise::from_sigma(0.05))
                .is_reset()
        );
        let after = tilt(&filter);
        assert!(
            (after - before).abs() < 1e-3 * before,
            "tilt variance {before} -> {after}"
        );
    }

    #[test]
    fn a_forward_axis_near_vertical_refuses_both_gnss_heading_and_course() {
        // A tailsitter hovering: body x 25° from the sky, moving at 15 m/s with its velocity
        // held, so nothing but the geometry refuses the course.
        let mut filter = Eskf::default();
        let state = State {
            attitude: attitude_of(0.0, 1.134, 0.0),
            velocity: Velocity::ned(15.0, 0.0, 0.0),
            ..State::default()
        };
        let _ = filter
            .seed(state, Covariance::from_sigmas([0.5; STATES]))
            .expect("a sane seed");
        hold_velocity(&mut filter, Velocity::ned(15.0, 0.0, 0.0));
        let noise = HeadingNoise::from_sigma(0.05);
        assert_eq!(
            filter.fuse_gnss_heading(filter.now(), Radians::from_radians(0.3), noise),
            Fusion::Unobservable
        );
        assert_eq!(
            filter.fuse_course(filter.now(), noise),
            Fusion::Unobservable
        );
    }

    #[test]
    fn a_course_disagreeing_with_a_magnetometer_is_not_adopted() {
        // Nose north on the magnetometer, tracking east: a crab no sideslip allowed for.
        let east = Velocity::ned(0.0, 15.0, 0.0);
        let mut filter = cruising(east);
        let north = measured(attitude_of(0.0, 0.0, 0.0), 0.0);
        let noise = HeadingNoise::from_sigma(0.02);
        assert!(
            filter
                .fuse_mag_heading(filter.now(), north, noise)
                .is_reset()
        );
        hold(&mut filter, 10.0, 10, |filter| {
            let _ = filter.fuse_mag_heading(filter.now(), north, noise);
            hold_velocity(filter, east);
            let outcome = filter.fuse_course(filter.now(), noise);
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().course.recovered, 0);
    }
}
