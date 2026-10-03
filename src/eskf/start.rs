//! Starting the filter, from a still window, one sample or a caller's seed. Equations (5)–(8),
//! the reference `α₀` of (30) a window at rest measures, and (42) on a seed.
//!
//! Entry points: [`Eskf::initialize`], [`Eskf::initialize_coarse`], [`Eskf::initialize_from`]
//! and [`Eskf::alignment_of`]. A start is worked out whole ([`Startup`]), checked, then
//! committed, so a refused one commits nothing.

use crate::health::Diagnostics;
use crate::init::{
    self, Alignment, Coarse, GyroBias, InitError, Measured, StaticSample, StaticWindow,
};
use crate::math::{below_floor, enforce_symmetry};
use crate::propagate::ImuSample;
use crate::state::{Covariance, Offset, State};
use crate::units::{Altitude, Timestamp};

use super::{Eskf, Unestablished};

impl Eskf {
    /// Align from a window of samples taken while the vehicle was, ideally, still.
    /// Equations (5)–(8), and `α₀` of (30).
    ///
    /// The window does not have to be a genuine static interval. If it is — long enough
    /// and within the tolerances [`Initialization`](crate::Initialization) sets — the
    /// result is [`Alignment::Static`] and the filter starts with the covariance that
    /// configuration describes. If it is not, the result is [`Alignment::Coarse`],
    /// carrying what was measured: the filter still runs, with attitude uncertainty
    /// inflated to match, and reports [`Status::Aligning`](crate::Status::Aligning) until that
    /// uncertainty comes down. Refusing instead would be a launch restriction, and a filter that
    /// will not start is worth less than one that starts and says how much to trust it.
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
    /// rather than fused — see [`Fusion::Reset`](crate::Fusion::Reset).
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
        let startup = self.startup(window)?;
        let alignment = startup.alignment;
        self.commit_startup(startup);
        Ok(alignment)
    }

    /// The start `window` gives, worked out and checked but not committed: what
    /// [`initialize`](Self::initialize) commits and [`alignment_of`](Self::alignment_of)
    /// reports, so the two cannot disagree.
    fn startup(&self, window: &StaticWindow) -> Result<Startup, InitError> {
        let measured = window.measured()?;
        let alignment = init::classify(&measured, &self.config.init, self.config.gravity);
        // One answer to "was the vehicle on the ground", read by the gyroscope bias of
        // (7), the barometric reference of (30), and what this start establishes.
        // Measured from the window rather than read off `alignment`, because a window too
        // short to align an attitude from can still be a window of a parked vehicle.
        let at_rest = init::at_rest(measured.peaks, &self.config.init, self.config.gravity);
        let gyro_bias = measured.gyro_bias(at_rest, &self.config);
        let reference = at_rest.then(|| {
            window
                .alpha0()
                .map(|(reference, variance)| (reference, Offset::independent(variance)))
        });
        self.checked_startup(
            alignment,
            &measured,
            gyro_bias,
            at_rest,
            measured.end,
            reference,
        )
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
    /// As [`initialize`](Self::initialize), whose work it does up to the commit.
    pub fn alignment_of(&self, window: &StaticWindow) -> Result<Alignment, InitError> {
        self.startup(window).map(|startup| startup.alignment)
    }

    /// Start from a single IMU sample, with no window at all.
    ///
    /// For the launch that never offers one: a hand launch, a deck that is always moving,
    /// a restart at altitude. Tilt comes from one accelerometer reading, which carries the
    /// sensor's full noise and whatever the vehicle's own acceleration was at that
    /// instant, so the covariance is inflated accordingly and the filter reports
    /// [`Status::Aligning`](crate::Status::Aligning).
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
        // Treated as a window of one, so the same finiteness, motion and averaging measures apply —
        // an average of one sample being that sample. The window is most of this frame, which stays
        // well under `update`'s.
        let mut window = StaticWindow::new();
        window.push(StaticSample {
            imu,
            ..StaticSample::default()
        })?;
        // A window of one: its own average, with no rotation to smear it and no second
        // velocity to difference against — a caller with GNSS in hand has a window, not
        // this entry point.
        let measured = window.measured()?;
        let alignment = Alignment::Coarse(Coarse::not_stationary(&measured, self.config.gravity));
        // The same rule `initialize` applies: the gyroscope bias is weighed only where the
        // sample says the vehicle was on the ground, and one reading is weighed by its own
        // noise, which at a sample's interval leaves it mostly to the prior.
        let at_rest = init::at_rest(measured.peaks, &self.config.init, self.config.gravity);
        let gyro_bias = measured.gyro_bias(at_rest, &self.config);
        // That reading establishes nothing, which is why it is not passed on as one. A
        // window shows rest by holding still over a span of time and this one spans none:
        // an accelerometer reading `γ` for an instant is a hover as readily as a vehicle
        // on the ground, and this entry point exists for the launches that are moving.
        // The barometric reference is left alone: one sample does not establish one, and
        // a restart at altitude should keep the reference the flight began with.
        let startup =
            self.checked_startup(alignment, &measured, gyro_bias, false, imu.time, None)?;
        self.commit_startup(startup);
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
    /// through [`from_body_to_ned`](crate::Attitude::from_body_to_ned), a ROS one through
    /// [`from_flu_to_enu`](crate::Attitude::from_flu_to_enu), which is where the conventions and
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
    /// confident seed reports [`Status::Healthy`](crate::Status::Healthy) straight away, a coarse
    /// one [`Status::Aligning`](crate::Status::Aligning) until it converges.
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
        // (42): a seed built by another filter's arithmetic differs from its transpose in the
        // last bit, and nothing after a start repairs it before the first gate reads it.
        let mut seed = *covariance.as_matrix();
        enforce_symmetry(&mut seed);
        // Nothing a seed carries is unestablished: the caller vouched for every quantity,
        // heading included, so no first measurement overwrites one.
        self.start(
            state,
            Covariance::from_matrix(seed),
            Unestablished::default(),
            time,
            false,
        );
        self.note_alignment();
        Ok(Alignment::Seeded)
    }

    /// Work out a start, the nominal state of (7) with `gyro_bias` and the covariance of (8),
    /// and check the whole of it before anything is committed. `reference` is the barometric
    /// reference the start sets: `None` leaves the one held alone, `Some(None)` clears it.
    ///
    /// Refused as [`InitError::NotFinite`] when the state, the covariance or the reference is
    /// not finite. Every sample was finite, but their averages can still overflow what (5),
    /// (8) and (30) square: one accelerometer reading of `f32::MAX` levels to a NaN tilt
    /// variance, and one barometer reading of it gives an infinite `P_bb`.
    fn checked_startup(
        &self,
        alignment: Alignment,
        measured: &Measured,
        gyro_bias: GyroBias,
        settled: bool,
        time: Timestamp,
        reference: Option<Option<(Altitude, Offset)>>,
    ) -> Result<Startup, InitError> {
        let state = init::nominal_state(measured, self.declination, gyro_bias.estimate);
        // The bias of (7) as committed, so that what it absorbed is not charged a second
        // time as motion the window could not vouch for; see `init::coarse_sigmas`.
        let (sigma_tilt, sigma_yaw) = init::attitude_sigmas(
            &self.config.init,
            alignment,
            measured,
            state.gyro_bias,
            self.config.gravity,
        );
        let covariance = init::initial_covariance(
            &self.config.init,
            &state.attitude,
            sigma_tilt,
            sigma_yaw,
            measured.level_scatter,
            gyro_bias.sigmas,
            self.config.gravity,
        );
        let reference_finite = reference.flatten().is_none_or(|(altitude, offset)| {
            altitude.as_meters().is_finite() && offset.variance.is_finite()
        });
        if !(state.is_finite() && covariance.is_finite() && reference_finite) {
            return Err(InitError::NotFinite);
        }
        Ok(Startup {
            alignment,
            state,
            covariance,
            unestablished: Unestablished::after(settled, measured.field.is_some()),
            time,
            settled,
            magnetic_north: measured.field.is_some(),
            reference,
        })
    }

    /// Commit a start: the nominal state equation (7) built, the covariance of (8), fresh
    /// health, and the barometric reference the start sets, if any.
    ///
    /// A start the window showed at rest clears the origin, on the same evidence that
    /// establishes its position: both are the claim *zero is here*, and an origin held
    /// from before says zero is somewhere else. The next geodetic fix places a new one.
    /// A start taken in motion keeps it, because its position is unestablished and the
    /// first fix is adopted about the origin the flight already has. The two move
    /// together or a still short window would report an established position of `(0,0,0)`
    /// about an origin nothing put under it.
    fn commit_startup(&mut self, startup: Startup) {
        self.start(
            startup.state,
            startup.covariance,
            startup.unestablished,
            startup.time,
            startup.settled,
        );
        // (6) leveled the heading from the window's field with the declination it held.
        self.magnetic_north = startup.magnetic_north;
        if startup.settled {
            self.origin = None;
        }
        if let Some(reference) = startup.reference {
            self.establish_reference(reference);
        }
        self.note_alignment();
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
        self.estimate.restart(time, state);
        // Diagnostics first: `commit_covariance` counts into them, and a start sitting on the
        // floor is a fact about this filter's life rather than the last one's.
        self.diagnostics = Diagnostics::default();
        self.commit_covariance(covariance, self.surviving_offset());
        self.unestablished = unestablished;
        self.magnetic_north = false;
        self.angular_rate = None;
        self.initialized = true;
        self.time = time;
        self.earliest = if at_rest { Timestamp::ZERO } else { time };
        // A fresh start is unaligned until its own covariance says otherwise, which
        // `note_alignment` reads at the end of each entry point.
        self.aligned = false;
        // A hold belongs to the life it held; the next outage anchors afresh.
        self.anchor = None;
    }
}

/// A start from a window or a sample, worked out and checked but not yet committed; see
/// [`Eskf::checked_startup`].
struct Startup {
    alignment: Alignment,
    state: State,
    covariance: Covariance,
    unestablished: Unestablished,
    time: Timestamp,
    /// Whether the start showed the vehicle at rest.
    settled: bool,
    /// Whether (6) leveled the heading from a magnetometer, so it is referred to north
    /// through the declination alone.
    magnetic_north: bool,
    /// The barometric reference the start sets: `None` keeps the one held, `Some(None)`
    /// clears it.
    reference: Option<Option<(Altitude, Offset)>>,
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

#[cfg(test)]
mod tests {

    use crate::config::{Config, GRAVITY};
    use crate::eskf::Eskf;
    use crate::eskf::fixtures::*;

    use crate::health::Status;
    use crate::init::tests::{gravity_at, still, turning};
    use crate::init::{Alignment, Coarse, InitError, StaticSample, StaticWindow};

    use crate::state::{Covariance, ErrorState, State};
    use crate::units::{
        Acceleration, Altitude, AltitudeNoise, AngularRate, Attitude, Position, PositionNoise,
        Quaternion, Seconds, Timestamp, Velocity, VelocityNoise,
    };

    /// That `initialize_coarse` weighs its one reading, `Measured::gyro_bias`, rather than
    /// taking it whole; the weighing itself is tested in `init.rs`.
    #[test]
    fn one_still_sample_starts_the_gyroscope_bias_no_less_certain_than_the_prior() {
        let mut filter = Eskf::default();
        let sample = still()
            .imu
            .with_gyro(AngularRate::body(0.05, 0.05, 0.05))
            .timed(Timestamp::from_micros(5_000), Seconds::from_secs(0.005));
        assert!(filter.initialize_coarse(sample).is_ok());
        let prior = Config::default().init.sigma_gyro_bias.as_rad_per_s();
        let sigma = filter.covariance().variance(ErrorState::GyroBiasX).sqrt();
        assert!(sigma <= prior && sigma > 0.99 * prior, "{sigma}");
        assert!(filter.state().gyro_bias.vector().x.abs() < 1e-3);
    }

    #[test]
    fn a_window_is_still_against_the_gravity_it_is_configured_with() {
        // Short of the default `γ` by just over `max_accel_deviation`, 1.961, and short of
        // 9.79 by just under it: moving at the default, at rest at the site, and only a start
        // at rest takes the window's barometric reference.
        let force = GRAVITY - 1.966;
        let window = window_at(100.0).map(|sample| StaticSample {
            imu: sample.imu.with_accel(Acceleration::body(0.0, 0.0, -force)),
            ..sample
        });
        let start = |config| {
            let mut filter = Eskf::new(config).unwrap();
            let alignment = filter
                .initialize_over(&window, Seconds::from_secs(0.25))
                .expect("a usable window");
            (alignment, filter.baro_reference().is_some())
        };
        assert_eq!(start(at_site()), (Alignment::Static, true));
        assert!(matches!(
            start(Config::default()),
            (Alignment::Coarse(Coarse::NotStationary { .. }), false)
        ));
    }

    #[test]
    fn the_tilt_a_biased_accelerometer_levels_in_is_the_bias_over_gravity() {
        // (8): `P_θβa = −(σ_βa² / γ)[d̂]×`, so the correlation scales as 1/γ.
        let cross = |config| {
            let mut filter = Eskf::new(config).unwrap();
            let _ = filter
                .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
                .expect("a still window");
            let m = *filter.covariance().as_matrix();
            m.fixed_view::<3, 3>(
                ErrorState::AttitudeX.index(),
                ErrorState::AccelBiasX.index(),
            )
            .abs()
            .max()
        };
        let ratio = cross(at_site()) / cross(Config::default());
        assert!((ratio - GRAVITY / SITE_GRAVITY).abs() < 1e-5, "{ratio}");
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
    fn a_seed_quaternion_is_normalized_and_a_zero_one_is_refused() {
        let (state, covariance) = seed();
        // A zero quaternion is no rotation.
        let zero = State {
            attitude: Attitude::from_body_to_ned(Quaternion {
                w: 0.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            ..state
        };
        let mut filter = Eskf::default();
        assert_eq!(filter.seed(zero, covariance), Err(InitError::NotFinite));
        assert!(!filter.is_initialized());

        // One `f32::MAX` component overflows the norm, which `nalgebra` alone normalizes to a
        // finite zero the seed would accept. It is the half turn about z.
        let huge = State {
            attitude: Attitude::from_body_to_ned(Quaternion {
                w: 0.0,
                x: 0.0,
                y: 0.0,
                z: f32::MAX,
            }),
            ..state
        };
        assert!(filter.seed(huge, covariance).is_ok());
        assert_eq!(
            filter.state().attitude.body_to_ned(),
            Quaternion {
                w: 0.0,
                x: 0.0,
                y: 0.0,
                z: 1.0
            }
        );
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

    #[test]
    fn a_static_window_commits_the_attitude_it_leveled() {
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
            "leveled to ({roll}, {pitch})"
        );
        assert_eq!(yaw, 0.0, "no magnetometer observed the rotation about it");
    }

    #[test]
    fn a_still_window_commits_its_gyroscope_bias_and_a_moving_one_does_not() {
        // About the offset by ±10⁻⁴ rad/s, so the window measures how well it knows it.
        let offset = AngularRate::body(0.01, -0.02, 0.003);
        let mut window = [still(); 8];
        for (index, sample) in window.iter_mut().enumerate() {
            let noise = if index % 2 == 0 { 1e-4 } else { -1e-4 };
            sample.imu = sample
                .imu
                .with_gyro(AngularRate::from_vector(offset.vector().add_scalar(noise)));
        }

        let mut filter = Eskf::default();
        assert_eq!(
            filter
                .initialize_over(&window, Seconds::from_secs(0.25))
                .expect("0.022 rad/s is well inside the tolerance"),
            Alignment::Static
        );
        assert!(
            (filter.state().gyro_bias.vector() - offset.vector()).norm() < 1e-5,
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
    fn one_sample_is_leveled_like_a_window_of_one() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_coarse(still().imu.with_accel(gravity_at(0.0, 0.35, 0.0)))
            .expect("a finite sample");

        let (roll, pitch, _) = filter.state().attitude.euler_angles();
        assert!(
            roll.abs() < 1e-6 && (pitch - 0.35).abs() < 1e-5,
            "leveled to ({roll}, {pitch})"
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
}
