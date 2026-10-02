//! Where the filter is: the origin of (43)–(44), the declination `D_m` of (6) and (35), and the
//! barometric reference `α₀` of (30) with its offset, (30′).
//!
//! Entry points: [`Eskf::set_origin`], [`Eskf::origin`], [`Eskf::geodetic_position`],
//! [`Eskf::set_magnetic_declination`], [`Eskf::magnetic_declination`],
//! [`Eskf::set_baro_reference`] and [`Eskf::baro_reference`].

use crate::geodetic::{Geodetic, LocalOrigin};
use crate::math::{exp_quat, wrap_pi};
use crate::state::{Offset, State};
use crate::units::{Altitude, AltitudeNoise, Attitude, Radians};
use nalgebra::Vector3;

use super::Eskf;

impl Eskf {
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
    pub(super) fn establish_reference(&mut self, reference: Option<(Altitude, Offset)>) {
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

    /// The offset a fresh covariance keeps: the reference's own variance, and no correlation
    /// with an error state that has just been replaced. A start that keeps the reference the
    /// flight had keeps its error too, since nothing about the reference changed.
    pub(super) fn surviving_offset(&self) -> Offset {
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
    /// be written in `f32` meters from the new origin.
    #[must_use = "a refused origin leaves the filter to place its own on the first fix"]
    pub fn set_origin(&mut self, origin: Geodetic) -> bool {
        let Some(new) = LocalOrigin::new(origin) else {
            return false;
        };
        if let Some(old) = self.origin {
            let position = new.to_ned(old.to_geodetic(self.estimate.state().position));
            // An origin further from the vehicle than `f32` reaches, such as a height of
            // 1e38 m, which `LocalOrigin::new` has no reason to refuse on its own.
            if !position.is_finite() {
                return false;
            }
            self.estimate.commit(State {
                position,
                ..*self.estimate.state()
            });
        }
        self.place_origin(new);
        true
    }

    /// Hold `origin` as the navigation origin, and read the site's declination from the
    /// magnetic model there unless the caller has set one. Every placement reads the model:
    /// [`set_origin`](Self::set_origin) and the coarse start's through here, and (44) through
    /// [`declination_at`](Self::declination_at) and
    /// [`learn_declination`](Self::learn_declination) directly, at the fix.
    ///
    /// GOALS.md differentiator 7: the site is the one thing the model needs, and the origin is
    /// the moment the filter learns it, as PX4 learns it from its first valid fix
    /// (`updateWorldMagneticModel`, `EKF/aid_sources/magnetometer/mag_control.cpp:642-644` at
    /// `c4e4ef98`).
    ///
    /// A heading referred to north through the declination alone is turned by the change,
    /// about navigation down, since the heading (6) leveled or a magnetometer set was the
    /// magnetic heading plus the old value and is that plus the new one. A static start is
    /// the usual case: it levels before any fix names the site, and the first fix arrives a
    /// declination later. Left alone, that is a standing innovation of the whole change on
    /// every heading (14.0° at 56° N, 44° E), which the gate of (37) reads as a disturbed
    /// magnetometer and turns away until [`Config::recovery`](crate::Config::recovery)
    /// adopts one. The covariance is kept as it was: the turn composes on the left, so the
    /// body-frame error `δθ` of (2) is the same error before and after it, and the tilt a
    /// window leveled against its accelerometer bias by (8) keeps that correlation in the
    /// body axes it was built in. A heading any true source has vouched for is not turned,
    /// and the change arrives as an innovation, as a caller's does. GNSS position and velocity
    /// do not count as one, though they correct heading through the correlations while the
    /// vehicle accelerates: a first origin arrives before that matters, and a caller moving
    /// the origin mid-flight to a distant site sets the declination itself.
    pub(super) fn place_origin(&mut self, origin: LocalOrigin) {
        if let Some(learned) = self.declination_at(origin.geodetic()) {
            self.learn_declination(learned);
        }
        self.origin = Some(origin);
    }

    /// What the magnetic model says at `site`, committing nothing: its declination there, and
    /// the turn about navigation down that gives a heading referred to north through the
    /// declination alone (zero for any other). `None` where there is nothing to learn: a
    /// declination the caller set, or no model.
    pub(super) fn declination_at(&self, site: Geodetic) -> Option<(Radians, f32)> {
        if self.declination_set {
            return None;
        }
        let declination = model_declination(site)?;
        let change = wrap_pi(declination.as_radians() - self.declination.as_radians());
        let turns = self.initialized && self.magnetic_north;
        Some((declination, if turns { change } else { 0.0 }))
    }

    /// Commit what [`declination_at`](Self::declination_at) read: the declination half of
    /// [`place_origin`](Self::place_origin), apart for (44), which places the origin with the
    /// turned heading before it commits the turn.
    pub(super) fn learn_declination(&mut self, (declination, turn): (Radians, f32)) {
        self.declination = declination;
        if turn != 0.0 {
            let mut turned =
                exp_quat(Vector3::z() * turn) * self.estimate.state().attitude.quaternion();
            turned.renormalize();
            self.estimate.commit(State {
                attitude: Attitude::from_quaternion(turned),
                ..*self.estimate.state()
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
    /// reads the declination from a World Magnetic Model table wherever it places its
    /// [`origin`](Self::origin), and turns a heading only the magnetometer has referred to
    /// north along with it (see [`Geodetic::magnetic_declination`]). A value set here is the
    /// caller's and the model never overrides it; that is the call for a site the table
    /// describes badly or a date far from its epoch. Zero until one or the other.
    ///
    /// Held by the filter rather than [`Config`](crate::Config) because it is a property of where
    /// the vehicle is, like the [`origin`](Self::origin) and `α₀`, and a vehicle that powers on
    /// before a GNSS fix learns its site only once a fix arrives. PX4 tracks it at runtime from the
    /// last valid GNSS position (`EKF/estimator_interface.h:485` at `c4e4ef98`).
    ///
    /// Read wherever a magnetic heading becomes a true one: the heading a static window
    /// commits by (6), every [`fuse_mag_heading`](Self::fuse_mag_heading) by (35), and the
    /// heading adoption and recovery. So set it before [`initialize`](Self::initialize) when
    /// the site is known. It moves no state. A change once heading is established arrives as
    /// an innovation of exactly the change on the next heading; a large one against the
    /// heading's σ is [`Fusion::Rejected`](crate::Fusion::Rejected) until
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
            .map(|origin| origin.to_geodetic(self.estimate.state().position))
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

#[cfg(test)]
mod tests {

    use crate::config::{Config, Correlation};
    use crate::eskf::Eskf;
    use crate::eskf::fixtures::*;

    use crate::geodetic::{Geodetic, LocalOrigin};
    use crate::health::{Fusion, GnssFusion, Propagation};
    use crate::init::Alignment;
    use crate::init::tests::{still, turning};

    use crate::observation::mag::tests::measured;

    use crate::state::{Covariance, ErrorState};
    use crate::units::{
        Altitude, AltitudeNoise, AngularRate, HeadingNoise, MagField, Position, PositionNoise,
        Radians, Seconds,
    };

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
        // covariance with p_D, and a second reading two meters higher moves α₀, not the
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
    fn a_reference_with_no_uncertainty_is_refused() {
        let mut filter = initialized();
        assert!(
            !filter.set_baro_reference(Altitude::from_meters(52.0), AltitudeNoise::from_sigma(0.0))
        );
        assert_eq!(filter.baro_reference(), None);
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

    /// A site whose declination is large, about 14° east, so a heading that
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
        assert_eq!(yaw_of(&filter), 0.0, "leveled at declination zero");
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
        // Heading leveled from the window's magnetometer, then a dual-antenna heading
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

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn the_origin_goes_under_the_antenna_the_turned_heading_puts() {
        // A magnetometer-leveled heading turned 14.0° by the first fix: the arm has to be
        // read after the turn, or the origin sits 0.24 m from where the estimate puts the
        // antenna.
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
        let _ = filter.fuse_gnss_geodetic(filter.now(), east_of_moscow(), noise, mast());
        let rotation = filter.state().attitude.quaternion();
        let antenna = Position::from_vector(rotation * mast().vector());
        let origin = filter.origin().expect("placed by the first fix");
        assert!(near(origin.to_ned(east_of_moscow()), antenna));
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
}
