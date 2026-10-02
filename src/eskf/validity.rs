//! What the estimate can claim: per-quantity [`Validity`] from the covariance against
//! [`Config::accuracy`](crate::Config::accuracy), the prediction an arming check reads, and the
//! alignment latch behind [`Status::Aligning`](crate::Status::Aligning).
//!
//! Entry points: [`Eskf::validity`], [`Eskf::predicted_validity`], [`Eskf::attitude_variance`]
//! and [`Eskf::is_aligned`].

use crate::config::{ALIGNED_HEADING, ALIGNED_TILT};
use crate::health::{SourceHealth, Validity};
use crate::propagate::project;
use crate::state::{AttitudeVariance, Covariance, ErrorState};
use crate::units::Radians;

use super::Eskf;

impl Eskf {
    /// How uncertain the attitude is about each navigation axis: tilt about north and east,
    /// heading about down.
    ///
    /// The numbers [`validity`](Self::validity) tests against
    /// [`Accuracy`](crate::Accuracy), and the ones to plot beside a tilt or a heading. The
    /// attitude rows of [`covariance`](Self::covariance) are in body axes and mean tilt and
    /// heading only while the vehicle is level; see [`AttitudeVariance`]. All zero before
    /// the filter is initialized, which is the covariance it then holds.
    pub fn attitude_variance(&self) -> AttitudeVariance {
        AttitudeVariance::of(&self.estimate.state().attitude, &self.covariance)
    }

    /// Whether the attitude has **ever** met [`ALIGNED_TILT`] and [`ALIGNED_HEADING`] since
    /// initialization — the test behind [`Status::Aligning`](crate::Status::Aligning).
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
    /// useless as a report that the filter has not finished starting up. PX4 and ArduPilot both
    /// latch it for the same reason: `tilt_align` and `tiltAlignComplete` are only ever tested
    /// while false (`src/modules/ekf2/EKF/control.cpp:73-78` at `c4e4ef98e9`,
    /// `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp:520-525` at `368dc0c428`).
    pub const fn is_aligned(&self) -> bool {
        self.aligned
    }

    /// Latch [`is_aligned`](Self::is_aligned) if the attitude now meets the bar.
    ///
    /// Called from every path that can move the attitude covariance or establish heading.
    /// A latch is state, so it cannot be derived on read the way [`Status`](crate::Status) is — and
    /// reading it lazily would make the answer depend on whether anyone asked.
    pub(super) fn note_alignment(&mut self) {
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
        let variance = AttitudeVariance::of(&self.estimate.state().attitude, p);
        variance.tilt_north <= sigma * sigma && variance.tilt_east <= sigma * sigma
    }

    /// Whether heading has been established and is within `bar`.
    ///
    /// A heading nothing observed fails whatever its variance: stillness never observes yaw,
    /// and a prior on a yaw nobody measured is not an estimate of one.
    fn heading_within(&self, p: &Covariance, bar: Radians) -> bool {
        let sigma = bar.as_radians();
        !self.unestablished.heading
            && AttitudeVariance::of(&self.estimate.state().attitude, p).heading <= sigma * sigma
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
    /// window brings it in and the gyroscope's noise and bias take it back out through (20),
    /// on a schedule the covariance knows and no acceptance timer does. At
    /// [`ImuNoise`](crate::ImuNoise)'s defaults an unaided start holds tilt for 10.3 s from a
    /// window that measured its gyroscope and 4.84 s from one whose gyroscope never scattered,
    /// so a horizon under that arms and one over it does not, which the current value alone
    /// cannot say.
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
            self.estimate.state(),
            self.covariance,
            self.config.accuracy.horizon,
            &self.config,
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
    /// [`Status`](crate::Status), [`predicted_validity`](Self::predicted_validity), the course and
    /// the recovery guards all ask.
    pub(super) fn accepted_recently(&self, source: &SourceHealth) -> bool {
        source.is_fresh(&self.config.timeouts)
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

#[cfg(test)]
mod tests {

    use crate::config::{Accuracy, Config, GRAVITY};
    use crate::eskf::Eskf;
    use crate::eskf::fixtures::*;

    use crate::health::{Propagation, Status, Validity};
    use crate::init::tests::still;
    use crate::init::{Alignment, Coarse, StaticSample};

    use crate::observation::mag::tests::attitude_of;

    use crate::state::{AttitudeVariance, Covariance, ErrorState, STATES, State};
    use crate::units::{
        Altitude, AltitudeNoise, AngularRate, HeadingNoise, MagField, Position, PositionNoise,
        Radians, Seconds,
    };

    /// The margin [`Accuracy`]'s defaults buy, measured rather than derived, from a static start
    /// with a magnetometer in the window at [`ImuNoise`](crate::ImuNoise)'s defaults: 10.3 s of
    /// unaided tilt and 274 s of heading from a window that measured its gyroscope, 4.84 s and
    /// 51.4 s from one whose gyroscope never scattered, whose `ω̄` is weighed under the
    /// configured density.
    ///
    /// The gyroscope-bias prior reaches attitude through (20)'s `−I Δt` and grows as
    /// `σ_βg² t²`, so the window sets both figures. Measured, it leaves tilt to the white
    /// noise's `σ_g² t`, 10.4 s alone, and heading to that and the bias walk. A test rather than
    /// a comment because [`Accuracy`] cites the numbers: change a bar or a density and this says
    /// by how much the margin moved.
    #[test]
    fn an_unaided_start_holds_its_attitude_for_the_margin_the_defaults_buy() {
        // Forty samples 50 ms apart, the gyroscope alternating ±1.9 × 10⁻³ rad/s on every
        // axis: blocks enough for (8″), and `ω̄` known to 3 × 10⁻⁴ rad/s.
        let mut measured = [StaticSample {
            mag: Some(MagField::body(0.22, 0.0, 0.44)),
            ..still()
        }; 40];
        for (index, sample) in measured.iter_mut().enumerate() {
            let rate = if index % 2 == 0 { 1.9e-3 } else { -1.9e-3 };
            sample.imu = sample.imu.with_gyro(AngularRate::body(rate, rate, rate));
        }
        let (tilt, heading) = held(&measured, Seconds::from_secs(0.05));
        assert!((tilt - 10.3).abs() < 0.1, "tilt held {tilt} s");
        assert!((heading - 274.0).abs() < 2.0, "heading held {heading} s");

        // A gyroscope that never scatters, held or coarsely quantized, measures nothing of
        // itself, so the configured density is weighed against the prior.
        let (tilt, heading) = held(&window_with_mag(), Seconds::from_secs(0.25));
        assert!((tilt - 4.84).abs() < 0.05, "tilt held {tilt} s");
        assert!((heading - 51.4).abs() < 0.3, "heading held {heading} s");
    }

    /// How long a static start from `window` holds tilt and heading unaided.
    fn held(window: &[StaticSample], dt: Seconds) -> (f32, f32) {
        let mut filter = Eskf::default();
        assert_eq!(filter.initialize_over(window, dt), Ok(Alignment::Static));

        let dt = Seconds::from_secs(0.005);
        let holding_still = still().imu;
        let mut tilt_held = None;
        // Past the 274 s the measured window buys, at 5 ms.
        for step in 1..100_000 {
            assert_eq!(filter.step(holding_still, dt), Propagation::Propagated);
            let elapsed = step as f32 * dt.as_secs();
            let validity = filter.validity();
            if tilt_held.is_none() && !validity.tilt {
                tilt_held = Some(elapsed);
            }
            if !validity.heading {
                return (tilt_held.unwrap_or(f32::NAN), elapsed);
            }
        }
        (tilt_held.unwrap_or(f32::NAN), f32::NAN)
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

        // Past the 4.84 s this unscattered window buys, with a GNSS fix arriving at 2 Hz so
        // that aiding is never stale. A 100 m one, which holds the status out of `DeadReckoning`
        // and tells tilt nothing: a velocity would, through the accelerometer bias (8)
        // correlates it with.
        for step in 1..=1_200 {
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
                // And the barometer, whose silence would degrade the status by itself.
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
        // tilt and, its gyroscope unscattered, holds it for 4.84 s unaided, so a
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

        assert_eq!(ask(1.0), (true, true), "a second is inside the 4.84 s hold");
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
}
