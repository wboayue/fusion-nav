//! Propagation, equations (9)–(22), and the coast of (22′) across a gap.
//!
//! Entry point: [`Eskf::predict`].

use crate::health::Propagation;
use crate::propagate::{self, ImuSample, Propagated, propagate};

use super::Eskf;

impl Eskf {
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
    /// as [`Propagation::StepTooLong`]. Either way the timers advance, so [`Status`](crate::Status)
    /// degrades on schedule. It is `Δt` that is tested rather than an interval, because a gap is
    /// time no sample describes: a driver that integrated across a stall hands over an increment
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
    /// job. So an unaided filter's [`Validity`](crate::Validity) flags go false in the order their
    /// variances cross [`Config::accuracy`](crate::Config::accuracy), and [`Status`](crate::Status)
    /// does not follow: it reads the aiding timers and an alignment that has already latched.
    /// The one exception is the position hold of [`Config::hold`](crate::Config::hold), which a
    /// committed step runs while nothing aids the filter.
    pub fn predict(&mut self, imu: ImuSample) -> Propagation {
        let outcome = self.propagate_or_coast(imu);
        // After the step returns rather than inside it, so that the hold's update sits beside
        // the propagation's frame instead of above it ([measured]).
        //
        // [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#measured-cost-by-function
        match outcome {
            Propagation::Propagated => self.step_yaw_estimator(imu),
            Propagation::Coasted { .. } => self.restart_yaw_estimator(),
            // The clock did not move: nothing was missed.
            Propagation::NotInitialized | Propagation::InvalidStep { .. } => return outcome,
            // It did, and the yaw estimator integrated none of it.
            Propagation::StepTooLong { .. }
            | Propagation::NotFinite
            | Propagation::InvalidInterval { .. }
            | Propagation::StateNotFinite => {
                self.restart_yaw_estimator();
                return outcome;
            }
        }
        self.hold_if_unaided();
        outcome
    }

    /// [`predict`](Self::predict) without the hold: refuse, coast or integrate the step.
    #[inline(never)]
    fn propagate_or_coast(&mut self, imu: ImuSample) -> Propagation {
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
                *self.estimate.state(),
                self.covariance,
                self.offset,
                dt,
                &self.config,
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
            *self.estimate.state(),
            self.covariance,
            self.offset,
            imu,
            &self.config,
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
        self.estimate.step(self.time, propagated.state);
        self.angular_rate = propagated.omega;
        self.commit_covariance(propagated.covariance, propagated.offset);
        self.note_alignment();
        self.diagnostics.propagation.record(outcome);
        outcome
    }

    /// Record a refused step and hand the outcome back, as [`refuse`](super::fuse::refuse) does for
    /// a measurement.
    ///
    /// Same reason: one place maps an outcome to what
    /// [`PropagationHealth`](crate::PropagationHealth) counts, so a guard added to
    /// [`predict`](Self::predict) cannot forget to count it. Whether the timers moved is
    /// the guard's business, not this one's.
    fn refuse_step(&mut self, outcome: Propagation) -> Propagation {
        self.diagnostics.propagation.record(outcome);
        outcome
    }
}

#[cfg(test)]
mod tests {

    use crate::config::{Coast, Config, GRAVITY};
    use crate::eskf::Eskf;
    use crate::eskf::fixtures::*;

    use crate::health::{Fusion, Propagation};
    use crate::init::Alignment;
    use crate::init::tests::still;

    use crate::observation::mag::tests::attitude_of;

    use crate::propagate::ImuSample;
    use crate::state::{Covariance, ErrorState, STATES, State};
    use crate::units::{
        Acceleration, AngularRate, Attitude, Position, PositionNoise, Seconds, Velocity,
    };

    use nalgebra::Vector3;

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
    /// was initialized with. Without them a static start would hold `Initialization`'s sigmas
    /// for the whole flight.
    #[test]
    fn a_step_grows_the_covariance() {
        let mut filter = unheld();
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

    /// Level and still, seeded with a tilt the propagation can leak into velocity.
    fn seeded_level(config: Config) -> Eskf {
        let mut filter = Eskf::new(config).unwrap();
        let mut sigmas = [0.5f32; STATES];
        sigmas[ErrorState::AttitudeX.index()] = 0.05;
        sigmas[ErrorState::AttitudeY.index()] = 0.05;
        let _ = filter
            .seed(State::default(), Covariance::from_sigmas(sigmas))
            .expect("a sane seed");
        filter
    }

    #[test]
    fn a_vehicle_at_rest_under_its_own_gravity_does_not_fall() {
        // (11): the specific force a still vehicle reads is the site's `γ`.
        let reading = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, -SITE_GRAVITY),
        );
        let fallen = |config| {
            let mut filter = seeded_level(config);
            for _ in 0..100 {
                assert!(
                    filter
                        .step(reading, Seconds::from_secs(0.01))
                        .is_propagated()
                );
            }
            filter.state().velocity.to_array()[2]
        };
        assert!(fallen(at_site()).abs() < 1e-5, "{}", fallen(at_site()));
        let off = fallen(Config::default());
        assert!((off - (GRAVITY - SITE_GRAVITY)).abs() < 1e-4, "{off}");
    }

    #[test]
    fn a_coast_leaks_tilt_into_velocity_through_the_gravity_configured() {
        // (22′) with no noise of its own: the horizontal velocity variance a coast adds is the
        // tilt's, through (17)'s `γ`, so it scales as `γ²`.
        let quiet = |config: Config| Config {
            imu: crate::config::ImuNoise {
                gyro_white: 0.0,
                accel_white: 0.0,
                gyro_bias_walk: 0.0,
                accel_bias_walk: 0.0,
            },
            coast: Some(Coast {
                acceleration: 0.0,
                rotation: 0.0,
            }),
            ..config
        };
        let grown = |config| {
            let mut filter = seeded_level(quiet(config));
            let before = filter.covariance().variance(ErrorState::VelocityNorth);
            assert!(matches!(
                filter.step(still().imu, Seconds::from_secs(1.2)),
                Propagation::Coasted { .. }
            ));
            filter.covariance().variance(ErrorState::VelocityNorth) - before
        };
        let ratio = grown(at_site()) / grown(Config::default());
        let expected = (SITE_GRAVITY / GRAVITY) * (SITE_GRAVITY / GRAVITY);
        // 1.1e-4 off, the rest of `F` over the gap; read against one `γ`, the ratio would be 1.
        assert!(
            (ratio - expected).abs() < 3e-4,
            "{ratio} against {expected}"
        );
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
                hold: None,
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
        let rotation = filter.state().attitude.quaternion();
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
}
