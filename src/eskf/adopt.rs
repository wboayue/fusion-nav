//! Adoption: a measurement taken whole rather than fused, for a quantity the start never
//! established or a source locked out past [`Config::recovery`](crate::Config::recovery), and the
//! caller's own resets. The writers here are what an adoption commits; the decision to adopt is
//! each source's, in `fuse.rs` and `heading.rs`. Also the barometric reference a first altitude
//! reads from the estimate, (30) solved for `α₀` with its row of (30′).
//!
//! Entry points: [`Eskf::reset_position_to`] and [`Eskf::reset_velocity_to`].

use crate::frames::Ned;
use crate::state::{ErrorState, Offset, State};
use crate::units::{
    Altitude, AltitudeNoise, Position, PositionNoise, Timestamp, Velocity, VelocityNoise,
};

use super::Eskf;

impl Eskf {
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
    /// no innovation to dilute it. See [`Fusion::NotFinite`](crate::Fusion::NotFinite) and
    /// [`Fusion::InvalidNoise`](crate::Fusion::InvalidNoise).
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
    // Out of line, as `adopt_velocity`: inlined, its copy of `P` lands in the `fuse_*` frame that
    // `update` then stacks on. Called after `update` returns, it adds nothing to the deepest path.
    #[inline(never)]
    pub(super) fn adopt_position<const N: usize>(
        &mut self,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
        axes: [ErrorState; N],
    ) {
        let (z, r) = (position.vector(), noise.variance());
        let mut adopted = self.estimate.state().position.vector();
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
        self.estimate.commit(State {
            position: Position::ned(adopted[0], adopted[1], adopted[2]),
            ..*self.estimate.state()
        });
        self.reset_block(axes, variances);
    }

    /// Recover GNSS height: adopt the down axis, and let the barometer read its reference
    /// again against it.
    ///
    /// A lockout of GNSS height is a barometer holding the height somewhere the receiver
    /// disagrees with, so the reference that put it there is dropped with the height it
    /// described, and the next altitude reads one from the estimate as a start that left
    /// none does — where
    /// [`Config::baro_reference_from_estimate`](crate::Config::baro_reference_from_estimate) allows
    /// it. Where it does not, the caller owns the reference and it is kept, only decorrelated from
    /// the height `reset_block` replaced.
    pub(super) fn adopt_height(&mut self, position: Position<Ned>, noise: PositionNoise<Ned>) {
        self.adopt_position(position, noise, [ErrorState::PositionDown]);
        if self.config.baro_reference_from_estimate {
            self.establish_reference(None);
        }
    }

    /// Take a checked velocity solution as the velocity, all three axes.
    // Out of line for the reason `adopt_position` is.
    #[inline(never)]
    pub(super) fn adopt_velocity(&mut self, velocity: Velocity<Ned>, noise: VelocityNoise<Ned>) {
        self.estimate.commit(State {
            velocity,
            ..*self.estimate.state()
        });
        self.reset_block(
            [
                ErrorState::VelocityNorth,
                ErrorState::VelocityEast,
                ErrorState::VelocityDown,
            ],
            noise.variance(),
        );
    }

    /// Read `α₀` from the estimate at one altitude, `α̂₀ = α + p̂_D` — (30) solved for α₀ with
    /// `z = p̂_D`, so that the altitude lands on the estimate — correlated with the height it
    /// was read against, `P_bb = P_DD + R_m` and `P_xb = −P[:, D]` of (30′). See
    /// [`fuse_baro_altitude`](Self::fuse_baro_altitude) for why.
    pub(super) fn reference_from_estimate(
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
}

/// The position axes, as a first adoption and a caller's reset take them.
pub(super) const POSITION: [ErrorState; 3] = [
    ErrorState::PositionNorth,
    ErrorState::PositionEast,
    ErrorState::PositionDown,
];

/// The horizontal half of a GNSS fix, as its recovery adopts it.
pub(super) const HORIZONTAL: [ErrorState; 2] =
    [ErrorState::PositionNorth, ErrorState::PositionEast];

#[cfg(test)]
mod tests {

    use crate::config::{Config, GRAVITY, Recovery};
    use crate::eskf::Eskf;
    use crate::eskf::fixtures::*;
    use crate::frames::Ned;

    use crate::health::{Fusion, GnssFusion, Propagation};
    use crate::init::tests::still;

    use crate::propagate::ImuSample;
    use crate::state::{Covariance, ErrorState};
    use crate::units::{
        Acceleration, Altitude, AltitudeNoise, AngularRate, Position, PositionNoise, Seconds,
        Velocity, VelocityNoise,
    };

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
    fn a_reset_below_the_floor_is_floored_and_counted_rather_than_refused() {
        // The seed is refused because it writes the covariance in whole; a reset writes one
        // block against an estimate that exists, so the floor repairs it instead. This is
        // what keeps `floored` reachable from the public API at all, since
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

    #[test]
    fn an_adopted_fix_is_referred_to_the_imu() {
        let mut filter = coarse();
        let yaw = filter.state().attitude.quaternion();
        let at_antenna = Position::ned(10.0, 20.0, -5.0);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let adopted = filter.fuse_gnss_position(filter.now(), at_antenna, noise, mast());
        assert!(adopted.is_reset(), "{adopted:?}");
        let imu = at_antenna.vector() - yaw * mast().vector();
        assert!(near(filter.state().position, Position::from_vector(imu)));
    }

    #[test]
    fn an_adopted_velocity_is_the_imus_not_the_antennas_swing() {
        let mut filter = coarse();
        let imu = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 0.5),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );
        assert!(filter.step(imu, DT).is_propagated());
        let rotation = filter.state().attitude.quaternion();
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

    // ---- recovery from gate lockout ----

    /// A kilometer north of a vehicle that has not moved.
    fn far() -> Position<Ned> {
        Position::ned(1000.0, 0.0, 0.0)
    }

    #[test]
    fn a_fix_rejected_past_the_timeout_is_adopted_and_its_height_is_left_to_its_own_gate() {
        let mut filter = unheld();
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

        // Half a meter down: inside the height gate, and where an adoption of all three axes
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
        // take a kilometer back in, which is about 30 s at rest.
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
        let mut filter = unheld();
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
}
