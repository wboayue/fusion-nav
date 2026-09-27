//! Magnetometer observation models. Equations (34)–(36), and the levelling variance
//! (36′) that the heading's own noise does not cover.

use nalgebra::{ComplexField, RealField, SVector, Vector3};

use crate::frames::Body;
use crate::init;
use crate::math::wrap_pi;
use crate::observation::heading::heading_jacobian;
use crate::state::{Covariance, ErrorState, State};
use crate::units::{HeadingNoise, MagField, Radians};
use crate::update::Observation;

/// `y`, the yaw error a magnetic field reports. Equations (34) and (35).
///
/// (34) rotates the measurement into the navigation frame with the attitude estimate,
/// `m̃_n = R(q̂) m_b`. Were that attitude exact, the horizontal part would make the
/// declination angle with North; whatever is left over is yaw error, and (35) is that
/// residual negated, `y = −wrap(atan2(m̃_E, m̃_N) − D_m)`.
///
/// Formed directly rather than as a difference of two angles, and wrapped **before** the
/// result is used, so a vehicle heading near ±180° produces a small innovation rather than
/// a spurious 2π one.
///
/// This is [`init::heading_from_mag`](crate::init::heading_from_mag) less the estimated
/// yaw: `R(q̂) = Rz(ψ̂) R₀`, so the two `atan2` arguments differ by exactly `ψ̂` and
/// `y = wrap(ψ_mag − ψ̂)`. They stay separate because reaching the other would cost a
/// quaternion-to-Euler round trip here, and because (6) commits a heading where this
/// innovates against one; the test below pins the identity so the declination sign and
/// the wrap convention cannot drift apart between them.
///
/// A field of exactly zero — a stopped or disconnected magnetometer — gives
/// `atan2(0, 0) = 0` and reports the vehicle as `D_m` off rather than producing NaN, the
/// way [`init::level_from_accel`](crate::init::level_from_accel) is written to. Whether
/// the field is worth believing is the application's question, since this crate does no
/// calibration and a field strength test is part of one.
pub(crate) fn heading_innovation(
    state: &State,
    field: MagField<Body>,
    declination: Radians,
) -> f32 {
    // (34): the measurement in navigation axes, where the horizontal part is a heading.
    let m_n = state.attitude.quaternion().to_rotation_matrix() * field.vector();
    // (35).
    -wrap_pi(RealField::atan2(m_n.y, m_n.x) - declination.as_radians())
}

/// A magnetic heading as the update reads it: `y` from (35), `H` from (36), and the
/// caller's variance widened by the levelling of (36′).
///
/// One degree of freedom, because the three-axis field is reduced to a single scalar
/// before the update sees it. That is the decision `GOALS.md` records under
/// [magnetometer without magnetic-field states](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#magnetometer-without-magnetic-field-states):
/// this crate carries no field states to absorb hard- and soft-iron error, so a
/// disturbance that reached roll and pitch would corrupt the two quantities gravity
/// determines well. Equations (31)–(33) describe the three-axis alternative and are
/// deliberately not built.
///
/// # Why `R` is not the caller's number alone
///
/// (34) levels the field with the *estimated* attitude, so the heading it produces is
/// only as good as that attitude: a tilt error tips the field and turns its horizontal
/// part by `tan δ` times as much, the same leak equation (8′) charges a coarse window's
/// heading prior for. `noise` cannot carry it, because the caller hands over a field and
/// never sees the rotation applied to it — the filter performs the levelling and holds
/// the tilt covariance, so the filter is the only layer that can price it.
///
/// (36′) adds it to `R` rather than to `H`. The exact Jacobian of (35) carries the term
/// too, and using it is worse than dropping it: it corrects tilt — which gravity already
/// determines an order of magnitude better — from a scalar carrying 3° of noise, and
/// measured `mission` at 9.0° of tilt error and 1.26 m/s² of accelerometer bias against
/// 0.58° and 0.053 for the form here. Widening `S` says the heading is less trustworthy
/// than its own noise suggests without claiming it observes the tilt that made it so.
/// That is `R` inflation with no cross-covariance, and it suffices because velocity fusion
/// keeps correcting the tilt it prices; an error shared unchanged across readings needs the
/// cross-covariance too, which is (30′). On `moving_start`, whose coarse start is where an unpriced levelling error
/// is largest, it is worth tilt 2.653° → 1.720, yaw 3.777° → 0.940, `nees_att`
/// 1.634 → 0.231, and 840 falsely-valid attitude quantity-epochs → 0.
///
/// `R(q̂)` is formed twice, once for (35) and once for (36). Sharing it would thread a
/// rotation matrix through both signatures to save about twenty multiplications on a call
/// that already runs a 15 × 15 update; the obvious form wins that trade under
/// `AGENTS.md`'s rule, and a measurement is what would reopen it.
pub(crate) fn heading_observation(
    state: &State,
    covariance: &Covariance,
    field: MagField<Body>,
    declination: Radians,
    noise: HeadingNoise,
) -> Observation<1> {
    let h = heading_jacobian(state);
    // Navigation down in body axes, `R(q̂)ᵀe₃`: (36)'s row read back as a column.
    let down = h
        .fixed_view::<1, 3>(0, ErrorState::AttitudeX.index())
        .transpose();
    let r_m =
        SVector::<f32, 1>::new(noise.variance() + levelling_variance(covariance, field, down));
    Observation {
        y: SVector::<f32, 1>::new(heading_innovation(state, field, declination)),
        h,
        h_b: SVector::<f32, 1>::zeros(),
        r_m,
        r_gain: r_m,
    }
}

/// `tan²δ · σ_tilt²`, the variance (34)'s levelling adds to a heading. Equation (36′).
///
/// `tan δ` is [`init::heading_sensitivity`](crate::init::heading_sensitivity), the same
/// ratio (8′) uses, read off this field rather than configured. It is an angle between a
/// field and a direction, so the frame the two share does not matter; the direction here
/// is `down`, navigation down resolved in body axes, which is (36)'s Jacobian row
/// transposed — `e₃ᵀR(q̂)` read as a column is `R(q̂)ᵀe₃`. It must be a unit vector, as a
/// row of a rotation is, or `d × f̂` below is not one.
///
/// `σ_tilt²` is the largest eigenvalue of the tilt block, the attitude covariance on the
/// horizontal plane: the variance of tilt about the worst horizontal axis. To first order
/// only tilt about the field's own horizontal direction `f̂` leaks, so `f̂ᵀ P f̂` is the exact
/// price of one reading. The bound is kept because it is the bound, not for a margin: fused
/// as white, the exact form lost (`gnss_outage` `pos_h` 2.630 m against 2.227), since
/// consecutive headings share a tilt error velocity fusion corrects only over seconds; under
/// (24′), which prices that sharing as the magnetometer's `τ`, the two agree to 0.3 % on
/// position and neither is overconfident on 50 seeds. `EQUATIONS.md` has the derivation and
/// both measurements.
///
/// The eigenvalue depends on no choice of axes, where the larger of two diagonals does: on
/// an anisotropic block the north/east and body x/y pairs give different maxima, and the
/// north/east one moves with yaw. It is never below either diagonal, so it errs toward an
/// `R` too large, which only slows the heading's correction, rather than too small, which is
/// what produced 840 falsely-valid attitude epochs on `moving_start` without the term.
///
/// The block is taken on the basis `f̂`, `down × f̂` in body axes; the eigenvalue is the same on any orthonormal basis of the plane, and
/// this one needs no rotation of `P`, which stays in the body axes of (2).
///
/// Zero where the field is horizontal — nothing to tip — and zero where
/// [`heading_sensitivity`](crate::init::heading_sensitivity) refuses a field with no
/// horizontal part at all, which observes no heading for the tilt to spoil.
fn levelling_variance(covariance: &Covariance, field: MagField<Body>, down: Vector3<f32>) -> f32 {
    let Some(sensitivity) = init::heading_sensitivity(field, down) else {
        return 0.0;
    };
    let f = sensitivity.horizontal;
    let g = down.cross(&f);
    let theta = ErrorState::AttitudeX.index();
    let p_theta = covariance.as_matrix().fixed_view::<3, 3>(theta, theta);
    let (pf, pg) = (p_theta * f, p_theta * g);
    let (a, b, d) = (f.dot(&pf), f.dot(&pg), g.dot(&pg));
    let half = 0.5 * (a - d);
    let tilt_variance = 0.5 * (a + d) + ComplexField::sqrt(half * half + b * b);
    sensitivity.tan_dip * sensitivity.tan_dip * tilt_variance
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::init::heading_from_mag;
    use crate::state::{AttitudeVariance, STATES};
    use crate::units::Attitude;
    use core::f32::consts::PI;
    use nalgebra::{UnitQuaternion, Vector3};

    /// The dip the corpus carries, 1.107 rad, quoted by `init::heading_from_mag`.
    const DIP: f32 = 1.107;

    /// The navigation-frame field for a declination and the dip above, unit magnitude.
    ///
    /// East-positive declination puts magnetic north `d` east of true north, and dip is
    /// positive downward, which is the down-positive frame's `+z`.
    fn field_ned(declination: f32) -> Vector3<f32> {
        Vector3::new(
            DIP.cos() * declination.cos(),
            DIP.cos() * declination.sin(),
            DIP.sin(),
        )
    }

    pub(crate) fn attitude_of(roll: f32, pitch: f32, yaw: f32) -> Attitude {
        Attitude::body_to_ned(UnitQuaternion::from_euler_angles(roll, pitch, yaw))
    }

    /// What a magnetometer reads on a vehicle at `attitude` in the field of `declination`.
    ///
    /// Shared with `eskf.rs`, which needs a field that means a heading rather than an
    /// arbitrary vector, the way it shares `init::tests`' windows.
    pub(crate) fn measured(attitude: Attitude, declination: f32) -> MagField<Body> {
        let body = attitude.quaternion().inverse() * field_ned(declination);
        MagField::body(body.x, body.y, body.z)
    }

    fn state_at(attitude: Attitude) -> State {
        State {
            attitude,
            ..State::default()
        }
    }

    #[test]
    fn a_field_synthesised_for_the_estimated_attitude_is_no_news() {
        let attitude = attitude_of(0.3, -0.2, 1.1);
        let innovation = heading_innovation(
            &state_at(attitude),
            measured(attitude, -0.06),
            Radians::from_radians(-0.06),
        );
        assert!(innovation.abs() < 1e-5, "innovation {innovation}");
    }

    #[test]
    fn a_yaw_error_at_non_zero_tilt_is_recovered() {
        // Zero tilt would pass with (34) omitted: `atan2` on the body field is the same
        // number there. The 0.3 rad of roll and -0.2 of pitch are what make this a test
        // of the rotation rather than of the arctangent, since at this dip an unlevelled
        // projection is wrong by about tan(1.107) = 1.96 times the tilt.
        let truth = attitude_of(0.3, -0.2, 1.1);
        let estimate = attitude_of(0.3, -0.2, 1.1 + 0.15);
        let innovation = heading_innovation(
            &state_at(estimate),
            measured(truth, -0.06),
            Radians::from_radians(-0.06),
        );
        assert!(
            (innovation + 0.15).abs() < 1e-4,
            "a yaw estimate 0.15 rad high should innovate -0.15, got {innovation}"
        );
    }

    #[test]
    fn declination_shifts_the_heading_by_exactly_d_m() {
        // The same field read against two declinations 0.2 rad apart. Nothing else moves,
        // so the difference is the conversion from magnetic heading to true and not the
        // geometry above it.
        let attitude = attitude_of(0.3, -0.2, 1.1);
        let field = measured(attitude, -0.06);
        let state = state_at(attitude);
        let at_charted = heading_innovation(&state, field, Radians::from_radians(-0.06));
        let at_shifted = heading_innovation(&state, field, Radians::from_radians(0.14));
        assert!(
            (at_shifted - at_charted - 0.2).abs() < 1e-5,
            "{at_charted} and {at_shifted} should differ by the 0.2 rad of declination"
        );
    }

    #[test]
    fn a_vehicle_heading_near_the_half_turn_innovates_the_short_way() {
        // 179° estimated, -179° measured: the case (35)'s note names, and the obvious
        // test of it. It is not a test of the wrap — it passes with the wrap deleted,
        // because `atan2` already returns its result in (-π, π] and a declination this
        // small cannot carry the difference out of that range. What it does catch is a
        // heading normalized into [0, 2π) somewhere above it, which would report 358°.
        // The test below is the one that fails without (35)'s wrap.
        let truth = attitude_of(0.0, 0.0, -179.0_f32.to_radians());
        let estimate = attitude_of(0.0, 0.0, 179.0_f32.to_radians());
        let innovation = heading_innovation(
            &state_at(estimate),
            measured(truth, 0.0),
            Radians::from_radians(0.0),
        );
        assert!(
            (innovation - 2.0_f32.to_radians()).abs() < 1e-4,
            "expected 2 degrees the short way, got {} degrees",
            innovation.to_degrees()
        );
        assert!(innovation.abs() <= PI);
    }

    #[test]
    fn a_heading_error_past_the_half_turn_innovates_the_short_way_round() {
        // The wrap of (35), and the geometry that actually exercises it: `atan2` is
        // already inside (-π, π], so only the declination can carry the difference out
        // of it. At D_m = 0.35 rad — 20°, an ordinary mid-latitude figure, where the
        // -0.06 the corpus is replayed at is too small to reach this at any heading —
        // and an estimate 2.93 rad from the truth, `atan2` reads -3.003 and the raw
        // difference is -3.353. Wrapped, the innovation is -2.93, the error itself;
        // unwrapped it is +3.353, which points the correction the long way round and is
        // larger than the error it is correcting. An unestablished heading is where a
        // 2.93 rad error is ordinary rather than absurd.
        let declination = Radians::from_radians(0.35);
        let truth = attitude_of(0.0, 0.0, 0.0);
        let estimate = attitude_of(0.0, 0.0, 2.93);
        let innovation =
            heading_innovation(&state_at(estimate), measured(truth, 0.35), declination);
        assert!(
            (innovation + 2.93).abs() < 1e-4,
            "expected -2.93, got {innovation}"
        );
        assert!(innovation.abs() <= PI, "{innovation} is outside (-π, π]");
    }

    #[test]
    fn the_innovation_is_the_levelled_heading_less_the_estimated_yaw() {
        // (35) and (6) are one computation: `R(q̂) = Rz(ψ̂) R₀`, so this innovation is
        // `wrap(ψ_mag − ψ̂)` exactly. Pinned because the two are written separately — a
        // declination sign or a wrap convention that drifts in one shows up here.
        let truth = attitude_of(0.25, 0.4, -2.9);
        let estimate = attitude_of(0.25, 0.4, -2.9 + 0.3);
        let field = measured(truth, -0.06);
        let declination = Radians::from_radians(-0.06);

        let (roll, pitch, yaw) = estimate.euler_angles();
        let levelled = heading_from_mag(
            field,
            Radians::from_radians(roll),
            Radians::from_radians(pitch),
            declination,
        );

        let innovation = heading_innovation(&state_at(estimate), field, declination);
        assert!(
            (innovation - wrap_pi(levelled.as_radians() - yaw)).abs() < 1e-5,
            "(35) gave {innovation}, (6) less the estimated yaw gave {}",
            wrap_pi(levelled.as_radians() - yaw)
        );
    }

    #[test]
    fn a_dead_magnetometer_reading_zero_reports_a_heading_rather_than_a_nan() {
        // `atan2(0, 0)` is 0 rather than undefined, so a field of exactly zero says the
        // vehicle is pointing `D_m` away from where it thinks it is. Finite, gateable,
        // and the application's to prevent; the value that matters here is that it is
        // not NaN, which no gate turns down.
        let innovation = heading_innovation(
            &state_at(Attitude::level()),
            MagField::body(0.0, 0.0, 0.0),
            Radians::from_radians(-0.06),
        );
        assert_eq!(innovation, -0.06);
    }

    #[test]
    fn the_jacobian_reads_yaw_out_of_the_attitude_block_and_nothing_else() {
        let h = heading_jacobian(&state_at(Attitude::level()));
        let dx = SVector::<f32, STATES>::from_fn(|i, _| i as f32 + 1.0);
        // Level, so the third row of `R(q̂)` is e₃ᵀ and the Jacobian picks the ninth
        // error state, `AttitudeZ`.
        assert_eq!((h * dx)[0], 9.0);
        for i in 0..STATES {
            if i != ErrorState::AttitudeZ.index() {
                assert_eq!(h[(0, i)], 0.0, "state {i} is not the yaw axis");
            }
        }
    }

    #[test]
    fn the_jacobian_leans_with_the_tilt_and_not_with_the_yaw() {
        // `e₃ᵀ R(q̂)` is the navigation down axis in body coordinates, which a yaw about
        // that same axis leaves alone and a tilt does not. A Jacobian written as a
        // constant e₃ᵀ passes at zero tilt and is wrong by the 1/cos θ of (36) elsewhere.
        let yawed = heading_jacobian(&state_at(attitude_of(0.0, 0.0, 1.1)));
        assert_eq!(yawed[(0, ErrorState::AttitudeZ.index())], 1.0);

        let pitched = heading_jacobian(&state_at(attitude_of(0.0, 0.4, 1.1)));
        assert!(
            (pitched[(0, ErrorState::AttitudeZ.index())] - 0.4_f32.cos()).abs() < 1e-6,
            "the yaw column should be cos(pitch)"
        );
        assert!(
            (pitched[(0, ErrorState::AttitudeX.index())] + 0.4_f32.sin()).abs() < 1e-6,
            "and the roll column -sin(pitch)"
        );
    }

    /// A covariance carrying one tilt σ on both horizontal attitude axes, nothing else.
    fn tilt_covariance(sigma: f32) -> Covariance {
        let mut sigmas = [0.0; STATES];
        sigmas[ErrorState::AttitudeX.index()] = sigma;
        sigmas[ErrorState::AttitudeY.index()] = sigma;
        Covariance::from_sigmas(sigmas)
    }

    #[test]
    fn a_filter_certain_of_its_tilt_adds_nothing_to_the_callers_variance() {
        // (36′) prices the levelling, and a levelling done with a known attitude costs
        // nothing. The boundary that says the term is a function of `P` rather than a
        // blanket inflation of `R`.
        let attitude = attitude_of(0.1, 0.1, 0.5);
        let observation = heading_observation(
            &state_at(attitude),
            &tilt_covariance(0.0),
            measured(attitude, -0.06),
            Radians::from_radians(-0.06),
            HeadingNoise::from_sigma(0.25),
        );
        assert_eq!(observation.r_m[0], 0.0625);
        assert!(observation.y[0].abs() < 1e-5);
    }

    #[test]
    fn the_levelling_variance_is_the_dip_squared_times_the_tilt_variance() {
        // (36′) at the dip the corpus carries: tan(1.107) = 2.0, so a tilt σ of 0.02 rad
        // is worth 0.04 of heading and four times the tilt's own variance. That factor
        // is why the term is not negligible against a magnetometer's own noise.
        let attitude = attitude_of(0.0, 0.0, 0.0);
        let observation = heading_observation(
            &state_at(attitude),
            &tilt_covariance(0.02),
            measured(attitude, 0.0),
            Radians::from_radians(0.0),
            HeadingNoise::from_sigma(0.05),
        );
        let expected = 0.05 * 0.05 + DIP.tan() * DIP.tan() * 0.02 * 0.02;
        assert!(
            (observation.r_m[0] - expected).abs() < 1e-7,
            "R = {} rather than {expected}",
            observation.r_m[0]
        );
    }

    #[test]
    fn a_horizontal_field_has_no_levelling_error_to_price() {
        // Dip zero: the field lies in the horizontal plane, so tipping it about a
        // horizontal axis turns its horizontal part by nothing to first order. (36′) is
        // free exactly where the geometry says it should be, which a constant inflation
        // of `R` would not be.
        let observation = heading_observation(
            &state_at(Attitude::level()),
            &tilt_covariance(0.3),
            MagField::body(0.22, 0.0, 0.0),
            Radians::from_radians(0.0),
            HeadingNoise::from_sigma(0.05),
        );
        // Against the noise's own variance rather than a literal: `0.05f32` squared is
        // 0.0025000002, and a tolerance wide enough to hide that is wide enough to hide
        // a small inflation too.
        let bare = HeadingNoise::from_sigma(0.05).variance();
        assert_eq!(observation.r_m[0], bare);
    }

    #[test]
    fn a_field_with_no_horizontal_part_is_not_priced_infinitely() {
        // Straight down: `heading_sensitivity` refuses it rather than dividing by zero.
        // The heading such a field reports is meaningless and the gate is what turns it
        // down; a NaN here would make `S` NaN, fail the Cholesky, and deliver it as
        // `StateInvalid`, which blames the filter for the sensor.
        let observation = heading_observation(
            &state_at(Attitude::level()),
            &tilt_covariance(0.3),
            MagField::body(0.0, 0.0, 0.49),
            Radians::from_radians(0.0),
            HeadingNoise::from_sigma(0.05),
        );
        assert!(observation.r_m[0].is_finite());
        assert_eq!(
            observation.r_m[0],
            HeadingNoise::from_sigma(0.05).variance()
        );
    }

    #[test]
    fn heading_variance_is_the_heading_rows_h_p_ht() {
        // (36)'s row and `AttitudeVariance` are two statements of what heading
        // uncertainty is; they must agree at any attitude.
        let state = state_at(attitude_of(0.4, 1.2, -2.0));
        let mut p = crate::state::CovarianceMatrix::from_fn(|i, j| 0.01 * (1 + i.min(j)) as f32);
        p.fill_diagonal(0.2);
        let covariance = Covariance::from_matrix(p);
        let h = heading_jacobian(&state);
        let hpht = (h * covariance.as_matrix() * h.transpose())[0];
        let heading = AttitudeVariance::of(&state.attitude, &covariance).heading;
        assert!((hpht - heading).abs() < 1e-6, "{hpht} against {heading}");
    }

    /// Navigation down in body axes, `R(q̂)ᵀe₃`, derived from the attitude rather than read
    /// off (36)'s row the way `heading_observation` reads it.
    fn down_of(attitude: Attitude) -> Vector3<f32> {
        attitude.quaternion().inverse() * Vector3::z()
    }

    /// A body-frame covariance whose tilt block is anisotropic and correlated, the case in
    /// which a choice of axes shows: 0.02 about one horizontal direction, 0.002 about the
    /// other, on a vehicle rolled and pitched far enough that body x/y are not horizontal.
    fn anisotropic(attitude: &Attitude) -> Covariance {
        let r = attitude.quaternion().to_rotation_matrix().into_inner();
        let azimuth = nalgebra::Rotation3::from_axis_angle(&Vector3::z_axis(), 0.6).into_inner();
        let ned = azimuth
            * nalgebra::Matrix3::from_diagonal(&Vector3::new(0.02, 0.002, 0.3))
            * azimuth.transpose();
        let mut p = crate::state::CovarianceMatrix::identity();
        let theta = ErrorState::AttitudeX.index();
        p.fixed_view_mut::<3, 3>(theta, theta)
            .copy_from(&(r.transpose() * ned * r));
        Covariance::from_matrix(p)
    }

    #[test]
    fn the_levelling_variance_prices_the_worst_horizontal_axis() {
        // The largest eigenvalue of the north/east block, found here by rotating `P` onto
        // navigation axes and decomposing it — the long way round the code does not take.
        // The field lies 1.1 rad from the axis tilt is least certain about, so pricing the
        // field axis alone, or the larger of the north and east diagonals, comes out smaller.
        let attitude = attitude_of(0.3, -0.4, 0.9);
        let state = state_at(attitude);
        let covariance = anisotropic(&attitude);
        let observation = heading_observation(
            &state,
            &covariance,
            measured(attitude, 1.7),
            Radians::from_radians(1.7),
            HeadingNoise::from_sigma(0.05),
        );
        let r = attitude.quaternion().to_rotation_matrix().into_inner();
        let theta = ErrorState::AttitudeX.index();
        let ned = r * covariance.as_matrix().fixed_view::<3, 3>(theta, theta) * r.transpose();
        let largest = ned
            .fixed_view::<2, 2>(0, 0)
            .into_owned()
            .symmetric_eigenvalues()
            .max();
        let expected = HeadingNoise::from_sigma(0.05).variance() + DIP.tan() * DIP.tan() * largest;
        assert!(
            (observation.r_m[0] - expected).abs() < 1e-5 * expected,
            "R = {} rather than {expected}",
            observation.r_m[0]
        );
        let diagonal = DIP.tan() * DIP.tan() * ned[(0, 0)].max(ned[(1, 1)]);
        assert!(observation.r_m[0] - HeadingNoise::from_sigma(0.05).variance() > 1.1 * diagonal);
    }

    #[test]
    fn the_levelling_variance_does_not_depend_on_where_the_vehicle_points() {
        // One vehicle, one covariance, one field in body axes, turned about down through
        // five yaws spanning the circle with the field turning with it: nothing the magnetometer or the
        // filter holds has changed, only the heading of both. The larger of the north and
        // east diagonals fails this, since turning an anisotropic tilt block through yaw
        // moves variance between them — the axis choice that moved `f16771dd`'s
        // `nu_mag_yaw` with nothing about the log changing.
        let levelling = |yaw: f32| {
            let attitude = attitude_of(0.3, -0.4, yaw);
            let covariance = anisotropic(&attitude_of(0.3, -0.4, 0.0));
            // The same body-frame block at every yaw: `anisotropic` rotated onto yaw zero.
            let field = measured(attitude, yaw + 1.7);
            levelling_variance(&covariance, field, down_of(attitude))
        };
        let at_zero = levelling(0.0);
        for yaw in [0.5, 1.3, 2.9, -2.2, -0.7] {
            let turned = levelling(yaw);
            assert!(
                (turned - at_zero).abs() < 1e-5 * at_zero,
                "at yaw {yaw} the price is {turned} against {at_zero}"
            );
        }
    }

    #[test]
    fn on_its_tail_the_levelling_reads_tilt_and_not_heading() {
        // Pitched 90° nose-up, body x is navigation up: the body x and y variances (36′)
        // once read are heading's and one tilt's. How uncertain heading is has nothing to
        // do with how badly the field was levelled.
        let attitude = attitude_of(0.0, core::f32::consts::FRAC_PI_2, 0.3);
        let field = measured(attitude, 0.0);
        let down = down_of(attitude);
        let levelling = |heading: f32| {
            let block = AttitudeVariance {
                tilt_north: 1e-3,
                tilt_east: 1e-3,
                heading,
            }
            .in_body(&attitude);
            let mut p = crate::state::CovarianceMatrix::identity();
            let theta = ErrorState::AttitudeX.index();
            p.fixed_view_mut::<3, 3>(theta, theta).copy_from(&block);
            levelling_variance(&Covariance::from_matrix(p), field, down)
        };
        let (tight, loose) = (levelling(1e-4), levelling(0.5));
        assert!(
            tight > 0.0,
            "a dipping field has a levelling error to price"
        );
        assert!(
            (tight - loose).abs() < 1e-6 * tight.max(1e-9) + 1e-9,
            "heading's variance leaked into the levelling: {tight} against {loose}"
        );
    }
}
