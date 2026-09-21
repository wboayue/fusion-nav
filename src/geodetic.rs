//! Geodetic positions and the local tangent plane the filter navigates in.
//! Equations (43)–(44).
//!
//! The filter navigates in Cartesian NED about a fixed origin, and GNSS reports latitude,
//! longitude and height. Something has to convert, and the conversion is only right if it
//! is taken about the same origin the filter's position is relative to. An application
//! converting on its own side has nothing to check that against: the filter's origin is
//! where the vehicle was at initialization, and the application's is whatever point it
//! chose. So the filter owns the origin — see
//! [`Eskf::fuse_gnss_geodetic`](crate::Eskf::fuse_gnss_geodetic) — and this module is
//! the conversion it owns it with.

use nalgebra::{ComplexField, Matrix3, RealField, Vector3};

use crate::frames::Ned;
use crate::units::Position;

/// WGS84 semi-major axis, in meters: a defining parameter of the ellipsoid,
/// NGA.STND.0036 v1.0.0 Table 3.1.
const WGS84_A: f64 = 6_378_137.0;

/// WGS84 first eccentricity squared, NGA.STND.0036 v1.0.0 Table 3.5. Written to the digits
/// the standard prints rather than derived from the flattening: it asks that derived
/// constants keep them, so that precision stays consistent between parameters.
const WGS84_E2: f64 = 6.694_379_990_141e-3;

/// A position on the WGS84 ellipsoid: latitude, longitude, and height.
///
/// `f64` throughout, where the rest of the crate is `f32`: at a latitude of 50° an `f32`
/// degree has a resolution of 4 × 10⁻⁶ °, which is 0.4 m on the ground. Positions
/// relative to the origin are small, so everything after the conversion is `f32` again.
///
/// Height is above the WGS84 ellipsoid for the conversion to be exact, which is what
/// receivers compute (u-blox `height`, PX4 `altitude_ellipsoid_m`). Height above mean
/// sea level works if used consistently, but it misplaces the ellipsoid by the local geoid
/// undulation — up to 100 m — and so scales horizontal distance by up to 1.6 × 10⁻⁵:
/// 1.6 cm per kilometre from the origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Geodetic {
    latitude: f64,
    longitude: f64,
    height: f64,
}

impl Geodetic {
    /// Construct from latitude and longitude in degrees, and height in meters.
    pub fn from_degrees(latitude: f64, longitude: f64, height: f64) -> Self {
        Self::from_radians(latitude.to_radians(), longitude.to_radians(), height)
    }

    /// Construct from latitude and longitude in radians, and height in meters.
    pub const fn from_radians(latitude: f64, longitude: f64, height: f64) -> Self {
        Self {
            latitude,
            longitude,
            height,
        }
    }

    /// Construct from the integer encoding u-blox `NAV-PVT`, MAVLink `GPS_RAW_INT`, and
    /// PX4's older `vehicle_gps_position` all share: latitude and longitude in 10⁻⁷
    /// degrees, height in millimeters.
    pub fn from_degrees_e7(latitude: i32, longitude: i32, height_mm: i32) -> Self {
        Self::from_degrees(
            f64::from(latitude) * 1e-7,
            f64::from(longitude) * 1e-7,
            f64::from(height_mm) * 1e-3,
        )
    }

    /// Latitude in degrees.
    pub fn latitude_deg(self) -> f64 {
        self.latitude.to_degrees()
    }

    /// Longitude in degrees.
    pub fn longitude_deg(self) -> f64 {
        self.longitude.to_degrees()
    }

    /// Latitude in radians.
    pub const fn latitude_rad(self) -> f64 {
        self.latitude
    }

    /// Longitude in radians.
    pub const fn longitude_rad(self) -> f64 {
        self.longitude
    }

    /// Height in meters, on the receiver's datum.
    pub const fn height(self) -> f64 {
        self.height
    }

    /// Whether every coordinate is a number, neither NaN nor infinite.
    pub(crate) fn is_finite(self) -> bool {
        self.latitude.is_finite() && self.longitude.is_finite() && self.height.is_finite()
    }
}

/// The origin of the navigation frame, and the local tangent plane about it.
/// Equation (43).
///
/// Exact, not approximated: a geodetic position goes to Earth-centered Cartesian
/// coordinates, the origin's are subtracted, and the difference is rotated onto the
/// origin's north, east and down. Every step is closed-form geometry on the WGS84
/// ellipsoid, so there is no range at which the conversion starts to lose accuracy; the
/// only rounding is the final narrowing to `f32`, 1 mm at 10 km. The inverse is exact to
/// micrometres too — see [`to_geodetic`](Self::to_geodetic).
///
/// What *does* change with range is not the conversion but the frame: a plane leaves a
/// curved Earth. At a horizontal distance `d` the plane sits `d²/2R` above the surface
/// under it — 8 cm at 1 km, 7.8 m at 10 km — so `-p_D` there is not height above the
/// origin, and down there is tilted `d/R` from local vertical. See
/// [`EQUATIONS.md`](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#geodetic-origin)
/// for what that means for the barometer.
///
/// PX4 projects with an azimuthal equidistant projection on a sphere
/// (`MapProjection::project` in `src/lib/geo/geo.cpp`): a map rather than a plane, with a
/// spherical Earth's scale, 0.3% off at mid latitudes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalOrigin {
    origin: Geodetic,
    /// The origin in Earth-centered, Earth-fixed coordinates, in meters.
    ecef: Vector3<f64>,
    /// `Cₑⁿ`: rotates an ECEF difference onto the origin's north, east, down.
    ecef_to_ned: Matrix3<f64>,
}

impl LocalOrigin {
    /// The tangent plane about `origin`, or `None` for a point that cannot be one: a
    /// coordinate that is not a number, or a latitude beyond ±90°.
    pub fn new(origin: Geodetic) -> Option<Self> {
        Self::is_usable(origin).then(|| Self::about(origin))
    }

    /// The tangent plane about a point already known to be usable.
    fn about(origin: Geodetic) -> Self {
        let (sin_phi, cos_phi) = ComplexField::sin_cos(origin.latitude);
        let (sin_lambda, cos_lambda) = ComplexField::sin_cos(origin.longitude);
        // Rows are the origin's north, east and down, written in ECEF.
        #[rustfmt::skip]
        let ecef_to_ned = Matrix3::new(
            -sin_phi * cos_lambda, -sin_phi * sin_lambda,  cos_phi,
            -sin_lambda,            cos_lambda,            0.0,
            -cos_phi * cos_lambda, -cos_phi * sin_lambda, -sin_phi,
        );
        Self {
            origin,
            ecef: ecef(origin),
            ecef_to_ned,
        }
    }

    /// Where the origin is.
    pub const fn geodetic(&self) -> Geodetic {
        self.origin
    }

    /// A geodetic position as NED meters from the origin. Equation (43).
    pub fn to_ned(&self, point: Geodetic) -> Position<Ned> {
        let ned = self.ecef_to_ned * (ecef(point) - self.ecef);
        // Narrowed only after the subtraction, which is where the f64 was needed.
        Position::from_vector(ned.map(|meters| meters as f32))
    }

    /// NED meters from the origin as a geodetic position: equation (43) run backwards.
    ///
    /// The rotation inverts exactly (it is orthonormal, so its inverse is its transpose);
    /// ECEF to geodetic is the one step with no closed form this simple, and is solved by
    /// fixed-point iteration; see `geodetic` in this module.
    pub fn to_geodetic(&self, position: Position<Ned>) -> Geodetic {
        let ned = position.vector().map(f64::from);
        geodetic(self.ecef + self.ecef_to_ned.transpose() * ned)
    }

    /// The origin whose tangent plane puts `fix` at `estimate`. Equation (44).
    ///
    /// For a filter that has been navigating before it knew where it was: it has a
    /// position relative to its own start, and the first fix says where that start was.
    /// The origin is placed so the two agree, rather than at the fix, which would step the
    /// estimate by however far the vehicle had moved. PX4 does the same when its origin is
    /// first set with aiding already active (`collect_gps`, `_pos_ref.reproject(-pos)`).
    ///
    /// Solved by iteration, because the answer depends on itself: stepping `-estimate`
    /// from the fix needs the origin's axes, and those depend on where the origin is. Each
    /// pass takes the axes from the previous guess, starting at the fix. Away from the
    /// poles each pass shrinks the error by about `|estimate| / R`: measured at Zurich,
    /// five passes leave 5 nm at 10 km, 20 µm at 100 km, and 13 mm at 300 km.
    ///
    /// Near a pole the axes turn with longitude, and each pass shrinks the error only by
    /// `|estimate|` over the distance to the pole — which grows it once the estimate is the
    /// longer of the two. There the origin may not exist at all: an origin `r` from the
    /// pole puts a point `e` due east of it `√(r² + e²)` from the pole, so a fix 11 m from
    /// the pole is 100 m east of no origin. So the result is checked, not assumed: the fix
    /// must land on the estimate to within what the `f32` state resolves there — 1 mm, or
    /// one part in 2²³ of `|estimate|` beyond 8 km — so a placement that passes steps
    /// nothing the state could represent.
    ///
    /// `None` if the fix cannot be an origin — see [`new`](Self::new) — or no origin was
    /// found that puts it at the estimate.
    pub fn placing(fix: Geodetic, estimate: Position<Ned>) -> Option<Self> {
        const PASSES: usize = 5;
        if !Self::is_usable(fix) {
            return None;
        }
        let offset = estimate.vector().map(f64::from);
        let fix = ecef(fix);
        let mut origin = Self::about(geodetic(fix));
        for _ in 0..PASSES {
            origin = Self::about(geodetic(fix - origin.ecef_to_ned.transpose() * offset));
        }
        // Measured in f64, before the narrowing `to_ned` would add.
        let landed = origin.ecef_to_ned * (fix - origin.ecef);
        let tolerance = f64::max(1e-3, f64::from(f32::EPSILON) * offset.norm());
        ((landed - offset).norm() <= tolerance).then_some(origin)
    }

    /// Whether `origin` can be one: finite, and a latitude that exists. The poles are
    /// fine — north there is whichever meridian the longitude names.
    fn is_usable(origin: Geodetic) -> bool {
        origin.is_finite() && origin.latitude.abs() <= core::f64::consts::FRAC_PI_2
    }
}

/// Earth-centered, Earth-fixed coordinates of a geodetic position, in meters.
///
/// `N` is the prime-vertical radius of curvature at that latitude; see Groves,
/// *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, (2.112).
fn ecef(point: Geodetic) -> Vector3<f64> {
    let (sin_phi, cos_phi) = ComplexField::sin_cos(point.latitude);
    let (sin_lambda, cos_lambda) = ComplexField::sin_cos(point.longitude);
    let n = prime_vertical_radius(sin_phi);
    let h = point.height;
    Vector3::new(
        (n + h) * cos_phi * cos_lambda,
        (n + h) * cos_phi * sin_lambda,
        (n * (1.0 - WGS84_E2) + h) * sin_phi,
    )
}

/// Geodetic position of ECEF coordinates: the inverse of [`ecef`].
///
/// Longitude is direct. Latitude is the fixed point of
/// `φ = atan2(z + e² N(φ) sin φ, p)`, where `p` is the distance from the polar axis;
/// each pass shrinks the error by about `e²`, 1/150, so five passes from the geocentric
/// latitude take the worst case to below a micrometre on the ground. A fixed count rather
/// than a tolerance, so the cost is the same every call.
///
/// Height uses the form that stays finite at the poles, rather than `p / cos φ − N`.
fn geodetic(ecef: Vector3<f64>) -> Geodetic {
    const PASSES: usize = 5;
    let p = ComplexField::hypot(ecef.x, ecef.y);
    let longitude = RealField::atan2(ecef.y, ecef.x);
    let mut latitude = RealField::atan2(ecef.z, p);
    for _ in 0..PASSES {
        let sin_phi = ComplexField::sin(latitude);
        let n = prime_vertical_radius(sin_phi);
        latitude = RealField::atan2(ecef.z + WGS84_E2 * n * sin_phi, p);
    }
    let (sin_phi, cos_phi) = ComplexField::sin_cos(latitude);
    let height = p * cos_phi + ecef.z * sin_phi
        - WGS84_A * ComplexField::sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi);
    Geodetic {
        latitude,
        longitude,
        height,
    }
}

/// `N`, the radius of curvature in the prime vertical, at a latitude given by its sine.
fn prime_vertical_radius(sin_phi: f64) -> f64 {
    WGS84_A / ComplexField::sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Zurich, a mid latitude where east and north scales differ visibly.
    fn zurich() -> Geodetic {
        Geodetic::from_degrees(47.3977, 8.5456, 488.0)
    }

    #[test]
    fn the_origin_is_zero() {
        let origin = LocalOrigin::new(zurich()).expect("usable");
        assert!(origin.to_ned(zurich()).vector().norm() < 1e-6);
    }

    #[test]
    fn ecef_matches_the_ellipsoid_where_it_is_known_by_heart() {
        // On the equator at the prime meridian: the semi-major axis along x.
        let equator = ecef(Geodetic::from_degrees(0.0, 0.0, 0.0));
        assert!((equator - Vector3::new(WGS84_A, 0.0, 0.0)).norm() < 1e-6);
        // At the north pole: the semi-minor axis, b = a √(1 − e²), along z.
        let b = 6_356_752.314_245;
        let pole = ecef(Geodetic::from_degrees(90.0, 0.0, 0.0));
        assert!((pole - Vector3::new(0.0, 0.0, b)).norm() < 1e-3, "{pole:?}");
    }

    #[test]
    fn ecef_and_back_is_lossless_everywhere() {
        for (lat, lon, h) in [
            (47.3977, 8.5456, 488.0),
            (-33.9, 151.2, -30.0),
            (89.9999, 45.0, 1000.0),
            (90.0, 0.0, 0.0),
            (-90.0, 0.0, 0.0),
            (0.0, -179.9999, 12_000.0),
            (64.1, -21.9, 400_000.0),
        ] {
            let point = Geodetic::from_degrees(lat, lon, h);
            let back = geodetic(ecef(point));
            let error = (ecef(back) - ecef(point)).norm();
            assert!(error < 1e-6, "{lat}, {lon}, {h}: {error} m");
        }
    }

    #[test]
    fn a_round_trip_returns_what_went_in_at_any_range() {
        let origin = LocalOrigin::new(zurich()).expect("usable");
        // Up to 100 km, where the f32 position itself resolves 8 mm.
        for p in [
            Position::ned(1234.5, -876.25, -120.0),
            Position::ned(10_000.0, 10_000.0, 0.0),
            Position::ned(-100_000.0, 60_000.0, -3000.0),
        ] {
            let back = origin.to_ned(origin.to_geodetic(p));
            let error = (back.vector() - p.vector()).norm();
            assert!(error < 1e-2, "{p:?}: {error} m");
        }
    }

    #[test]
    fn range_is_the_straight_line_distance() {
        // A plane measures chords, so the NED distance from the origin is exactly the
        // distance through ECEF — nothing a projection could say about any map.
        let origin = LocalOrigin::new(zurich()).expect("usable");
        let far = Geodetic::from_degrees(47.9, 9.3, 1200.0);
        let chord = (ecef(far) - ecef(zurich())).norm();
        let ned = f64::from(origin.to_ned(far).vector().norm());
        assert!((ned - chord).abs() < 1e-2, "{ned} vs {chord}");
    }

    #[test]
    fn it_matches_the_converter() {
        // tools/ulog2replay.py::geodetic_to_ned about zurich(), to 1 km and to 10 km.
        let origin = LocalOrigin::new(zurich()).expect("usable");
        for (point, expected) in [
            (
                Geodetic::from_degrees(47.4077, 8.5556, 498.0),
                [1111.9223, 754.814, -9.8584],
            ),
            (
                Geodetic::from_degrees(47.4877, 8.6756, 488.0),
                [10015.096, 9797.691, 15.3833],
            ),
        ] {
            let p = origin.to_ned(point).to_array();
            for (got, want) in p.into_iter().zip(expected) {
                assert!((got - want).abs() < 2e-3, "{p:?} vs {expected:?}");
            }
        }
    }

    #[test]
    fn at_range_the_plane_rises_off_the_surface() {
        // 10 km out at the origin's height, the plane is d²/2R above the ground.
        let origin = LocalOrigin::new(zurich()).expect("usable");
        let down = origin
            .to_ned(Geodetic::from_degrees(47.4877, 8.5456, 488.0))
            .z();
        assert!((7.0..9.0).contains(&down), "{down} m below the plane");
    }

    #[test]
    fn up_is_negative_down() {
        let origin = LocalOrigin::new(zurich()).expect("usable");
        let above = Geodetic::from_degrees(47.3977, 8.5456, 500.0);
        assert!((origin.to_ned(above).z() + 12.0).abs() < 1e-5);
    }

    #[test]
    fn crossing_the_antimeridian_is_a_short_step() {
        let origin =
            LocalOrigin::new(Geodetic::from_degrees(-17.0, 179.9999, 0.0)).expect("usable");
        let east = origin.to_ned(Geodetic::from_degrees(-17.0, -179.9999, 0.0));
        let e = east.y();
        assert!(e > 0.0 && e < 25.0, "0.0002° east, got {e} m");
    }

    #[test]
    fn the_integer_encoding_is_the_same_position() {
        let e7 = Geodetic::from_degrees_e7(473_977_000, 85_456_000, 488_000);
        let p = LocalOrigin::new(zurich())
            .expect("usable")
            .to_ned(e7)
            .vector();
        assert!(p.norm() < 1e-3, "{p:?}");
    }

    #[test]
    fn placing_puts_the_fix_at_the_estimate_even_far_out() {
        for estimate in [
            Position::ned(35.0, -12.0, -4.0),
            Position::ned(8_000.0, -6_000.0, -300.0),
            Position::ned(210_000.0, 210_000.0, -100.0),
        ] {
            let origin = LocalOrigin::placing(zurich(), estimate).expect("usable");
            let p = origin.to_ned(zurich());
            assert!(
                (p.vector() - estimate.vector()).norm() < 0.05,
                "{p:?} != {estimate:?}"
            );
        }
    }

    #[test]
    fn a_pole_is_an_origin_and_nonsense_is_not() {
        let pole = Geodetic::from_degrees(90.0, 0.0, 0.0);
        // 1 km down the prime meridian from the pole is 1 km south.
        let origin = LocalOrigin::new(pole).expect("a pole is an origin");
        let p = origin.to_ned(origin.to_geodetic(Position::ned(-1000.0, 0.0, 0.0)));
        assert!((p.x() + 1000.0).abs() < 1e-3 && p.y().abs() < 1e-3, "{p:?}");

        assert_eq!(
            LocalOrigin::new(Geodetic::from_degrees(f64::NAN, 0.0, 0.0)),
            None
        );
        assert_eq!(
            LocalOrigin::new(Geodetic::from_degrees(91.0, 0.0, 0.0)),
            None
        );
    }

    #[test]
    fn placing_refuses_an_origin_that_does_not_exist() {
        // 11 m from the pole, nothing is 100 m due east of any origin. The iteration used
        // to return its last guess anyway, putting the fix at (99, -11, 0).
        let near_pole = Geodetic::from_degrees(89.9999, 90.0, 0.0);
        assert_eq!(
            LocalOrigin::placing(near_pole, Position::ned(0.0, 100.0, 0.0)),
            None
        );
        let near_pole = Geodetic::from_degrees(89.999, 90.0, 0.0);
        assert_eq!(
            LocalOrigin::placing(near_pole, Position::ned(0.0, 300.0, 0.0)),
            None
        );
        // Where one does exist, near the pole is no obstacle.
        let estimate = Position::ned(-100.0, 0.0, 0.0);
        let origin = LocalOrigin::placing(near_pole, estimate).expect("100 m south exists");
        let p = origin.to_ned(near_pole);
        assert!((p.vector() - estimate.vector()).norm() < 1e-3, "{p:?}");
    }
}
