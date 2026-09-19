//! Geodetic positions and the local tangent plane the filter navigates in.
//! Equations (43)–(44).
//!
//! The filter navigates in Cartesian NED about a fixed origin, and GNSS reports latitude,
//! longitude and height. Something has to convert, and the conversion is only right if it
//! is taken about the same origin the filter's position is relative to. When the
//! application did the conversion, nothing checked that: the filter's origin was "where
//! the vehicle was at initialization", and the application's was whatever point it had
//! chosen. So the filter owns the origin — see
//! [`Eskf::fuse_gnss_geodetic`](crate::Eskf::fuse_gnss_geodetic) — and this module is
//! the conversion it owns it with.

use nalgebra::{ComplexField, Vector3};

use crate::frames::Ned;
use crate::units::Position;

/// WGS84 semi-major axis, in meters.
const WGS84_A: f64 = 6_378_137.0;

/// WGS84 first eccentricity squared.
const WGS84_E2: f64 = 6.694_379_990_141e-3;

/// A position on the WGS84 ellipsoid: latitude, longitude, and height.
///
/// `f64` throughout, where the rest of the crate is `f32`: at a latitude of 50° an `f32`
/// degree has a resolution of 4 × 10⁻⁶ °, which is 0.4 m on the ground. Positions
/// relative to the origin are small, so everything after the conversion is `f32` again.
///
/// Height is whatever the receiver reports — above the ellipsoid or above mean sea level —
/// used consistently. Only differences in it reach the filter, so the datum does not
/// matter as long as it does not change mid-flight.
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

    fn is_finite(self) -> bool {
        self.latitude.is_finite() && self.longitude.is_finite() && self.height.is_finite()
    }
}

/// The origin of the navigation frame, and the local tangent plane about it.
/// Equation (43).
///
/// North and east are arc lengths along the meridian and the parallel through the origin,
/// using the ellipsoid's radii of curvature there. That is a first-order expansion about
/// the origin, so it is exact at the origin and loses accuracy with range: the error
/// grows as the product of the north and east offsets over the Earth's radius — about
/// 0.2 m at 1 km by 1 km at mid latitudes, 17 m at 10 km by 10 km. The same expansion as
/// `tools/ulog2replay.py`, so the replay corpus and the filter agree about where a fix is.
///
/// PX4 projects with an azimuthal equidistant projection on a sphere instead
/// (`MapProjection::project` in `src/lib/geo/geo.cpp`), which keeps range from the
/// origin exact at any distance but takes a spherical Earth's scale, 0.3% off at mid
/// latitudes. Either can replace the other behind this type without the API changing.
///
/// Not usable within a few meters of a pole, where the parallel through the origin has no
/// length and east is undefined.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalOrigin {
    origin: Geodetic,
    /// `M + h₀`: meters of northing per radian of latitude.
    meridian_radius: f64,
    /// `(N + h₀) cos φ₀`: meters of easting per radian of longitude.
    parallel_radius: f64,
}

impl LocalOrigin {
    /// The tangent plane about `origin`.
    pub fn new(origin: Geodetic) -> Self {
        let sin_phi = ComplexField::sin(origin.latitude);
        let cos_phi = ComplexField::cos(origin.latitude);
        let w2 = 1.0 - WGS84_E2 * sin_phi * sin_phi;
        // Radii of curvature in the meridian (M) and in the prime vertical (N).
        let meridian = WGS84_A * (1.0 - WGS84_E2) / (w2 * ComplexField::sqrt(w2));
        let prime_vertical = WGS84_A / ComplexField::sqrt(w2);
        Self {
            origin,
            meridian_radius: meridian + origin.height,
            parallel_radius: (prime_vertical + origin.height) * cos_phi,
        }
    }

    /// Where the origin is.
    pub const fn geodetic(&self) -> Geodetic {
        self.origin
    }

    /// A geodetic position as NED meters from the origin. Equation (43).
    pub fn to_ned(&self, point: Geodetic) -> Position<Ned> {
        let north = (point.latitude - self.origin.latitude) * self.meridian_radius;
        let east = wrap_pi(point.longitude - self.origin.longitude) * self.parallel_radius;
        let down = -(point.height - self.origin.height);
        // Narrowed only after the subtraction, which is where the f64 was needed.
        Position::from_vector(Vector3::new(north as f32, east as f32, down as f32))
    }

    /// NED meters from the origin as a geodetic position. The exact inverse of
    /// [`to_ned`](Self::to_ned), so a round trip returns what went in.
    pub fn to_geodetic(&self, position: Position<Ned>) -> Geodetic {
        let p = position.vector();
        Geodetic {
            latitude: self.origin.latitude + f64::from(p.x) / self.meridian_radius,
            longitude: wrap_pi(self.origin.longitude + f64::from(p.y) / self.parallel_radius),
            height: self.origin.height - f64::from(p.z),
        }
    }

    /// The origin whose tangent plane puts `fix` at `estimate`. Equation (44).
    ///
    /// For a filter that has been navigating before it knew where it was: it has a
    /// position relative to its own start, and the first fix says where that start was.
    /// The origin is placed so the two agree, rather than at the fix, which would step the
    /// estimate by however far the vehicle had moved. PX4 does the same when its origin is
    /// first set with aiding already active (`collect_gps`, `_pos_ref.reproject(-pos)`).
    ///
    /// The radii are taken at the fix rather than at the origin being solved for; the
    /// difference is second order in the displacement.
    pub fn placing(fix: Geodetic, estimate: Position<Ned>) -> Self {
        Self::new(Self::new(fix).to_geodetic(Position::from_vector(-estimate.vector())))
    }

    pub(crate) fn is_usable(origin: Geodetic) -> bool {
        origin.is_finite() && origin.latitude.abs() < core::f64::consts::FRAC_PI_2
    }
}

/// An angle wrapped into `(-π, π]`, so a flight across the antimeridian is not a
/// 40 000 km step east.
fn wrap_pi(angle: f64) -> f64 {
    use core::f64::consts::{PI, TAU};
    let wrapped = angle - TAU * ComplexField::floor((angle + PI) / TAU);
    if wrapped == -PI { PI } else { wrapped }
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
        let origin = LocalOrigin::new(zurich());
        assert_eq!(origin.to_ned(zurich()), Position::zero());
    }

    #[test]
    fn a_round_trip_returns_what_went_in() {
        let origin = LocalOrigin::new(zurich());
        let p = Position::ned(1234.5, -876.25, -120.0);
        let back = origin.to_ned(origin.to_geodetic(p));
        assert!(
            (back.vector() - p.vector()).norm() < 1e-3,
            "{back:?} != {p:?}"
        );
    }

    #[test]
    fn scales_match_the_converter() {
        // tools/ulog2replay.py::geodetic_to_ned for 0.01° each way from zurich():
        //   (1111.8711, 754.9557, -10.0)
        let origin = LocalOrigin::new(zurich());
        let p = origin
            .to_ned(Geodetic::from_degrees(47.4077, 8.5556, 498.0))
            .vector();
        assert!((p.x - 1111.8711).abs() < 1e-3, "north {}", p.x);
        assert!((p.y - 754.9557).abs() < 1e-3, "east {}", p.y);
        assert_eq!(p.z, -10.0, "down is negative height");
    }

    #[test]
    fn up_is_negative_down() {
        let origin = LocalOrigin::new(zurich());
        let above = Geodetic::from_degrees(47.3977, 8.5456, 500.0);
        assert_eq!(origin.to_ned(above).z(), -12.0);
    }

    #[test]
    fn crossing_the_antimeridian_is_a_short_step() {
        let origin = LocalOrigin::new(Geodetic::from_degrees(-17.0, 179.9999, 0.0));
        let east = origin.to_ned(Geodetic::from_degrees(-17.0, -179.9999, 0.0));
        let e = east.y();
        assert!(e > 0.0 && e < 25.0, "0.0002° east, got {e} m");
    }

    #[test]
    fn the_integer_encoding_is_the_same_position() {
        let e7 = Geodetic::from_degrees_e7(473_977_000, 85_456_000, 488_000);
        let p = LocalOrigin::new(zurich()).to_ned(e7).vector();
        assert!(p.norm() < 1e-3, "{p:?}");
    }

    #[test]
    fn placing_puts_the_fix_at_the_estimate() {
        let estimate = Position::ned(35.0, -12.0, -4.0);
        let origin = LocalOrigin::placing(zurich(), estimate);
        let p = origin.to_ned(zurich());
        assert!(
            (p.vector() - estimate.vector()).norm() < 1e-2,
            "{p:?} != {estimate:?}"
        );
    }

    #[test]
    fn poles_and_nonsense_are_not_origins() {
        assert!(LocalOrigin::is_usable(zurich()));
        assert!(!LocalOrigin::is_usable(Geodetic::from_degrees(
            90.0, 0.0, 0.0
        )));
        assert!(!LocalOrigin::is_usable(Geodetic::from_degrees(
            f64::NAN,
            0.0,
            0.0
        )));
    }
}
