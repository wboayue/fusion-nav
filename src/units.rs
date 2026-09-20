//! Typed quantities the filter's API is expressed in.
//!
//! Types here make the caller write down the claims that cause navigation bugs, and leave
//! out the ones that do not:
//!
//! * **Frame.** Vector quantities carry their [`Frame`], and their constructors name it —
//!   `Position::ned(..)`, `AngularRate::body(..)` — so a sign error from ENU or FLU input
//!   is a conversion the caller asked for, not one they forgot. `Position::enu(..).to_ned()`
//!   and `AngularRate::flu(..)` do the conversion here, once.
//! * **Value versus noise.** A measurement and its uncertainty are different types, and a
//!   noise type is built `from_sigma` or `from_variance`, so a standard deviation cannot
//!   arrive where a variance was meant.
//! * **Sign conventions** that differ from NED, like [`Altitude`], which is positive up.
//!
//! Units are SI throughout and are named in a constructor only where a source commonly
//! supplies something else — degrees, a receiver's σ. A component of a position is an
//! `f32` in meters, not a `Meters`.
//!
//! The numeric payloads are `nalgebra` types — [`Vector3<f32>`] and
//! [`UnitQuaternion<f32>`] — so an application already using `nalgebra` can unwrap a
//! quantity and keep working, and the filter's internals get `nalgebra`'s fixed-size
//! matrix algebra without a second vector type to convert through.

use core::fmt;
use core::marker::PhantomData;

use nalgebra::{UnitQuaternion, Vector3};

use crate::frames::{Body, Enu, Frame, Ned};

/// Vehicle attitude: the rotation from [`Body`](crate::Body) to [`Ned`](crate::Ned).
///
/// Equation (7). Hamilton convention, scalar first, and normalization is maintained by
/// [`UnitQuaternion`] rather than by the filter remembering to renormalize.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attitude(UnitQuaternion<f32>);

impl Attitude {
    /// Level and pointing north.
    pub fn level() -> Self {
        Self(UnitQuaternion::identity())
    }

    /// Wrap a quaternion rotating body to NED.
    pub const fn from_quaternion(q: UnitQuaternion<f32>) -> Self {
        Self(q)
    }

    /// The underlying quaternion, body to NED.
    pub const fn quaternion(self) -> UnitQuaternion<f32> {
        self.0
    }

    /// Roll, pitch, yaw in radians, from the ZYX sequence
    /// `R = Rz(yaw) Ry(pitch) Rx(roll)`.
    pub fn euler_angles(self) -> (f32, f32, f32) {
        self.0.euler_angles()
    }
}

impl Default for Attitude {
    fn default() -> Self {
        Self::level()
    }
}

impl From<UnitQuaternion<f32>> for Attitude {
    fn from(q: UnitQuaternion<f32>) -> Self {
        Self(q)
    }
}

macro_rules! scalar {
    (
        $(#[$meta:meta])*
        $name:ident, unit = $unit:literal, new = $new:ident, get = $get:ident
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
        pub struct $name(f32);

        impl $name {
            /// Zero.
            pub const ZERO: Self = Self(0.0);

            #[doc = concat!("Construct from a value in ", $unit, ".")]
            pub const fn $new(value: f32) -> Self {
                Self(value)
            }

            #[doc = concat!("The value in ", $unit, ".")]
            pub const fn $get(self) -> f32 {
                self.0
            }
        }
    };
}

scalar!(
    /// A time interval. The filter never reads a clock; `dt` is always supplied.
    Seconds,
    unit = "seconds",
    new = from_secs,
    get = as_secs
);

impl Seconds {
    /// Whether this is a real, forward step in time: positive and not NaN. `is_nan` is
    /// spelled out because `<= 0.0` alone is false for NaN.
    pub(crate) fn is_usable_step(self) -> bool {
        self.0 > 0.0 && !self.0.is_nan()
    }
}

scalar!(
    /// An angle.
    Radians,
    unit = "radians",
    new = from_radians,
    get = as_radians
);

impl Radians {
    /// Construct from a value in degrees, the unit charts and datasheets use — magnetic
    /// declination, most obviously.
    pub const fn from_degrees(degrees: f32) -> Self {
        Self(degrees.to_radians())
    }
}

scalar!(
    /// A length, such as a position standard deviation.
    Meters,
    unit = "meters",
    new = from_meters,
    get = as_meters
);

scalar!(
    /// A speed, such as a velocity standard deviation.
    MetersPerSecond,
    unit = "meters per second",
    new = from_m_per_s,
    get = as_m_per_s
);

scalar!(
    /// A scalar acceleration, such as a specific-force tolerance or an accelerometer bias
    /// standard deviation.
    MetersPerSecond2,
    unit = "meters per second squared",
    new = from_m_per_s2,
    get = as_m_per_s2
);

scalar!(
    /// A scalar angular rate, such as a gyroscope tolerance or a gyroscope bias standard
    /// deviation.
    RadiansPerSecond,
    unit = "radians per second",
    new = from_rad_per_s,
    get = as_rad_per_s
);

scalar!(
    /// Barometric altitude above the barometer's own reference, positive **up**.
    ///
    /// Not NED down-position: the filter absorbs the sign and the unknown reference
    /// offset. Equation (30).
    Altitude,
    unit = "meters",
    new = from_meters,
    get = as_meters
);

macro_rules! noise {
    (
        $(#[$meta:meta])*
        $name:ident, sigma = $sigma_unit:literal, variance = $variance_unit:literal
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub struct $name {
            variance: f32,
        }

        impl $name {
            #[doc = concat!("From a standard deviation in ", $sigma_unit, ".")]
            pub const fn from_sigma(sigma: f32) -> Self {
                Self::from_variance(sigma * sigma)
            }

            #[doc = concat!("From a variance in ", $variance_unit, ".")]
            pub const fn from_variance(variance: f32) -> Self {
                Self { variance }
            }

            #[doc = concat!("The variance, in ", $variance_unit, ".")]
            pub const fn variance(self) -> f32 {
                self.variance
            }

            /// Whether the variance is a number, neither NaN nor infinite.
            pub(crate) fn is_finite(self) -> bool {
                self.variance.is_finite()
            }

            /// Whether the variance is one a real sensor could have: strictly positive.
            /// See [`Fusion::InvalidNoise`](crate::Fusion::InvalidNoise).
            pub(crate) fn is_positive(self) -> bool {
                self.variance > 0.0
            }
        }
    };
}

noise!(
    /// Noise on a barometric altitude, `R` of equation (30).
    AltitudeNoise,
    sigma = "meters",
    variance = "meters squared"
);

noise!(
    /// Noise on a magnetic heading, `R` of equation (36). On the heading, not on the
    /// field components.
    HeadingNoise,
    sigma = "radians",
    variance = "radians squared"
);

/// `Clone`, `Copy`, `PartialEq` and `Debug` for a type holding a `Vector3<f32>` field and a
/// frame marker.
///
/// Written out rather than derived: `derive` would add an `F: Clone` bound, and the frame
/// markers are only ever type-level.
macro_rules! vector_impls {
    ($name:ident, $field:ident, $unit:literal) => {
        impl<F: Frame> Clone for $name<F> {
            fn clone(&self) -> Self {
                *self
            }
        }

        impl<F: Frame> Copy for $name<F> {}

        impl<F: Frame> PartialEq for $name<F> {
            fn eq(&self, other: &Self) -> bool {
                self.$field == other.$field
            }
        }

        impl<F: Frame> fmt::Debug for $name<F> {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                let v = self.$field;
                write!(
                    f,
                    "{}<{}>({}, {}, {}) {}",
                    stringify!($name),
                    F::NAME,
                    v.x,
                    v.y,
                    v.z,
                    $unit
                )
            }
        }
    };
}

macro_rules! framed {
    (
        $(#[$meta:meta])*
        $name:ident, unit = $unit:literal
    ) => {
        $(#[$meta])*
        pub struct $name<F: Frame> {
            value: Vector3<f32>,
            frame: PhantomData<F>,
        }

        impl<F: Frame> $name<F> {
            #[doc = concat!("From an `nalgebra` vector whose components are in ", $unit, ".")]
            ///
            /// The frame is the type parameter, so name it: `Position::<Ned>::from_vector` or
            /// a binding with a stated type. The frame-named constructors say it for you.
            pub const fn from_vector(value: Vector3<f32>) -> Self {
                Self {
                    value,
                    frame: PhantomData,
                }
            }

            /// All components zero.
            pub fn zero() -> Self {
                Self::from_vector(Vector3::zeros())
            }

            #[doc = concat!("The components as an `nalgebra` vector, in ", $unit, ".")]
            pub const fn vector(self) -> Vector3<f32> {
                self.value
            }

            #[doc = concat!("The components as an array, in ", $unit, ".")]
            ///
            /// For an application on a different `nalgebra` version, or none, so nothing
            /// ties it to this crate's.
            pub fn to_array(self) -> [f32; 3] {
                self.value.into()
            }

            /// First component on the axes of `F`: north, east for ENU, or body forward.
            pub fn x(self) -> f32 {
                self.value.x
            }

            /// Second component on the axes of `F`: east, north for ENU, or body right.
            pub fn y(self) -> f32 {
                self.value.y
            }

            /// Third component on the axes of `F`: down, up for ENU, or body down.
            pub fn z(self) -> f32 {
                self.value.z
            }

            /// Whether every component is a number, neither NaN nor infinite.
            pub(crate) fn is_finite(self) -> bool {
                self.value.iter().all(|v| v.is_finite())
            }
        }

        impl<F: Frame> Default for $name<F> {
            fn default() -> Self {
                Self::zero()
            }
        }

        vector_impls!($name, value, $unit);
    };
}

framed!(
    /// Position relative to the navigation origin, in meters.
    ///
    /// See [`Eskf::origin`](crate::Eskf::origin) for where that is.
    Position,
    unit = "meters"
);

framed!(
    /// Velocity, in meters per second.
    Velocity,
    unit = "meters per second"
);

framed!(
    /// Specific force, an accelerometer bias, or a navigation-frame acceleration, in
    /// meters per second squared.
    ///
    /// The frame says which. `Acceleration<Body>` is what an accelerometer reads —
    /// specific force, which includes the reaction to gravity and is why a level,
    /// stationary sensor reads `-γ` on its down axis. `Acceleration<Ned>` is how the
    /// vehicle is actually accelerating, gravity excluded. Equation (11) relates them.
    Acceleration,
    unit = "meters per second squared"
);

framed!(
    /// Angular rate, or a gyroscope bias, in radians per second.
    AngularRate,
    unit = "radians per second"
);

framed!(
    /// A magnetic field measurement.
    ///
    /// Scale is irrelevant — only the direction is fused — but the components must be
    /// hard- and soft-iron calibrated. See
    /// [magnetometer, heading only](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#magnetometer-heading-only).
    MagField,
    unit = "arbitrary units"
);

/// Constructors that name the navigation frame, and the ENU conversion into it.
///
/// ENU to NED swaps the horizontal axes and flips the vertical: `(n, e, d) = (y, x, -z)`.
/// An exact signed permutation, so nothing is lost.
macro_rules! navigation {
    ($name:ident) => {
        impl $name<Ned> {
            /// From north, east, down components.
            pub fn ned(north: f32, east: f32, down: f32) -> Self {
                Self::from_vector(Vector3::new(north, east, down))
            }
        }

        impl $name<Enu> {
            /// From east, north, up components, as ROS and most GIS tools use. Convert
            /// with [`to_ned`](Self::to_ned) before handing it to the filter.
            pub fn enu(east: f32, north: f32, up: f32) -> Self {
                Self::from_vector(Vector3::new(east, north, up))
            }

            /// The same vector in north, east, down.
            pub fn to_ned(self) -> $name<Ned> {
                let enu = self.value;
                $name::ned(enu.y, enu.x, -enu.z)
            }
        }
    };
}

navigation!(Position);
navigation!(Velocity);
navigation!(Acceleration);

/// Constructors that name the body frame, and the FLU conversion into it.
///
/// The body frame is forward-right-down. ROS REP 103 bodies, and many IMU breakouts, are
/// forward-left-up: the same forward axis, with the other two negated. An exact signed
/// permutation, so nothing is lost.
macro_rules! body {
    ($name:ident, $what:literal) => {
        impl $name<Body> {
            #[doc = concat!($what, " from forward, right, down components.")]
            pub fn body(forward: f32, right: f32, down: f32) -> Self {
                Self::from_vector(Vector3::new(forward, right, down))
            }

            #[doc = concat!($what, " from forward, left, up components, converted to")]
            /// forward, right, down.
            pub fn flu(forward: f32, left: f32, up: f32) -> Self {
                Self::body(forward, -left, -up)
            }
        }
    };
}

body!(Acceleration, "Specific force");
body!(AngularRate, "Angular rate");
body!(MagField, "Field");

impl AngularRate<Body> {
    /// Angular rate from forward, right, down components in degrees per second, which is
    /// what most MEMS gyroscope drivers report.
    pub fn body_deg_per_s(forward: f32, right: f32, down: f32) -> Self {
        Self::body(forward.to_radians(), right.to_radians(), down.to_radians())
    }
}

macro_rules! noise3 {
    (
        $(#[$meta:meta])*
        $name:ident, sigma = $sigma_unit:literal, variance = $variance_unit:literal
    ) => {
        $(#[$meta])*
        pub struct $name<F: Frame> {
            variance: Vector3<f32>,
            frame: PhantomData<F>,
        }

        impl<F: Frame> $name<F> {
            #[doc = concat!("From per-axis standard deviations in ", $sigma_unit, ".")]
            pub fn from_sigma(x: f32, y: f32, z: f32) -> Self {
                Self::from_variance(x * x, y * y, z * z)
            }

            #[doc = concat!("From per-axis variances in ", $variance_unit, " — the diagonal")]
            /// of a covariance, as ROS messages carry it.
            pub fn from_variance(x: f32, y: f32, z: f32) -> Self {
                Self {
                    variance: Vector3::new(x, y, z),
                    frame: PhantomData,
                }
            }

            #[doc = concat!("The per-axis variances, in ", $variance_unit, ".")]
            pub const fn variance(self) -> Vector3<f32> {
                self.variance
            }

            /// Whether every variance is a number, neither NaN nor infinite.
            pub(crate) fn is_finite(self) -> bool {
                self.variance.iter().all(|v| v.is_finite())
            }

            /// Whether every variance is one a real sensor could have: strictly positive.
            /// See [`Fusion::InvalidNoise`](crate::Fusion::InvalidNoise).
            pub(crate) fn is_positive(self) -> bool {
                self.variance.iter().all(|v| *v > 0.0)
            }
        }

        vector_impls!($name, variance, $variance_unit);
    };
}

noise3!(
    /// Noise on a position measurement, `R` of equation (28), on the axes of `F`.
    ///
    /// A GNSS receiver's reported accuracy needs a floor before it becomes this; see
    /// [`Eskf::fuse_gnss_position`](crate::Eskf::fuse_gnss_position).
    PositionNoise,
    sigma = "meters",
    variance = "meters squared"
);

noise3!(
    /// Noise on a velocity measurement, `R` of equation (29), on the axes of `F`.
    ///
    /// Floored the same way as [`PositionNoise`]; see
    /// [`Eskf::fuse_gnss_velocity`](crate::Eskf::fuse_gnss_velocity).
    VelocityNoise,
    sigma = "meters per second",
    variance = "meters squared per second squared"
);

impl PositionNoise<Ned> {
    /// From a receiver's horizontal and vertical accuracy, `eph` and `epv` — standard
    /// deviations in meters, as u-blox (`hAcc`, `vAcc`), MAVLink and PX4 report them.
    pub fn horizontal_vertical(horizontal: f32, vertical: f32) -> Self {
        Self::from_sigma(horizontal, horizontal, vertical)
    }
}

impl VelocityNoise<Ned> {
    /// From a receiver's speed accuracy, `sAcc` — one standard deviation in meters per
    /// second, applied to every axis.
    pub fn from_speed_accuracy(speed: f32) -> Self {
        Self::from_sigma(speed, speed, speed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enu_converts_to_ned_by_permutation() {
        let ned = Position::enu(1.0, 2.0, 3.0).to_ned();
        assert_eq!(ned, Position::ned(2.0, 1.0, -3.0));
        let ned = Velocity::enu(-4.0, 5.0, -6.0).to_ned();
        assert_eq!(ned, Velocity::ned(5.0, -4.0, 6.0));
    }

    #[test]
    fn flu_converts_to_frd_by_negating_left_and_up() {
        assert_eq!(
            AngularRate::flu(0.1, 0.2, 0.3),
            AngularRate::body(0.1, -0.2, -0.3)
        );
        // A level FLU accelerometer at rest reads +g up; FRD reads -g down.
        assert_eq!(
            Acceleration::flu(0.0, 0.0, 9.8),
            Acceleration::body(0.0, 0.0, -9.8)
        );
    }

    #[test]
    fn degrees_are_converted_where_they_arrive() {
        let rate = AngularRate::body_deg_per_s(180.0, 0.0, -90.0);
        assert!((rate.x() - core::f32::consts::PI).abs() < 1e-6);
        assert!((rate.z() + core::f32::consts::FRAC_PI_2).abs() < 1e-6);
        assert!((Radians::from_degrees(-3.5).as_radians() + 0.061_086).abs() < 1e-6);
    }

    #[test]
    fn sigma_and_variance_name_the_same_noise() {
        assert_eq!(
            PositionNoise::horizontal_vertical(1.5, 3.0),
            PositionNoise::<Ned>::from_variance(2.25, 2.25, 9.0)
        );
        assert_eq!(
            VelocityNoise::from_speed_accuracy(0.5).variance(),
            Vector3::repeat(0.25)
        );
        assert_eq!(
            AltitudeNoise::from_sigma(2.0),
            AltitudeNoise::from_variance(4.0)
        );
    }

    #[test]
    fn components_come_out_as_plain_numbers() {
        let p = Position::ned(1.0, 2.0, 3.0);
        assert_eq!(p.to_array(), [1.0, 2.0, 3.0]);
        assert_eq!((p.x(), p.y(), p.z()), (1.0, 2.0, 3.0));
    }
}
