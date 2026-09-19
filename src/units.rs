//! Typed quantities the filter's API is expressed in.
//!
//! Every quantity crossing the API boundary names its unit in its constructor, so a
//! caller cannot pass degrees where radians are expected without writing the conversion
//! down. Vector quantities additionally carry their [`Frame`].
//!
//! The numeric payloads are `nalgebra` types — [`Vector3<f32>`] and
//! [`UnitQuaternion<f32>`] — so an application already using `nalgebra` can unwrap a
//! quantity and keep working, and the filter's internals get `nalgebra`'s fixed-size
//! matrix algebra without a second vector type to convert through.

use core::fmt;
use core::marker::PhantomData;

use nalgebra::{UnitQuaternion, Vector3};

use crate::frames::Frame;

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

scalar!(
    /// Variance of a scalar altitude measurement.
    AltitudeVariance,
    unit = "meters squared",
    new = from_m2,
    get = as_m2
);

scalar!(
    /// Variance of a scalar heading measurement.
    HeadingVariance,
    unit = "radians squared",
    new = from_rad2,
    get = as_rad2
);

macro_rules! framed {
    (
        $(#[$meta:meta])*
        $name:ident, unit = $unit:literal, new = $new:ident, get = $get:ident
    ) => {
        $(#[$meta])*
        pub struct $name<F: Frame> {
            value: Vector3<f32>,
            frame: PhantomData<F>,
        }

        impl<F: Frame> $name<F> {
            #[doc = concat!("Construct from components in ", $unit, ".")]
            pub fn $new(x: f32, y: f32, z: f32) -> Self {
                Self::from_vector(Vector3::new(x, y, z))
            }

            #[doc = concat!("Construct from a vector whose components are in ", $unit, ".")]
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

            #[doc = concat!("The components, in ", $unit, ".")]
            pub const fn $get(self) -> Vector3<f32> {
                self.value
            }
        }

        // Written out rather than derived: `derive` would add an `F: Clone` bound, and the
        // frame markers are only ever type-level.
        impl<F: Frame> Clone for $name<F> {
            fn clone(&self) -> Self {
                *self
            }
        }

        impl<F: Frame> Copy for $name<F> {}

        impl<F: Frame> Default for $name<F> {
            fn default() -> Self {
                Self::zero()
            }
        }

        impl<F: Frame> PartialEq for $name<F> {
            fn eq(&self, other: &Self) -> bool {
                self.value == other.value
            }
        }

        impl<F: Frame> fmt::Debug for $name<F> {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(
                    f,
                    "{}<{}>({}, {}, {}) {}",
                    stringify!($name),
                    F::NAME,
                    self.value.x,
                    self.value.y,
                    self.value.z,
                    $unit
                )
            }
        }
    };
}

framed!(
    /// Position relative to the navigation origin.
    Position,
    unit = "meters",
    new = from_meters,
    get = as_meters
);

framed!(
    /// Velocity.
    Velocity,
    unit = "meters per second",
    new = from_m_per_s,
    get = as_m_per_s
);

framed!(
    /// Specific force, or an accelerometer bias.
    Acceleration,
    unit = "meters per second squared",
    new = from_m_per_s2,
    get = as_m_per_s2
);

framed!(
    /// Angular rate, or a gyroscope bias.
    AngularRate,
    unit = "radians per second",
    new = from_rad_per_s,
    get = as_rad_per_s
);

framed!(
    /// A magnetic field measurement.
    ///
    /// Scale is irrelevant — only the direction is fused — but the components must be
    /// hard- and soft-iron calibrated. See
    /// [magnetometer, heading only](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#magnetometer-heading-only).
    MagField,
    unit = "arbitrary units",
    new = from_components,
    get = as_components
);

framed!(
    /// Per-axis variance of a position measurement, on the axes of `F`.
    ///
    /// A GNSS receiver's reported accuracy needs a floor before it becomes this; see
    /// [`Eskf::fuse_gnss_position`](crate::Eskf::fuse_gnss_position).
    PositionVariance,
    unit = "meters squared",
    new = from_m2,
    get = as_m2
);

framed!(
    /// Per-axis variance of a velocity measurement, on the axes of `F`.
    ///
    /// Floored the same way as [`PositionVariance`]; see
    /// [`Eskf::fuse_gnss_velocity`](crate::Eskf::fuse_gnss_velocity).
    VelocityVariance,
    unit = "meters squared per second squared",
    new = from_m2_per_s2,
    get = as_m2_per_s2
);

impl<F: Frame> PositionVariance<F> {
    /// The same variance on all three axes.
    pub fn isotropic(variance: f32) -> Self {
        Self::from_m2(variance, variance, variance)
    }
}

impl<F: Frame> VelocityVariance<F> {
    /// The same variance on all three axes.
    pub fn isotropic(variance: f32) -> Self {
        Self::from_m2_per_s2(variance, variance, variance)
    }
}
