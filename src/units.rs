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

use core::f32::consts::FRAC_1_SQRT_2;
use core::fmt;
use core::marker::PhantomData;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::frames::{Body, Enu, Frame, Ned};

/// Vehicle attitude: the rotation from [`Body`](crate::Body) to [`Ned`](crate::Ned).
///
/// Equation (7). Hamilton convention, scalar first, and normalization is maintained by
/// [`UnitQuaternion`] rather than by the filter remembering to renormalize.
///
/// Every constructor names the convention it takes, and there is no
/// `From<UnitQuaternion<f32>>`, for the reason the module docs give for keeping
/// `From<[f32; 3]>` off the framed vectors: `.into()` would claim body-to-NED for a
/// quaternion that is a stored inverse or an ENU one. A seed is where that costs most,
/// because it is the one input with no residual to expose it: the filter runs on an
/// attitude wrong by a frame, reports [`Status::Healthy`](crate::Status::Healthy) if the
/// seed covariance was confident, and nothing gates. How wrong depends on the attitude —
/// the level case is a half turn, which is why a level bench check is the one that cannot
/// find it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attitude(UnitQuaternion<f32>);

/// The quaternion's four components, scalar first, rather than `nalgebra`'s own `Format`,
/// which would need its `defmt` feature and a second optional dependency edge to name it.
#[cfg(feature = "defmt")]
impl defmt::Format for Attitude {
    fn format(&self, f: defmt::Formatter<'_>) {
        let q = self.0.quaternion();
        defmt::write!(
            f,
            "Attitude({=f32}, {=f32}, {=f32}, {=f32})",
            q.w,
            q.i,
            q.j,
            q.k
        )
    }
}

impl Attitude {
    /// Level and pointing north.
    pub fn level() -> Self {
        Self(UnitQuaternion::identity())
    }

    /// Wrap a quaternion that already rotates body FRD to NED. Equation (7).
    ///
    /// The seed path from either production autopilot, and it converts nothing: PX4's
    /// `vehicle_attitude.q` is the "rotation from the FRD body frame to the NED earth
    /// frame", Hamilton and scalar-first (`msg/versioned/VehicleAttitude.msg:2,10`), and
    /// ArduPilot publishes `AP_AHRS::get_quat_body_to_ned`
    /// (`libraries/AP_AHRS/AP_AHRS.h:672`).
    /// [`flu_to_nwu`](Self::flu_to_nwu) maps `fusion-ahrs`'s three conventions onto these
    /// constructors, `Convention::Ned` included.
    ///
    /// Read at PX4-Autopilot `c4e4ef98` (v1.18.0-beta1) and ardupilot `368dc0c4`.
    pub const fn body_to_ned(q: UnitQuaternion<f32>) -> Self {
        Self(q)
    }

    /// Invert a quaternion that rotates NED to body FRD.
    ///
    /// Storing the inverse — the direction-cosine matrix that takes a navigation vector
    /// into the body frame — is a common convention, and it is the mistake no other check
    /// reaches. An inverted attitude is finite, is a unit quaternion, and is identity
    /// wherever the true one is, so it survives every gate and every static test; it
    /// differs only in the sign of every rotation the vehicle actually has.
    pub fn ned_to_body(q: UnitQuaternion<f32>) -> Self {
        Self(q.inverse())
    }

    /// Convert a quaternion that rotates body FLU to ENU, the ROS REP 103 pair.
    ///
    /// Two-sided, because both frames differ: `q_{NED←FRD} = r_nav ⊗ q ⊗ r_body⁻¹`, where
    /// `r_nav` is `q_{NED←ENU}` and `r_body` is `q_{FRD←FLU}`. Rotating only the
    /// navigation frame is the half-applied form, and it is not obviously wrong: it
    /// reports the same heading as this one and the vehicle upside down, so a level bench
    /// check that reads a compass agrees with it.
    pub fn flu_to_enu(q: UnitQuaternion<f32>) -> Self {
        let r_nav = ned_from_enu();
        let r_body = frd_from_flu();
        Self(r_nav * q * r_body.inverse())
    }

    /// Convert a quaternion that rotates body FLU to NWU, as Madgwick-family filters
    /// report it.
    ///
    /// [`fusion-ahrs`](https://crates.io/crates/fusion-ahrs) is the one to hand: its
    /// quaternion rotates the sensor frame into the earth frame, its default
    /// `Convention::Nwu` puts north, west, up on the earth axes, and a level sensor then
    /// reads gravity on `+z`, which is an FLU body. `Convention::Enu` is
    /// [`flu_to_enu`](Self::flu_to_enu) and `Convention::Ned` is
    /// [`body_to_ned`](Self::body_to_ned).
    ///
    /// Two-sided for the reason [`flu_to_enu`](Self::flu_to_enu) is. Both halves are the
    /// same half turn here, so the conversion is a conjugation and a vehicle that is only
    /// rolled comes through unchanged — a second attitude that cannot tell a conversion
    /// from no conversion at all.
    pub fn flu_to_nwu(q: UnitQuaternion<f32>) -> Self {
        let r_nav = ned_from_nwu();
        let r_body = frd_from_flu();
        Self(r_nav * q * r_body.inverse())
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

/// `q_{NED←ENU}`: the half turn about the north-east bisector, `(1, 1, 0)/√2`.
///
/// The signed permutation [`Position::to_ned`] applies to a vector, `(n, e, d) =
/// (y, x, -z)`, written as a rotation. Swapping two axes alone would be a reflection;
/// negating the third is what leaves determinant `+1` and a rotation to compose with.
fn ned_from_enu() -> UnitQuaternion<f32> {
    // w = cos(π/2) = 0, vector = sin(π/2)·axis = the axis itself. Unit by construction,
    // so taken unchecked rather than normalized through a division by 1 ± ε.
    UnitQuaternion::new_unchecked(Quaternion::new(0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0))
}

/// `q_{NED←NWU}`: the half turn about north, which negates west and up to give east and
/// down.
fn ned_from_nwu() -> UnitQuaternion<f32> {
    half_turn_about_first_axis()
}

/// `q_{FRD←FLU}`: the half turn about forward, the rotation form of
/// [`AngularRate::flu`]'s `(f, r, d) = (f, -l, -u)`.
fn frd_from_flu() -> UnitQuaternion<f32> {
    half_turn_about_first_axis()
}

/// A half turn about the first axis: the second and third are negated, the first is kept.
///
/// Written once because NWU-to-NED and FLU-to-FRD are the same rotation on different
/// frames, and named twice above so that a call site says which of the two it is.
fn half_turn_about_first_axis() -> UnitQuaternion<f32> {
    UnitQuaternion::new_unchecked(Quaternion::new(0.0, 1.0, 0.0, 0.0))
}

macro_rules! scalar {
    (
        $(#[$meta:meta])*
        $name:ident, unit = $unit:literal, new = $new:ident, get = $get:ident
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

        #[cfg(feature = "defmt")]
        impl<F: Frame> defmt::Format for $name<F> {
            fn format(&self, f: defmt::Formatter<'_>) {
                let v = self.$field;
                defmt::write!(
                    f,
                    "{=str}<{=str}>({=f32}, {=f32}, {=f32}) {=str}",
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
    /// A GNSS receiver's reported accuracy is bounded on both sides before it becomes
    /// this; see [`PositionNoise::clamped`] and
    /// [`Eskf::fuse_gnss_position`](crate::Eskf::fuse_gnss_position).
    PositionNoise,
    sigma = "meters",
    variance = "meters squared"
);

noise3!(
    /// Noise on a velocity measurement, `R` of equation (29), on the axes of `F`.
    ///
    /// Bounded the same way as [`PositionNoise`]; see [`VelocityNoise::clamped`] and
    /// [`Eskf::fuse_gnss_velocity`](crate::Eskf::fuse_gnss_velocity).
    VelocityNoise,
    sigma = "meters per second",
    variance = "meters squared per second squared"
);

/// One σ held between two bounds, for the `clamped` constructors below.
///
/// Written with comparisons rather than [`f32::clamp`], which panics when `min > max`,
/// and rather than [`f32::max`], which returns the bound when the σ is NaN and would
/// turn a measurement [`Fusion::NotFinite`](crate::Fusion::NotFinite) exists to refuse
/// into a plausible-looking number. Every comparison against NaN is false, so a NaN σ
/// falls through unchanged and is refused where every other non-finite noise is.
///
/// Bounds the wrong way round saturate to `max` rather than refusing: `clamped` is on the
/// per-measurement path, and a filter that cannot panic has no better answer available.
fn clamp_sigma(sigma: f32, min: f32, max: f32) -> f32 {
    let floored = if sigma < min { min } else { sigma };
    if floored > max { max } else { floored }
}

impl PositionNoise<Ned> {
    /// From a receiver's horizontal and vertical accuracy, `eph` and `epv` — standard
    /// deviations in meters, as u-blox (`hAcc`, `vAcc`), MAVLink and PX4 report them.
    ///
    /// Also how a two-dimensional fix is expressed: give the vertical axis a σ large
    /// enough that its Kalman gain is negligible against the height uncertainty the
    /// filter already holds, and say in the calling code which it is. A kilometre is
    /// comfortably that for any vehicle this filter runs on, and is a number a reader
    /// recognises as deliberate where `1e6` reads as arbitrary.
    ///
    /// Unbounded on purpose, which is the whole difference from [`clamped`](Self::clamped):
    /// a σ standing for an axis the receiver did not measure is a sentinel, not an
    /// accuracy, and a cap would quietly turn it back into a measurement.
    pub fn horizontal_vertical(horizontal: f32, vertical: f32) -> Self {
        Self::from_sigma(horizontal, horizontal, vertical)
    }

    /// The same, with each σ held between `min_sigma` and `max_sigma` in meters.
    ///
    /// For the axes the receiver actually measured. Both are capped, so this is not the
    /// way to drop one — `clamped(eph, 1000.0, 0.5, 100.0)` gives the vertical axis
    /// σ = 100 m, which is a height measurement the filter will lean on rather than the
    /// one being declined. A two-dimensional fix goes through
    /// [`horizontal_vertical`](Self::horizontal_vertical) instead, bounding `eph` first
    /// if it came from the receiver.
    ///
    /// A receiver's accuracy estimate is its view of its own geometry and residuals, and
    /// under multipath it stays small while the fix is metres wrong — so both production
    /// estimators bound it on both sides rather than trusting it. ArduPilot writes the
    /// two-sided form directly, `constrain_ftype(gpsPosAccuracy, _gpsHorizPosNoise, 100)`
    /// at `AP_NavEKF3_PosVelFusion.cpp:836` — the ordinary GNSS branch, not the identical
    /// line at `:807`, which is the synthetic-zero-velocity case opened at `:789`.
    /// PX4 floors at `ekf2_gps_p_noise`
    /// (`EKF/aid_sources/gnss/gps_control.cpp:358`) and caps at `ekf2_noaid_noise`, 10 m,
    /// but only while GNSS is the sole horizontal aid (`:363-364`) — the cap is about
    /// what the filter can afford to lean on, not about the fix.
    ///
    /// Keeping both bounds here rather than in [`Config`](crate::Config) keeps them
    /// travelling with the measurement they describe, which is the same reason `R` is a
    /// per-call argument at all.
    ///
    /// Read at PX4-Autopilot `c4e4ef98` (v1.18.0-beta1) and ardupilot `368dc0c4`.
    pub fn clamped(horizontal: f32, vertical: f32, min_sigma: f32, max_sigma: f32) -> Self {
        let horizontal = clamp_sigma(horizontal, min_sigma, max_sigma);
        Self::from_sigma(
            horizontal,
            horizontal,
            clamp_sigma(vertical, min_sigma, max_sigma),
        )
    }
}

impl VelocityNoise<Ned> {
    /// From a receiver's speed accuracy, `sAcc` — one standard deviation in meters per
    /// second, applied to every axis.
    ///
    /// Isotropic, because that is what the receiver said: `sAcc` is a single scalar
    /// (u-blox `sAcc`, PX4 `s_variance_m_s`, ArduPilot `speed_accuracy`) and it carries no
    /// claim about the vertical axis being worse. Both production estimators nonetheless
    /// loosen vertical before fusing — PX4 by exactly 1.5,
    /// `Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f))`
    /// (`EKF/aid_sources/gnss/gps_control.cpp:321`), the ratio it names elsewhere as "a
    /// typical ratio of vacc/hacc" (`gnss_height_control.cpp:62`); ArduPilot by flooring
    /// the axes differently, `_gpsHorizVelNoise` 0.3 m/s against `_gpsVertVelNoise` 0.5
    /// on copter (`AP_NavEKF3_PosVelFusion.cpp:821-822`, `AP_NavEKF3.cpp:23-24`).
    ///
    /// That is a policy about receivers rather than a property of this fix, so it stays
    /// with the caller, who writes it as
    /// [`horizontal_vertical`](Self::horizontal_vertical)`(sacc, 1.5 * sacc)` for PX4's
    /// shape. `R` describing the measurement is what lets a caller with a better number —
    /// a receiver reporting `vAcc` separately, a dual-frequency fix — supply it.
    ///
    /// Read at PX4-Autopilot `c4e4ef98` (v1.18.0-beta1) and ardupilot `368dc0c4`.
    pub fn from_speed_accuracy(speed: f32) -> Self {
        Self::from_sigma(speed, speed, speed)
    }

    /// From separate horizontal and vertical speed accuracies, standard deviations in
    /// meters per second.
    ///
    /// The shape both production estimators fuse in; see
    /// [`from_speed_accuracy`](Self::from_speed_accuracy) for the ratios they use and why
    /// applying one is the caller's call.
    ///
    /// Also how a solution with no usable vertical velocity is expressed — the case both
    /// platforms gate on a flag, PX4 `vel_ned_valid` (`msg/SensorGps.msg:59`) and
    /// ArduPilot `have_vertical_velocity` (`AP_GPS.h:214`). Give the down axis a σ large
    /// enough that its Kalman gain is negligible against the vertical velocity
    /// uncertainty the filter holds; 1000 m/s is unambiguously that and reads as
    /// deliberate. Unbounded for the reason
    /// [`PositionNoise::horizontal_vertical`] is — a declined axis is a sentinel, and a
    /// cap would turn it back into a measurement.
    pub fn horizontal_vertical(horizontal: f32, vertical: f32) -> Self {
        Self::from_sigma(horizontal, horizontal, vertical)
    }

    /// From horizontal and vertical speed accuracies, each held between `min_sigma` and
    /// `max_sigma` in meters per second. An isotropic `sacc` is
    /// `clamped(sacc, sacc, min, max)`.
    ///
    /// Bounded on both sides for the reason [`PositionNoise::clamped`] is, and per axis
    /// because ArduPilot is: it floors the axes at different parameters, 0.3 m/s
    /// horizontal against 0.5 vertical on copter, under one shared 50 m/s cap
    /// (`AP_NavEKF3_PosVelFusion.cpp:821-822`, `AP_NavEKF3.cpp:23-24`). PX4 floors at
    /// `ekf2_gps_v_noise`, 0.5 m/s, and caps nothing
    /// (`EKF/aid_sources/gnss/gps_control.cpp:320`).
    ///
    /// Bounds are for axes the receiver measured. A solution with no usable vertical
    /// velocity goes through
    /// [`horizontal_vertical`](Self::horizontal_vertical), for the reason
    /// [`PositionNoise::clamped`] gives.
    ///
    /// Read at PX4-Autopilot `c4e4ef98` (v1.18.0-beta1) and ardupilot `368dc0c4`.
    pub fn clamped(horizontal: f32, vertical: f32, min_sigma: f32, max_sigma: f32) -> Self {
        let horizontal = clamp_sigma(horizontal, min_sigma, max_sigma);
        Self::from_sigma(
            horizontal,
            horizontal,
            clamp_sigma(vertical, min_sigma, max_sigma),
        )
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Rotation3};

    use super::*;

    /// Two rotations agreeing to within a tolerance, compared as rotations: a quaternion
    /// and its negation are the same attitude, and the conversions below produce whichever
    /// sign the multiplication lands on.
    fn assert_same_rotation(left: UnitQuaternion<f32>, right: UnitQuaternion<f32>) {
        assert!(
            left.angle_to(&right) < 1.0e-6,
            "{left} and {right} differ by {} rad",
            left.angle_to(&right)
        );
    }

    /// One physical attitude — nose east, rolled 90° right — written out as the body axes
    /// each convention would express it in, in the order (NED from FRD, ENU from FLU, NWU
    /// from FLU). The columns of a body-to-navigation rotation are the body axes in the
    /// navigation frame, so each triad is that matrix.
    ///
    /// Deliberately neither level nor axis-aligned in yaw: its quaternion is
    /// `(½, ½, ½, ½)`, every component nonzero, so a conversion that rotates only the
    /// navigation frame lands somewhere else. At level attitude it would not.
    fn nose_east_rolled_right() -> [UnitQuaternion<f32>; 3] {
        // Nose east, rolled right through 90°: the body down axis points north and the
        // body right axis points at the ground.
        let ned_from_frd = Matrix3::from_columns(&[
            Vector3::new(0.0, 1.0, 0.0), // forward is east
            Vector3::new(0.0, 0.0, 1.0), // right is down
            Vector3::new(1.0, 0.0, 0.0), // down is north
        ]);
        let enu_from_flu = Matrix3::from_columns(&[
            Vector3::new(1.0, 0.0, 0.0),  // forward is east
            Vector3::new(0.0, 0.0, 1.0),  // left is up
            Vector3::new(0.0, -1.0, 0.0), // up is south
        ]);
        let nwu_from_flu = Matrix3::from_columns(&[
            Vector3::new(0.0, -1.0, 0.0), // forward is east, which is -west
            Vector3::new(0.0, 0.0, 1.0),  // left is up
            Vector3::new(-1.0, 0.0, 0.0), // up is south
        ]);
        [ned_from_frd, enu_from_flu, nwu_from_flu]
            .map(|m| UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(m)))
    }

    #[test]
    fn every_convention_maps_one_attitude_to_the_same_body_to_ned_quaternion() {
        let [ned_from_frd, enu_from_flu, nwu_from_flu] = nose_east_rolled_right();

        assert_same_rotation(
            Attitude::body_to_ned(ned_from_frd).quaternion(),
            ned_from_frd,
        );
        assert_same_rotation(
            Attitude::flu_to_enu(enu_from_flu).quaternion(),
            ned_from_frd,
        );
        assert_same_rotation(
            Attitude::flu_to_nwu(nwu_from_flu).quaternion(),
            ned_from_frd,
        );
    }

    #[test]
    fn a_navigation_only_conversion_keeps_the_heading_and_turns_the_vehicle_over() {
        let [ned_from_frd, enu_from_flu, _] = nose_east_rolled_right();

        // What rotating only the navigation frame computes: an attitude a whole rotation
        // away from the truth, which this attitude shows and a level one does not.
        let half_applied = ned_from_enu() * enu_from_flu;
        assert!(half_applied.angle_to(&ned_from_frd) > 1.0);

        // Level, pointing east. The half-applied form differs by a half turn about body
        // forward — the vehicle inverted — and reports the same heading, which is the one
        // number a bench check reads.
        let level = UnitQuaternion::identity();
        let (half_roll, _, half_yaw) = (ned_from_enu() * level).euler_angles();
        let (roll, _, yaw) = Attitude::flu_to_enu(level).euler_angles();
        assert!((half_yaw - yaw).abs() < 1.0e-6);
        assert!((half_roll.abs() - core::f32::consts::PI).abs() < 1.0e-6);
        assert!(roll.abs() < 1.0e-6);
    }

    #[test]
    fn an_identity_in_each_convention_is_the_attitude_that_convention_calls_level() {
        let (roll, pitch, yaw) = Attitude::flu_to_enu(UnitQuaternion::identity()).euler_angles();
        // ENU pairs east with body forward, so its identity is a vehicle pointing east.
        assert!(roll.abs() < 1.0e-6 && pitch.abs() < 1.0e-6);
        assert!((yaw - core::f32::consts::FRAC_PI_2).abs() < 1.0e-6);

        // NWU pairs north with body forward, so its identity is ours.
        assert_same_rotation(
            Attitude::flu_to_nwu(UnitQuaternion::identity()).quaternion(),
            Attitude::level().quaternion(),
        );

        // And NWU's conversion is a conjugation by a half turn about that shared first
        // axis, so a vehicle that is only rolled comes through with its own quaternion.
        let rolled = UnitQuaternion::from_euler_angles(0.4, 0.0, 0.0);
        assert_same_rotation(Attitude::flu_to_nwu(rolled).quaternion(), rolled);
    }

    #[test]
    fn a_stored_inverse_is_inverted_rather_than_wrapped() {
        let [ned_from_frd, ..] = nose_east_rolled_right();
        let frd_from_ned = ned_from_frd.inverse();

        assert_same_rotation(
            Attitude::ned_to_body(frd_from_ned).quaternion(),
            ned_from_frd,
        );
        // The error the inversion exists to prevent: still a unit quaternion, still
        // finite, and a whole rotation away from the truth.
        assert!(frd_from_ned.angle_to(&ned_from_frd) > 1.0);
    }

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
    fn clamping_bounds_a_sigma_on_both_sides() {
        // Inside the bounds, the receiver's own number survives untouched.
        assert_eq!(
            PositionNoise::clamped(1.5, 3.0, 0.5, 100.0),
            PositionNoise::horizontal_vertical(1.5, 3.0)
        );
        // An optimistic fix is floored, an implausible one capped, per axis.
        assert_eq!(
            PositionNoise::clamped(0.01, 250.0, 0.5, 100.0),
            PositionNoise::horizontal_vertical(0.5, 100.0)
        );
        assert_eq!(
            VelocityNoise::clamped(0.02, 0.02, 0.5, 50.0).variance(),
            Vector3::repeat(0.25)
        );
        // Per axis, so PX4's 1.5 vertical ratio survives the bounds it is applied under.
        assert_eq!(
            VelocityNoise::clamped(0.4, 0.6, 0.3, 50.0),
            VelocityNoise::horizontal_vertical(0.4, 0.6)
        );
    }

    #[test]
    fn a_clamped_nan_stays_nan_rather_than_becoming_a_bound() {
        // `f32::max` would return the bound here, and the fix would be fused with an
        // invented accuracy instead of refused as Fusion::NotFinite.
        let noise = PositionNoise::clamped(f32::NAN, 3.0, 0.5, 100.0);
        assert!(noise.variance()[0].is_nan());
        assert!(!noise.is_finite());
    }

    #[test]
    fn bounds_the_wrong_way_round_saturate_rather_than_panicking() {
        assert_eq!(
            VelocityNoise::clamped(1.0, 1.0, 10.0, 5.0).variance(),
            Vector3::repeat(25.0)
        );
    }

    #[test]
    fn a_dropped_vertical_axis_is_a_large_sigma_on_that_axis_alone() {
        let noise = VelocityNoise::horizontal_vertical(0.3, 1000.0);
        assert_eq!(noise.variance()[0], 0.09);
        assert_eq!(noise.variance()[2], 1.0e6);
        assert!(noise.is_positive());
    }

    #[test]
    fn components_come_out_as_plain_numbers() {
        let p = Position::ned(1.0, 2.0, 3.0);
        assert_eq!(p.to_array(), [1.0, 2.0, 3.0]);
        assert_eq!((p.x(), p.y(), p.z()), (1.0, 2.0, 3.0));
    }
}
