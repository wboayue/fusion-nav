//! Reference frames as compile-time markers.
//!
//! A frame is a zero-sized type parameter on a quantity, so that a
//! [`Position<Enu>`](crate::Position) cannot be passed where a `Position<Ned>` is expected.
//! Frames carry no data and cost nothing at runtime.

mod sealed {
    pub trait Sealed {}
}

/// A reference frame a quantity can be expressed in.
///
/// Sealed: frames are a closed set, because the filter's Jacobians depend on the
/// conventions of [`Ned`] and [`Body`] specifically.
pub trait Frame: sealed::Sealed + Copy + core::fmt::Debug {
    /// Short label used in `Debug` output, such as `"NED"`.
    const NAME: &'static str;
}

macro_rules! frame {
    ($(#[$meta:meta])* $name:ident, $label:literal) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
        pub struct $name;

        impl sealed::Sealed for $name {}

        impl Frame for $name {
            const NAME: &'static str = $label;
        }
    };
}

frame!(
    /// Local tangent navigation frame: North, East, Down.
    ///
    /// Down-positive, so gravity has a positive `z` component. This is the frame the
    /// filter navigates in.
    Ned,
    "NED"
);

frame!(
    /// Local tangent frame: East, North, Up.
    ///
    /// Not used by the filter. It exists so that an ENU quantity is a compile error at the
    /// boundary rather than a sign error in flight.
    Enu,
    "ENU"
);

frame!(
    /// Vehicle body frame: x forward, y right, z down.
    ///
    /// IMU and magnetometer measurements and the IMU biases live here.
    Body,
    "body"
);
