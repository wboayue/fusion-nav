//! Numbers in `Display` output, written without core's float formatting.
//!
//! `write!(f, "{}", x)` on an `f32` links `core::fmt::float`, whose shortest and exact
//! digit generators index slices and reach `core::panicking`: `panic-check/run.sh` fails on
//! both thumb targets at both gated optimization levels the moment one float is printed
//! that way. Integer formatting passes. So every number an outcome prints goes through
//! [`Fixed`], which rounds to a fixed count of decimals and writes the two halves as
//! integers.

use core::fmt;

/// An `f32` printed with a fixed count of decimals, through integer formatting only.
///
/// Rounds half away from zero. A value whose scaled magnitude does not fit a `u64` prints
/// as `out of range` rather than as the saturated cast, which would be a number the value
/// never was.
pub(crate) struct Fixed {
    value: f32,
    decimals: Decimals,
}

/// How many digits follow the point. A closed set, so the scale is a match rather than a
/// `pow` or an index either of which the panic gate would have to prove in range.
#[derive(Clone, Copy)]
pub(crate) enum Decimals {
    Two,
    Three,
}

impl Fixed {
    pub(crate) const fn new(value: f32, decimals: Decimals) -> Self {
        Self { value, decimals }
    }
}

impl fmt::Display for Fixed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let value = self.value;
        if value.is_nan() {
            return f.write_str("NaN");
        }
        let sign = if value.is_sign_negative() { "-" } else { "" };
        if value.is_infinite() {
            return write!(f, "{sign}inf");
        }
        let (scale, width) = match self.decimals {
            Decimals::Two => (100u64, 2),
            Decimals::Three => (1000u64, 3),
        };
        let scaled = value.abs() * scale as f32 + 0.5;
        // 2^63, exactly representable: anything at or above it saturates the cast below.
        if scaled >= 9.223_372e18 {
            return f.write_str("out of range");
        }
        let scaled = scaled as u64;
        // `-0.00` says a sign survived rounding that no digit carries.
        let sign = if scaled == 0 { "" } else { sign };
        write!(f, "{sign}{}.{:0width$}", scaled / scale, scaled % scale)
    }
}

#[cfg(test)]
mod tests {
    use std::format;

    use super::*;

    fn two(value: f32) -> std::string::String {
        format!("{}", Fixed::new(value, Decimals::Two))
    }

    #[test]
    fn rounds_to_the_decimals_asked_for() {
        assert_eq!(two(2.7), "2.70");
        assert_eq!(two(0.314_159), "0.31");
        assert_eq!(two(12.0), "12.00");
        assert_eq!(format!("{}", Fixed::new(0.0026, Decimals::Three)), "0.003");
        assert_eq!(format!("{}", Fixed::new(1.5, Decimals::Three)), "1.500");
    }

    #[test]
    fn a_negative_value_keeps_its_sign_unless_it_rounds_to_zero() {
        assert_eq!(two(-2.7), "-2.70");
        assert_eq!(two(-0.001), "0.00");
        assert_eq!(two(-0.0), "0.00");
    }

    #[test]
    fn what_is_not_a_number_is_named() {
        assert_eq!(two(f32::NAN), "NaN");
        assert_eq!(two(f32::INFINITY), "inf");
        assert_eq!(two(f32::NEG_INFINITY), "-inf");
        assert_eq!(two(f32::MAX), "out of range");
        assert_eq!(two(-1.0e20), "out of range");
    }
}
