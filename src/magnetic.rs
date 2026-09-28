//! Magnetic declination from PX4's World Magnetic Model table, for the site the origin names.
//!
//! GOALS.md differentiator 7 lists declination as derived: "a magnetic model, given the GNSS
//! origin". The filter reads it from here when it places an origin and no caller has set one;
//! see [`Eskf::set_magnetic_declination`](crate::Eskf::set_magnetic_declination).
//!
//! The table is PX4's `declination_table`, `src/lib/world_magnetic_model/geo_magnetic_tables.hpp`
//! at PX4-Autopilot `c4e4ef98`: WMM-2020 evaluated at epoch 2024.41257, every 10° of latitude
//! and longitude, in units of [`SCALE`] degrees. The lookup is PX4's `get_table_data`
//! (`geo_mag_declination.cpp:56-104` there): bilinear between the four corners about the point.
//! Both are the ones `tools/ulog2replay.py` ports to reproduce EKF2's own declination on the
//! corpus, so the replay harness's `declination_model=` key compares this lookup against that
//! one on every log with a fix.
//!
//! What it costs, measured on `panic-check`'s ELF (fat LTO, `opt-level = "s"`) with the
//! `magnetic-model` feature and without: 1408 bytes of `.rodata`, the table, and 1096 of `.text`
//! on `thumbv6m-none-eabi` (1520 on `thumbv7em-none-eabihf`), about 2.5 KB of flash. Hence the
//! feature, which GOALS.md's derived-configuration table asks for, "optional, for its flash
//! cost": off, none of it is linked.
//!
//! ArduPilot ships the same shape (`AP_Declination`, a `float[19][37]` regenerated from IGRF),
//! and neither takes a date: a table is fixed at the epoch it was generated for, and secular
//! variation accumulates until it is regenerated. The WMM itself is US-government work in the
//! public domain; the table and lookup are PX4's, under the BSD-3-Clause licence below, which
//! redistribution requires be retained.
//!
//! ```text
//! Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//! 1. Redistributions of source code must retain the above copyright
//!    notice, this list of conditions and the following disclaimer.
//! 2. Redistributions in binary form must reproduce the above copyright
//!    notice, this list of conditions and the following disclaimer in
//!    the documentation and/or other materials provided with the
//!    distribution.
//! 3. Neither the name PX4 nor the names of its contributors may be
//!    used to endorse or promote products derived from this software
//!    without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//! "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//! LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//! FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//! COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//! BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//! OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//! AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//! LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//! ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//! POSSIBILITY OF SUCH DAMAGE.
//! ```

use crate::geodetic::Geodetic;
use crate::units::Radians;

/// Degrees per table unit: PX4's `WMM_DECLINATION_SCALE_TO_DEGREES`.
const SCALE: f32 = 0.005_451_435;

/// Grid spacing in degrees, on both axes.
const RESOLUTION: f32 = 10.0;

/// Rows are latitude −90° to 90°, columns longitude −180° to 180°.
#[rustfmt::skip]
const TABLE: [[i16; 37]; 19] = [
    // -90°
    [
        27264, 25429, 23595, 21761, 19926, 18092, 16258, 14423, 12589, 10754, 8920, 7086, 5251,
        3417, 1583, -252, -2086, -3921, -5755, -7589, -9424, -11258, -13092, -14927, -16761,
        -18596, -20430, -22264, -24099, -25933, -27768, -29602, -31436, 32767, 30933, 29098,
        27264
    ],
    // -80°
    [
        23650, 21416, 19382, 17521, 15799, 14184, 12647, 11165, 9721, 8303, 6904, 5519, 4143,
        2772, 1397, 9, -1403, -2848, -4333, -5863, -7437, -9056, -10718, -12423, -14175, -15984,
        -17864, -19835, -21922, -24152, -26545, -29105, -31803, 31464, 28718, 26092, 23650
    ],
    // -70°
    [
        15755, 14282, 13094, 12077, 11159, 10281, 9392, 8457, 7454, 6381, 5257, 4109, 2971,
        1865, 794, -267, -1359, -2527, -3797, -5172, -6632, -8142, -9669, -11188, -12690,
        -14181, -15686, -17253, -18969, -20997, -23675, -27707, 32110, 25303, 20617, 17720,
        15755
    ],
    // -60°
    [
        8903, 8634, 8328, 8030, 7755, 7481, 7150, 6689, 6037, 5171, 4116, 2948, 1775, 704, -211,
        -998, -1763, -2635, -3706, -4982, -6396, -7846, -9240, -10513, -11630, -12572, -13320,
        -13839, -14019, -13549, -11322, -3523, 5353, 8167, 8943, 9057, 8903
    ],
    // -50°
    [
        5806, 5839, 5774, 5672, 5587, 5541, 5499, 5359, 4991, 4286, 3216, 1875, 470, -754,
        -1647, -2218, -2623, -3091, -3835, -4934, -6270, -7627, -8830, -9771, -10388, -10623,
        -10400, -9587, -7986, -5492, -2431, 460, 2679, 4178, 5097, 5594, 5806
    ],
    // -40°
    [
        4186, 4282, 4284, 4229, 4159, 4118, 4119, 4103, 3914, 3342, 2253, 728, -912, -2273,
        -3151, -3597, -3767, -3819, -4016, -4665, -5740, -6891, -7817, -8365, -8453, -8028,
        -7069, -5603, -3811, -2034, -509, 772, 1872, 2790, 3486, 3942, 4186
    ],
    // -30°
    [
        3161, 3250, 3275, 3251, 3182, 3094, 3026, 2988, 2851, 2337, 1229, -384, -2070, -3364,
        -4111, -4446, -4503, -4252, -3790, -3609, -4059, -4874, -5597, -5938, -5777, -5133,
        -4109, -2845, -1594, -619, 83, 710, 1372, 2016, 2561, 2948, 3161
    ],
    // -20°
    [
        2485, 2532, 2542, 2534, 2481, 2375, 2255, 2172, 2018, 1486, 353, -1228, -2777, -3865,
        -4390, -4481, -4240, -3623, -2707, -1902, -1669, -2082, -2768, -3241, -3254, -2856,
        -2179, -1328, -531, -35, 229, 542, 1013, 1529, 1985, 2318, 2485
    ],
    // -10°
    [
        2072, 2064, 2032, 2018, 1980, 1883, 1758, 1657, 1461, 874, -261, -1719, -3046, -3887,
        -4129, -3855, -3229, -2397, -1510, -734, -274, -342, -854, -1379, -1586, -1478, -1143,
        -624, -116, 121, 159, 321, 720, 1197, 1625, 1939, 2072
    ],
    // 0°
    [
        1847, 1810, 1742, 1723, 1703, 1621, 1500, 1371, 1102, 444, -660, -1952, -3043, -3628,
        -3589, -3035, -2219, -1401, -724, -169, 254, 350, 38, -401, -658, -711, -608, -338, -43,
        36, -42, 45, 413, 896, 1354, 1703, 1847
    ],
    // 10°
    [
        1699, 1705, 1653, 1662, 1681, 1620, 1480, 1275, 878, 122, -954, -2080, -2929, -3256,
        -3012, -2356, -1536, -796, -265, 129, 466, 610, 425, 85, -155, -268, -299, -222, -121,
        -175, -335, -312, 14, 509, 1030, 1470, 1699
    ],
    // 20°
    [
        1494, 1649, 1708, 1799, 1884, 1855, 1681, 1355, 781, -117, -1209, -2197, -2803, -2893,
        -2528, -1885, -1137, -461, 13, 332, 601, 746, 640, 383, 178, 51, -55, -135, -228, -437,
        -694, -758, -503, -19, 561, 1114, 1494
    ],
    // 30°
    [
        1160, 1544, 1816, 2050, 2215, 2221, 2017, 1567, 802, -275, -1438, -2341, -2756, -2673,
        -2242, -1629, -939, -294, 184, 499, 738, 885, 855, 696, 542, 408, 230, -8, -317, -718,
        -1109, -1275, -1099, -642, -30, 613, 1160
    ],
    // 40°
    [
        765, 1379, 1900, 2314, 2577, 2622, 2393, 1831, 876, -403, -1685, -2574, -2892, -2722,
        -2248, -1623, -936, -278, 252, 629, 908, 1109, 1199, 1184, 1104, 944, 647, 195, -387,
        -1030, -1576, -1826, -1696, -1253, -625, 77, 765
    ],
    // 50°
    [
        437, 1218, 1936, 2528, 2919, 3036, 2799, 2116, 928, -623, -2091, -3030, -3327, -3121,
        -2604, -1925, -1180, -449, 193, 719, 1153, 1519, 1805, 1977, 1991, 1784, 1297, 527,
        -430, -1381, -2085, -2373, -2234, -1771, -1112, -355, 437
    ],
    // 60°
    [
        198, 1087, 1936, 2674, 3216, 3452, 3243, 2411, 855, -1158, -2926, -3941, -4206, -3936,
        -3339, -2560, -1696, -817, 27, 812, 1534, 2191, 2754, 3160, 3315, 3098, 2397, 1190,
        -317, -1699, -2593, -2902, -2725, -2214, -1502, -680, 198
    ],
    // 70°
    [
        -111, 863, 1803, 2648, 3305, 3634, 3405, 2285, 58, -2678, -4700, -5589, -5627, -5143,
        -4352, -3383, -2317, -1205, -83, 1026, 2098, 3108, 4014, 4748, 5204, 5214, 4533, 2927,
        575, -1644, -2988, -3441, -3276, -2730, -1962, -1068, -111
    ],
    // 80°
    [
        -1054, -90, 801, 1527, 1946, 1812, 731, -1632, -4654, -6842, -7732, -7698, -7114, -6204,
        -5096, -3864, -2554, -1197, 184, 1572, 2947, 4292, 5581, 6779, 7828, 8630, 8988, 8486,
        6313, 2026, -1910, -3677, -4009, -3636, -2914, -2019, -1054
    ],
    // 90°
    [
        -30607, -28773, -26938, -25104, -23269, -21435, -19601, -17766, -15932, -14097, -12263,
        -10429, -8594, -6760, -4926, -3091, -1257, 578, 2412, 4246, 6081, 7915, 9749, 11584,
        13418, 15253, 17087, 18921, 20756, 22590, 24424, 26259, 28093, 29928, 31762, -32441,
        -30607
    ],
];

/// The declination at `site`, east-positive, from the table: `None` for a latitude or
/// longitude that is not a number.
///
/// Latitude is clamped to ±90° and longitude wrapped into ±180°, as PX4's lookup does. Height
/// is ignored, as both upstream tables ignore it: the field's direction changes by well under
/// the table's own interpolation error across any altitude a vehicle flies at.
pub(crate) fn declination_at(site: Geodetic) -> Option<Radians> {
    // In f32, as PX4 computes it. The converter's f64 port agrees to 1e-4° (the test below),
    // and in f64 the lookup linked 4496 bytes of `.text` on `thumbv6m-none-eabi` in software
    // doubles, against 1096 here.
    let (latitude, longitude) = (site.latitude_deg() as f32, site.longitude_deg() as f32);
    if !latitude.is_finite() || !longitude.is_finite() {
        return None;
    }
    let latitude = latitude.clamp(-90.0, 90.0);
    // One turn either way, as PX4's `get_table_data` wraps, then clamped: a longitude beyond
    // ±540° is not one a receiver reports.
    let mut longitude = longitude;
    if longitude > 180.0 {
        longitude -= 360.0;
    }
    if longitude < -180.0 {
        longitude += 360.0;
    }
    let longitude = longitude.clamp(-180.0, 180.0);

    // The south-west corner of the cell, one short of the last row and column so that the
    // north-east corner exists. Truncation is the floor here: both offsets are non-negative.
    let row = (((latitude + 90.0) / RESOLUTION) as usize).min(TABLE.len() - 2);
    let column = (((longitude + 180.0) / RESOLUTION) as usize).min(TABLE[0].len() - 2);
    let (south, north) = (TABLE.get(row)?, TABLE.get(row + 1)?);
    let corner = |line: &[i16; 37], at: usize| line.get(at).map(|&v| f32::from(v));
    let (sw, se) = (corner(south, column)?, corner(south, column + 1)?);
    let (nw, ne) = (corner(north, column)?, corner(north, column + 1)?);

    let lat_scale = ((latitude - (row as f32 * RESOLUTION - 90.0)) / RESOLUTION).clamp(0.0, 1.0);
    let lon_scale =
        ((longitude - (column as f32 * RESOLUTION - 180.0)) / RESOLUTION).clamp(0.0, 1.0);
    let southern = lon_scale * (se - sw) + sw;
    let northern = lon_scale * (ne - nw) + nw;
    Some(Radians::from_degrees(
        (lat_scale * (northern - southern) + southern) * SCALE,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn degrees_at(latitude: f64, longitude: f64) -> f32 {
        let radians = declination_at(Geodetic::from_degrees(latitude, longitude, 0.0));
        radians
            .map(|d| d.as_radians().to_degrees())
            .unwrap_or(f32::NAN)
    }

    /// PX4's own `test_geo_lookup.cpp` values at grid points, within its tolerance of
    /// 0.4 + 1.0 degrees, as `tools/ulog2replay.py`'s self-test checks its port.
    #[test]
    fn grid_points_agree_with_px4s_own_test() {
        assert!((degrees_at(-50.0, -180.0) - 31.7).abs() <= 1.4);
        assert!((degrees_at(-50.0, -100.0) - 27.2).abs() <= 1.4);
    }

    /// `table_declination` in `tools/ulog2replay.py` at the same points, in degrees: the two
    /// lookups are one rule, so they agree to the f32 the answer is returned in. The
    /// corpus repeats the comparison at every log's first fix (`declination_model=`).
    #[test]
    fn the_converters_port_gives_the_same_answer() {
        for (latitude, longitude, converter) in [
            (56.41, 43.76, 13.816_002),
            (47.397742, 8.545594, 3.405_605),
            (-33.9, 151.2, 13.030_712),
        ] {
            let got = degrees_at(latitude, longitude);
            assert!(
                (got - converter).abs() < 1e-4,
                "{latitude}, {longitude}: {got} against {converter}"
            );
        }
    }

    #[test]
    fn halfway_between_two_columns_is_their_mean() {
        let mean = (degrees_at(-50.0, -180.0) + degrees_at(-50.0, -170.0)) / 2.0;
        assert!((degrees_at(-50.0, -175.0) - mean).abs() < 1e-5);
    }

    #[test]
    fn longitude_wraps_and_latitude_clamps() {
        assert_eq!(degrees_at(0.0, 190.0), degrees_at(0.0, -170.0));
        assert_eq!(degrees_at(0.0, -190.0), degrees_at(0.0, 170.0));
        assert_eq!(degrees_at(95.0, 20.0), degrees_at(90.0, 20.0));
    }

    #[test]
    fn a_site_that_is_not_a_number_has_no_declination() {
        assert!(declination_at(Geodetic::from_degrees(f64::NAN, 0.0, 0.0)).is_none());
        assert!(declination_at(Geodetic::from_degrees(0.0, f64::INFINITY, 0.0)).is_none());
    }
}
