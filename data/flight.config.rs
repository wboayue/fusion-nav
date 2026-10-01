// A Config derived from `flight.csv` by `cargo run --example replay -- --derive`.
// Each value says where it came from; the run's evidence is on stderr.
// As --set: --set gravity=9.80118
fusion_nav::Config {
    imu: fusion_nav::ImuNoise {
        // The default; the window measured a floor of 2.402e-4, 62x under it.
        gyro_white: 0.015,
        // The default; the window measured a floor of 2.448e-3, 143x under it.
        accel_white: 0.35,
        // Bias walks: the default. Not measured: an Allan variance needs a soak of hours.
        gyro_bias_walk: 0.00011,
        accel_bias_walk: 0.0022,
    },
    // Barometer `R` is per call to fuse_baro_altitude, not Config: the window's σ, 0.398 m, is its floor.
    // Gates: the default percentile. ε over the 95 / 99 / 99.9 % bounds, against 5 / 1 / 0.1:
    //   gnss_pos: 0.0 % / 0.0 % / 0.0 % of 20
    //   gnss_hgt: 5.0 % / 5.0 % / 0.0 % of 20
    //   gnss_vel: 0.0 % / 0.0 % / 0.0 % of 20
    //   baro: 0.0 % / 0.0 % / 0.0 % of 65
    //   mag: 0.0 % / 0.0 % / 0.0 % of 49
    gates: fusion_nav::Gates::at(fusion_nav::Percentile::P999),
    timeouts: fusion_nav::Timeouts::default(), // the mission's
    recovery: fusion_nav::Recovery {
        // The defaults, PX4's; beside each, the longest rejection run that ended in an acceptance with recovery off, and recoveries at the default.
        gnss_position: Some(fusion_nav::Seconds::from_secs(7.0)), // no rejection resolved; recovered 0
        gnss_height: Some(fusion_nav::Seconds::from_secs(5.0)), // no rejection resolved; recovered 0
        gnss_velocity: Some(fusion_nav::Seconds::from_secs(7.0)), // no rejection resolved; recovered 0
        baro_altitude: Some(fusion_nav::Seconds::from_secs(5.0)), // no rejection resolved; recovered 0
        mag_heading: Some(fusion_nav::Seconds::from_secs(7.0)), // no rejection resolved; recovered 0
        gnss_heading: Some(fusion_nav::Seconds::from_secs(7.0)), // no rejection resolved; recovered 0
        course: Some(fusion_nav::Seconds::from_secs(7.0)), // no rejection resolved; recovered 0
    },
    correlation: fusion_nav::Correlation {
        // τ = −T / ln ρ from each source's lag-one autocorrelation fused white, a lower bound since an innovation is whiter than the error behind it: printed only where it exceeds the default.
        gnss_position: Some(fusion_nav::Seconds::from_secs(8.5)), // ρ -0.030 over 20 rows, white within 2/√n: the default
        gnss_height: Some(fusion_nav::Seconds::from_secs(40.0)), // ρ 0.042 over 20 rows, white within 2/√n: the default
        gnss_velocity: Some(fusion_nav::Seconds::from_secs(0.5)), // ρ -0.045 over 20 rows, white within 2/√n: the default
        baro_altitude: Some(fusion_nav::Seconds::from_secs(0.26)), // ρ -0.164 over 65 rows, white within 2/√n: the default
        mag_heading: Some(fusion_nav::Seconds::from_secs(1.3)), // ρ -0.106 over 49 rows, white within 2/√n: the default
        gnss_heading: Some(fusion_nav::Seconds::from_secs(0.25)), // not in this log: the default
        course: Some(fusion_nav::Seconds::from_secs(1.4)), // not in this log: the default
    },
    init: fusion_nav::Initialization::default(), // the start's tolerances and priors
    accuracy: fusion_nav::Accuracy::default(), // the mission's
    // WGS-84 normal gravity at the log's origin, 40.1164° -88.3697° 200 m.
    gravity: 9.80118,
    // IMU interval after the start: median 20.0 ms, 99.9 % 20.0 ms, longest ordinary 20.0 ms; widest step among ordinary intervals 1.0x; 0 dropouts of 600 intervals.
    // The default: 0.025 s with margin is under it, and no dropout falls between the two.
    max_predict_dt: fusion_nav::Seconds::from_secs(0.1),
    // The default: the log has no gap to coast.
    coast: Some(fusion_nav::Coast { acceleration: 2.0, rotation: 0.1 }),
    // The default: under 1800 s of barometer beside GNSS height, too short to read a drift.
    // GNSS height / barometer rejections at each walk: 0: 0/0, 0.02: 0/0, 0.05: 0/0, 0.13: 0/0.
    baro_offset_walk: 0.13,
    baro_reference_from_estimate: true, // policy
}
