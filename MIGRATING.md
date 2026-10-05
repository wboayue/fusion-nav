# Coming from PX4 or ArduPilot

How a tuned EKF2 or EKF3 maps onto `fusion-nav`. [GUIDE.md](GUIDE.md) is how to use the filter, and
[GLOSSARY.md](GLOSSARY.md#coming-from-px4-or-ardupilot) where the two estimators use a word differently.

A tuned EKF2 or EKF3 does not carry across by renaming. Most of what those estimators take as
parameters is here either a per-call argument, because it describes one measurement, or derived,
because the filter can find it. Read from source at PX4-Autopilot `c4e4ef98` and ardupilot
`368dc0c4`; PX4's firmware defaults are the ones in its `params_*.yaml`, which override the
initializers in `EKF/common.h`.

What is not a rename:

* **Measurement noise is an argument, and their "noise" parameters are floors.** `EKF2_GPS_P_NOISE`,
  `EKF2_GPS_V_NOISE`, `EK3_POSNE_M_NSE` and `EK3_VELNE_M_NSE` bound what the receiver reports
  (`eph`, `epv`, `sacc`); `PositionNoise::clamped` and `VelocityNoise::clamped` take the same
  bounds per call, `SigmaBounds::new(0.5, EKF2_NOAID_NOISE)` horizontally and 1.5 times the floor
  vertically for PX4. `EKF2_BARO_NOISE` (3.5 m on PX4 firmware) and `EK3_ALT_M_NSE` become each
  call's `AltitudeNoise`, and `EKF2_HEAD_NOISE` and `EK3_YAW_M_NSE` each call's `HeadingNoise`.
* **Gates are percentiles, theirs are σ multiples.** `Gates` holds a `Gate<DOF>` per source at a
  chi-square percentile (`P999` by default). A 1-D gate converts exactly, `Gate::<1>::new(k²)`:
  `EKF2_BARO_GATE` 5 is `new(25.0)`, `EKF2_HDG_GATE` 2.6 is `new(6.76)`, and ArduPilot's gates are
  in hundredths of σ, so `EK3_YAW_I_GATE` 300 is `new(9.0)`. Above one dimension they are not
  chi-square tests: PX4 tests each axis at `kσ` and rejects the source if any fails, which the
  ellipse `Gate::<N>::new(k²)` sits just inside, and ArduPilot tests `Σν² ≤ k²ΣS`, which is
  `new(N·k²)` at equal variances. Both platforms gate GNSS height apart from horizontal position,
  as `Gates::gnss_height` does, PX4 with `EKF2_GPS_P_GATE`. PX4's heading-mode magnetometer is
  gated per field component under `EKF2_MAG_GATE`, which has no exact counterpart.
* **Noise densities, not per-step σ.** `EKF2_GYR_NOISE`, `EKF2_ACC_NOISE` and the bias noises are
  σ per filter step; `ImuNoise` takes densities. Multiply by `√Δt` at `EKF2_PREDICT_US` (10 ms) or
  ArduPilot's 12 ms to reproduce them; `ImuNoise::default()`'s doc says why its white noise sits
  at ten times that.
* **A delay is the timestamp.** `SENS_GPS0_DELAY` (`EKF2_GPS_DELAY` before the rename),
  `EKF2_BARO_DELAY`, `EKF2_MAG_DELAY`, `GPS1_DELAY_MS` and `EK3_HGT_DELAY` are subtracted from a
  measurement's arrival time by the caller, and `time` carries the result. Past
  `LATENCY_HORIZON`, 0.3 s against `EKF2_DELAY_MAX`'s 200 ms, a measurement is `OutOfHorizon`.
* **The antenna offset is an argument.** `antenna` on each GNSS `fuse_*` is `SENS_GPS0_OFF*`
  (`EKF2_GPS_POS_*` before the rename) less `EKF2_IMU_POS*`, or `GPS1_POS_*` less `INS_POS1_*`.
  The estimate is the IMU's; neither PX4's output at the center of gravity nor its output predictor
  (`EKF2_TAU_VEL`, `EK3_TAU_OUTPUT`) has a counterpart, and `angular_rate()` moves the estimate to
  any other point. A dual-antenna heading's mounting angle, `EKF2_GPS_YAW_OFF` or the baseline
  `GPS1_MB_OFS_*`, stays the caller's subtraction: it is a constant angle, where an arm needs the
  filter's attitude and rate.
* **Declination is looked up.** `EKF2_DECL_TYPE` bit 0 and `COMPASS_AUTODEC` are the filter's
  default: a WMM table, read where the origin is placed. `EKF2_MAG_DECL` (degrees) and
  `COMPASS_DEC` (radians) are `set_magnetic_declination`, which the table never overrides. PX4
  re-reads its table every 10 s as the vehicle moves; this, like ArduPilot, reads it once.
* **Height has one absolute.** GNSS height is the reference and the barometer's offset is
  estimated, equation (30′), which is PX4's default `EKF2_HGT_REF` of GNSS without the choice.
  `baro_offset_walk` is PX4's `baro_bias_nsd`, 0.13, a constant there rather than a parameter.
* **Recovery constants are constants there too.** `Recovery`'s 7 s and 5 s are PX4's
  `reset_timeout_max` and `hgt_fusion_timeout_max`, and ArduPilot's `posRetryTime*` and
  `hgtRetryTime*` are the same shape; none is a parameter on either platform, and neither can
  turn recovery off, which `Recovery::OFF` does.
* **A long IMU interval is coasted.** PX4 and ArduPilot clamp a step to twice the expected period
  and lose the rest; `max_predict_dt` and `Config::coast` coast it on the estimated velocity and
  grow the covariance for the time that passed.

| `Config` | PX4 | ArduPilot |
| -------- | --- | --------- |
| `imu.gyro_white`, `imu.accel_white` | `EKF2_GYR_NOISE`, `EKF2_ACC_NOISE`, converted | `EK3_GYRO_P_NSE`, `EK3_ACC_P_NSE`, converted |
| `imu.gyro_bias_walk`, `imu.accel_bias_walk` | `EKF2_GYR_B_NOISE`, `EKF2_ACC_B_NOISE`, converted | `EK3_GBIAS_P_NSE`, `EK3_ABIAS_P_NSE`, converted; the defaults |
| `gates.*` | `EKF2_{GPS_P,GPS_V,BARO,MAG,HDG}_GATE`, as above | `EK3_{POS,VEL,HGT,YAW}_I_GATE`, as above |
| `timeouts.dead_reckoning_after` | `EKF2_NOAID_TOUT` | none; `deadReckonDeclare_ms`, a constant |
| `recovery.*` | none; constants | none; constants |
| `correlation.*` | none; neither models a source's error as correlated in time | none |
| `coast` | none | none |
| `hold` | `EKF2_NOAID_NOISE`; the fake position, fused as white | `EK3_NOAID_M_NSE`; `AID_NONE`'s synthetic position, fused as white |
| `init.sigma_tilt` | `EKF2_ANGERR_INIT` | none; 0.1 rad, a constant |
| `init.sigma_accel_bias`, `init.sigma_gyro_bias` | `EKF2_ABIAS_INIT`, `EKF2_GBIAS_INIT` | `EK3_ACC_BIAS_LIM` × 0.2; a per-sensor constant |
| `init.sigma_position`, `init.sigma_velocity` | the GNSS noise parameters, reused as a prior | the same |
| `init.min_duration`, `max_gyro_rate`, `max_accel_deviation` | none; tilt levels on low-passed readings | none; one sample after 1 s |
| `accuracy.position`, `accuracy.velocity` | `COM_POS_FS_EPH`, `COM_VEL_FS_EVH`, in commander, on a 2-D norm | `FS_EKF_THRESH`, a variance ratio |
| `accuracy.tilt`, `accuracy.heading`, `accuracy.horizon` | none | none |
| `max_predict_dt` | `EKF2_PREDICT_US`, whose clamp it replaces | none |
| `baro_offset_walk` | `baro_bias_nsd`, a constant | none |
| `baro_reference_from_estimate` | always on, no parameter | none |

What is left out:

* **Refused, each for a stated reason.** `EKF2_GPS_CHECK`, `EKF2_REQ_*` and `EK3_GPS_CHECK` are
  the application's, since the filter never sees satellites or dilution (and the first fix it is
  handed places the origin, so hand it one that passed). `EKF2_GYR_B_LIM`, `EKF2_ABL_LIM` and
  `EK3_ACC_BIAS_LIM`'s clamp saturate a state, which this filter does not do. `EKF2_*_CTRL`,
  `EKF2_SENS_EN` and `EK3_SRC*` select sources, which here is which `fuse_*` the caller calls.
  Magnetic-field states, airspeed, range, flow, wind, drag and multiple lanes are
  [non-goals](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#non-goals).
* **Candidates, each needing evidence first,** neither refused nor built: a magnetometer
  disturbance check (`EKF2_MAG_CHECK`,
  [#172](https://github.com/wboayue/fusion-nav/issues/172)) and bad vertical-accelerometer
  detection (PX4's `bad_acc_vertical`, ArduPilot's `badIMUdata`,
  [#174](https://github.com/wboayue/fusion-nav/issues/174)), reported through `Diagnostics`. PX4's
  `EKF2_POS_LOCK` is `fuse_stationary` here, called by the application rather than set.
* **Declined until a log shows need:** a barometer ground-effect dead zone (`EKF2_GND_EFF_DZ`,
  `EK3_GND_EFF_DZ`) and inhibiting accelerometer-bias learning under hard maneuvers
  (`EKF2_ABL_ACCLIM`). Each is a threshold on the airframe, a knob data could settle.
