#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyulog==1.2.4"]
# ///
"""Convert a PX4 ULog flight log into the fusion-nav replay CSV format.

    uv run tools/ulog2replay.py data/logs/flight.ulg -o data/logs/flight.csv

The output feeds `cargo run --example replay -- <csv>`. See `examples/replay.rs`
for the schema.

With --reference, EKF2's own solution and innovation test ratios are written to a
second file on the same timebase. That file is deliberately separate from the
replay input: the filter must never be fed the reference it is being compared
against, and an alignment bug then shows up as a visible offset rather than as
silently-wrong aiding.

Converters live outside the test path on purpose. Replaying ULog directly would
make the Rust test suite depend on PX4 tooling, which is exactly the
no-hardware-no-toolchain property the validation claim rests on (GOALS.md,
"Harness constraint"). This script runs once; its CSV output is what CI reads.

Topic and field names have changed repeatedly across PX4 releases, so each source
lists candidates newest first and the converter records what it actually used in
the output header. Verified against v1.16 and against a v1.5-era log.

Depends on pyulog, declared inline above (PEP 723) so that `uv run` resolves it
without a virtualenv to create or keep current. A tool run a few times a year is
exactly the one whose setup instructions rot; this way there are none.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

# WGS84, for the local tangent plane about the first GNSS fix.
WGS84_A = 6_378_137.0
WGS84_E2 = 6.694_379_990_141e-3

# PX4 publishes no barometer or magnetic-heading variance, so the converter
# supplies both. Neither production estimator ever receives one from a driver
# either -- both read a parameter -- which is what makes a constant here honest
# rather than a shortcut. Read at PX4-Autopilot c4e4ef98 (v1.18.0-beta1) and
# ardupilot 368dc0c4.
#
# Barometer, sigma = 2.0 m: what both platforms use for the common vehicle. PX4
# ekf2_baro_noise{2.0f} (EKF/common.h:346), squared at
# EKF/aid_sources/barometer/baro_height_control.cpp:62; ArduPilot _baroAltNoise,
# squared at AP_NavEKF3_PosVelFusion.cpp:1416, whose default is vehicle-dependent
# (AP_NavEKF3.cpp:20-151) -- 2.0 m for copter, Rover and the fallback, 3.0 m for
# Plane, 0.01 m for Sub. The multirotor number, not a universal one.
#
# Heading, sigma = 0.3 rad: PX4's ekf2_head_noise{3.0e-1f} (EKF/common.h:403),
# used at EKF/aid_sources/magnetometer/mag_control.cpp:603. ArduPilot sits looser
# again, YAW_M_NSE 0.5 rad (AP_NavEKF3.cpp:472), used unfloored on the compass
# path -- case yawFusionMethod::MAGNETOMETER: R_YAW = sq(frontend->_yawNoise) at
# AP_NavEKF3_MagFusion.cpp:975-978. PX4's is the tighter of the two and the
# corpus is PX4 logs, so it is the one taken: a heading claimed better than
# either production filter achieves would make the gate optimistic once gating
# is real.
DEFAULT_BARO_VARIANCE = 4.0  # m^2, sigma = 2.0 m
DEFAULT_MAG_VARIANCE = 0.09  # rad^2, sigma = 0.3 rad

# A *_timestamp_relative of INT32_MAX means "no sample in this message".
INVALID_RELATIVE = 2_147_483_647

GNSS_TOPICS = ["sensor_gps", "vehicle_gps_position"]

# Geodetic field spellings, newest first: (fields, lat/lon scale, alt scale).
# v1.14ish moved from scaled integers to plain floats and renamed everything.
# Ellipsoid height before MSL: the tangent plane is exact on the ellipsoid, and
# MSL misplaces it by the geoid undulation (see Geodetic in src/geodetic.rs).
GNSS_GEODETIC = [
    (("latitude_deg", "longitude_deg", "altitude_ellipsoid_m"), 1.0, 1.0),
    (("lat", "lon", "alt_ellipsoid"), 1e-7, 1e-3),
    (("latitude_deg", "longitude_deg", "altitude_msl_m"), 1.0, 1.0),
    (("lat", "lon", "alt"), 1e-7, 1e-3),
]

# Aggregate innovation ratios. PX4 renamed mag_test_ratio to hdg_test_ratio.
EKF2_RATIOS = [
    ("pos_test_ratio",),
    ("vel_test_ratio",),
    ("hgt_test_ratio",),
    ("hdg_test_ratio", "mag_test_ratio"),
]

# EKF2 lays its state vector out the same way in every version that logs one --
# quat[0:4], vel[4:7], pos[7:10], gyro bias[10:13], accel bias[13:16] -- so only
# the covariance indexing and the bias units move across releases.
EKF2_STATE_BG = 10
EKF2_STATE_BA = 13

# Where each quantity sits on the covariance diagonal, keyed on the pair
# (n_states, entries in `covariances`). Neither count alone separates the three
# eras: n_states=24 is both the state-indexed covariance and the first
# error-state one, and a 24-entry covariance is both the state-indexed one and
# the error-state one with terrain. The pair does, and no field name does.
# Read at PX4-Autopilot c4e4ef98 and v1.15.4.
#
# (25, 24) err24 -- error-state, mapped by State:: in
#   EKF/python/ekf_derivation/generated/state.h:44-54, and the bias states are
#   rates. Three commits got it there: 84b6b472b4 ("change delta angle and delta
#   velocity bias states to accel and gyro bias"), 0d6c2c8ce9 ("EKF2:
#   Error-State Kalman Filter"), and 68980b59e2, which added the terrain state
#   that makes the count 25 against a 24-entry covariance.
# (24, 23) err23 -- the same error-state order before the terrain state: v1.15,
#   whose msg/EstimatorStates.msg carries float32[23] covariances and whose
#   state.h puts quat_nominal, vel, pos, gyro_bias and accel_bias at 0, 3, 6, 9
#   and 12 with State::size 23. Mapped as err24 was, every v1.15 log failed on
#   `covariances[23]`.
# (24, 24) quat24 -- indexed like the state vector instead, so four quaternion
#   entries at [0..3] push vel to [4..6] and pos to [7..9], and the bias states
#   are a delta angle in rad and a delta velocity in m/s over one filter update.
#   `att` is None because a quaternion covariance becomes a rotation-vector one
#   only through the full 4x4 block, and the log carries the diagonal alone.
# Anything else is a different filter -- an LPE log reports n_states=10 -- and is
#   refused rather than mapped.
EKF2_LAYOUTS = {
    (25, 24): {"name": "err24", "att": 0, "vel": 3, "pos": 6, "bg": 9, "ba": 12,
               "bias_is_rate": True},
    (24, 23): {"name": "err23", "att": 0, "vel": 3, "pos": 6, "bg": 9, "ba": 12,
               "bias_is_rate": True},
    (24, 24): {"name": "quat24", "att": None, "vel": 4, "pos": 7, "bg": 10, "ba": 13,
               "bias_is_rate": False},
}

REFERENCE_TOPICS = [
    "vehicle_local_position",
    "vehicle_attitude",
    "estimator_states",
    "estimator_status",
    "vehicle_status",
    "vtol_vehicle_status",
]


class ConversionError(Exception):
    pass


def geodetic_to_ned(lat, lon, alt, lat0, lon0, alt0):
    """Local tangent plane about (lat0, lon0, alt0). Degrees in, meters out.

    Exact, the same conversion as `LocalOrigin::to_ned` in src/geodetic.rs, so the
    replay corpus and the filter agree about where a fix is: both points to
    Earth-centered coordinates, then the difference rotated onto the origin's
    north, east and down.
    """
    x, y, z = ecef(lat, lon, alt)
    x0, y0, z0 = ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    phi, lam = math.radians(lat0), math.radians(lon0)
    sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
    north = -sp * cl * dx - sp * sl * dy + cp * dz
    east = -sl * dx + cl * dy
    down = -cp * cl * dx - cp * sl * dy - sp * dz
    return north, east, down


def ecef(lat, lon, alt):
    """Earth-centered, Earth-fixed meters of a WGS84 position in degrees."""
    phi, lam = math.radians(lat), math.radians(lon)
    s = math.sin(phi)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    return (
        (n + alt) * math.cos(phi) * math.cos(lam),
        (n + alt) * math.cos(phi) * math.sin(lam),
        (n * (1.0 - WGS84_E2) + alt) * s,
    )


def median(values):
    """Median of a sequence, or None when it is empty."""
    ordered = sorted(values)
    if not ordered:
        return None
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def pick(ulog, names):
    """First available dataset among `names`, or None."""
    available = {d.name for d in ulog.data_list}
    for name in names:
        if name in available:
            return ulog.get_dataset(name)
    return None


def column(dataset, name, index=None):
    key = name if index is None else f"{name}[{index}]"
    if key not in dataset.data:
        raise ConversionError(f"`{dataset.name}` has no field `{key}`")
    return dataset.data[key]


def stamps(dataset):
    """Timestamps as signed Python ints.

    ULog timestamps are uint64. Subtracting a later origin from an earlier sample
    wraps to ~1.8e19 instead of going negative, which silently produced garbage
    times in the reference file before this cast was added.
    """
    return [int(v) for v in dataset.data["timestamp"]]


def fields_for(dataset, candidates):
    """First candidate tuple whose fields all exist, or None."""
    for entry in candidates:
        names = entry[0] if isinstance(entry, tuple) and isinstance(entry[0], tuple) else entry
        if all(n in dataset.data for n in names):
            return entry
    return None


def convert_imu(ulog, rows, used):
    dataset = pick(ulog, ["sensor_combined"])
    if dataset is None:
        if pick(ulog, ["vehicle_imu"]) is not None:
            raise ConversionError(
                "only `vehicle_imu` is present, which publishes integrated "
                "delta-angle and delta-velocity. The replay schema carries rates "
                "and specific force; re-log with `sensor_combined` enabled."
            )
        raise ConversionError("no `sensor_combined` topic: nothing to propagate")
    used["imu"] = dataset.name
    t = stamps(dataset)
    gyro = [column(dataset, "gyro_rad", i) for i in range(3)]
    accel = [column(dataset, "accelerometer_m_s2", i) for i in range(3)]
    for k in range(len(t)):
        rows.append((t[k], "imu", [g[k] for g in gyro] + [a[k] for a in accel], []))
    return dataset


def is_3d_fix(fix, k):
    """Whether sample `k` is a 3D fix. A receiver logging no `fix_type` is taken at its word."""
    return fix is None or fix[k] >= 3


def convert_gnss(ulog, rows, used):
    dataset = pick(ulog, GNSS_TOPICS)
    if dataset is None:
        print("warning: no GNSS topic; position and velocity aiding omitted", file=sys.stderr)
        return
    # Some drivers declare the ellipsoid field and never fill it; an all-zero height
    # column would flatten every fix onto the origin's height. Skip to the MSL one.
    filled = [c for c in GNSS_GEODETIC if c[0][2] not in dataset.data or dataset.data[c[0][2]].any()]
    geodetic = fields_for(dataset, filled) or fields_for(dataset, GNSS_GEODETIC)
    if geodetic is None:
        raise ConversionError(
            f"`{dataset.name}` has no recognised geodetic fields; looked for "
            + " and ".join("/".join(c[0]) for c in GNSS_GEODETIC)
        )
    (lat_f, lon_f, alt_f), angle_scale, alt_scale = geodetic
    before = len(rows)

    t = stamps(dataset)
    lat = column(dataset, lat_f)
    lon = column(dataset, lon_f)
    alt = column(dataset, alt_f)
    fix = dataset.data.get("fix_type")
    eph = column(dataset, "eph")
    epv = column(dataset, "epv")

    has_velocity = all(
        f in dataset.data for f in ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s", "s_variance_m_s")
    )
    if has_velocity:
        vn, ve, vd = (column(dataset, f) for f in ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s"))
        speed_sigma = column(dataset, "s_variance_m_s")
        valid = dataset.data.get("vel_ned_valid")
    else:
        print("warning: GNSS topic has no NED velocity; velocity aiding omitted", file=sys.stderr)

    origin = None
    for k in range(len(t)):
        if not is_3d_fix(fix, k):
            continue
        phi = float(lat[k]) * angle_scale
        lam = float(lon[k]) * angle_scale
        height = float(alt[k]) * alt_scale
        if origin is None:
            origin = (phi, lam, height)
            print(
                f"navigation origin: {phi:.7f}, {lam:.7f}, {height:.2f} m",
                file=sys.stderr,
            )
        north, east, down = geodetic_to_ned(phi, lam, height, *origin)
        # eph/epv are standard deviations; the filter wants variances. Passed through
        # unfloored, deliberately: this file reproduces what the receiver reported, and
        # PX4's own max(eph, EKF2_GPS_P_NOISE) is a fusion-time decision the consumer
        # makes. See Eskf::fuse_gnss_position.
        horizontal = float(eph[k]) ** 2
        vertical = float(epv[k]) ** 2
        rows.append((t[k], "gnss_pos", [north, east, down], [horizontal, horizontal, vertical]))

        if has_velocity and (valid is None or valid[k]):
            variance = float(speed_sigma[k]) ** 2
            rows.append(
                (t[k], "gnss_vel", [vn[k], ve[k], vd[k]], [variance, variance, variance])
            )

    if origin is None:
        print("warning: no 3D GNSS fix in the log", file=sys.stderr)
    tally(rows, before, used, "gnss", f"{dataset.name} ({lat_f})")


def from_sensor_combined(dataset, value_fields, relative_field, rows, source, variances):
    """Older PX4 carried baro and mag inside sensor_combined.

    The values repeat at the IMU rate, so *_timestamp_relative is both the true
    sample time and the dedupe key.
    """
    t = stamps(dataset)
    relative = dataset.data.get(relative_field)
    columns = [column(dataset, f) for f in value_fields]
    previous = None
    for k in range(len(t)):
        if relative is not None:
            offset = int(relative[k])
            if offset == INVALID_RELATIVE:
                continue
            stamp = t[k] + offset
        else:
            stamp = t[k]
        if stamp == previous:
            continue
        previous = stamp
        rows.append((stamp, source, [c[k] for c in columns], list(variances)))


def tally(rows, before, used, key, topic):
    """Record a source only if it actually produced rows."""
    if len(rows) > before:
        used[key] = topic
        return True
    print(f"warning: `{topic}` carried no usable {key} samples", file=sys.stderr)
    return False


def convert_baro(ulog, rows, used, variance, sensor_combined):
    before = len(rows)
    dataset = pick(ulog, ["vehicle_air_data"])
    if dataset is not None:
        t = stamps(dataset)
        altitude = column(dataset, "baro_alt_meter")
        for k in range(len(t)):
            rows.append((t[k], "baro", [altitude[k]], [variance]))
        tally(rows, before, used, "baro", dataset.name)
        return
    if sensor_combined is not None and "baro_alt_meter" in sensor_combined.data:
        from_sensor_combined(
            sensor_combined, ("baro_alt_meter",), "baro_timestamp_relative",
            rows, "baro", (variance,),
        )
        tally(rows, before, used, "baro", "sensor_combined")
        return
    print("warning: no barometer topic; altitude aiding omitted", file=sys.stderr)


def convert_mag(ulog, rows, used, variance, sensor_combined):
    before = len(rows)
    dataset = pick(ulog, ["vehicle_magnetometer"])
    if dataset is not None:
        t = stamps(dataset)
        components = [column(dataset, "magnetometer_ga", i) for i in range(3)]
        for k in range(len(t)):
            rows.append((t[k], "mag", [c[k] for c in components], [variance]))
        tally(rows, before, used, "mag", dataset.name)
        return
    if sensor_combined is not None and "magnetometer_ga[0]" in sensor_combined.data:
        from_sensor_combined(
            sensor_combined,
            tuple(f"magnetometer_ga[{i}]" for i in range(3)),
            "magnetometer_timestamp_relative",
            rows, "mag", (variance,),
        )
        tally(rows, before, used, "mag", "sensor_combined")
        return
    dataset = pick(ulog, ["sensor_mag"])
    if dataset is not None:
        t = stamps(dataset)
        components = [column(dataset, axis) for axis in ("x", "y", "z")]
        for k in range(len(t)):
            rows.append((t[k], "mag", [c[k] for c in components], [variance]))
        tally(rows, before, used, "mag", dataset.name)
        return
    print("warning: no magnetometer topic; heading aiding omitted", file=sys.stderr)


def dropouts(ulog):
    """Report logging dropouts, which are not sensor dropouts.

    A gap in the output means the SD card missed those messages, not that the
    sensor stopped. Replaying one as a single long `dt` would propagate as though
    the vehicle really had dead-reckoned through it.
    """
    if not ulog.dropouts:
        return None
    total = sum(d.duration for d in ulog.dropouts)
    note = (
        f"{len(ulog.dropouts)} logging dropouts totalling {total} ms; "
        "gaps in this file are missing log data, not stopped sensors"
    )
    print(f"warning: {note}", file=sys.stderr)
    return note


def pinned_pyulog():
    """The version the PEP 723 header above declares, or None if it cannot be read.

    Read back out of this file rather than repeated as a constant. The header is the
    pin -- `uv run` resolves it and `data/fetch.sh` parses the same line -- and a
    second copy is what would go stale, which is the whole failure this function is
    here to avoid printing.
    """
    try:
        text = Path(__file__).read_text()
    except OSError:
        return None
    match = re.search(r'^# dependencies = \["pyulog==([^"]+)"\]', text, re.M)
    return match.group(1) if match else None


def open_ulog(path, topics=None):
    """Parse a ULog, or say how to get the pinned pyulog that parses it."""
    try:
        from pyulog import ULog
    except ImportError:
        pin = pinned_pyulog()
        remedy = (
            f"Or install that same version here: uv venv && uv pip install pyulog=={pin}"
            if pin
            else "The pinned version could not be read back out of this file's PEP 723 "
            "header, so it is no longer where `data/fetch.sh` looks for it either."
        )
        raise ConversionError(
            "pyulog is not available to this interpreter. Run this script through uv,\n"
            "which resolves the pinned version with no virtualenv to keep current:\n"
            "  uv run tools/ulog2replay.py ...\n"
            f"{remedy}"
        ) from None
    return ULog(str(path), topics)


def convert(path, baro_variance, mag_variance):
    ulog = open_ulog(path)
    rows = []
    used = {}
    note = dropouts(ulog)
    sensor_combined = convert_imu(ulog, rows, used)
    convert_gnss(ulog, rows, used)
    convert_baro(ulog, rows, used, baro_variance, sensor_combined)
    convert_mag(ulog, rows, used, mag_variance, sensor_combined)
    # The IMU sample interval, for --reference's bias scaling only. Taken here
    # because this is where `sensor_combined` is already open, and nothing in the
    # replay output depends on it.
    t = stamps(sensor_combined)
    imu_dt = median([(b - a) * 1e-6 for a, b in zip(t, t[1:]) if b > a])
    return rows, used, note, imu_dt


def write_rows(rows, out, note):
    """Sort by time, rebase to the first sample, and write the replay schema."""
    rows.sort(key=lambda r: r[0])
    if not rows:
        raise ConversionError("nothing to write")
    t0 = rows[0][0]

    with open(out, "w", newline="") as handle:
        for line in note:
            handle.write(f"# {line}\n")
        handle.write("t_s,source,v0,v1,v2,v3,v4,v5,var0,var1,var2\n")
        for timestamp, source, values, variances in rows:
            t = (timestamp - t0) * 1e-6  # ULog timestamps are microseconds
            values = list(values) + [None] * (6 - len(values))
            variances = list(variances) + [None] * (3 - len(variances))
            cells = [f"{t:.6f}", source]
            cells += ["" if v is None else f"{v:.6g}" for v in values + variances]
            handle.write(",".join(cells) + "\n")
    return len(rows), t0


def ekf2_update_period(ulog, imu_dt):
    """EKF2's mean filter update period, and a sentence saying where it came from.

    Wanted only to turn a pre-84b6b472b4 delta-angle/delta-velocity bias into a
    rate, and it has to be the quantity PX4 divides by, which is `_dt_ekf_avg`:
    `getGyroBias() { return _state.delta_ang_bias / _dt_ekf_avg; }` with the
    variance over `sq(_dt_ekf_avg)` (EKF/ekf.h:239-244 at ae3070bbf1^).
    `_dt_ekf_avg` is a running mean of the realized step, seeded at the target
    (`0.99f * _dt_ekf_avg + 0.01f * input`, EKF/ekf.cpp:289).

    So it is a **mean**, not an integer multiple of the IMU interval, and the
    down-sampler is built to hold that mean: it fires when the accumulated
    `delta_ang_dt` reaches `_target_dt - _imu_collection_time_adj`, then moves
    the adjustment by `0.01f * (delta_ang_dt - _target_dt)` -- a feedback term
    whose comment says it is there "so that we meet the average EKF update rate
    requirement" (EKF/imu_down_sampler.cpp:36-43 at ae3070bbf1^). On a 250 Hz
    IMU against a 10 ms target it alternates two-sample and three-sample steps
    and averages 10 ms; it does not settle at 12.

    Hence `max(target, imu_dt)`: the loop holds the mean at the target while a
    sample is shorter than it, and nothing can subdivide a sample longer than
    it. The second case is real -- a 50 Hz log runs a 20 ms period against a
    10 ms target -- and so is the first: rounding a 4 ms IMU up to 12 ms scales
    every bias and bias sigma in that log 20 % low.

    The topic's own publication interval is no use for any of this: on two
    corpus logs `estimator_status` publishes at 5 Hz and on a third
    `estimator_states` publishes at 1 Hz.
    """
    micros = ulog.initial_parameters.get("EKF2_PREDICT_US")
    if micros:
        target = float(micros) * 1e-6
        provenance = f"EKF2_PREDICT_US {int(micros)} us"
    else:
        # FILTER_UPDATE_PERIOD_MS{10} at EKF/estimator_interface.h:267, read at
        # ae3070bbf1^ -- the commit that replaced the constant with the
        # parameter. Every pre-2022 log predates it and carries no parameter.
        target = 0.010
        provenance = "no EKF2_PREDICT_US; FILTER_UPDATE_PERIOD_MS 10 ms"
    if not imu_dt or imu_dt <= 0:
        return target, provenance
    period = max(target, imu_dt)
    held = "the mean the down-sampler holds" if period == target else "one IMU sample, longer than the target"
    return period, f"{provenance}; {held}, against a {imu_dt * 1e3:.3f} ms IMU interval"


# One row per source topic, since EKF2 publishes these at three different rates
# and resampling them onto one grid would be interpolation, which is a statistic
# and not a converter's business. Union schema with blank cells, the way the
# replay input handles sources of different arities.
REFERENCE_COLUMNS = [
    "pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d",
    "q0", "q1", "q2", "q3", "att_reset",
    "ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z",
    "sigma_pos_n", "sigma_pos_e", "sigma_pos_d",
    "sigma_vel_n", "sigma_vel_e", "sigma_vel_d",
    "sigma_att_n", "sigma_att_e", "sigma_att_d", "sigma_att_total",
    "sigma_ba_x", "sigma_ba_y", "sigma_ba_z",
    "sigma_bg_x", "sigma_bg_y", "sigma_bg_z",
    "r_gnss_pos", "r_gnss_vel", "r_baro", "r_mag",
    "mode",
]


def reference_local(local, rows):
    """EKF2's position and velocity, from `vehicle_local_position`."""
    t = stamps(local)
    names = ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d"]
    columns = [column(local, f) for f in ("x", "y", "z", "vx", "vy", "vz")]
    for k in range(len(t)):
        rows.append((t[k], "ekf2_local", {n: c[k] for n, c in zip(names, columns)}))


def reference_attitude(attitude, rows):
    """EKF2's attitude, from `vehicle_attitude`, as the quaternion it logs.

    Written through unconverted: `vehicle_attitude.q` is Hamilton, scalar-first,
    body FRD to NED (`msg/versioned/VehicleAttitude.msg:2,10` at PX4 c4e4ef98),
    the convention the replay output's `q0..q3` carry, so the two diff by name.
    Euler angles are what `tools/replay_report.py` does not draw, because ZYX
    cannot separate roll from yaw at 90 deg of pitch.

    `att_reset` carries `quat_reset_counter` where the topic has it, because
    without it a reset reads as divergence.
    """
    t = stamps(attitude)
    q = [column(attitude, "q", i) for i in range(4)]
    resets = attitude.data.get("quat_reset_counter")
    for k in range(len(t)):
        values = {f"q{i}": q[i][k] for i in range(4)}
        if resets is not None:
            values["att_reset"] = resets[k]
        rows.append((t[k], "ekf2_att", values))


def reference_states(states, layout, period, rows):
    """EKF2's biases and covariance diagonal, from whichever topic carries them.

    Biases are scaled to rates on the era that logs deltas, so the column means
    the same thing on every log. Sigmas are standard deviations, matching the
    replay output's.

    The attitude ones are in **NED**, and the name says so because the frames do
    not match. PX4 stores the error-state attitude covariance in the navigation
    frame -- `getRotVarNed` returns the diagonal as stored while `getRotVarBody`
    rotates it by `R_to_earth' (.) R_to_earth` (EKF/ekf_helper.cpp:926-937 at
    c4e4ef98) -- while this crate's `delta_theta` is a local body-frame
    perturbation, equation (36) giving the navigation-frame error as
    `R(q) delta_theta`. Rotating either diagonal into the other frame needs the
    off-diagonals no log carries, so `sigma_att_total`, the square root of the
    trace, is emitted beside them: a trace is invariant under rotation, which
    makes it the one attitude scalar comparable to the replay output's
    `sigma_att_x/y/z` without an off-diagonal on either side.

    Not emitted, deliberately: a tilt or yaw sigma. PX4's `getTiltVariance` sums
    two NED variances where (36') takes the larger of two body-frame ones, and
    its `getYawVar` has no counterpart in the replay output, which publishes the
    body diagonal only. Naming either of those pairs alike would be a comparison
    of two different quantities.
    """
    t = stamps(states)
    bias = {
        "ba": [column(states, "states", EKF2_STATE_BA + i) for i in range(3)],
        "bg": [column(states, "states", EKF2_STATE_BG + i) for i in range(3)],
    }
    entries = sum(1 for name in states.data if name.startswith("covariances["))
    cov = [column(states, "covariances", i) for i in range(entries)]
    scale = 1.0 if layout["bias_is_rate"] else 1.0 / period

    for k in range(len(t)):
        values = {}
        for kind, axes in (("ba", "xyz"), ("bg", "xyz")):
            for i, axis in enumerate(axes):
                values[f"{kind}_{axis}"] = bias[kind][i][k] * scale
                variance = cov[layout[kind] + i][k]
                values[f"sigma_{kind}_{axis}"] = math.sqrt(max(0.0, variance)) * scale
        for kind, axes in (("pos", "ned"), ("vel", "ned")):
            for i, axis in enumerate(axes):
                variance = cov[layout[kind] + i][k]
                values[f"sigma_{kind}_{axis}"] = math.sqrt(max(0.0, variance))
        if layout["att"] is not None:
            att = [max(0.0, cov[layout["att"] + i][k]) for i in range(3)]
            for axis, variance in zip("ned", att):
                values[f"sigma_att_{axis}"] = math.sqrt(variance)
            values["sigma_att_total"] = math.sqrt(sum(att))
        rows.append((t[k], "ekf2_states", values))


def reference_ratios(status, ratio_fields, rows):
    """EKF2's aggregate innovation test ratios, from `estimator_status`."""
    t = stamps(status)
    names = ["r_gnss_pos", "r_gnss_vel", "r_baro", "r_mag"]
    columns = [column(status, f) if f else None for f in ratio_fields]
    for k in range(len(t)):
        rows.append((
            t[k],
            "ekf2_ratio",
            {n: c[k] for n, c in zip(names, columns) if c is not None},
        ))


# The vehicle's flight regime, for shading the report and nothing else. PX4 tells
# EKF2 whether it is flying fixed-wing or transitioning (`flags.is_fixed_wing`,
# `flags.in_transition`, src/modules/ekf2/EKF2.cpp:2830-2831 at c4e4ef98); this
# filter is told nothing, and a reader of a VTOL log needs to see where the
# regime changed to judge whether that matters.
#
# `vehicle_status.vehicle_type` is never read. Its constants were renumbered
# (7cb6464cfb: rotary wing 0, fixed wing 1) and put back (a150fc05af, 50626f6848:
# 1 and 2) under one field name, and ULog records names, not constants: 3949f175
# logs a quadrotor as 0. What is read instead is fixed outside PX4 -- MAVLink's
# MAV_TYPE (`vehicle_status.system_type`) and MAV_VTOL_STATE
# (`vtol_vehicle_status.vehicle_vtol_state`, msg/versioned/VtolVehicleStatus.msg:1)
# -- or told apart by field name, as the three bools that preceded the VTOL state
# in 2b7efeacca are.

# MAV_VTOL_STATE, which has not moved since v1.10.
VTOL_STATES = {0: "undefined", 1: "to_fw", 2: "to_mc", 3: "mc", 4: "fw"}

# MAV_TYPE for a vehicle that is not a VTOL: one regime for the whole log.
# Anything else -- a rover, a boat, a value outside MAVLink's enum such as
# a299e722's 202 -- is `other` rather than a guess.
MAV_TYPE_MC = {2, 3, 4, 13, 14, 15, 29, 35}  # quad, coax, heli, hexa, octo, tri, dodeca, deca
MAV_TYPE_FW = {1}


def classify_mode(is_vtol, system_type, vtol_state=None, legacy=None):
    """The regime one status sample describes.

    `vtol_state` is MAV_VTOL_STATE; `legacy` is the pre-2b7efeacca triple
    `(vtol_in_rw_mode, vtol_in_trans_mode, in_transition_to_fw)`. A VTOL with
    neither logged has no regime to report, which is `undefined`.
    """
    if not is_vtol:
        if system_type in MAV_TYPE_MC:
            return "mc"
        if system_type in MAV_TYPE_FW:
            return "fw"
        return "other"
    if vtol_state is not None:
        return VTOL_STATES.get(vtol_state, "undefined")
    if legacy is not None:
        rotary, transition, to_fw = legacy
        if transition:
            return "to_fw" if to_fw else "to_mc"
        return "mc" if rotary else "fw"
    return "undefined"


def reference_mode(status, vtol, rows):
    """One `vehicle_mode` row per change of regime.

    A vehicle that is not a VTOL has one regime for the whole log, so one row, at
    the first status sample. A VTOL takes its regime from `vtol_vehicle_status`,
    whichever era of that topic the log carries.
    """
    if status is None or "is_vtol" not in status.data or "system_type" not in status.data:
        return None
    t = stamps(status)
    system_type = int(status.data["system_type"][0])
    if not any(status.data["is_vtol"]):
        rows.append((t[0], "vehicle_mode", {"mode": classify_mode(False, system_type)}))
        return f"vehicle_status.system_type (MAV_TYPE {system_type})"

    samples = []
    if vtol is not None and "vehicle_vtol_state" in vtol.data:
        states = vtol.data["vehicle_vtol_state"]
        for k, when in enumerate(stamps(vtol)):
            samples.append((when, classify_mode(True, system_type, vtol_state=int(states[k]))))
        provenance = "vtol_vehicle_status.vehicle_vtol_state"
    elif vtol is not None and "vtol_in_rw_mode" in vtol.data:
        legacy = [vtol.data[f] for f in
                  ("vtol_in_rw_mode", "vtol_in_trans_mode", "in_transition_to_fw")]
        for k, when in enumerate(stamps(vtol)):
            triple = tuple(bool(c[k]) for c in legacy)
            samples.append((when, classify_mode(True, system_type, legacy=triple)))
        provenance = "vtol_vehicle_status.vtol_in_rw_mode/vtol_in_trans_mode"
    else:
        samples.append((t[0], classify_mode(True, system_type)))
        provenance = "is_vtol with no vtol_vehicle_status"

    previous = None
    for when, mode in samples:
        if mode != previous:
            rows.append((when, "vehicle_mode", {"mode": mode}))
            previous = mode
    return provenance


def ekf2_states(ulog):
    """The topic carrying EKF2's state vector and its layout key, or `(None, None)`.

    Newer builds log `estimator_states`; older ones put the same fields in
    `estimator_status`. The key is `(n_states, covariance entries)`, which is what
    `EKF2_LAYOUTS` is indexed by.
    """
    states = pick(ulog, ["estimator_states", "estimator_status"])
    if states is None or "states[0]" not in states.data:
        return None, None
    entries = sum(1 for name in states.data if name.startswith("covariances["))
    return states, (int(states.data["n_states"][0]), entries)


def layout_label(key):
    """How `--screen` and the reference header name a layout key."""
    if key is None:
        return "none"
    layout = EKF2_LAYOUTS.get(key)
    return layout["name"] if layout is not None else "unmapped"


def write_reference(ulog, out, t0, imu_dt, source_name):
    """EKF2's own solution and innovation ratios, for a side-by-side diff."""
    local = pick(ulog, ["vehicle_local_position"])
    attitude = pick(ulog, ["vehicle_attitude"])
    status = pick(ulog, ["estimator_status"])
    states, key = ekf2_states(ulog)
    vehicle = pick(ulog, ["vehicle_status"])
    vtol = pick(ulog, ["vtol_vehicle_status"])
    if local is None and status is None and attitude is None:
        print("warning: no EKF2 topics; reference not written", file=sys.stderr)
        return False

    ratio_fields = None
    if status is not None:
        ratio_fields = [next((n for n in c if n in status.data), None) for c in EKF2_RATIOS]
        if any(f is None for f in ratio_fields):
            print(
                "warning: `estimator_status` is missing some test ratios; "
                f"found {[f for f in ratio_fields if f]}",
                file=sys.stderr,
            )

    layout, period, provenance = None, None, None
    if key is not None:
        layout = EKF2_LAYOUTS.get(key)
        if layout is None:
            print(
                f"warning: `{states.name}` reports n_states={key[0]} with {key[1]} "
                "covariance entries, which is none of the EKF2 layouts; states and "
                "covariance omitted",
                file=sys.stderr,
            )
        else:
            period, provenance = ekf2_update_period(ulog, imu_dt)

    rows = []
    if local is not None:
        reference_local(local, rows)
    if attitude is not None:
        reference_attitude(attitude, rows)
    if layout is not None:
        reference_states(states, layout, period, rows)
    if status is not None and ratio_fields:
        reference_ratios(status, ratio_fields, rows)
    mode = reference_mode(vehicle, vtol, rows)
    rows.sort(key=lambda r: r[0])

    with open(out, "w", newline="") as handle:
        for line in reference_note(local, attitude, states, layout, key,
                                   period, provenance, ratio_fields, mode, source_name):
            handle.write(f"# {line}\n")
        handle.write("t_s,source," + ",".join(REFERENCE_COLUMNS) + "\n")
        for timestamp, source, values in rows:
            cells = [f"{(timestamp - t0) * 1e-6:.6f}", source]
            for name in REFERENCE_COLUMNS:
                value = values.get(name)
                if value is None:
                    cells.append("")
                elif isinstance(value, str):
                    cells.append(value)
                else:
                    cells.append(f"{value:.6g}")
            handle.write(",".join(cells) + "\n")
    return True


def reference_note(local, attitude, states, layout, key, period, provenance,
                   ratio_fields, mode, source_name):
    """The `#` header: what each row kind came from, and every caveat on it."""
    kinds = []
    if local is not None:
        kinds.append("ekf2_local=vehicle_local_position")
    if attitude is not None:
        kinds.append("ekf2_att=vehicle_attitude")
    if layout is not None:
        kinds.append(f"ekf2_states={states.name}")
    if ratio_fields:
        kinds.append("ekf2_ratio=estimator_status")
    if mode:
        kinds.append(f"vehicle_mode={mode}")

    note = [
        "EKF2's own solution, for comparison. Never filter input.",
        # Names the log, so a consumer pairing this with a replay can refuse a
        # reference from a different flight. The replay input's own header says
        # `Converted from <name>.ulg`, and the two have to agree.
        f"Converted from {source_name} by tools/ulog2replay.py",
        "Rows: " + ", ".join(kinds),
    ]
    if ratio_fields:
        note.append("Ratio fields: " + ", ".join(f or "(missing)" for f in ratio_fields))
    if layout is None:
        if key is not None:
            note.append(f"No states or covariance: n_states={key[0]} with {key[1]} "
                        "covariance entries is not an EKF2 layout.")
    else:
        note.append(f"EKF2 layout: {layout['name']}, n_states={key[0]} with {key[1]} "
                    "covariance entries.")
        if layout["bias_is_rate"]:
            note.append("Bias states are rates as logged; nothing scaled.")
        else:
            note.append(
                f"Bias states are delta-angle/delta-velocity, scaled to rates by a "
                f"{period:.4f} s filter update period ({provenance})."
            )
        if layout["att"] is None:
            note.append(
                "sigma_att_* are blank: this era logs a quaternion covariance "
                "diagonal, and four entries become a rotation-vector sigma only "
                "through the full 4x4 block."
            )
        else:
            note.append(
                "sigma_att_n/e/d are NED, the frame PX4 stores the error-state "
                "attitude covariance in; the replay output's sigma_att_x/y/z are "
                "body-frame, and rotating either needs off-diagonals no log "
                "carries. Compare sigma_att_total, a trace being invariant."
            )
    note.append(ekf2_origin_note(local))
    note.append("Same timebase as the replay CSV: rebased to its first sample.")
    return note


def ekf2_origin_note(local):
    """EKF2's local-position origin, which #8 needs before it can align tracks.

    A `#` line rather than a column because it does not move: it is one geodetic
    point per log, and the replay harness already parses a `#` header for the
    truth file's scenario and seed. Two corpus logs report `xy_global` false with
    the reference fields all zero, so their x/y/z are origin-relative with no
    origin -- which is a fact about those logs and has to be said, not filled in.
    """
    if local is None:
        return "EKF2 origin: none (no vehicle_local_position)"
    if "xy_global" not in local.data or "ref_lat" not in local.data:
        return "EKF2 origin: none (vehicle_local_position carries no reference fields)"
    if not any(local.data["xy_global"]):
        return "EKF2 origin: none (xy_global false)"
    index = next(k for k, flag in enumerate(local.data["xy_global"]) if flag)
    lat = local.data["ref_lat"][index]
    lon = local.data["ref_lon"][index]
    alt = local.data["ref_alt"][index]
    return f"EKF2 origin: {lat:.9g} {lon:.9g} {alt:.9g}"


SCREEN_TOPICS = GNSS_TOPICS + [
    "sensor_combined",
    "vehicle_imu",
    "vehicle_imu_status",
    "estimator_states",
    "estimator_status",
    "vehicle_status",
    "vtol_vehicle_status",
]


def release(encoded):
    """`ver_sw_release` as `vMAJOR.MINOR.PATCH`, suffixed unless a release, or `none`.

    PX4 packs it as 0xMMmmpptt, the low byte a firmware type: dev from 0, alpha from 64,
    beta from 128, rc from 192, release at 255 (`FIRMWARE_TYPE` in
    src/lib/version/version.c:49-58 at PX4-Autopilot c4e4ef98e9). Dropping it is how
    `3949f175`, an rc build, reads as v1.16.0.
    """
    if not encoded:
        return "none"
    kind = encoded & 0xFF
    suffix = ("" if kind == 255 else "-rc" if kind >= 192 else "-beta" if kind >= 128
              else "-alpha" if kind >= 64 else "-dev")
    return f"v{(encoded >> 24) & 0xFF}.{(encoded >> 16) & 0xFF}.{(encoded >> 8) & 0xFF}{suffix}"


def vibration_metric(encoded):
    """Which quantity `accel_vibration_metric` is on this build: `dv`, `accel` or `unknown`.

    PX4 f2ae8ae814 changed it, under the same field name, from a filtered
    |dv - dv_prev| over one integration interval, in m/s, to |a - a_prev| over raw
    samples, in m/s^2. So 0.094 on `2c42096b` (v1.11.3) and 32 on a v1.15 log are
    not a quiet airframe and a loud one. No rescaling makes them one quantity --
    the old metric differences averages, which is where vibration is filtered out
    -- so the screen names the metric rather than converting it. The commit is on
    master between v1.12 and v1.13.0-beta1: a v1.12 dev build could be either.
    `estimator_status.vibe[2]` on older builds is the delta-velocity one.
    """
    if not encoded:
        return "unknown"
    major, minor = (encoded >> 24) & 0xFF, (encoded >> 16) & 0xFF
    if (major, minor) >= (1, 13):
        return "accel"
    if (major, minor) == (1, 12) and encoded & 0xFF != 255:
        return "unknown"
    return "dv"


def screen_gnss(dataset):
    """What a receiver reports about itself, over the samples it calls a 3D fix.

    A constant `eph` and satellite count is the tell of a simulated receiver: a
    real one's accuracy moves with the sky it sees.
    """
    keys = ("gnss", "fix_max", "eph_min", "eph_max", "sats_min", "sats_max")
    if dataset is None:
        return dict.fromkeys(keys, "none")
    fix = dataset.data.get("fix_type")
    sats = dataset.data.get("satellites_used")
    eph = dataset.data.get("eph")
    fixed = [k for k in range(len(dataset.data["timestamp"])) if is_3d_fix(fix, k)]

    def span(values, form):
        if values is None or not fixed:
            return "none", "none"
        chosen = [float(values[k]) for k in fixed]
        return form(min(chosen)), form(max(chosen))

    eph_min, eph_max = span(eph, lambda v: f"{v:.2f}")
    sats_min, sats_max = span(sats, lambda v: f"{v:.0f}")
    return dict(zip(keys, (
        dataset.name,
        "none" if fix is None else f"{max(int(f) for f in fix)}",
        eph_min, eph_max, sats_min, sats_max,
    )))


def screen_vibration(ulog):
    """Clipped accelerometer samples, and the 95th percentile of the vibration metric.

    A percentile, not the maximum: touchdown alone reads 0.55 on `3949f175`, a
    simulation whose median is 0.008. `vehicle_imu_status` carries one metric per
    IMU instance, pooled here; older builds put the accelerometer's in
    `estimator_status.vibe[2]` instead. A metric that is zero throughout was never
    computed -- LPE's `7592c9b2` logs the field and fills it with nothing -- so it
    reads `none` rather than as a quiet airframe.
    """
    combined = pick(ulog, ["sensor_combined"])
    clipping = None if combined is None else combined.data.get("accelerometer_clipping")
    clip = "none" if clipping is None else f"{sum(1 for c in clipping if c)}"

    metrics = [
        float(v)
        for d in ulog.data_list if d.name == "vehicle_imu_status"
        for v in d.data.get("accel_vibration_metric", [])
    ]
    if not metrics:
        status = pick(ulog, ["estimator_status"])
        if status is not None and "vibe[2]" in status.data:
            metrics = [float(v) for v in status.data["vibe[2]"]]
    if not any(metrics):
        return {"clip": clip, "vib_p95": "none"}
    metrics.sort()
    return {"clip": clip, "vib_p95": f"{metrics[(95 * (len(metrics) - 1)) // 100]:.3f}"}


def screen_regime(vehicle, vtol):
    """The airframe, and the regime changes a VTOL made, from the rows `--reference` writes.

    `type` is `mc`, `fw` or `other` off `vehicle_status`, or `vtol`. A VTOL whose
    regime nothing logged has changes nobody can count, which is `none` and not 0.
    """
    rows = []
    provenance = reference_mode(vehicle, vtol, rows)
    if provenance is None:
        return {"type": "none", "mode_changes": "none"}
    if any(vehicle.data["is_vtol"]):
        # MAV_VTOL_STATE_UNDEFINED is a sample that says nothing, often the first one at
        # boot. Skipped rather than counted, so it neither hides the changes after it nor
        # adds two of its own mid-flight.
        modes = [v["mode"] for _, _, v in rows if v["mode"] != "undefined"]
        changes = sum(1 for a, b in zip(modes, modes[1:]) if a != b)
        return {"type": "vtol", "mode_changes": f"{changes}" if modes else "none"}
    return {"type": rows[0][2]["mode"], "mode_changes": "0"}


def screen(ulog):
    """One `key=value` line saying whether a candidate log fills a corpus gap.

    Only what the ULog alone can say. What the vehicle did -- extent, speed, tilt
    -- and whether its start was still are the replay harness's `summary` keys,
    because the harness is the one place a statistic is computed and `at_rest` is
    the filter's claim to make. Every value is one number or one word, so a gap's
    criteria can be written as `data/expect.sh` pairs and tested against it.
    """
    info = ulog.msg_info_dict
    hardware = str(info.get("ver_hw", "none")).replace(" ", "_")
    _, key = ekf2_states(ulog)
    keys = {
        "sitl": "yes" if "SITL" in hardware else "no",
        "hw": hardware,
        "sw": release(info.get("ver_sw_release", 0)),
        "duration": f"{(ulog.last_timestamp - ulog.start_timestamp) * 1e-6:.0f}",
    }
    keys.update(screen_gnss(pick(ulog, GNSS_TOPICS)))
    keys.update(screen_vibration(ulog))
    keys["vib_metric"] = vibration_metric(info.get("ver_sw_release", 0))
    keys["ekf2"] = layout_label(key)
    keys["vehicle_imu"] = "yes" if pick(ulog, ["vehicle_imu"]) is not None else "no"
    keys.update(screen_regime(pick(ulog, ["vehicle_status"]), pick(ulog, ["vtol_vehicle_status"])))
    return "screen " + " ".join(f"{k}={v}" for k, v in keys.items())


class Fixture:
    """A stand-in for a pyulog dataset: a name and a dict of field arrays."""

    def __init__(self, name, **fields):
        self.name = name
        self.data = fields


class FixtureLog:
    """A stand-in for a pyulog `ULog`: the datasets, and `get_dataset` by name."""

    def __init__(self, *datasets):
        self.data_list = list(datasets)

    def get_dataset(self, name):
        return next(d for d in self.data_list if d.name == name)


def self_test():
    """Literal fixtures for what no corpus log can check about the converter.

    None of the five logs is a VTOL and every one fuses a near-level attitude, so
    neither a mode read from the wrong field nor a quaternion written scalar-last
    would show in their output: a scalar-last quaternion is still a rotation.
    """
    failures = []

    def expect(what, got, want):
        if got != want:
            failures.append(f"{what}: got {got!r}, want {want!r}")

    for args, want in [
        ((False, 2), "mc"), ((False, 13), "mc"), ((False, 1), "fw"),
        ((False, 10), "other"), ((False, 202), "other"),
        ((True, 20, 3), "mc"), ((True, 20, 1), "to_fw"), ((True, 20, 4), "fw"),
        ((True, 20, 2), "to_mc"), ((True, 20, 0), "undefined"),
        ((True, 20), "undefined"),
    ]:
        expect(f"classify_mode{args}", classify_mode(*args), want)
    for triple, want in [
        ((True, False, False), "mc"), ((False, True, True), "to_fw"),
        ((False, True, False), "to_mc"), ((False, False, False), "fw"),
    ]:
        expect(f"legacy {triple}", classify_mode(True, 21, legacy=triple), want)

    # 3949f175's combination: a quadrotor (MAV_TYPE 2) built inside the window
    # where `vehicle_type` numbered rotary wing 0. Read numerically under the
    # other era's constants, 0 is no vehicle at all.
    rows = []
    status = Fixture("vehicle_status", timestamp=[10, 20], is_vtol=[0, 0],
                     system_type=[2, 2], vehicle_type=[0, 0])
    reference_mode(status, None, rows)
    expect("3949f175 mode rows", [(t, v["mode"]) for t, _, v in rows], [(10, "mc")])

    # A VTOL mission: one row per change, not per message.
    rows = []
    status = Fixture("vehicle_status", timestamp=[0], is_vtol=[1], system_type=[20],
                     vehicle_type=[1])
    vtol = Fixture("vtol_vehicle_status", timestamp=[1, 2, 3, 4, 5, 6, 7],
                   vehicle_vtol_state=[3, 3, 1, 4, 4, 2, 3])
    reference_mode(status, vtol, rows)
    expect("vtol mode rows", [(t, v["mode"]) for t, _, v in rows],
           [(1, "mc"), (3, "to_fw"), (4, "fw"), (6, "to_mc"), (7, "mc")])

    # Pitched 90 deg: (cos 45, 0, sin 45, 0), scalar first. Scalar-last would put
    # the pair in q1 and q3 -- a half turn about a tilted axis, still a rotation.
    rows = []
    half = math.sqrt(0.5)
    attitude = Fixture("vehicle_attitude", timestamp=[0],
                       **{"q[0]": [half], "q[1]": [0.0], "q[2]": [half], "q[3]": [0.0]})
    reference_attitude(attitude, rows)
    expect("attitude row", {k: rows[0][2][k] for k in ("q0", "q1", "q2", "q3")},
           {"q0": half, "q1": 0.0, "q2": half, "q3": 0.0})

    # --screen. A receiver's span is taken over its 3D fixes only: the 2D fix at
    # eph 9.0 is outside it, and counting it would make a constant receiver vary.
    gps = Fixture("sensor_gps", timestamp=[0, 1, 2, 3], fix_type=[2, 3, 3, 6], eph=[9.0, 0.9, 0.9, 0.9],
                  satellites_used=[4, 10, 10, 10])
    expect("constant receiver", screen_gnss(gps),
           {"gnss": "sensor_gps", "fix_max": "6", "eph_min": "0.90", "eph_max": "0.90",
            "sats_min": "10", "sats_max": "10"})
    gps = Fixture("vehicle_gps_position", timestamp=[0, 1], eph=[1.4, 2.1])
    expect("receiver with no fix_type", screen_gnss(gps),
           {"gnss": "vehicle_gps_position", "fix_max": "none", "eph_min": "1.40",
            "eph_max": "2.10", "sats_min": "none", "sats_max": "none"})
    expect("no receiver", screen_gnss(None)["eph_min"], "none")
    expect("vtol regime", screen_regime(status, vtol), {"type": "vtol", "mode_changes": "4"})
    expect("vtol with no regime logged", screen_regime(status, None),
           {"type": "vtol", "mode_changes": "none"})
    # Undefined at boot, and again mid-flight: still the four changes of the mission above.
    vtol = Fixture("vtol_vehicle_status", timestamp=[0, 1, 2, 3, 4, 5, 6, 7],
                   vehicle_vtol_state=[0, 3, 1, 4, 0, 4, 2, 3])
    expect("vtol regime past undefined", screen_regime(status, vtol),
           {"type": "vtol", "mode_changes": "4"})
    expect("fixed wing", screen_regime(
        Fixture("vehicle_status", timestamp=[0], is_vtol=[0], system_type=[1]), None),
        {"type": "fw", "mode_changes": "0"})
    expect("receiver with no eph", screen_gnss(Fixture("sensor_gps", timestamp=[0]))["eph_max"],
           "none")
    # LPE's `7592c9b2` logs `vibe[2]` and never fills it: zero throughout is unmeasured.
    lpe = FixtureLog(Fixture("estimator_status", **{"vibe[2]": [0.0, 0.0, 0.0]}))
    expect("vibration never computed", screen_vibration(lpe)["vib_p95"], "none")
    imus = FixtureLog(
        Fixture("vehicle_imu_status", accel_vibration_metric=[0.01] * 19 + [0.55]),
        Fixture("vehicle_imu_status", accel_vibration_metric=[0.02] * 20),
    )
    expect("vibration pooled, touchdown excluded", screen_vibration(imus)["vib_p95"], "0.020")
    for encoded, want in [
        (0x010B03FF, "dv"), (0x010C01FF, "dv"), (0x010C0100, "unknown"),
        (0x010D0080, "accel"), (0x010F0400, "accel"), (0, "unknown"),
    ]:
        expect(f"vibration metric {encoded:#x}", vibration_metric(encoded), want)
    # 25 states against 24 entries, 24 against 23, 24 against 24: only the pair names each.
    expect("v1.16 layout", layout_label((25, 24)), "err24")
    expect("v1.15 layout", layout_label((24, 23)), "err23")
    expect("state-indexed layout", layout_label((24, 24)), "quat24")
    expect("LPE layout", layout_label((10, 10)), "unmapped")
    expect("no estimator", layout_label(None), "none")
    expect("v1.15 velocity index", EKF2_LAYOUTS[(24, 23)]["vel"], 3)
    rows = []
    v115 = Fixture("estimator_states", timestamp=[0], n_states=[24],
                   **{f"states[{i}]": [0.0] for i in range(24)},
                   **{f"covariances[{i}]": [float(i)] for i in range(23)})
    expect("v1.15 key", ekf2_states(FixtureLog(v115))[1], (24, 23))
    reference_states(v115, EKF2_LAYOUTS[(24, 23)], None, rows)
    expect("v1.15 sigma_vel_n", rows[0][2]["sigma_vel_n"], math.sqrt(3.0))
    expect("v1.15 sigma_ba_z", rows[0][2]["sigma_ba_z"], math.sqrt(14.0))
    expect("release", release(0x010B03FF), "v1.11.3")
    expect("rc", release(0x011000C0), "v1.16.0-rc")
    expect("dev", release(0x010A0000), "v1.10.0-dev")
    expect("no release", release(0), "none")

    for failure in failures:
        print(f"FAIL {failure}", file=sys.stderr)
    print(f"ulog2replay self-test: {'FAIL' if failures else 'ok'}", file=sys.stderr)
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("ulog", type=Path, nargs="?", help="input .ulg file")
    parser.add_argument("-o", "--output", type=Path, help="output CSV (default: alongside input)")
    parser.add_argument(
        "--reference", type=Path, nargs="?", const=True,
        help="also write EKF2's solution and test ratios "
             "(default: <output>.reference.csv)",
    )
    parser.add_argument(
        "--baro-variance", type=float, default=DEFAULT_BARO_VARIANCE,
        help=f"m^2, PX4 does not log one (default: {DEFAULT_BARO_VARIANCE})",
    )
    parser.add_argument(
        "--mag-variance", type=float, default=DEFAULT_MAG_VARIANCE,
        help=f"rad^2 on heading, PX4 does not log one (default: {DEFAULT_MAG_VARIANCE})",
    )
    parser.add_argument(
        "--self-test", action="store_true",
        help="run the fixtures no corpus log can check, and exit",
    )
    parser.add_argument(
        "--screen", action="store_true",
        help="print one `screen` line of what the log could cover in the corpus, "
             "write nothing, and exit",
    )
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.ulog is None:
        parser.error("a .ulg file is required")
    if args.screen:
        try:
            print(screen(open_ulog(args.ulog, SCREEN_TOPICS)))
        except ConversionError as e:
            print(f"ulog2replay: {e}", file=sys.stderr)
            return 1
        return 0

    output = args.output or args.ulog.with_suffix(".csv")
    try:
        rows, used, dropout_note, imu_dt = convert(
            args.ulog, args.baro_variance, args.mag_variance
        )
        note = [
            f"Converted from {args.ulog.name} by tools/ulog2replay.py",
            "Topics used: " + ", ".join(f"{k}={v}" for k, v in sorted(used.items())),
            f"baro variance {args.baro_variance} m^2 and mag heading variance "
            f"{args.mag_variance} rad^2 are assumed; PX4 logs neither.",
            "Timestamps rebased to the first sample.",
        ]
        if dropout_note:
            note.append(dropout_note)
        count, t0 = write_rows(rows, output, note)

        if args.reference:
            target = (
                args.reference
                if isinstance(args.reference, Path)
                else output.with_suffix(".reference.csv")
            )
            # A second open, with its own topic filter: the replay pass and this
            # one want disjoint topics, and the largest corpus log is 219 MB.
            # Do not fold them into one unfiltered open.
            reference = open_ulog(args.ulog, REFERENCE_TOPICS)
            if write_reference(reference, target, t0, imu_dt, args.ulog.name):
                print(f"reference -> {target}", file=sys.stderr)
    except ConversionError as e:
        print(f"ulog2replay: {e}", file=sys.stderr)
        return 1

    for source, topic in sorted(used.items()):
        print(f"  {source:<5} {topic}", file=sys.stderr)
    print(f"{count} rows -> {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
