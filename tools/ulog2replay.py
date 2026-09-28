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

# The sphere PX4's local x/y are projected on (`src/lib/geo/geo.h:55` at
# c4e4ef98); `px4_reproject` owns the rest.
PX4_EARTH_RADIUS = 6_371_000.0

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

# Dual-antenna GNSS heading, sigma = 0.1 rad where the receiver logs no accuracy:
# PX4's hard-coded gnss_heading_noise{0.1f} (EKF/common.h:390), the floor its
# R_YAW = sq(fmaxf(yaw_acc, gnss_heading_noise)) reads a missing accuracy as
# (EKF/aid_sources/gnss/gnss_yaw_control.cpp:138-139). So a log without one is
# fused at exactly what its EKF2 fused it at.
DEFAULT_GNSS_HEADING_VARIANCE = 0.01  # rad^2, sigma = 0.1 rad

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

# The MSL height beside each height field above, and its scale: the datum EKF2's
# `ref_alt` is in (`msg/versioned/VehicleLocalPosition.msg:57`, `EKF2.cpp:1738`
# at c4e4ef98), which the first fix's own two heights convert to. An MSL field
# is its own counterpart.
MSL_OF = {
    "altitude_ellipsoid_m": ("altitude_msl_m", 1.0),
    "alt_ellipsoid": ("alt", 1e-3),
    "altitude_msl_m": ("altitude_msl_m", 1.0),
    "alt": ("alt", 1e-3),
}

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


def px4_reproject(x, y, lat0, lon0):
    """Degrees of the point PX4's local (x, y) names about (lat0, lon0).

    PX4's local x/y are its `MapProjection`, azimuthal equidistant on a sphere
    (`project` at `src/lib/geo/geo.cpp:67-88` at c4e4ef98, and still how
    `EstimatorInterface::getPosition` forms x/y since the state became lat/lon,
    `estimator_interface.cpp:613-625`); this is `reproject` (`geo.cpp:90-111`),
    term for term. Reading EKF2's x/y as tangent-plane metres instead is wrong
    by the sphere's scale against the ellipsoid's, about 0.2 % of the distance
    from its origin at mid-latitudes: 3.7 m north on `89a498ce`, which flies
    4.07 km out. The one place in this repository that cites these lines.
    """
    x_rad, y_rad = x / PX4_EARTH_RADIUS, y / PX4_EARTH_RADIUS
    c = math.hypot(x_rad, y_rad)
    if c == 0.0:
        return lat0, lon0
    phi0, lam0 = math.radians(lat0), math.radians(lon0)
    sin_c, cos_c = math.sin(c), math.cos(c)
    lat = math.asin(cos_c * math.sin(phi0) + x_rad * sin_c * math.cos(phi0) / c)
    lon = lam0 + math.atan2(y_rad * sin_c,
                            c * math.cos(phi0) * cos_c - x_rad * math.sin(phi0) * sin_c)
    return math.degrees(lat), math.degrees(lon)


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


def gnss_yaw_enabled(params):
    """Whether this log's EKF2 fused the receiver's `heading` as a dual-antenna yaw.

    The field alone does not say: a driver may fill it from anything, and a ULog records
    no meaning beside a name. The log's own configuration does. `EKF2_GPS_CTRL` bit 3
    since PX4 8962cf2d25 (`src/modules/ekf2/params_gnss.yaml:5-17` at c4e4ef98), and
    `EKF2_AID_MASK` bit 7, `USE_GPS_YAW`, before it (`EKF/common.h` at 8962cf2d25~1).
    """
    if "EKF2_GPS_CTRL" in params:
        return bool(int(params["EKF2_GPS_CTRL"]) & (1 << 3))
    return bool(int(params.get("EKF2_AID_MASK", 0)) & (1 << 7))


def convert_gnss(ulog, rows, used, heading_variance=DEFAULT_GNSS_HEADING_VARIANCE):
    dataset = pick(ulog, GNSS_TOPICS)
    if dataset is None:
        print("warning: no GNSS topic; position and velocity aiding omitted", file=sys.stderr)
        return None
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
    msl_f, msl_scale = MSL_OF[alt_f]
    msl = dataset.data.get(msl_f)
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

    # `heading` is already the body frame's (msg/SensorGps.msg:76), the antenna mounting
    # having been taken out upstream, so it is written as it reads.
    heading = column(dataset, "heading") if "heading" in dataset.data else None
    if heading is not None and not gnss_yaw_enabled(ulog.initial_parameters):
        heading = None
    heading_accuracy = dataset.data.get("heading_accuracy")

    origin = None
    for k in range(len(t)):
        if not is_3d_fix(fix, k):
            continue
        phi = float(lat[k]) * angle_scale
        lam = float(lon[k]) * angle_scale
        height = float(alt[k]) * alt_scale
        if origin is None:
            # The same fix's MSL height rides along, None where the receiver logs
            # none, so a consumer can move an MSL altitude onto this datum.
            at_msl = float(msl[k]) * msl_scale if msl is not None and msl[k] else None
            origin = (phi, lam, height, at_msl)
            print(
                f"navigation origin: {phi:.7f}, {lam:.7f}, {height:.2f} m",
                file=sys.stderr,
            )
        north, east, down = geodetic_to_ned(phi, lam, height, *origin[:3])
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

        if heading is not None and math.isfinite(heading[k]):
            sigma = float(heading_accuracy[k]) if heading_accuracy is not None else 0.0
            variance = sigma**2 if math.isfinite(sigma) and sigma > 0 else heading_variance
            rows.append((t[k], "gnss_yaw", [float(heading[k])], [variance]))

    if origin is None:
        print("warning: no 3D GNSS fix in the log", file=sys.stderr)
    tally(rows, before, used, "gnss", f"{dataset.name} ({lat_f})")
    return origin


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


def convert(path, baro_variance, mag_variance, heading_variance=DEFAULT_GNSS_HEADING_VARIANCE):
    ulog = open_ulog(path)
    rows = []
    used = {}
    note = dropouts(ulog)
    sensor_combined = convert_imu(ulog, rows, used)
    origin = convert_gnss(ulog, rows, used, heading_variance)
    convert_baro(ulog, rows, used, baro_variance, sensor_combined)
    convert_mag(ulog, rows, used, mag_variance, sensor_combined)
    # The IMU sample interval, for --reference's bias scaling only. Taken here
    # because this is where `sensor_combined` is already open, and nothing in the
    # replay output depends on it.
    t = stamps(sensor_combined)
    imu_dt = median([(b - a) * 1e-6 for a, b in zip(t, t[1:]) if b > a])
    # Header lines the harness configures itself from: what EKF2 on this log used.
    delays, delays_note = measurement_delays(ulog.initial_parameters)
    parameters = [
        origin_note(origin),
        declination_note(ulog.initial_parameters, origin and origin[:2]),
        gnss_noise_note(ulog.initial_parameters),
        delays_note,
    ]
    return rows, used, note, imu_dt, parameters, origin, delays


def write_rows(rows, out, note, delays):
    """Sort by time, rebase to the first sample, and write the replay schema.

    `t_meas_s` is a row's time less its source's delay in `delays`, microseconds by
    source name, and blank where there is none: when the measurement was taken, where
    `t_s` is when it was logged.
    """
    rows.sort(key=lambda r: r[0])
    if not rows:
        raise ConversionError("nothing to write")
    t0 = rows[0][0]

    with open(out, "w", newline="") as handle:
        for line in note:
            handle.write(f"# {line}\n")
        handle.write("t_s,source,v0,v1,v2,v3,v4,v5,var0,var1,var2,t_meas_s\n")
        for timestamp, source, values, variances in rows:
            t = (timestamp - t0) * 1e-6  # ULog timestamps are microseconds
            values = list(values) + [None] * (6 - len(values))
            variances = list(variances) + [None] * (3 - len(variances))
            cells = [f"{t:.6f}", source]
            cells += ["" if v is None else f"{v:.6g}" for v in values + variances]
            delay = delays.get(source, 0)
            cells.append(f"{(timestamp - delay - t0) * 1e-6:.6f}" if delay else "")
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
    "xy_reset", "z_reset", "vxy_reset", "vz_reset",
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


# EKF2's own reset counters, beside the states they step. A reset is an event in
# that filter, and without the counter a step across one reads as divergence from
# this one -- the same reason `att_reset` carries `quat_reset_counter`.
LOCAL_RESETS = [
    ("xy_reset", "xy_reset_counter"),
    ("z_reset", "z_reset_counter"),
    ("vxy_reset", "vxy_reset_counter"),
    ("vz_reset", "vz_reset_counter"),
]


def position_axes(local, origin):
    """Which of EKF2's position axes can be written in the replay frame, as
    `("ned" | "ne" | "", reason)`.

    Horizontal needs an EKF2 origin (`ekf2_origin_of`) and a replay one. Down
    also needs the fix's MSL height, which `place` moves EKF2's heights onto the
    ellipsoid by.
    """
    ekf2_origin, missing = ekf2_origin_of(local)
    if ekf2_origin is None:
        return "", missing
    if origin is None:
        return "", "the replay input has no GNSS fix"
    if origin[3] is None:
        return "ne", "the fix logs no MSL height"
    return "ned", None


def place(x, y, z, ref_lat, ref_lon, ref_alt, origin):
    """PX4's local (x, y, z) about EKF2's origin, as north, east, down in the
    replay frame about `origin`, `(lat, lon, height, MSL height)`.

    Reprojected by `px4_reproject` and placed by `geodetic_to_ned`, exactly as a
    fix is. Height is `ref_alt - z` (`estimator_interface.cpp:628` at c4e4ef98),
    MSL, moved onto the ellipsoid by the first fix's own difference between its
    two heights, the geoid height there; data/README.md, "What `--reference`
    writes", gives what it is worth. With no MSL height the shift is zero and
    down is not a replay-frame height, which `position_axes` reports.
    """
    lat0, lon0, height0, msl0 = origin
    geoid = 0.0 if msl0 is None else height0 - msl0
    lat, lon = px4_reproject(x, y, ref_lat, ref_lon)
    return geodetic_to_ned(lat, lon, ref_alt - z + geoid, lat0, lon0, height0)


def reference_local(local, rows, origin):
    """EKF2's position and velocity, from `vehicle_local_position`, and its reset counters.

    Position is written in the replay frame wherever `position_axes` allows, each
    row placed about that row's own EKF2 origin, which `093e806a` moves once
    mid-log and `7ce66f0d` moves in height. The
    alternative, EKF2's x/y plus one offset between the two origins, is not a
    rigid shift, because the two frames differ in scale. A row without an origin
    (`xy_global` false) has no position here. With no axes placed, x/y/z are
    EKF2's own. Velocity stays EKF2's: its north differs from the replay origin's
    by the arc between them, 0.04 deg at 4 km.
    """
    axes, _ = position_axes(local, origin)
    t = stamps(local)
    names = ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d"]
    columns = [column(local, f) for f in ("x", "y", "z", "vx", "vy", "vz")]
    resets = [(n, local.data[f]) for n, f in LOCAL_RESETS if f in local.data]
    if axes:
        refs = [column(local, f) for f in ("xy_global", "z_global", "ref_lat", "ref_lon",
                                           "ref_alt")]
    for k in range(len(t)):
        values = {n: c[k] for n, c in zip(names, columns)}
        if axes:
            xy_global, z_global, *ref = (c[k] for c in refs)
            values["pos_n"] = values["pos_e"] = values["pos_d"] = None
            if xy_global:
                north, east, down = place(*(float(c[k]) for c in columns[:3]),
                                          *map(float, ref), origin)
                values["pos_n"], values["pos_e"] = north, east
                if axes == "ned" and z_global:
                    values["pos_d"] = down
        values.update((n, int(c[k])) for n, c in resets)
        rows.append((t[k], "ekf2_local", values))


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


def write_reference(ulog, out, t0, imu_dt, source_name, origin):
    """EKF2's own solution and innovation ratios, for a side-by-side diff.

    `origin` is the replay input's navigation origin, the first 3D fix as
    (lat, lon, height, MSL height) in degrees and metres, or None when the log
    has no fix.
    """
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
        reference_local(local, rows, origin)
    if attitude is not None:
        reference_attitude(attitude, rows)
    if layout is not None:
        reference_states(states, layout, period, rows)
    if status is not None and ratio_fields:
        reference_ratios(status, ratio_fields, rows)
    mode = reference_mode(vehicle, vtol, rows)
    rows.sort(key=lambda r: r[0])

    with open(out, "w", newline="") as handle:
        for line in reference_note(local, attitude, states, layout, key, period,
                                   provenance, ratio_fields, mode, source_name,
                                   status, origin, ulog.initial_parameters):
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
                   ratio_fields, mode, source_name, status, origin, params):
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
    note.append(estimator_note(params))
    ekf2_origin, missing = ekf2_origin_of(local)
    note.append("EKF2 origin: " + (f"none ({missing})" if ekf2_origin is None
                                   else "{:.9g} {:.9g} {:.9g}".format(*ekf2_origin)))
    note.append(origin_offset_note(ekf2_origin, origin))
    note.append(position_note(local, origin))
    note.append(ekf2_aiding_note(status, params))
    note.append("Same timebase as the replay CSV: rebased to its first sample.")
    return note


def ekf2_origin_of(local):
    """EKF2's first local-position origin as `(lat, lon, alt)`, or None and why.

    A `#` line rather than a column because it does not move: it is one geodetic
    point per log, and the replay harness already parses a `#` header for the
    truth file's scenario and seed. Two corpus logs report `xy_global` false with
    the reference fields all zero, so their x/y/z are origin-relative with no
    origin -- which is a fact about those logs and has to be said, not filled in.
    """
    if local is None:
        return None, "no vehicle_local_position"
    if "xy_global" not in local.data or "ref_lat" not in local.data:
        return None, "vehicle_local_position carries no reference fields"
    if not any(local.data["xy_global"]):
        return None, "xy_global false"
    index = next(k for k, flag in enumerate(local.data["xy_global"]) if flag)
    return tuple(float(local.data[f][index]) for f in ("ref_lat", "ref_lon", "ref_alt")), None


def origin_offset_note(ekf2_origin, origin):
    """Where EKF2's first origin sits in the replay input's frame.

    A fact about the log, not a shift to apply: `reference_local` places every
    position itself, since the two frames differ in scale as well as origin. The
    two origins are both first fixes, but of different runs of the receiver: on
    `2c42096b` EKF2's was set 770 s before logging began, 3.67 m south and
    2.14 m east of the first fix the log holds. It is `place` at (0, 0, 0), so
    down is `none` where the fix logs no MSL height.
    """
    if ekf2_origin is None:
        return "EKF2 origin in replay frame: none (EKF2 reports no origin)"
    if origin is None:
        return "EKF2 origin in replay frame: none (the replay input has no GNSS fix)"
    # `+ 0.0` turns a rounded -0.0 into 0.0, which prints without its sign.
    north, east, down = (round(v, 3) + 0.0 for v in place(0.0, 0.0, 0.0, *ekf2_origin, origin))
    down_text = "none" if origin[3] is None else f"{down:.3f}"
    return f"EKF2 origin in replay frame: {north:.3f} {east:.3f} {down_text} m"


def position_note(local, origin):
    """Which `pos_*` columns are in the replay frame, as `position_axes` decides.

    The one line a consumer reads before comparing a position, so a column still
    in EKF2's own frame is never set beside this filter's.
    """
    axes, reason = position_axes(local, origin)
    if axes == "ned":
        return "EKF2 position in replay frame: n e d"
    if axes:
        return f"EKF2 position in replay frame: n e ({reason})"
    return f"EKF2 position in replay frame: none ({reason})"


def estimator_note(params):
    """Which PX4 estimator published the reference, from the parameters that select it.

    The row kinds are named `ekf2_*` whatever produced them, and on one corpus log
    (`7592c9b2`) that is LPE: `vehicle_local_position` and `vehicle_attitude` are
    the vehicle's estimate, not EKF2's. `SYS_MC_EST_GROUP` selected it until the
    per-module switches replaced it, 1 for LPE with attitude_estimator_q and 2 for
    EKF2 since 66ffc834d3 dropped INAV's 0 (`src/modules/systemlib/system_params.c:98-99`
    at that commit); `EKF2_EN` and `LPE_EN` after.
    """
    group = params.get("SYS_MC_EST_GROUP")
    if params.get("EKF2_EN") == 1 or group == 2:
        name = "ekf2"
    elif params.get("LPE_EN") == 1 or group == 1:
        name = "lpe"
    else:
        name = "unknown"
    chosen = ", ".join(f"{k} {params[k]}" for k in ("SYS_MC_EST_GROUP", "EKF2_EN", "LPE_EN")
                       if k in params)
    return f"Estimator: {name} ({chosen or 'no selection parameter'})"


# `control_mode_flags` bits that say which aiding EKF2 fused, the ones this
# comparison needs to name a divergence's cause. Their positions have not moved
# since the constants were written into the message (`msg/estimator_status.msg:30-39`
# at e5d428bd65, 2018) and still match `filter_control_status_u`
# (`src/modules/ekf2/EKF/common.h:574-586` at c4e4ef98), which covers every
# corpus log. A ULog records the bitmask, never these names -- see AGENTS.md, "A
# ULog field name does not pin its meaning" -- so the table is checked against
# both ends of that range rather than trusted to the field name.
EKF2_AIDING = [(2, "gnss_pos"), (4, "mag_hdg"), (5, "mag_3d"), (9, "baro_hgt"), (11, "gps_hgt")]

# The height source EKF2 converges to, under either name the parameter has had:
# `EKF2_HGT_REF` since 8962cf2d25 (`src/modules/ekf2/module.yaml:87-104` at
# c4e4ef98) and `EKF2_HGT_MODE` before it, one enum in both
# (`ekf2_params.c:642-649` at 8962cf2d25^).
HEIGHT_REFERENCES = {0: "baro", 1: "gps", 2: "range", 3: "vision"}


def ekf2_aiding_note(status, params):
    """The share of `control_mode_flags` samples on which each aiding flag is set,
    and the height reference EKF2 was configured to converge to.

    A share rather than a yes or no because a height source can start or stop
    mid-log, and a comparison of height has to know whether it did. Both halves
    because neither alone names the reference: on `2c42096b` (`EKF2_HGT_MODE`
    baro) `baro_hgt` reads 1.00 and `gps_hgt` 0.00, but a build with
    `EKF2_HGT_REF` fuses both at once, reading 1.00 on each, and the parameter is
    what says which one the estimate follows at low frequency.
    """
    name = next((n for n in ("EKF2_HGT_REF", "EKF2_HGT_MODE") if n in params), None)
    if name is None:
        reference = "height reference unknown (no EKF2_HGT_REF or EKF2_HGT_MODE)"
    else:
        value = int(params[name])
        reference = f"height reference {HEIGHT_REFERENCES.get(value, value)} ({name} {value})"
    if status is None or "control_mode_flags" not in status.data or not len(status.data["control_mode_flags"]):
        return f"EKF2 aiding: {reference}; flags unknown (no control_mode_flags)"
    flags = [int(v) for v in status.data["control_mode_flags"]]
    shares = [
        f"{flag} {sum(1 for v in flags if v >> bit & 1) / len(flags):.2f}"
        for bit, flag in EKF2_AIDING
    ]
    return (f"EKF2 aiding: {reference}; share of control_mode_flags samples: "
            + ", ".join(shares))


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


# PX4's magnetic declination table and its lookup, ported from
# src/lib/world_magnetic_model/geo_magnetic_tables.hpp and geo_mag_declination.cpp:56-108
# at PX4-Autopilot c4e4ef98 (table last regenerated at f2bca92221: WMM-2020, epoch 2024.41).
# EKF2 applies the table compiled into its own firmware to its first GNSS fix when
# EKF2_DECL_TYPE bit 0 is set. This is PX4's current one, so on an older build the value
# can differ from what that build applied by the model's secular change -- a few tenths of
# a degree on the corpus, where the saved EKF2_MAG_DECL and this table disagree by at most
# 0.36 deg -- rather than being that build's value exactly.
#
# The table and lookup are PX4's, under its licence, retained here as it requires:
#
# Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in
# the documentation and/or other materials provided with the
# distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
# used to endorse or promote products derived from this software
# without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Rows are latitude -90 to 90, columns longitude -180 to 180, 10 deg apart, in units of
# DECLINATION_SCALE degrees.
DECLINATION_SCALE = 0.00545143529
DECLINATION_TABLE = (
    (27264, 25429, 23595, 21761, 19926, 18092, 16258, 14423, 12589, 10754, 8920, 7086, 5251, 3417, 1583, -252, -2086, -3921, -5755, -7589, -9424, -11258, -13092, -14927, -16761, -18596, -20430, -22264, -24099, -25933, -27768, -29602, -31436, 32767, 30933, 29098, 27264),
    (23650, 21416, 19382, 17521, 15799, 14184, 12647, 11165, 9721, 8303, 6904, 5519, 4143, 2772, 1397, 9, -1403, -2848, -4333, -5863, -7437, -9056, -10718, -12423, -14175, -15984, -17864, -19835, -21922, -24152, -26545, -29105, -31803, 31464, 28718, 26092, 23650),
    (15755, 14282, 13094, 12077, 11159, 10281, 9392, 8457, 7454, 6381, 5257, 4109, 2971, 1865, 794, -267, -1359, -2527, -3797, -5172, -6632, -8142, -9669, -11188, -12690, -14181, -15686, -17253, -18969, -20997, -23675, -27707, 32110, 25303, 20617, 17720, 15755),
    (8903, 8634, 8328, 8030, 7755, 7481, 7150, 6689, 6037, 5171, 4116, 2948, 1775, 704, -211, -998, -1763, -2635, -3706, -4982, -6396, -7846, -9240, -10513, -11630, -12572, -13320, -13839, -14019, -13549, -11322, -3523, 5353, 8167, 8943, 9057, 8903),
    (5806, 5839, 5774, 5672, 5587, 5541, 5499, 5359, 4991, 4286, 3216, 1875, 470, -754, -1647, -2218, -2623, -3091, -3835, -4934, -6270, -7627, -8830, -9771, -10388, -10623, -10400, -9587, -7986, -5492, -2431, 460, 2679, 4178, 5097, 5594, 5806),
    (4186, 4282, 4284, 4229, 4159, 4118, 4119, 4103, 3914, 3342, 2253, 728, -912, -2273, -3151, -3597, -3767, -3819, -4016, -4665, -5740, -6891, -7817, -8365, -8453, -8028, -7069, -5603, -3811, -2034, -509, 772, 1872, 2790, 3486, 3942, 4186),
    (3161, 3250, 3275, 3251, 3182, 3094, 3026, 2988, 2851, 2337, 1229, -384, -2070, -3364, -4111, -4446, -4503, -4252, -3790, -3609, -4059, -4874, -5597, -5938, -5777, -5133, -4109, -2845, -1594, -619, 83, 710, 1372, 2016, 2561, 2948, 3161),
    (2485, 2532, 2542, 2534, 2481, 2375, 2255, 2172, 2018, 1486, 353, -1228, -2777, -3865, -4390, -4481, -4240, -3623, -2707, -1902, -1669, -2082, -2768, -3241, -3254, -2856, -2179, -1328, -531, -35, 229, 542, 1013, 1529, 1985, 2318, 2485),
    (2072, 2064, 2032, 2018, 1980, 1883, 1758, 1657, 1461, 874, -261, -1719, -3046, -3887, -4129, -3855, -3229, -2397, -1510, -734, -274, -342, -854, -1379, -1586, -1478, -1143, -624, -116, 121, 159, 321, 720, 1197, 1625, 1939, 2072),
    (1847, 1810, 1742, 1723, 1703, 1621, 1500, 1371, 1102, 444, -660, -1952, -3043, -3628, -3589, -3035, -2219, -1401, -724, -169, 254, 350, 38, -401, -658, -711, -608, -338, -43, 36, -42, 45, 413, 896, 1354, 1703, 1847),
    (1699, 1705, 1653, 1662, 1681, 1620, 1480, 1275, 878, 122, -954, -2080, -2929, -3256, -3012, -2356, -1536, -796, -265, 129, 466, 610, 425, 85, -155, -268, -299, -222, -121, -175, -335, -312, 14, 509, 1030, 1470, 1699),
    (1494, 1649, 1708, 1799, 1884, 1855, 1681, 1355, 781, -117, -1209, -2197, -2803, -2893, -2528, -1885, -1137, -461, 13, 332, 601, 746, 640, 383, 178, 51, -55, -135, -228, -437, -694, -758, -503, -19, 561, 1114, 1494),
    (1160, 1544, 1816, 2050, 2215, 2221, 2017, 1567, 802, -275, -1438, -2341, -2756, -2673, -2242, -1629, -939, -294, 184, 499, 738, 885, 855, 696, 542, 408, 230, -8, -317, -718, -1109, -1275, -1099, -642, -30, 613, 1160),
    (765, 1379, 1900, 2314, 2577, 2622, 2393, 1831, 876, -403, -1685, -2574, -2892, -2722, -2248, -1623, -936, -278, 252, 629, 908, 1109, 1199, 1184, 1104, 944, 647, 195, -387, -1030, -1576, -1826, -1696, -1253, -625, 77, 765),
    (437, 1218, 1936, 2528, 2919, 3036, 2799, 2116, 928, -623, -2091, -3030, -3327, -3121, -2604, -1925, -1180, -449, 193, 719, 1153, 1519, 1805, 1977, 1991, 1784, 1297, 527, -430, -1381, -2085, -2373, -2234, -1771, -1112, -355, 437),
    (198, 1087, 1936, 2674, 3216, 3452, 3243, 2411, 855, -1158, -2926, -3941, -4206, -3936, -3339, -2560, -1696, -817, 27, 812, 1534, 2191, 2754, 3160, 3315, 3098, 2397, 1190, -317, -1699, -2593, -2902, -2725, -2214, -1502, -680, 198),
    (-111, 863, 1803, 2648, 3305, 3634, 3405, 2285, 58, -2678, -4700, -5589, -5627, -5143, -4352, -3383, -2317, -1205, -83, 1026, 2098, 3108, 4014, 4748, 5204, 5214, 4533, 2927, 575, -1644, -2988, -3441, -3276, -2730, -1962, -1068, -111),
    (-1054, -90, 801, 1527, 1946, 1812, 731, -1632, -4654, -6842, -7732, -7698, -7114, -6204, -5096, -3864, -2554, -1197, 184, 1572, 2947, 4292, 5581, 6779, 7828, 8630, 8988, 8486, 6313, 2026, -1910, -3677, -4009, -3636, -2914, -2019, -1054),
    (-30607, -28773, -26938, -25104, -23269, -21435, -19601, -17766, -15932, -14097, -12263, -10429, -8594, -6760, -4926, -3091, -1257, 578, 2412, 4246, 6081, 7915, 9749, 11584, 13418, 15253, 17087, 18921, 20756, 22590, 24424, 26259, 28093, 29928, 31762, -32441, -30607),
)


def table_declination(latitude, longitude):
    """Declination in degrees from PX4's table, interpolated bilinearly as PX4 does."""
    res, lat_min, lat_max, lon_min, lon_max = 10.0, -90.0, 90.0, -180.0, 180.0
    latitude = min(max(latitude, lat_min), lat_max)
    if longitude > lon_max:
        longitude -= 360.0
    if longitude < lon_min:
        longitude += 360.0
    lat0 = min(max(math.floor(latitude / res) * res, lat_min), lat_max - res)
    lon0 = min(max(math.floor(longitude / res) * res, lon_min), lon_max - res)
    i, j = int((lat0 - lat_min) / res), int((lon0 - lon_min) / res)
    sw, se = DECLINATION_TABLE[i][j], DECLINATION_TABLE[i][j + 1]
    nw, ne = DECLINATION_TABLE[i + 1][j], DECLINATION_TABLE[i + 1][j + 1]
    lat_scale = min(max((latitude - lat0) / res, 0.0), 1.0)
    lon_scale = min(max((longitude - lon0) / res, 0.0), 1.0)
    south = lon_scale * (se - sw) + sw
    north = lon_scale * (ne - nw) + nw
    return (lat_scale * (north - south) + south) * DECLINATION_SCALE


def origin_note(origin):
    """The header line naming where the replay's NED frame sits, which `examples/replay.rs`
    reads to look its own magnetic model up at the site (`declination_model=`) and, under
    `--declination model`, hands the filter as its origin.

    `origin` is the first 3D fix as (lat, lon, height, MSL height), degrees and metres; the
    height is on the datum the fixes were converted on. A log with no fix writes the line
    anyway, as `none`, so its absence says the file predates it.
    """
    if origin is None:
        return "Navigation origin none (no 3D fix)"
    return f"Navigation origin {origin[0]:.9f} {origin[1]:.9f} {origin[2]:.3f} (lat deg, lon deg, height m; the first 3D fix)"


def declination_note(params, origin):
    """The header line `examples/replay.rs` reads its magnetic declination from.

    The declination the log's own EKF2 applied, by its own rule
    (`Ekf::getMagDeclination`, EKF/aid_sources/magnetometer/mag_control.cpp:617-636 at
    c4e4ef98): with `EKF2_DECL_TYPE` bit 0 set and a fix, PX4's table at that fix --
    `origin`, the first 3D fix, as (lat, lon) in degrees. Otherwise `EKF2_MAG_DECL` only
    when bit 1 (save) is set and it is non-zero, and zero if not -- EKF2 ignores a stale
    parameter it was not told to keep. LPE's `ATT_MAG_DECL` is taken as it reads. Degrees
    east-positive, as `Config`'s is.

    Not the parameter first, because with bit 0 set the parameter is only what an earlier
    flight saved at disarm: `7ce66f0d` reads 0 there, a first flight at its site, while its
    EKF2 flew on the table. A log with no GNSS and no parameter reads 0, and nothing in
    it can say where north is.
    """
    decl_type = int(params.get("EKF2_DECL_TYPE", 0))
    if decl_type & 1 and origin is not None:
        degrees = table_declination(*origin)
        source = "PX4's table at the first fix, as EKF2_DECL_TYPE bit 0 has EKF2 use"
    elif "EKF2_DECL_TYPE" in params or "EKF2_MAG_DECL" in params:
        saved = float(params.get("EKF2_MAG_DECL", 0.0))
        if decl_type & 2 and saved != 0.0:
            degrees, source = saved, "EKF2_MAG_DECL"
        else:
            degrees, source = 0.0, "EKF2 had neither a fix to look one up nor a saved value"
    else:
        degrees = float(params.get("ATT_MAG_DECL", 0.0))
        source = "ATT_MAG_DECL" if "ATT_MAG_DECL" in params else "no declination parameter, so zero"
    return f"Magnetic declination {math.radians(degrees):.6f} rad ({degrees:.2f} deg, {source})"


# The parameters PX4 floors and caps a receiver's reported accuracy with, and
# their defaults at c4e4ef98 (`src/modules/ekf2/params_gnss.yaml:30-72`,
# `module.yaml:67-71` for EKF2_NOAID_NOISE) for a log that does not carry one.
GNSS_NOISE_PARAMETERS = [
    ("EKF2_GPS_P_NOISE", 0.5),
    ("EKF2_GPS_V_NOISE", 0.3),
    ("EKF2_NOAID_NOISE", 10.0),
]


def gnss_noise_note(params):
    """The header line `examples/replay.rs --r-policy px4` reads its floors from.

    The values this log's EKF2 bounded its receiver with, so a floored replay is
    compared against the `R` EKF2 actually fused rather than a default it may not
    have run: `2c42096b` flew `EKF2_GPS_V_NOISE` 0.3, not the 0.5 #113 floored it
    at. A parameter the log does not carry falls back to PX4's default and says so. The rule the harness applies to them is its own, cited
    there; this line carries values only.
    """
    cells = []
    for name, default in GNSS_NOISE_PARAMETERS:
        if name in params:
            cells.append(f"{name} {float(params[name]):.6g}")
        else:
            cells.append(f"{name} {default:.6g} (PX4 default; not in the log)")
    return "GNSS noise parameters: " + ", ".join(cells)


# How EKF2 dates a measurement: its timestamp less a configured delay, per source.
# Baro and mag at `src/modules/ekf2/EKF/estimator_interface.cpp:145,230`; GNSS in the
# sensors module since SENS_GPS0_DELAY (`src/modules/sensors/vehicle_gps_position/
# VehicleGPSPosition.cpp:168-169`), in EKF2 as EKF2_GPS_DELAY before it. All at
# c4e4ef98. The first name a log carries is the one it ran.
MEASUREMENT_DELAYS = [
    (("gnss_pos", "gnss_vel", "gnss_yaw"), ("SENS_GPS0_DELAY", "EKF2_GPS_DELAY")),
    (("baro",), ("EKF2_BARO_DELAY",)),
    (("mag",), ("EKF2_MAG_DELAY",)),
]


def measurement_delays(params):
    """Each aiding source's delay in microseconds, and the header line saying so.

    A receiver's fix describes where the vehicle was when it was computed, and EKF2
    fuses it there, `EKF2_GPS_DELAY` (110 ms by default) before it was logged. This
    carries EKF2's own figure for this log into `t_meas_s`, so a replay dates each
    measurement as EKF2 did rather than as current. A parameter the log does not carry
    is no delay, and the line says so.
    """
    delays, cells = {}, []
    for sources, names in MEASUREMENT_DELAYS:
        name = next((n for n in names if n in params), None)
        millis = float(params[name]) if name else 0.0
        for source in sources:
            delays[source] = int(round(millis * 1000))
        label = sources[0].split("_")[0]
        cells.append(f"{label} {millis:.6g} ms ({name or 'not in the log'})")
    return delays, "Measurement delays: " + ", ".join(cells)


def vibration_metric(encoded):
    """Which quantity `accel_vibration_metric` is on this build: `dv`, `accel` or `unknown`.

    PX4 f2ae8ae814 changed it, under the same field name, from a filtered
    |dv - dv_prev| over one integration interval, in m/s, to |a - a_prev| over raw
    samples, in m/s^2. So 0.094 on `2c42096b` (v1.11.3) and 32 on a v1.15 log are
    not a quiet airframe and a loud one. No rescaling makes them one quantity --
    the old metric differences averages, which is where vibration is filtered out
    -- so the screen names the metric rather than converting it. The commit is on
    master 1945 commits after v1.13.0-alpha1 and before v1.13.0-beta1, so a v1.12 or
    v1.13 build short of beta could be either.
    `estimator_status.vibe[2]` on older builds is the delta-velocity one. A version
    outside PX4's v1 series is a vendor's own numbering -- `a299e722` reports v6.5.22
    -- and says nothing about which PX4 it forked.
    """
    major, minor = (encoded >> 24) & 0xFF, (encoded >> 16) & 0xFF
    if not encoded or major != 1:
        return "unknown"
    kind = encoded & 0xFF
    if (major, minor) > (1, 13) or ((major, minor) == (1, 13) and kind >= 128):
        return "accel"
    if (major, minor) in ((1, 12), (1, 13)) and kind != 255:
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


def screen_imu_rate(dataset):
    """`sensor_combined`'s median rate in Hz, or `none`.

    A log can carry it at 5 Hz -- `9ae507d6`, a 3.8 km fixed-wing flight whose logger
    profile sampled it slowly -- and the replay then refuses every step as too long. Only
    a full conversion and replay said so before. The median, as the harness takes its own
    rate, because burst logging makes the minimum and the mean lie.
    """
    if dataset is None:
        return {"imu_hz": "none"}
    t = stamps(dataset)
    dt = median([(b - a) * 1e-6 for a, b in zip(t, t[1:]) if b > a])
    return {"imu_hz": "none" if not dt else f"{1.0 / dt:.0f}"}


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
    keys.update(screen_imu_rate(pick(ulog, ["sensor_combined"])))
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

    No corpus log is a VTOL and every one fuses a near-level attitude, so
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
        (0x010D0040, "unknown"), (0x010D0000, "unknown"), (0x010D00FF, "accel"),
        (0x06051680, "unknown"),
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
    # PX4's own test_geo_lookup.cpp values at grid points, within its own 0.4 + 1.0 deg.
    for lat, lon, want in [(-50, -180, 31.7), (-50, -100, 27.2)]:
        got = round(table_declination(lat, lon), 1)
        expect(f"table at ({lat}, {lon}) within PX4's tolerance", abs(got - want) <= 1.4, True)
    # Halfway between two grid points is the mean of the two, exactly.
    expect("bilinear midpoint", round(table_declination(-50, -175), 9),
           round((table_declination(-50, -180) + table_declination(-50, -170)) / 2, 9))
    expect("longitude wraps", table_declination(0, 190), table_declination(0, -170))
    expect("bilinear between rows", round(table_declination(-45, -180), 9),
           round((table_declination(-50, -180) + table_declination(-40, -180)) / 2, 9))
    # The origin the table is read at is the first 3D fix, not the first message.
    import numpy
    gps = Fixture("sensor_gps", timestamp=numpy.array([0, 1, 2]), fix_type=[2, 3, 3],
                  latitude_deg=numpy.array([10.0, 56.41, 56.5]),
                  longitude_deg=numpy.array([10.0, 43.76, 43.8]),
                  altitude_ellipsoid_m=numpy.array([0.0, 70.0, 71.0]),
                  eph=[1.0] * 3, epv=[1.0] * 3)
    expect("origin at the first 3D fix", convert_gnss(FixtureLog(gps), [], {})[:2], (56.41, 43.76))
    gps.data["altitude_msl_m"] = numpy.array([0.0, 52.0, 53.0])
    expect("the first fix's MSL height beside it", convert_gnss(FixtureLog(gps), [], {})[2:],
           (70.0, 52.0))
    geo = {"EKF2_DECL_TYPE": 3, "EKF2_MAG_DECL": 0.0}
    expect("bit 0 and a fix: the table", declination_note(geo, (56.41, 43.76)).split(" (")[0],
           f"Magnetic declination {math.radians(table_declination(56.41, 43.76)):.6f} rad")
    expect("bit 0, no fix, saved 0: zero", declination_note(geo, None).split(" rad")[0],
           "Magnetic declination 0.000000")
    expect("both bits clear: a stale parameter is ignored",
           declination_note({"EKF2_DECL_TYPE": 0, "EKF2_MAG_DECL": 13.0}, (56.41, 43.76)).split(" rad")[0],
           "Magnetic declination 0.000000")
    expect("bit 0 clear: the parameter",
           declination_note({"EKF2_DECL_TYPE": 2, "EKF2_MAG_DECL": 13.746335}, (56.41, 43.76)),
           "Magnetic declination 0.239919 rad (13.75 deg, EKF2_MAG_DECL)")
    expect("LPE", declination_note({"ATT_MAG_DECL": -2.0}, None).split(" rad")[0],
           "Magnetic declination -0.034907")
    expect("nothing", declination_note({}, None).split(" rad")[0], "Magnetic declination 0.000000")
    expect("origin line", origin_note((56.41, 43.76, 150.0, None)),
           "Navigation origin 56.410000000 43.760000000 150.000 (lat deg, lon deg, height m; the first 3D fix)")
    expect("no origin", origin_note(None), "Navigation origin none (no 3D fix)")
    # A 2.5 ms burst inside a true 20 ms period, a299e722's shape: the median holds.
    burst = Fixture("sensor_combined", timestamp=[0, 20000, 40000, 42500, 60000, 80000, 100000])
    expect("imu rate through bursts", screen_imu_rate(burst), {"imu_hz": "50"})
    expect("no imu", screen_imu_rate(None), {"imu_hz": "none"})
    expect("release", release(0x010B03FF), "v1.11.3")
    expect("rc", release(0x011000C0), "v1.16.0-rc")
    expect("dev", release(0x010A0000), "v1.10.0-dev")
    expect("no release", release(0), "none")

    # Reset counters ride on the position rows, as integers, and an era without
    # them leaves the cells blank rather than zero.
    rows = []
    local = Fixture("vehicle_local_position", timestamp=[0], x=[1.0], y=[2.0], z=[3.0],
                    vx=[0.0], vy=[0.0], vz=[0.0], xy_reset_counter=[2], z_reset_counter=[0])
    reference_local(local, rows, None)
    expect("reset counters", {k: rows[0][2].get(k) for k in ("xy_reset", "z_reset", "vxy_reset")},
           {"xy_reset": 2, "z_reset": 0, "vxy_reset": None})
    expect("no origin keeps EKF2's frame", [rows[0][2][k] for k in ("pos_n", "pos_e", "pos_d")],
           [1.0, 2.0, 3.0])
    # PX4's x at the equator is arc on a 6371 km sphere, so 0.001 deg of latitude
    # is 6371e3 pi/180e3 = 111.195 m of x, where the ellipsoid's tangent plane puts
    # the same point 110.575 m north. Reading x as tangent-plane metres is the
    # 0.62 m gap; a swapped x/y lands the point east.
    arc = PX4_EARTH_RADIUS * math.radians(0.001)
    expect("reproject north", tuple(round(v, 12) for v in px4_reproject(arc, 0.0, 0.0, 0.0)),
           (0.001, 0.0))
    expect("reproject east", tuple(round(v, 12) for v in px4_reproject(0.0, arc, 0.0, 0.0)),
           (0.0, 0.001))
    # Off the equator, back through `MapProjection::project` (cited at
    # `px4_reproject`), written out here: at latitude 0 every term carrying sin(lat0) vanishes, so
    # a sign error in one survives the two cases above.
    lat, lon = px4_reproject(3000.0, -1000.0, 36.37, 126.42)
    phi, phi0, dlam = math.radians(lat), math.radians(36.37), math.radians(lon - 126.42)
    arg = math.acos(math.sin(phi0) * math.sin(phi) + math.cos(phi0) * math.cos(phi) * math.cos(dlam))
    k = arg / math.sin(arg)
    back = (k * (math.cos(phi0) * math.sin(phi) - math.sin(phi0) * math.cos(phi) * math.cos(dlam))
            * PX4_EARTH_RADIUS, k * math.cos(phi) * math.sin(dlam) * PX4_EARTH_RADIUS)
    expect("reproject round trip", tuple(round(v, 6) for v in back), (3000.0, -1000.0))
    # The same point 2 m up from an EKF2 origin at 5 m MSL, against a replay origin
    # at 30 m ellipsoidal, 5 m MSL: 110.575 m north and 1.999 m up (the tangent
    # plane rises 1 mm over 110 m). The second row's EKF2 origin has moved 0.001
    # deg north and 10 m up with no height reference (`z_global` false): placed
    # about its own origin it is 221.150 m north with no down, where the first
    # row's origin would put it back at 110.575. A row with no origin has no
    # position, and a fix with no MSL height places horizontal only.
    rows = []
    local = Fixture("vehicle_local_position", timestamp=[0, 1, 2], x=[arc, arc, arc],
                    y=[0.0, 0.0, 0.0], z=[-2.0, -2.0, -2.0], vx=[0.0] * 3, vy=[0.0] * 3,
                    vz=[0.0] * 3, xy_global=[1, 1, 0], z_global=[1, 0, 0],
                    ref_lat=[0.0, 0.001, 0.0], ref_lon=[0.0] * 3, ref_alt=[5.0, 15.0, 5.0])
    reference_local(local, rows, (0.0, 0.0, 30.0, 5.0))
    expect("placed", [round(rows[0][2][k], 3) for k in ("pos_n", "pos_e", "pos_d")],
           [110.575, 0.0, -1.999])
    expect("each row about its own origin", [None if rows[1][2][k] is None
                                            else round(rows[1][2][k], 3)
                                            for k in ("pos_n", "pos_e", "pos_d")],
           [221.15, 0.0, None])
    expect("row with no origin", [rows[2][2][k] for k in ("pos_n", "pos_e", "pos_d")],
           [None, None, None])
    expect("placement note", position_note(local, (0.0, 0.0, 30.0, 5.0)),
           "EKF2 position in replay frame: n e d")
    rows = []
    reference_local(local, rows, (0.0, 0.0, 30.0, None))
    # Centimetres: unshifted, the point sits 23 m lower, a quarter-millimetre of arc.
    expect("no MSL height", [None if rows[0][2][k] is None else round(rows[0][2][k], 2)
                             for k in ("pos_n", "pos_e", "pos_d")], [110.57, 0.0, None])
    expect("no MSL height note", position_note(local, (0.0, 0.0, 30.0, None)),
           "EKF2 position in replay frame: n e (the fix logs no MSL height)")
    expect("no replay origin note", position_note(local, None),
           "EKF2 position in replay frame: none (the replay input has no GNSS fix)")
    # At the equator a thousandth of a degree is a(1 - e^2) pi/180e3 = 110.574 m of
    # latitude and (a + 10 m) pi/180e3 = 111.320 m of longitude at 10 m up, and 110 m of arc drops
    # 2 mm below the tangent plane: an origin north, east and 10 m above reads
    # positive, positive, and -9.998. A swapped argument order negates all three.
    expect("origin offset", origin_offset_note((0.001, 0.001, 10.0), (0.0, 0.0, 0.0, 0.0)),
           "EKF2 origin in replay frame: 110.574 111.320 -9.998 m")
    # A replay origin at 30 m on the ellipsoid and 5 m MSL, a geoid height of 25 m:
    # an EKF2 origin at 5 m MSL is the same height, not 25 m below it.
    expect("datum", origin_offset_note((0.0, 0.0, 5.0), (0.0, 0.0, 30.0, 5.0)),
           "EKF2 origin in replay frame: 0.000 0.000 0.000 m")
    expect("no MSL height", origin_offset_note((0.0, 0.0, 5.0), (0.0, 0.0, 30.0, None)),
           "EKF2 origin in replay frame: 0.000 0.000 none m")
    expect("no EKF2 origin", origin_offset_note(None, (0.0, 0.0, 0.0, 0.0)).split(" (")[0],
           "EKF2 origin in replay frame: none")
    expect("no replay origin", origin_offset_note((0.0, 0.0, 0.0), None).split(" (")[0],
           "EKF2 origin in replay frame: none")
    # One bit per sample beside a pair, so each share names its bit: an off-by-one
    # position moves a share to its neighbour.
    status = Fixture("estimator_status",
                     control_mode_flags=[1 << 2 | 1 << 9, 1 << 4, 1 << 5 | 1 << 11, 1 << 9])
    expect("aiding shares", ekf2_aiding_note(status, {"EKF2_HGT_REF": 1}),
           "EKF2 aiding: height reference gps (EKF2_HGT_REF 1); share of control_mode_flags "
           "samples: gnss_pos 0.25, mag_hdg 0.25, mag_3d 0.25, baro_hgt 0.50, gps_hgt 0.25")
    expect("the older name", ekf2_aiding_note(None, {"EKF2_HGT_MODE": 0}).split(";")[0],
           "EKF2 aiding: height reference baro (EKF2_HGT_MODE 0)")
    expect("neither", ekf2_aiding_note(None, {}).split(" (")[0],
           "EKF2 aiding: height reference unknown")
    expect("7592c9b2's estimator", estimator_note({"SYS_MC_EST_GROUP": 1}),
           "Estimator: lpe (SYS_MC_EST_GROUP 1)")
    expect("a later one", estimator_note({"EKF2_EN": 1, "LPE_EN": 0}),
           "Estimator: ekf2 (EKF2_EN 1, LPE_EN 0)")
    expect("no selection", estimator_note({}), "Estimator: unknown (no selection parameter)")
    expect("noise from the log and a default",
           gnss_noise_note({"EKF2_GPS_P_NOISE": 0.5, "EKF2_GPS_V_NOISE": 0.30000001192092896}),
           "GNSS noise parameters: EKF2_GPS_P_NOISE 0.5, EKF2_GPS_V_NOISE 0.3, "
           "EKF2_NOAID_NOISE 10 (PX4 default; not in the log)")

    delays, line = measurement_delays({"EKF2_GPS_DELAY": 110.0, "EKF2_BARO_DELAY": 0.0})
    expect("an older log's delays", line,
           "Measurement delays: gnss 110 ms (EKF2_GPS_DELAY), baro 0 ms (EKF2_BARO_DELAY), "
           "mag 0 ms (not in the log)")
    expect("both halves of a fix", (delays["gnss_pos"], delays["gnss_vel"]), (110000, 110000))
    # The heading field is not the evidence: a299e722 fills it with the configuration on,
    # every other corpus log leaves it NaN with the configuration off, and a log whose
    # driver fills it with the configuration off must not be read as a yaw.
    expect("GPS_CTRL bit 3", gnss_yaw_enabled({"EKF2_GPS_CTRL": 15}), True)
    expect("GPS_CTRL default", gnss_yaw_enabled({"EKF2_GPS_CTRL": 7}), False)
    expect("AID_MASK bit 7", gnss_yaw_enabled({"EKF2_AID_MASK": 385}), True)
    # 2c42096b's 131 has bit 7 set and its receiver logs no heading at all, which is the
    # other half of the rule: rows need the configuration *and* a finite heading.
    expect("AID_MASK 131", gnss_yaw_enabled({"EKF2_AID_MASK": 131}), True)
    expect("AID_MASK without", gnss_yaw_enabled({"EKF2_AID_MASK": 3}), False)
    expect("GPS_CTRL wins", gnss_yaw_enabled({"EKF2_GPS_CTRL": 7, "EKF2_AID_MASK": 128}), False)
    expect("neither", gnss_yaw_enabled({}), False)
    expect("the newer name wins", measurement_delays(
        {"SENS_GPS0_DELAY": 33.0, "EKF2_GPS_DELAY": 110.0})[0]["gnss_pos"], 33000)

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
        "--gnss-heading-variance", type=float, default=DEFAULT_GNSS_HEADING_VARIANCE,
        help="rad^2 on a dual-antenna heading whose receiver logs no accuracy "
             f"(default: {DEFAULT_GNSS_HEADING_VARIANCE}, PX4's floor)",
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
        rows, used, dropout_note, imu_dt, parameters, origin, delays = convert(
            args.ulog, args.baro_variance, args.mag_variance, args.gnss_heading_variance
        )
        note = [
            f"Converted from {args.ulog.name} by tools/ulog2replay.py",
            *parameters,
            "Topics used: " + ", ".join(f"{k}={v}" for k, v in sorted(used.items())),
            f"baro variance {args.baro_variance} m^2 and mag heading variance "
            f"{args.mag_variance} rad^2 are assumed; PX4 logs neither.",
            "Timestamps rebased to the first sample.",
        ]
        if any(row[1] == "gnss_yaw" for row in rows):
            note.append(
                f"gnss heading variance {args.gnss_heading_variance} rad^2 is assumed where "
                "the receiver logs no heading_accuracy."
            )
        if dropout_note:
            note.append(dropout_note)
        count, t0 = write_rows(rows, output, note, delays)

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
            if write_reference(reference, target, t0, imu_dt, args.ulog.name, origin):
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
