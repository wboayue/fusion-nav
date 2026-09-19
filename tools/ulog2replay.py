#!/usr/bin/env python3
"""Convert a PX4 ULog flight log into the fusion-nav replay CSV format.

    tools/ulog2replay.py data/logs/flight.ulg -o data/logs/flight.csv

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

Requires pyulog:  pip install pyulog
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# WGS84, for the local tangent plane about the first GNSS fix.
WGS84_A = 6_378_137.0
WGS84_E2 = 6.694_379_990_141e-3

# PX4 does not publish a barometer or magnetic-heading variance, so they are
# supplied here. Placeholders in the same sense as Config's defaults: plausible,
# not validated.
DEFAULT_BARO_VARIANCE = 4.0  # m^2
DEFAULT_MAG_VARIANCE = 0.05  # rad^2

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


def convert_gnss(ulog, rows, used):
    dataset = pick(ulog, GNSS_TOPICS)
    if dataset is None:
        print("warning: no GNSS topic; position and velocity aiding omitted", file=sys.stderr)
        return
    geodetic = fields_for(dataset, GNSS_GEODETIC)
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
        if fix is not None and fix[k] < 3:
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


def convert(path, baro_variance, mag_variance):
    try:
        from pyulog import ULog
    except ImportError:
        raise ConversionError(
            "pyulog is not installed. `pip install pyulog`, or use a virtualenv:\n"
            "  python3 -m venv .venv && .venv/bin/pip install pyulog"
        ) from None

    ulog = ULog(str(path))
    rows = []
    used = {}
    note = dropouts(ulog)
    sensor_combined = convert_imu(ulog, rows, used)
    convert_gnss(ulog, rows, used)
    convert_baro(ulog, rows, used, baro_variance, sensor_combined)
    convert_mag(ulog, rows, used, mag_variance, sensor_combined)
    return rows, used, note


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


def write_reference(ulog, out, t0):
    """EKF2's own solution and innovation ratios, for a side-by-side diff.

    One union schema with blank cells, matching how the replay input handles
    sources with different arities.
    """
    local = pick(ulog, ["vehicle_local_position"])
    status = pick(ulog, ["estimator_status"])
    if local is None and status is None:
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

    reference = []
    if local is not None:
        t = stamps(local)
        columns = [column(local, f) for f in ("x", "y", "z", "vx", "vy", "vz")]
        for k in range(len(t)):
            reference.append((t[k], "ekf2_state", [c[k] for c in columns], []))
    if status is not None and ratio_fields:
        t = stamps(status)
        columns = [column(status, f) if f else None for f in ratio_fields]
        for k in range(len(t)):
            reference.append(
                (t[k], "ekf2_ratio", [], [c[k] if c is not None else None for c in columns])
            )
    reference.sort(key=lambda r: r[0])

    with open(out, "w", newline="") as handle:
        handle.write("# EKF2's own solution, for comparison. Never filter input.\n")
        handle.write(f"# Ratio fields: {ratio_fields}\n")
        handle.write("# Same timebase as the replay CSV: rebased to its first sample.\n")
        handle.write("t_s,source,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,")
        handle.write("r_gnss_pos,r_gnss_vel,r_baro,r_mag\n")
        for timestamp, source, values, ratios in reference:
            values = list(values) + [None] * (6 - len(values))
            ratios = list(ratios) + [None] * (4 - len(ratios))
            cells = [f"{(timestamp - t0) * 1e-6:.6f}", source]
            cells += ["" if v is None else f"{v:.6g}" for v in values + ratios]
            handle.write(",".join(cells) + "\n")
    return True


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("ulog", type=Path, help="input .ulg file")
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
    args = parser.parse_args()

    output = args.output or args.ulog.with_suffix(".csv")
    try:
        rows, used, dropout_note = convert(
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
            from pyulog import ULog

            target = (
                args.reference
                if isinstance(args.reference, Path)
                else output.with_suffix(".reference.csv")
            )
            if write_reference(ULog(str(args.ulog)), target, t0):
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
