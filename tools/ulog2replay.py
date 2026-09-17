#!/usr/bin/env python3
"""Convert a PX4 ULog flight log into the fusion-nav replay CSV format.

    tools/ulog2replay.py data/logs/flight.ulg -o data/logs/flight.csv

The output feeds `cargo run --example replay -- <csv>`. See `examples/replay.rs`
for the schema.

With --reference, EKF2's own solution and innovation test ratios are written to a
second file on the same timestamps. That file is deliberately separate from the
replay input: the filter must never be fed the reference it is being compared
against, and an alignment bug then shows up as a visible offset rather than as
silently-wrong aiding.

Converters live outside the test path on purpose. Replaying ULog directly would
make the Rust test suite depend on PX4 tooling, which is exactly the
no-hardware-no-toolchain property the validation claim rests on (GOALS.md,
"Harness constraint"). This script runs once; its CSV output is what CI reads.

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
# supplied here. These are placeholders in the same sense as Config's defaults:
# plausible, not validated.
DEFAULT_BARO_VARIANCE = 4.0  # m^2
DEFAULT_MAG_VARIANCE = 0.05  # rad^2

# Topic and field names drift across PX4 releases, so each source lists
# candidates newest first. The converter reports which it actually used, because
# a silent fallback to a different sensor is how irreproducible numbers happen.
IMU_CANDIDATES = [
    ("sensor_combined", "gyro_rad", "accelerometer_m_s2"),
    ("vehicle_imu", "delta_angle", "delta_velocity"),
]
GNSS_CANDIDATES = ["sensor_gps", "vehicle_gps_position"]
BARO_CANDIDATES = [("vehicle_air_data", "baro_alt_meter")]
MAG_CANDIDATES = [
    ("vehicle_magnetometer", "magnetometer_ga"),
    ("sensor_mag", None),  # separate x/y/z fields
]


class ConversionError(Exception):
    pass


def geodetic_to_ned(lat, lon, alt, lat0, lon0, alt0):
    """Local tangent plane about (lat0, lon0, alt0). Degrees in, meters out.

    Small-angle on the WGS84 ellipsoid: exact enough over a flight, and it keeps
    the converter free of a geodesy dependency.
    """
    phi0 = math.radians(lat0)
    s = math.sin(phi0)
    denominator = 1.0 - WGS84_E2 * s * s
    meridian = WGS84_A * (1.0 - WGS84_E2) / denominator**1.5
    prime_vertical = WGS84_A / math.sqrt(denominator)

    north = math.radians(lat - lat0) * (meridian + alt0)
    east = math.radians(lon - lon0) * (prime_vertical + alt0) * math.cos(phi0)
    down = -(alt - alt0)
    return north, east, down


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


def convert(path, baro_variance, mag_variance, first_fix_only):
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

    # --- IMU ---------------------------------------------------------------
    imu = None
    for name, gyro_field, accel_field in IMU_CANDIDATES:
        dataset = pick(ulog, [name])
        if dataset is not None:
            imu = (dataset, gyro_field, accel_field)
            break
    if imu is None:
        raise ConversionError(
            "no IMU topic found; looked for "
            + ", ".join(name for name, _, _ in IMU_CANDIDATES)
        )
    dataset, gyro_field, accel_field = imu
    used["imu"] = dataset.name
    if dataset.name == "vehicle_imu":
        raise ConversionError(
            "`vehicle_imu` publishes integrated delta-angle and delta-velocity, "
            "which the replay schema does not carry. Re-log with "
            "`sensor_combined` enabled."
        )
    t = dataset.data["timestamp"]
    gyro = [column(dataset, gyro_field, i) for i in range(3)]
    accel = [column(dataset, accel_field, i) for i in range(3)]
    for k in range(len(t)):
        rows.append(
            (t[k], "imu", [g[k] for g in gyro] + [a[k] for a in accel], [])
        )

    # --- GNSS --------------------------------------------------------------
    gnss = pick(ulog, GNSS_CANDIDATES)
    if gnss is None:
        print("warning: no GNSS topic; position and velocity aiding omitted", file=sys.stderr)
    else:
        used["gnss"] = gnss.name
        gt = gnss.data["timestamp"]
        # 1e-7 degrees and millimeters, as PX4 logs them.
        lat = column(gnss, "lat") * 1e-7
        lon = column(gnss, "lon") * 1e-7
        alt = column(gnss, "alt") * 1e-3
        fix = gnss.data.get("fix_type")
        eph = column(gnss, "eph")
        epv = column(gnss, "epv")

        origin = None
        for k in range(len(gt)):
            if fix is not None and fix[k] < 3:
                continue
            if origin is None:
                origin = (lat[k], lon[k], alt[k])
                if first_fix_only:
                    print(
                        f"navigation origin: {origin[0]:.7f}, {origin[1]:.7f}, "
                        f"{origin[2]:.2f} m",
                        file=sys.stderr,
                    )
            north, east, down = geodetic_to_ned(lat[k], lon[k], alt[k], *origin)
            # eph/epv are standard deviations; the filter wants variances.
            horizontal = float(eph[k]) ** 2
            vertical = float(epv[k]) ** 2
            rows.append(
                (gt[k], "gnss_pos", [north, east, down], [horizontal, horizontal, vertical])
            )

            try:
                vn = column(gnss, "vel_n_m_s")[k]
                ve = column(gnss, "vel_e_m_s")[k]
                vd = column(gnss, "vel_d_m_s")[k]
                sigma = float(column(gnss, "s_variance_m_s")[k])
            except ConversionError:
                continue
            speed_variance = sigma**2
            rows.append(
                (
                    gt[k],
                    "gnss_vel",
                    [vn, ve, vd],
                    [speed_variance, speed_variance, speed_variance],
                )
            )
        if origin is None:
            print("warning: no 3D GNSS fix in the log", file=sys.stderr)

    # --- Barometer ---------------------------------------------------------
    for name, field in BARO_CANDIDATES:
        dataset = pick(ulog, [name])
        if dataset is None:
            continue
        used["baro"] = dataset.name
        bt = dataset.data["timestamp"]
        altitude = column(dataset, field)
        for k in range(len(bt)):
            rows.append((bt[k], "baro", [altitude[k]], [baro_variance]))
        break
    else:
        print("warning: no barometer topic; altitude aiding omitted", file=sys.stderr)

    # --- Magnetometer ------------------------------------------------------
    for name, field in MAG_CANDIDATES:
        dataset = pick(ulog, [name])
        if dataset is None:
            continue
        used["mag"] = dataset.name
        mt = dataset.data["timestamp"]
        if field is None:
            components = [column(dataset, axis) for axis in ("x", "y", "z")]
        else:
            components = [column(dataset, field, i) for i in range(3)]
        for k in range(len(mt)):
            rows.append((mt[k], "mag", [c[k] for c in components], [mag_variance]))
        break
    else:
        print("warning: no magnetometer topic; heading aiding omitted", file=sys.stderr)

    return rows, used


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
            # ULog timestamps are microseconds since boot.
            t = (timestamp - t0) * 1e-6
            values = list(values) + [None] * (6 - len(values))
            variances = list(variances) + [None] * (3 - len(variances))
            cells = [f"{t:.6f}", source]
            cells += ["" if v is None else f"{v:.6g}" for v in values + variances]
            handle.write(",".join(cells) + "\n")
    return len(rows), t0


def write_reference(path, out, t0):
    """EKF2's own solution and innovation test ratios, for side-by-side diff."""
    from pyulog import ULog

    ulog = ULog(str(path))
    local = pick(ulog, ["vehicle_local_position"])
    status = pick(ulog, ["estimator_status"])
    if local is None and status is None:
        print("warning: no EKF2 topics; reference not written", file=sys.stderr)
        return 0

    ratio_fields = ("pos_test_ratio", "vel_test_ratio", "hgt_test_ratio", "mag_test_ratio")
    with open(out, "w", newline="") as handle:
        handle.write("# EKF2's own solution, for comparison. Not filter input.\n")
        handle.write("t_s,source,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d\n")
        if local is not None:
            t = local.data["timestamp"]
            fields = [column(local, f) for f in ("x", "y", "z", "vx", "vy", "vz")]
            for k in range(len(t)):
                cells = [f"{(t[k] - t0) * 1e-6:.6f}", "ekf2_state"]
                cells += [f"{f[k]:.6g}" for f in fields]
                handle.write(",".join(cells) + "\n")
        if status is not None:
            handle.write("# t_s,source," + ",".join(ratio_fields) + "\n")
            t = status.data["timestamp"]
            try:
                ratios = [column(status, f) for f in ratio_fields]
            except ConversionError as e:
                print(f"warning: {e}; test ratios not written", file=sys.stderr)
                return 1
            for k in range(len(t)):
                cells = [f"{(t[k] - t0) * 1e-6:.6f}", "ekf2_test_ratio"]
                cells += [f"{r[k]:.4f}" for r in ratios]
                handle.write(",".join(cells) + "\n")
    return 1


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("ulog", type=Path, help="input .ulg file")
    parser.add_argument("-o", "--output", type=Path, help="output CSV (default: alongside input)")
    parser.add_argument(
        "--reference",
        type=Path,
        nargs="?",
        const=True,
        help="also write EKF2's solution and test ratios (default: <output>.reference.csv)",
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
        rows, used = convert(args.ulog, args.baro_variance, args.mag_variance, True)
        note = [
            f"Converted from {args.ulog.name} by tools/ulog2replay.py",
            "Topics used: " + ", ".join(f"{k}={v}" for k, v in sorted(used.items())),
            f"baro variance {args.baro_variance} m^2 and mag heading variance "
            f"{args.mag_variance} rad^2 are assumed; PX4 logs neither.",
            "Timestamps rebased to the first sample.",
        ]
        count, t0 = write_rows(rows, output, note)

        if args.reference:
            reference = (
                args.reference
                if isinstance(args.reference, Path)
                else output.with_suffix(".reference.csv")
            )
            write_reference(args.ulog, reference, t0)
            print(f"reference -> {reference}", file=sys.stderr)
    except ConversionError as e:
        print(f"ulog2replay: {e}", file=sys.stderr)
        return 1

    for source, topic in sorted(used.items()):
        print(f"  {source:<5} {topic}", file=sys.stderr)
    print(f"{count} rows -> {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
