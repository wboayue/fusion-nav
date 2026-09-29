#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///
"""Convert UrbanNav-HK-Medium-Urban-1 into the replay format, with its truth.

    uv run tools/urbannav2replay.py data/urbannav --receiver m8t -o out.csv --truth out.truth.csv

UrbanNav is the gate benchmark of #60: a car in Hong Kong's urban canyons, receivers that
are wrong by metres to hundreds of metres for tens of seconds while claiming a few, and a
SPAN-CPT solution to say they were wrong. `data/urbannav.txt` pins the three files this
reads and carries the terms: UrbanNav states no licence, so neither the inputs nor what
this writes are ever committed (GOALS.md, "Secondary sources").

No rosbag is read. The dataset publishes each piece this needs outside its 34 GB bag: the
IMU as a `rostopic echo -p` CSV, each receiver's own solution as NMEA inside the GNSS zip,
and the truth as text. So the converter is the standard library alone, and no ROS tooling
reaches it, let alone the test path (GOALS.md, "Harness constraint").

What each source is, and what this does to it:

* **IMU**, an Xsens MTi-10 at 400 Hz. Its axes are right, forward, up, which the at-rest
  specific force (+9.79 m/s^2 on z) and the gyroscope's z against the truth's heading rate
  both confirm; the replay's are forward, right, down, so (x, y, z) becomes (y, x, -z).
  Timed by `field.header.stamp`, which is UTC: one-second means of the gyroscope's yaw rate
  against the truth's heading differences agree at an offset of 0 s (0.15 deg/s RMS, against
  0.57 to 0.60 at +-0.2 s), and not at the 18 s of GPS time. The MTi-10's magnetometer is not published, so no heading row.
* **GNSS**, one u-blox receiver's `$PUBX,00`, the one sentence carrying its own accuracy:
  `hAcc` and `vAcc` as sigma, squared into the variance columns, and a height above the
  ellipsoid, the datum the truth's `H-Ell` is on. 3D fixes only (`G3`, `D3`): a `NF` row
  still carries a position, hundreds of metres out, which the receiver itself disowns.
  Velocity is speed and course over ground and `vVel` (positive down); u-blox publishes no
  speed accuracy in NMEA, so its variance is a constant, `VELOCITY_SIGMA`.
* **Truth**, SPAN-CPT post-processed in Inertial Explorer at 1 Hz. Its point is the SPAN's,
  0.14 m above the Xsens (`body_T_SPAN` in the dataset's `extrinsic.yaml`), so each row is
  moved to the Xsens, the point the filter estimates. Its velocity is on body axes and is
  rotated onto north, east, down by its own attitude. It carries no bias, so the bias
  columns are blank: nothing here knows them.

The antenna of both u-blox receivers is 0.86 m ahead of the Xsens and 0.31 m below it
(`ANTENNA_T_IMU`, measured by hand to +-0.1 m, and the maintainers' answer in
IPNL-POLYU/UrbanNavDataset#89, "in front and below"), written as the `# GNSS antenna` line.
"""

from __future__ import annotations

import argparse
import datetime
import io
import math
import sys
import zipfile
from pathlib import Path

from geodesy import geodetic_to_ned
from gnss_noise import GNSS_NOISE_PARAMETERS, gnss_noise_note
from replay_format import source_tag, write_replay, write_truth
from rotations import rotate, rotation

SEGMENT = "UrbanNav-HK-Medium-Urban-1"
IMU_FILE = "xsense_imu_medium_urban1.csv"
GNSS_FILE = "gnss_medium_urban1.zip"
TRUTH_FILE = "UrbanNav_TST_GT_raw_with_std.txt"

# The two receivers with a `$PUBX,00`, and so with an accuracy of their own. The other
# u-blox files and the phones carry GGA alone, which has no sigma to fuse or to judge.
RECEIVERS = {
    # GPS and BeiDou, single frequency: the hostile one. It reports an hAcc of 4.5 to 24 m
    # while 200 to 470 m out, for a minute at a time.
    "m8t": f"{SEGMENT}.ublox.m8t.GC.nmea",
    # Dual frequency with differential corrections: honest to its hAcc on this segment,
    # which is what makes it the test of rejecting good fixes at road speed.
    "f9p": f"{SEGMENT}.ublox.f9p.nmea",
}

# Where the antenna sits from the Xsens, forward, right, down, in metres: `ANTENNA_T_IMU`
# (0, 0.86, -0.31) on the dataset's right, forward, up axes.
ANTENNA_FRD = (0.86, 0.0, 0.31)

# Where the SPAN sits from the Xsens, forward, right, down: `body_T_SPAN` (0, 0, 0.14) on
# right, forward, up.
SPAN_FRD = (0.0, 0.0, -0.14)

# Speed accuracy the receiver does not publish, as a sigma in m/s on each axis. PX4's
# `EKF2_GPS_V_NOISE` default, the floor EKF2 puts under any receiver's own `sAcc`, so
# `--r-policy px4` fuses what raw does.
VELOCITY_SIGMA = dict(GNSS_NOISE_PARAMETERS)["EKF2_GPS_V_NOISE"]

# The course's sideslip, sigma in radians: 3 deg. The truth's own body velocity, above
# 3 m/s, puts the angle between travel and heading at 1.44 deg RMS with a 1.25 deg mean and
# 3.9 deg at its 99th percentile. The mean is a share every reading carries, so the sigma
# sits at twice the RMS rather than at it.
COURSE_SIDESLIP = math.radians(3.0)

# A 3D fix, with or without differential corrections. Everything else u-blox reports here
# (NF, DR, G2, D2, RK, TT) is not a position the receiver stands behind in three axes.
FIXED = {"G3", "D3"}


class ConversionError(Exception):
    pass


def nmea_ok(line):
    """True when a sentence's `*hh` checksum matches: the NMEA files interleave binary UBX."""
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, tail = line[1:].partition("*")
    want = 0
    for ch in body:
        want ^= ord(ch)
    return tail[:2].upper() == f"{want:02X}"


def degrees_of(value, hemisphere):
    """`ddmm.mmmm` or `dddmm.mmmm` and its hemisphere letter, as signed degrees."""
    whole = int(float(value) / 100)
    minutes = float(value) - 100 * whole
    sign = -1.0 if hemisphere in ("S", "W") else 1.0
    return sign * (whole + minutes / 60.0)


def utc_ns(date, hhmmss):
    """Nanoseconds since the Unix epoch of a UTC date and an NMEA `hhmmss.ss`."""
    midnight = datetime.datetime(date.year, date.month, date.day,
                                 tzinfo=datetime.timezone.utc)
    seconds = int(hhmmss[0:2]) * 3600 + int(hhmmss[2:4]) * 60 + float(hhmmss[4:])
    return int(midnight.timestamp()) * 10**9 + round(seconds * 1e9)


def read_nmea(text):
    """Every 3D `$PUBX,00` fix: (t_ns, lat, lon, h_ell, hacc, vacc, v_ned).

    The date comes from the first `$GNZDA` or `$GNRMC`, since `$PUBX,00` carries a time of
    day alone; a segment crossing midnight is refused rather than dated wrong.
    """
    date, fixes, last = None, [], None
    for raw in text.splitlines():
        line = raw.strip()
        if not nmea_ok(line):
            continue
        cells = line.split("*")[0].split(",")
        if date is None and cells[0].endswith("ZDA") and cells[2]:
            date = datetime.date(int(cells[4]), int(cells[3]), int(cells[2]))
        elif date is None and cells[0].endswith("RMC") and cells[9]:
            d = cells[9]
            date = datetime.date(2000 + int(d[4:6]), int(d[2:4]), int(d[0:2]))
        if cells[0] != "$PUBX" or cells[1] != "00" or date is None:
            continue
        if cells[8] not in FIXED or not cells[3] or not cells[11]:
            continue
        t = utc_ns(date, cells[2])
        if last is not None and t < last - 12 * 3600 * 10**9:
            raise ConversionError("the NMEA crosses midnight; date it per day")
        last = t
        speed = float(cells[11]) / 3.6  # km/h
        course = math.radians(float(cells[12]))
        fixes.append((
            t,
            degrees_of(cells[3], cells[4]),
            degrees_of(cells[5], cells[6]),
            float(cells[7]),
            float(cells[9]),
            float(cells[10]),
            (speed * math.cos(course), speed * math.sin(course), float(cells[13])),
        ))
    if not fixes:
        raise ConversionError("no 3D $PUBX,00 fix with a date")
    return fixes


def read_imu(handle):
    """(t_ns, gyro, accel) per row, forward-right-down, from the Xsens' right-forward-up."""
    header = handle.readline().strip().split(",")
    col = {name: i for i, name in enumerate(header)}
    names = ["field.header.stamp"] + [f"field.angular_velocity.{a}" for a in "xyz"] + \
            [f"field.linear_acceleration.{a}" for a in "xyz"]
    missing = [n for n in names if n not in col]
    if missing:
        raise ConversionError(f"IMU columns missing: {missing}")
    rows = []
    for line in handle:
        cells = line.split(",")
        t = int(cells[col[names[0]]])
        wx, wy, wz, ax, ay, az = (float(cells[col[n]]) for n in names[1:])
        rows.append((t, (wy, wx, -wz), (ay, ax, -az)))
    return rows


def read_truth(text):
    """(t_ns, lat, lon, h_ell, v_body_frd, roll, pitch, yaw) per row, radians.

    Latitude and longitude are three tokens each, degrees minutes seconds, which is why
    the columns are counted rather than named. Inertial Explorer's roll, pitch and heading
    are about forward, right and down on this vehicle frame, the replay's convention:
    at rest they read -1.74 and 0.44 deg where the Xsens' own gravity reads -1.54 and 0.53.
    """
    rows = []
    for line in text.splitlines():
        cells = line.split()
        if len(cells) != 30 or not cells[0][0].isdigit():
            continue
        dms = lambda d, m, s: math.copysign(abs(float(d)) + float(m) / 60 + float(s) / 3600,
                                           -1.0 if d.startswith("-") else 1.0)
        vx, vy, vz = (float(c) for c in cells[10:13])  # right, forward, up
        rows.append((
            round(float(cells[0]) * 1e9),
            dms(*cells[3:6]),
            dms(*cells[6:9]),
            float(cells[9]),
            (vy, vx, -vz),
            math.radians(float(cells[16])),
            math.radians(float(cells[17])),
            math.radians(float(cells[18])),
        ))
    if not rows:
        raise ConversionError("no truth rows")
    return rows


def convert(directory, receiver, out, truth_out):
    directory = Path(directory)
    imu_path, zip_path, truth_path = (directory / f for f in (IMU_FILE, GNSS_FILE, TRUTH_FILE))
    with zipfile.ZipFile(zip_path) as archive:
        nmea = archive.read(RECEIVERS[receiver]).decode("ascii", "replace")
    fixes = read_nmea(nmea)
    with open(imu_path) as handle:
        imu = read_imu(handle)
    truth = read_truth(truth_path.read_text())

    # The earliest thing in any of the three files, so no time is negative: the receiver and
    # the truth both start a fraction of a second before the first IMU sample.
    t0 = min(imu[0][0], fixes[0][0], truth[0][0])
    lat0, lon0, h0 = truth[0][1:4]
    name = f"urbannav-tst-{receiver}"
    tag = source_tag([imu_path, zip_path, truth_path])

    rows = []
    for t, gyro, accel in imu:
        rows.append((t, "imu", gyro + accel, (None,) * 3))
    for t, lat, lon, h, hacc, vacc, v in fixes:
        rows.append((t, "gnss_pos", geodetic_to_ned(lat, lon, h, lat0, lon0, h0),
                     (hacc ** 2, hacc ** 2, vacc ** 2)))
        rows.append((t, "gnss_vel", v, (VELOCITY_SIGMA ** 2,) * 3))
    # Stable, so a fix and its velocity keep the order written, and an IMU sample stamped
    # at the same nanosecond comes first.
    rows.sort(key=lambda r: r[0])

    header = [
        f"fusion-nav converted `{name}` from {SEGMENT}, source {tag}",
        f"Converted from {SEGMENT} ({RECEIVERS[receiver]}) by tools/urbannav2replay.py",
        "UrbanNav states no licence: never commit this file or anything drawn from it "
        "(data/urbannav.txt)",
        f"Course sideslip {COURSE_SIDESLIP:.6f} rad (3 deg; the truth's own sideslip, "
        "1.44 deg RMS above 3 m/s)",
        f"GNSS antenna {ANTENNA_FRD[0]:.3f} {ANTENNA_FRD[1]:.3f} {ANTENNA_FRD[2]:.3f} m "
        "(forward, right, down from the IMU; ANTENNA_T_IMU)",
        f"Navigation origin {lat0:.9f} {lon0:.9f} {h0:.3f} (lat deg, lon deg, height m; "
        "the first truth row)",
        # PX4's defaults, the parameters `--r-policy px4` reads: this is no PX4 log.
        gnss_noise_note({}),
        f"Velocity variance {VELOCITY_SIGMA ** 2:.6g} m^2/s^2 on every axis: u-blox NMEA "
        "publishes no speed accuracy",
    ]
    write_replay(out, header, rows, t0, 1e-9)

    truth_rows = []
    for t, lat, lon, h, v_body, roll, pitch, yaw in truth:
        r = rotation(roll, pitch, yaw)
        span = geodetic_to_ned(lat, lon, h, lat0, lon0, h0)
        arm = rotate(r, SPAN_FRD)
        position = tuple(p - a for p, a in zip(span, arm))
        truth_rows.append(((t - t0) * 1e-9, position, rotate(r, v_body), (roll, pitch, yaw)))
    write_truth(truth_out, [
        f"fusion-nav truth for `{name}.csv`, source {tag}",
        f"SPAN-CPT from {TRUTH_FILE}, moved to the Xsens (body_T_SPAN); "
        "no bias is known, so those columns are blank",
    ], truth_rows)
    return len(imu), len(fixes), len(truth)


def self_test():
    failures = 0

    def expect(what, got, want, tolerance=None):
        nonlocal failures
        ok = (abs(got - want) <= tolerance) if tolerance is not None else got == want
        if not ok:
            failures += 1
            print(f"FAIL {what}: got {got!r}, want {want!r}", file=sys.stderr)

    # Checksums: one real sentence from the segment, and the same with a digit changed.
    good = "$GNGST,023255.00,48,,,,3.6,3.8,6.8*46"
    expect("checksum", nmea_ok(good), True)
    expect("bad checksum", nmea_ok(good.replace("3.6", "3.7")), False)
    expect("no checksum", nmea_ok("$GNGST,023255.00"), False)

    expect("latitude", degrees_of("2218.07038", "N"), 22 + 18.07038 / 60, 1e-12)
    expect("west", degrees_of("11410.73832", "W"), -(114 + 10.73832 / 60), 1e-12)
    expect("utc", utc_ns(datetime.date(2021, 5, 17), "023255.50"),
           1621218775_500_000_000)

    def sentence(body):
        want = 0
        for ch in body:
            want ^= ord(ch)
        return f"${body}*{want:02X}"

    # Course 90 deg at 36 km/h is 10 m/s east; vVel is positive down. A NF row, and a
    # fix before any date, are both dropped.
    text = "\n".join([
        sentence("PUBX,00,023254.00,2218.07038,N,11410.73832,E,-0.845,G3,5.3,6.8,"
                 "36.0,90.00,0.236,,0.92,1.37,0.98,8,0,0"),
        sentence("GNZDA,023255.00,17,05,2021,00,00"),
        sentence("PUBX,00,023255.00,2218.07038,N,11410.73832,E,-0.845,G3,5.3,6.8,"
                 "36.0,90.00,0.236,,0.92,1.37,0.98,8,0,0"),
        sentence("PUBX,00,023256.00,2218.07038,N,11410.73832,E,-0.845,NF,5.3,6.8,"
                 "36.0,90.00,0.236,,0.92,1.37,0.98,8,0,0"),
    ])
    fixes = read_nmea(text)
    expect("fixes kept", len(fixes), 1)
    t, lat, lon, h, hacc, vacc, v = fixes[0]
    expect("fix time", t, 1621218775_000_000_000)
    expect("height is altRef", h, -0.845)
    expect("hAcc", hacc, 5.3)
    expect("vAcc", vacc, 6.8)
    expect("north", v[0], 0.0, 1e-9)
    expect("east", v[1], 10.0, 1e-9)
    expect("down", v[2], 0.236)

    # Right-forward-up to forward-right-down: an asymmetric row, so a swap cannot pass.
    handle = io.StringIO(
        "%time,field.header.stamp,field.angular_velocity.x,field.angular_velocity.y,"
        "field.angular_velocity.z,field.linear_acceleration.x,field.linear_acceleration.y,"
        "field.linear_acceleration.z\n"
        "0,1621218775548635005,0.1,0.2,0.3,1.0,2.0,9.8\n")
    (t, gyro, accel), = read_imu(handle)
    expect("imu time", t, 1621218775548635005)
    expect("gyro frd", gyro, (0.2, 0.1, -0.3))
    expect("accel frd", accel, (2.0, 1.0, -9.8))

    # A row copied from the truth file, with a heading of 90 deg made up for the check:
    # forward travel at 4 m/s is then 4 m/s east.
    row = ("1621218786.00 2158.00000  95604.00   22 18 04.18249  114 10 44.43134        3.358"
           "   0.000   4.000   0.000  -0.435   0.643  -0.105  0.0 0.0 90.0 3        0.124"
           "        0.153        0.116   0.0326857865     0.007     0.008     0.004"
           "   0.0056323488   0.0056190505   0.0000000000")
    (t, lat, lon, h, v_body, roll, pitch, yaw), = read_truth(row)
    expect("truth time", t, 1621218786_000_000_000)
    expect("truth lat", lat, 22 + 18 / 60 + 4.18249 / 3600, 1e-12)
    expect("truth h", h, 3.358)
    expect("body forward", v_body, (4.0, 0.0, -0.0))
    east = rotate(rotation(roll, pitch, yaw), v_body)
    expect("truth velocity north", east[0], 0.0, 1e-12)
    expect("truth velocity east", east[1], 4.0, 1e-12)
    # A roll of 90 deg takes body right onto down.
    down = rotate(rotation(math.pi / 2, 0.0, 0.0), (0.0, 1.0, 0.0))
    expect("roll right to down", down[2], 1.0, 1e-12)

    print(f"urbannav2replay self-test: {'FAIL' if failures else 'ok'}", file=sys.stderr)
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("directory", nargs="?", type=Path,
                        help="where data/fetch.sh --manifest data/urbannav.txt put the files")
    parser.add_argument("--receiver", choices=sorted(RECEIVERS), default="m8t")
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--truth", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not (args.directory and args.output and args.truth):
        parser.error("directory, -o and --truth are required")
    try:
        imu, fixes, truth = convert(args.directory, args.receiver, args.output, args.truth)
    except (ConversionError, OSError, KeyError) as error:
        print(f"urbannav2replay: {error}", file=sys.stderr)
        return 1
    print(f"{args.output}: {imu} IMU rows, {fixes} fixes; {args.truth}: {truth} truth rows",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
