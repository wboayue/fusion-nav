#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///
"""Convert one INSANE sequence into the replay format, with its truth.

    uv run tools/insane2replay.py data/insane --sequence outdoor_1 -o out.csv --truth out.truth.csv

INSANE (Brommer et al., IROS 2022, arXiv:2210.09114) is the accuracy benchmark of #9: a
3 kg quadcopter's own PX4 autopilot sensors, with RTK truth. `data/insane.txt` pins the
archives and carries the terms: BSD-2 with a no-Sell condition, so neither the inputs nor
what this writes are ever committed (GOALS.md, "Primary sources"). Each sequence ships as
CSVs, so this is the standard library alone.

**What the truth is built from decides what can be scored.** The dataset's
`gps_mag_orientation.m` (aau-cns/insane_dataset_tools) places the truth at the PX4 IMU from
RTK2's position, and fits its attitude by Wahba to the RTK1-to-RTK2 baseline, weighted 50,
and the PX4 magnetometer, weighted 1. So an RTK fix or the dual-antenna heading would be
scored against itself, and neither is written: the fixes are the PX4 autopilot's own
receiver, the one a flight controller fuses. Yaw truth is the baseline's, and the fused
magnetometer sets only the truth's rotation about the baseline, one tilt axis.

**Attitude truth is coarser than the filter.** Over one-second windows, the truth's own
rotation differs from the integrated gyroscope by a median 1.4 to 1.6 deg on these three
sequences (at the lag below), where the gyroscope is good to about 0.1 deg over a second.
And its tilt is off outright: at rest, the truth rotates the accelerometer's gravity 17 deg
from vertical on `outdoor_1` and 5 to 6 deg on the desert sequences, in the dataset's own
frames. That is the axis the magnetometer sets. So the attitude keys are written, since a
truth file carries them, and are neither pinned nor published (data/insane.sh).

**Its timeline lags the PX4 IMU's**, despite the dataset's own synchronization: that
one-second rotation mismatch is least with the truth moved 80 to 170 ms earlier
(`outdoor_1` 3.92 deg RMS at 0, 3.01 at -170 ms; `mars_19` 4.41 and 2.91; `mars_1` 3.93
and 3.01 at -80, the weakest, its median moving only 1.49 to 1.41). The dataset's own
`time_info.yaml` corroborates two of the three: `t_mag_gps - t_pximu_imugt`, the
magnetometer's offset from the IMU the truth attitude was built on, is 196, 7 and 166 ms.
Uncorrected, 170 ms is 1.2 m of position at `outdoor_1`'s 6.9 m/s. So each sequence's lag
is measured here, the same fit the dataset used and against the same gyroscope, and the
truth is moved by it; a sweep whose best lag sits on its edge is refused rather than
applied. `mars_3`, `mars_4` and `mars_14` are left out: their mismatch is 12 to 16 deg RMS at
every lag, carried by windows where the truth fails outright (medians 1.8 to 2.6 deg), so a
handful of failures would set their lag.

What each stream is, and what this does to it:

* **IMU**, the Pixhawk's at ~200 Hz. Its axes are forward, left, up (the dataset's "ENU"):
  +9.78 m/s^2 on z at rest, and the gyroscope's z has the sign of the truth's yaw rate.
  The replay's are forward, right, down, so (x, y, z) becomes (x, -y, -z).
* **GNSS**, `px4_gps.csv`: the dataset's own east, north, up about its reference, the
  frame the truth is on (within 5 cm of `geodesy.geodetic_to_ned`), with the receiver's own
  variances. Fixes and truth are both moved to the first RTK2 fix, 0.6 m from where the IMU
  starts: a static start puts the filter at zero, and the desert's reference is a kilometre
  away. Its velocity is not written: it is horizontal only, `v_z` reads 0 on every row, and
  it carries no accuracy. Dated `EKF2_GPS_DELAY`'s default before logging, since no
  parameter came with it. Against corrected truth, with each sequence's mean bias removed,
  `outdoor_1` is 2.05 m RMS at 0 ms and 1.87 at 350 ms, and `mars_19` is flat; but 350 ms
  is past the 320 ms (23') keeps (at 380 every fix is discarded), and at 300 ms `pos_h`
  moves 4.304 to 4.299 m, so the default stands.
* **Barometer**, pressure turned into height by PX4's own `getAltitudeFromPressure`.
* **Magnetometer**, the raw PX4 field the truth was built from (its intrinsic correction is
  commented out there), in gauss, on the IMU's axes as it comes: `R_pxmag_pximu` is not
  applied. Against the baseline's yaw, the filter's mean heading error is -8.8, +5.9 and
  +7.5 deg with that rotation (its transpose, as the truth script applies it), -6.1, -2.4
  and -2.6 without; the desert's is a 13.9 deg rotation. `px4_mag` is plausibly PX4's own
  body-frame field already, which the extrinsic would rotate twice. No declination line is
  written: the calibrations' `mag_var.dec` reads 3.4 deg west at Klagenfurt under the truth
  script's own `sph2cart(pi/2 + dec)`, where the filter's table reads 4.6 deg east, so
  data/insane.sh replays under `--declination model`, that table.
* **Truth**, at RTK2's epochs: the pose interpolated from the 80 Hz truth, and velocity the
  mean of the two receivers' Doppler, which is the vehicle centre's (the baseline's
  midpoint) and needs no attitude, moved to the IMU through `imu_velocity`'s 6 cm arm. The
  constructed velocity agrees with the differentiated truth position to a median 4 to 8
  cm/s. No bias is known.

The PX4 receiver's antenna offset is not in INSANE's calibration, so fixes are taken at the
IMU and no antenna line is written.

The three sequences, each covering what the others do not (the screen of all twenty GNSS
sequences is in #9):

* `outdoor_1`, the Klagenfurt model airfield, the only other site: 52 s still, a 24 m climb,
  a receiver 8 m high on average, and a barometer that departs from truth by up to 2.8 m
  during the climb and returns (5 s means; 0.35 m start to end).
* `mars_1`, Negev desert: the receiver claiming the smallest error (sigma 0.77 m
  horizontally) and 1.86 m RMS out, a mean normalized squared error of 3.9.
* `mars_19`, the longest log (371 s, 280 s of it with truth), hovering within 6 m: a
  barometer drifting 1.8 m from truth start to end (5 s means), and logging dropouts (596
  IMU intervals over two and a half periods) that flicker `Degraded`.
"""

from __future__ import annotations

import argparse
import bisect
import math
import re
import sys
import zipfile
from pathlib import Path

from geodesy import geodetic_to_ned
from gnss_noise import gnss_noise_note
from replay_format import source_tag, write_replay, write_truth
from rotations import (angle_between, cross, euler, matmul, quaternion_matrix, rotate,
                       rotation_vector_matrix, transpose)

CALIBRATION = "insane_sensor_calib_preprocessed.zip"

SEQUENCES = ("outdoor_1", "mars_1", "mars_19")

# PX4's EKF2_GPS_DELAY default at c4e4ef98 (`src/modules/ekf2/params_gnss.yaml`), in ns.
GNSS_DELAY_NS = 110_000_000

# The variances ulog2replay substitutes where PX4 logs none, for the same reasons.
BARO_VARIANCE = 4.0  # m^2, sigma = 2.0 m
MAG_VARIANCE = 0.09  # rad^2 on heading, sigma = 0.3 rad

# The lag sweep: one-second windows, every quarter second, in 10 ms steps.
LAGS_MS = range(-400, 201, 10)
WINDOW_ROWS = 80
STRIDE_ROWS = 20

# Body forward-left-up to forward-right-down, and east-north-up to north-east-down.
FLU_TO_FRD = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
ENU_TO_NED = ((0, 1, 0), (1, 0, 0), (0, 0, -1))


class ConversionError(Exception):
    pass


def nanoseconds(text):
    """Integer nanoseconds of a decimal seconds string, exactly: `f64` holds 1e9 s to 0.2 us."""
    whole, _, fraction = text.strip().partition(".")
    return int(whole) * 10**9 + int((fraction + "000000000")[:9])


def read_csv(archive, name):
    """(t_ns, floats...) per row, the header skipped where there is one."""
    rows = []
    for line in archive.read(name).decode().splitlines():
        cells = [c for c in line.split(",") if c.strip()]
        if not cells or not cells[0].strip()[0].isdigit():
            continue
        rows.append((nanoseconds(cells[0]),) + tuple(float(c) for c in cells[1:]))
    if not rows:
        raise ConversionError(f"{name}: no rows")
    return rows


def yaml_numbers(text, key):
    """Every number after `key:` up to the next blank line: the calibrations' matrices."""
    match = re.search(rf"^\s*{re.escape(key)}:(.*?)(?:\n\s*\n|\Z)", text, re.M | re.S)
    if not match:
        raise ConversionError(f"calibration has no {key}")
    return [float(x) for x in re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", match.group(1))]


def pressure_altitude(pa):
    """PX4's `getAltitudeFromPressure` (src/lib/atmosphere/atmosphere.cpp:55-72 at c4e4ef98)."""
    t1, a, r, g = 288.15, -6.5e-3, 287.1, 9.80665
    return ((pa / 101325.0) ** (-(a * r) / g) * t1 - t1) / a


def gyro_attitudes(imu):
    """The attitude the gyroscope alone integrates to at each IMU sample, from the first."""
    r = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    out = [r]
    for a, b in zip(imu, imu[1:]):
        dt = (b[0] - a[0]) * 1e-9
        r = matmul(r, rotation_vector_matrix(tuple(w * dt for w in a[4:7])))
        out.append(r)
    return out


def truth_lag(imu, truth):
    """The truth-to-IMU lag in ns, and the mismatch at it and at zero, in degrees RMS.

    For each lag, one-second windows of the truth's own body rotation against the
    gyroscope's over the same window moved by the lag. Truth's samples are 1 to 2 deg
    noisy, so a rate from neighbouring rows is noise; a second of rotation is not.
    """
    times = [row[0] for row in imu]
    integrated = gyro_attitudes(imu)
    poses = [quaternion_matrix(*row[4:8]) for row in truth]

    def gyro_at(t):
        return integrated[max(0, min(bisect.bisect(times, t) - 1, len(integrated) - 1))]

    mismatch = {}
    for lag_ms in LAGS_MS:
        lag = lag_ms * 1_000_000
        squares = []
        for i in range(0, len(truth) - WINDOW_ROWS, STRIDE_ROWS):
            j = i + WINDOW_ROWS
            truth_turn = matmul(transpose(poses[i]), poses[j])
            gyro_turn = matmul(transpose(gyro_at(truth[i][0] + lag)),
                               gyro_at(truth[j][0] + lag))
            squares.append(math.degrees(angle_between(gyro_turn, truth_turn)) ** 2)
        mismatch[lag_ms] = math.sqrt(sum(squares) / len(squares))
    best = min(mismatch, key=mismatch.get)
    if best in (LAGS_MS[0], LAGS_MS[-1]):
        raise ConversionError(f"the truth's lag is not bracketed: best at the sweep's edge, "
                              f"{best} ms")
    return best * 1_000_000, mismatch[best], mismatch[0]


def truth_pose(truth, times, t):
    """Position and quaternion at `t`, between the 80 Hz rows either side, or None outside.

    Linear in both: the rows are 12.5 ms apart, and `quaternion_matrix` normalizes. The
    later quaternion is taken on the earlier's hemisphere so the blend does not pass
    through zero.
    """
    i = bisect.bisect(times, t)
    if i <= 0 or i >= len(truth):
        return None
    a, b = truth[i - 1], truth[i]
    f = (t - a[0]) / (b[0] - a[0])
    sign = 1.0 if sum(x * y for x, y in zip(a[4:8], b[4:8])) >= 0 else -1.0
    position = tuple(x + f * (y - x) for x, y in zip(a[1:4], b[1:4]))
    q = tuple(x + f * (sign * y - x) for x, y in zip(a[4:8], b[4:8]))
    return position, q


def mean_rate(imu, times, t, half_ns=25_000_000):
    """The gyroscope's mean over +-25 ms about `t`, forward, left, up."""
    lo, hi = bisect.bisect(times, t - half_ns), bisect.bisect(times, t + half_ns)
    if hi <= lo:
        return None
    return tuple(sum(row[4 + k] for row in imu[lo:hi]) / (hi - lo) for k in range(3))


def rtk_velocity(rows, times, t, limit_ns=300_000_000):
    """One receiver's Doppler velocity at `t`, between its epochs either side, or None
    where they are further apart than `limit_ns`: a gap is not interpolated across."""
    i = bisect.bisect(times, t)
    if i <= 0 or i >= len(rows) or times[i] - times[i - 1] > limit_ns:
        return None
    a, b = rows[i - 1], rows[i]
    f = (t - a[0]) / (b[0] - a[0])
    return tuple(x + f * (y - x) for x, y in zip(a[4:7], b[4:7]))


def imu_velocity(centre, world_body, omega, arm):
    """The IMU's velocity from the vehicle centre's: `v + R (w x r)`, `r` centre to IMU.

    The centre is the RTK baseline's midpoint, so its velocity is the two receivers' mean
    and needs no attitude; only this 6 cm arm does, which keeps the truth's tilt error
    (17 deg at rest on `outdoor_1`) out of the velocity.
    """
    lever = rotate(world_body, cross(omega, arm))
    return tuple(v + dv for v, dv in zip(centre, lever))


def site_reference(archive, sequence):
    """The dataset's east-north-up reference, latitude, longitude and height."""
    text = archive.read(f"{sequence}_sensors/README.txt").decode()
    match = re.search(r"=\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)", text)
    if not match:
        raise ConversionError("README.txt names no reference coordinates")
    return tuple(float(x) for x in match.groups())


def convert(directory, sequence, out, truth_out):
    directory = Path(directory)
    if sequence not in SEQUENCES:
        raise ConversionError(f"{sequence} is not one of {sorted(SEQUENCES)}")
    zip_path, calibration_path = directory / f"{sequence}_sensors.zip", directory / CALIBRATION
    prefix = f"{sequence}_sensors"
    with zipfile.ZipFile(calibration_path) as archive:
        sensors = archive.read("insane_sensor_calib_preprocessed/sensor_calibration.yaml")
    # From the vehicle centre, the baseline's midpoint, to the IMU (`R_vc_pximu` is identity).
    arm = tuple(yaml_numbers(sensors.decode(), "p_vc_pximu"))

    with zipfile.ZipFile(zip_path) as archive:
        lat0, lon0, h0 = site_reference(archive, sequence)
        imu = read_csv(archive, f"{prefix}/px4_imu.csv")
        fixes = read_csv(archive, f"{prefix}/px4_gps.csv")
        baro = read_csv(archive, f"{prefix}/px4_baro.csv")
        field = read_csv(archive, f"{prefix}/px4_mag.csv")
        truth = read_csv(archive, f"{prefix}/ground_truth/ground_truth_80hz.csv")
        rtk1 = read_csv(archive, f"{prefix}/ground_truth/rtk_gps1_data_revised.csv")
        rtk2 = read_csv(archive, f"{prefix}/ground_truth/rtk_gps2_data_revised.csv")

    lag, mismatch, at_zero = truth_lag(imu, truth)
    # The replay's origin is the first RTK2 fix, within a metre of where the IMU starts,
    # since a static start puts the filter at zero: the desert's reference is a kilometre
    # from its flights.
    origin = rtk2[0][1:4]
    shift = geodetic_to_ned(*origin, lat0, lon0, h0)

    def ned(enu):
        return tuple(x - o for x, o in zip(rotate(ENU_TO_NED, enu), shift))

    t0 = min(imu[0][0], fixes[0][0], baro[0][0], field[0][0], truth[0][0] + lag)
    name = f"insane-{sequence}"
    tag = source_tag([zip_path, calibration_path])

    rows = []
    for t, ax, ay, az, wx, wy, wz in imu:
        rows.append((t, "imu", rotate(FLU_TO_FRD, (wx, wy, wz)) +
                     rotate(FLU_TO_FRD, (ax, ay, az)), ()))
    for t, _lat, _lon, _alt, e, n, u, var_e, var_n, var_u in fixes:
        rows.append((t, "gnss_pos", ned((e, n, u)), (var_n, var_e, var_u)))
    for t, pa in baro:
        rows.append((t, "baro", (pressure_altitude(pa),), (BARO_VARIANCE,)))
    for t, x, y, z, *_spherical in field:
        # On the IMU's axes as logged; `R_pxmag_pximu` is not applied (module docstring).
        gauss = tuple(c * 1e4 for c in rotate(FLU_TO_FRD, (x, y, z)))
        rows.append((t, "mag", gauss, (MAG_VARIANCE,)))
    # Stable, so an IMU sample stamped at the same nanosecond as a measurement comes first.
    rows.sort(key=lambda r: r[0])

    header = [
        f"fusion-nav converted `{name}` from INSANE {sequence}, source {tag}",
        f"Converted from INSANE {zip_path.name} by tools/insane2replay.py",
        "INSANE's licence forbids selling what derives from it: never commit this file or "
        "anything drawn from it (data/insane.txt)",
        f"Navigation origin {origin[0]:.9f} {origin[1]:.9f} {origin[2]:.3f} (lat deg, lon deg, "
        "height m; the first RTK2 fix)",
        # PX4's defaults, the parameters `--r-policy px4` reads: this is no PX4 log.
        gnss_noise_note({}),
        f"Measurement delays: gnss {GNSS_DELAY_NS / 1e6:.6g} ms (EKF2_GPS_DELAY's PX4 "
        "default; the data cannot pin one), baro 0 ms, mag 0 ms",
        "No declination and no antenna line: replay with --declination model, and the "
        "receiver's antenna is not in INSANE's calibration",
    ]
    write_replay(out, header, rows, t0, 1e-9, {"gnss_pos": GNSS_DELAY_NS})

    imu_times, truth_times = [row[0] for row in imu], [row[0] for row in truth]
    rtk1_times = [row[0] for row in rtk1]
    truth_rows = []
    for t, _lat, _lon, _alt, *v2 in rtk2:
        pose = truth_pose(truth, truth_times, t)
        v1 = rtk_velocity(rtk1, rtk1_times, t)
        omega = mean_rate(imu, imu_times, t + lag)
        if pose is None or v1 is None or omega is None:
            continue
        position, q = pose
        world_body = quaternion_matrix(*q)  # east-north-up from forward-left-up
        centre = tuple((a + b) / 2 for a, b in zip(v1, v2))
        velocity = imu_velocity(centre, world_body, omega, arm)
        attitude = euler(matmul(matmul(ENU_TO_NED, world_body), FLU_TO_FRD))
        truth_rows.append(((t + lag - t0) * 1e-9, ned(position),
                           rotate(ENU_TO_NED, velocity), attitude))
    if not truth_rows:
        raise ConversionError("no truth row falls inside the IMU's span")
    write_truth(truth_out, [
        f"fusion-nav truth for `{name}.csv`, source {tag}",
        "INSANE's truth at RTK2's epochs: position at the PX4 IMU, attitude from the RTK "
        "baseline and the PX4 magnetometer, velocity the two RTK receivers' mean Doppler "
        "moved to the IMU; no bias is known, so those columns are blank",
        f"Moved {lag / 1e6:+.0f} ms onto the PX4 IMU's clock: one-second rotation mismatch "
        f"against the gyroscope {mismatch:.2f} deg RMS there, {at_zero:.2f} at 0",
    ], truth_rows)
    return len(imu), len(fixes), len(truth_rows), lag / 1e6


def convert_fixture(motion):
    """`convert` end to end on a synthetic sequence: the truth moved by its lag and onto
    the first RTK2 fix, its velocity the receivers' mean, and the magnetometer's axes."""
    import tempfile
    failures = 0
    reference, first = (30.6, 34.87, 500.0), (30.601, 34.871, 510.0)
    shift = geodetic_to_ned(*first, *reference)  # where the first RTK2 fix sits, north-east-down
    start_enu = (shift[1], shift[0], -shift[2])
    imu, truth = motion(150_000_000)

    def csv(header, rows):
        return header + "\n" + "".join(",".join(f"{c / 1e9:.9f}" if i == 0 else repr(c)
                                                for i, c in enumerate(r)) + "\n" for r in rows)
    truth_rows = [(t, start_enu[0] + 0.1, start_enu[1] + 0.2, start_enu[2] + 0.3) + tuple(q)
                  for t, _, _, _, *q in truth]
    epoch = truth[40][0]  # an RTK2 epoch on a truth row, stamped as late as the truth
    files = {
        "README.txt": f"Reference\n\t [Latitude,Longitude,Altitude] = {reference[0]}, "
                      f"{reference[1]}, {reference[2]}\n",
        "px4_imu.csv": csv("t, a_x, a_y, a_z, w_x, w_y, w_z", imu),
        "px4_gps.csv": csv("t, lat, long, alt, p_x, p_y, p_z, cov_p_x, cov_p_y, cov_p_z,",
                           [(10**9, 0.0, 0.0, 0.0, start_enu[0] + 1.0, start_enu[1] + 2.0,
                             start_enu[2] + 3.0, 4.0, 5.0, 6.0)]),
        "px4_baro.csv": csv("t, p", [(10**9, 101325.0)]),
        "px4_mag.csv": csv("t, cart_x, cart_y, cart_z, spher_az, spher_el, spher_norm",
                           [(10**9, 1e-5, 2e-5, 3e-5, 0.0, 0.0, 0.0)]),
        "ground_truth/ground_truth_80hz.csv": csv("t, p_x, p_y, p_z, q_w, q_x, q_y, q_z",
                                                  truth_rows),
        "ground_truth/rtk_gps1_data_revised.csv": csv("", [
            (epoch - 10**8, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            (epoch + 10**8, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)]),
        "ground_truth/rtk_gps2_data_revised.csv": csv("", [
            (epoch,) + first + (3.0, 0.0, 0.0)]),
    }
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        with zipfile.ZipFile(directory / "mars_1_sensors.zip", "w") as archive:
            for name, text in files.items():
                archive.writestr(f"mars_1_sensors/{name}", text)
        with zipfile.ZipFile(directory / CALIBRATION, "w") as archive:
            archive.writestr("insane_sensor_calib_preprocessed/sensor_calibration.yaml",
                             "p_vc_pximu: [0, 0, 0]\n\n")
        convert(directory, "mars_1", directory / "out.csv", directory / "truth.csv")
        out = (directory / "out.csv").read_text().splitlines()
        rows = [line.split(",") for line in (directory / "truth.csv").read_text().splitlines()
                if line[0].isdigit()]

    def check(what, got, want, tolerance):
        nonlocal failures
        if any(abs(g - w) > tolerance for g, w in zip(got, want)):
            failures += 1
            print(f"FAIL {what}: got {got!r}, want {want!r}", file=sys.stderr)

    (row,) = rows
    # Stamped 150 ms late, so taken 150 ms before its stamp, on a clock that starts at 0.
    check("truth time", [float(row[0])], [epoch / 1e9 - 0.150], 1e-6)
    # 0.1 east, 0.2 north, 0.3 up of the first RTK2 fix; the fix 1, 2, 3 of it.
    check("truth position", [float(c) for c in row[1:4]], [0.2, 0.1, -0.3], 1e-4)
    fix = next(line.split(",") for line in out if ",gnss_pos," in line)
    check("fix position", [float(c) for c in fix[2:5]], [2.0, 1.0, -3.0], 1e-4)
    # East at 1 m/s and at 3: the centre moves east at 2.
    check("truth velocity", [float(c) for c in row[4:7]], [0.0, 2.0, 0.0], 1e-9)
    mag = next(line.split(",") for line in out if ",mag," in line)
    check("mag axes, gauss", [float(c) for c in mag[2:5]], [0.1, -0.2, -0.3], 1e-9)
    return failures


def self_test():
    failures = 0

    def expect(what, got, want, tolerance=None):
        nonlocal failures
        if tolerance is not None:
            ok = all(abs(g - w) <= tolerance for g, w in
                     zip(got if isinstance(got, tuple) else (got,),
                         want if isinstance(want, tuple) else (want,)))
        else:
            ok = got == want
        if not ok:
            failures += 1
            print(f"FAIL {what}: got {got!r}, want {want!r}", file=sys.stderr)

    # Seven decimals as the dataset writes them, and a value `float` would round.
    expect("ns", nanoseconds("1612483639.8658478"), 1612483639_865847800)
    expect("ns short", nanoseconds("1633587463.5"), 1633587463_500000000)
    expect("ns whole", nanoseconds("12"), 12_000_000_000)

    # An asymmetric vector, so a swapped or unsigned axis cannot pass.
    expect("flu to frd", rotate(FLU_TO_FRD, (1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))
    expect("enu to ned", rotate(ENU_TO_NED, (1.0, 2.0, 3.0)), (2.0, 1.0, -3.0))

    # PX4's own test: 70109 Pa is 3000 m within 0.5 m, and sea level is 0.
    expect("sea level", pressure_altitude(101325.0), 0.0, 1e-9)
    expect("3000 m", pressure_altitude(70109.0), 3000.0, 0.5)

    # A body turned 90 deg to the left in east-north-up (+z): facing north from facing
    # east. In north-east-down that is a yaw of 0 from the 90 deg a forward-east body has.
    east = quaternion_matrix(1.0, 0.0, 0.0, 0.0)
    north = quaternion_matrix(math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    frd = lambda r: euler(matmul(matmul(ENU_TO_NED, r), FLU_TO_FRD))
    expect("facing east", frd(east), (0.0, 0.0, math.pi / 2), 1e-12)
    expect("facing north", frd(north), (0.0, 0.0, 0.0), 1e-12)
    # Nose up 10 deg while facing east: a left-hand turn about body y in FLU is nose down,
    # so nose up is -10 deg about y, and pitch reads +10 in FRD.
    up = quaternion_matrix(math.cos(math.radians(-5)), 0.0, math.sin(math.radians(-5)), 0.0)
    expect("nose up", frd(up), (0.0, math.radians(10), math.pi / 2), 1e-12)

    calibration = ("p_vc_pximu: [0.06, 0, 0]\n\n"
                   "R_pxmag_pximu: [[0.99,0.03,4e-2],\n  [-0.03,0.99,-5e-3],\n"
                   "  [-4e-2,3.2e-3,0.99]]\n\n# next\n")
    expect("arm", yaml_numbers(calibration, "p_vc_pximu"), [0.06, 0.0, 0.0])
    expect("matrix", yaml_numbers(calibration, "R_pxmag_pximu"),
           [0.99, 0.03, 4e-2, -0.03, 0.99, -5e-3, -4e-2, 3.2e-3, 0.99])

    # A centre 1 m ahead of the IMU on a vehicle yawing left at 1 rad/s moves 1 m/s to the
    # left (north, facing east) though the IMU is still: v + R (w x r), `r` from the centre
    # back to the IMU, takes it back to zero.
    expect("lever", imu_velocity((0.0, 1.0, 0.0), east, (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
           (0.0, 0.0, 0.0), 1e-12)

    # Velocity between two epochs, and none across a gap.
    rtk = [(0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0), (100_000_000, 0.0, 0.0, 0.0, 3.0, 2.0, 1.0),
           (900_000_000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
    times = [row[0] for row in rtk]
    expect("rtk mid", rtk_velocity(rtk, times, 50_000_000), (2.0, 2.0, 2.0), 1e-12)
    expect("rtk gap", rtk_velocity(rtk, times, 500_000_000), None)

    # One step integrates the earlier sample's rate over the interval: 1 rad, not 5.
    step = gyro_attitudes([(0, 0.0, 0.0, 9.8, 0.0, 0.0, 1.0), (10**9, 0.0, 0.0, 9.8, 0.0, 0.0, 5.0)])
    expect("earlier sample", angle_between(step[0], step[1]), 1.0, 1e-9)
    # Two quarter turns, about x then about the *body's* z: body rates compose on the right.
    quarter = math.pi / 2
    turns = gyro_attitudes([(0, 0.0, 0.0, 9.8, quarter, 0.0, 0.0),
                            (10**9, 0.0, 0.0, 9.8, 0.0, 0.0, quarter),
                            (2 * 10**9, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0)])
    body = matmul(rotation_vector_matrix((quarter, 0.0, 0.0)),
                  rotation_vector_matrix((0.0, 0.0, quarter)))
    expect("body frame", angle_between(turns[2], body), 0.0, 1e-9)

    # q and -q are one rotation: halfway between them is that rotation, not zero.
    pose = truth_pose([(0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
                       (10, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0)], [0, 10], 5)
    expect("hemisphere", pose[1], (1.0, 0.0, 0.0, 0.0), 1e-12)

    # An analytic attitude, yaw then roll, with the body rates it implies: nothing here comes
    # from `gyro_attitudes`, so a fault there cannot build the truth that passes it.
    def angles(t):
        return (0.8 * math.sin(1.1 * t), 0.5 * math.sin(0.7 * t + 0.3),
                0.88 * math.cos(1.1 * t), 0.35 * math.cos(0.7 * t + 0.3))

    def body_rate(t):
        a, b, da, db = angles(t)
        return (db, math.sin(b) * da, math.cos(b) * da)  # Rx(b)^T (0, 0, a') + (b', 0, 0)

    def attitude(t):
        a, b, _, _ = angles(t)
        z = (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))
        x = (math.cos(b / 2), math.sin(b / 2), 0.0, 0.0)
        return (z[0] * x[0], z[0] * x[1], z[3] * x[1], z[3] * x[0])  # z (x) x

    def motion(late_ns, seconds=4.0):
        imu = [(round(k * 5e6), 0.0, 0.0, 9.8) + body_rate((k + 0.5) * 5e-3)
               for k in range(int(seconds / 5e-3))]
        truth = [(round(t * 1e9) + late_ns, 0.0, 0.0, 0.0) + attitude(t)
                 for t in (0.6 + k * 0.0125 for k in range(int((seconds - 1.2) / 0.0125)))]
        return imu, truth

    imu, truth = motion(150_000_000)
    lag, mismatch, at_zero = truth_lag(imu, truth)
    expect("lag", lag, -150_000_000)
    expect("lag beats zero", mismatch < at_zero, True)
    try:
        truth_lag(*motion(550_000_000))
        expect("a lag past the sweep is refused", False, True)
    except ConversionError:
        pass

    failures += convert_fixture(motion)

    print(f"insane2replay self-test: {'FAIL' if failures else 'ok'}", file=sys.stderr)
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("directory", nargs="?", type=Path,
                        help="where data/fetch.sh --manifest data/insane.txt put the files")
    parser.add_argument("--sequence", choices=sorted(SEQUENCES))
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--truth", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not (args.directory and args.sequence and args.output and args.truth):
        parser.error("directory, --sequence, -o and --truth are required")
    try:
        imu, fixes, truth, lag = convert(args.directory, args.sequence, args.output,
                                         args.truth)
    except (ConversionError, OSError, KeyError) as error:
        print(f"insane2replay: {error}", file=sys.stderr)
        return 1
    print(f"{args.output}: {imu} IMU rows, {fixes} fixes; {args.truth}: {truth} truth rows, "
          f"lag {lag:+.0f} ms", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
