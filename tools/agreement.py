#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy==2.5.3"]
# ///
"""Agreement with EKF2: the statistics #8 defines, each computed here and nowhere else.

    uv run tools/agreement.py --self-test

EKF2 is not truth, so nothing here is accuracy. Every figure is a distance
between two estimators fed the same log, and a divergence is a finding to
explain before it is an error on either side. The causes this module cannot
remove are named beside its output rather than inside it: the `R` policy
(`r_policy=`), the height reference each filter converges to, EKF2's resets,
and the attitude covariance three corpus logs do not carry.

No I/O: `tools/replay_report.py` reads the files and hands arrays in, for one
log (a section of its report) or for the corpus (`--corpus`). So a statistic is
tested against arrays whose answer is known by construction, and the harness's
own keys (`nis_`, `resets=`, `recovered=`) are never recomputed -- they are
read off the `summary` line and printed beside these (AGENTS.md, "One statistic,
one implementation"). The comparison against EKF2 is the one kind of statistic
the harness cannot own, since CI has no reference to give it.

Every array is paired, never interpolated: each EKF2 sample against the nearest
of ours within two of our sample intervals, as `attitude_difference` pairs
attitude, so a sample in a gap in either compares against nothing.
"""

from __future__ import annotations

import math
import sys

import numpy as np

# ---------------------------------------------------------------- attitude
#
# Quaternion geometry, for the report's pictures and for the comparison below.
# None of it is a statistic the harness publishes. Every function takes the
# Hamilton scalar-first body-to-NED convention the epoch and reference files
# both write (`Attitude::body_to_ned`), elementwise over numpy arrays.

QUATERNION = ["q0", "q1", "q2", "q3"]

#: Past this tilt heading is undefined. The swing-twist heading below is
#: singular only when the vehicle is inverted, where `q0` and `q3` both vanish,
#: and a few degrees short of that it turns small attitude noise into large
#: heading swings.
INVERTED = math.radians(170.0)


def quaternion_from_euler(roll, pitch, yaw):
    """ZYX Euler to quaternion, `R = Rz(yaw) Ry(pitch) Rx(roll)`.

    Only the truth file is still Euler (`examples/simulate.rs`). This direction
    is defined at every attitude, so truth passes through 90 deg of pitch
    losslessly; only the reverse is ambiguous.
    """
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def tilt_heading(q0, q1, q2, q3):
    """Tilt and heading, radians, from the swing-twist split about NED down.

    `q = twist(down) * swing(horizontal axis)`: the twist is heading, the
    swing's angle is tilt, the angle between body down and navigation down.
    These are the two quantities `Validity` and `Accuracy` split attitude into,
    and unlike ZYX roll and yaw they separate at 90 deg of pitch, where a
    tailsitter cruises. Heading is NaN past `INVERTED`, where the split is
    singular. `q` and `-q` give the same pair.
    """
    tilt = np.arccos(np.clip(1.0 - 2.0 * (q1 * q1 + q2 * q2), -1.0, 1.0))
    heading = np.mod(2.0 * np.arctan2(q3, q0) + math.pi, 2.0 * math.pi) - math.pi
    return tilt, np.where(tilt > INVERTED, np.nan, heading)


def conjugate(q):
    """The inverse of a unit quaternion."""
    q0, q1, q2, q3 = q
    return q0, -q1, -q2, -q3


def multiply(p, r):
    """The Hamilton product `p * r`."""
    p0, p1, p2, p3 = p
    r0, r1, r2, r3 = r
    return (p0 * r0 - p1 * r1 - p2 * r2 - p3 * r3,
            p0 * r1 + p1 * r0 + p2 * r3 - p3 * r2,
            p0 * r2 - p1 * r3 + p2 * r0 + p3 * r1,
            p0 * r3 + p1 * r2 - p2 * r1 + p3 * r0)


def rotation_vector(q):
    """`log(q)` as a rotation vector, radians, taking the shorter of the two
    rotations `q` and `-q` describe."""
    w, x, y, z = q
    sign = np.where(w < 0.0, -1.0, 1.0)
    w, x, y, z = w * sign, x * sign, y * sign, z * sign
    norm = np.sqrt(x * x + y * y + z * z)
    angle = 2.0 * np.arctan2(norm, w)
    # angle / norm tends to 2 as the rotation vanishes; guard the 0 / 0.
    scale = np.where(norm > 1e-12, angle / np.where(norm > 1e-12, norm, 1.0), 2.0)
    return x * scale, y * scale, z * scale


def rotation_difference(a, b):
    """The rotation vector of `a^-1 * b`, in `a`'s body axes, radians.

    `b = a * exp(d)`, the local perturbation of equation (3), which is the
    frame the epoch file's `sigma_att_x/y/z` are in -- so a band drawn from them
    around `d` compares like with like at any attitude.
    """
    return rotation_vector(multiply(conjugate(a), b))


def nearest(times, at, tolerance):
    """For each of `at`, the index of the nearest of the sorted `times`, or -1.

    `tolerance` refuses a pairing across a gap: a reference sample with no
    epoch near it compares against nothing rather than against the far side.
    """
    if len(times) == 1:
        pick = np.zeros(len(at), dtype=int)
    else:
        right = np.clip(np.searchsorted(times, at), 1, len(times) - 1)
        left = right - 1
        pick = np.where(np.abs(times[left] - at) <= np.abs(times[right] - at),
                        left, right)
    return np.where(np.abs(times[pick] - at) <= tolerance, pick, -1)


def attitude_difference(ours, reference):
    """`(t, (dx, dy, dz))`: each reference sample's rotation to ours, undecimated.

    Both arguments are `(t, {"q0": .., "q3": ..})` as read_columns returns them.
    Each reference sample is paired with the nearest of ours, never
    interpolated, within two of our sample intervals: a reference sample in a
    gap in ours compares against nothing. Undecimated so that a caller comparing
    other pairs of runs decides its own stride.
    """
    (t_ours, q_ours), (t_ref, q_ref) = ours, reference
    keep, pick = paired(t_ours, t_ref)
    d = rotation_difference(tuple(q_ref[n][keep] for n in QUATERNION),
                            tuple(q_ours[n][pick] for n in QUATERNION))
    return t_ref[keep], d




# ------------------------------------------------------------- comparison

#: A source's verdict holds until its next sample, and no longer than this many
#: of its median intervals: the multiple `tools/replay_report.py` shades an
#: outage at, so a logging dropout does not stretch one rejection across it.
HOLD = 5.0

#: Seconds averaged at each end of a height change: long enough that a single
#: barometer or GNSS excursion does not set the figure, the window the issue's
#: first measurement used.
END_WINDOW = 60.0

#: This filter's sources against the EKF2 test ratio that judges the same
#: measurement. `gnss_hgt` has none: EKF2's `hgt_test_ratio` belongs to whichever
#: height source is active. `baro` compares only where EKF2's height reference is
#: the barometer, for the same reason.
RATIO_OF = {"gnss_pos": "r_gnss_pos", "gnss_vel": "r_gnss_vel", "baro": "r_baro",
            "mag": "r_mag"}


def tolerance_of(t):
    """Two median sample intervals, the pairing tolerance `attitude_difference` uses."""
    return 2.0 * float(np.median(np.diff(t))) if len(t) > 1 else 0.0


def paired(t_ours, t_ref):
    """`(keep, pick)`: which reference samples have an epoch of ours near them,
    and its index."""
    pick = nearest(t_ours, t_ref, tolerance_of(t_ours))
    keep = pick >= 0
    return keep, pick[keep]


def intervals(t, flag, hold=None):
    """`[(start, end)]` over which `flag` holds, each sample's value held until
    the next sample or for `hold` seconds, whichever is sooner.

    `hold` defaults to `HOLD` median intervals of `t`. Consecutive flagged
    samples merge into one run, so a source rejected on every fix for a minute is
    one interval of a minute, however often it reports.
    """
    t = np.asarray(t, dtype=float)
    flag = np.asarray(flag, dtype=bool)
    if len(t) == 0:
        return []
    if hold is None:
        hold = HOLD * float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    ends = np.minimum(np.append(t[1:], t[-1] + hold), t + hold)
    runs = []
    for start, end in zip(t[flag], ends[flag]):
        if runs and start <= runs[-1][1]:
            runs[-1][1] = max(runs[-1][1], end)
        else:
            runs.append([start, end])
    return [tuple(run) for run in runs]


def total(runs):
    """Seconds covered by a list of disjoint runs; a float even when there are none,
    so no time prints as a time and not as a count."""
    return sum((end - start for start, end in runs), 0.0)


def overlap(a, b):
    """Seconds covered by both of two sorted lists of disjoint runs."""
    i = j = 0
    both = 0.0
    while i < len(a) and j < len(b):
        start, end = max(a[i][0], b[j][0]), min(a[i][1], b[j][1])
        if end > start:
            both += end - start
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return both


def distance(ours, theirs, sigma_ours=None, sigma_theirs=None):
    """`(rms, max, nd2)` of `theirs - ours`, NaN-skipping.

    `nd2` is the mean of `d^2 / (sigma_ours^2 + sigma_theirs^2)`: the distance in
    units of the two filters' own claimed uncertainty, near 1 or below where
    each covariance covers the other's estimate. It treats the two errors as
    independent, which they are not, since both filters read the same sensors,
    so it is a scale for comparing logs rather than a chi-square to test against.
    """
    d = np.asarray(theirs, dtype=float) - np.asarray(ours, dtype=float)
    good = np.isfinite(d)
    variance = None
    if sigma_ours is not None:
        theirs_sigma = np.asarray(sigma_theirs, float)
        variance = np.asarray(sigma_ours, float) ** 2 + theirs_sigma ** 2
        # A sigma EKF2 reports as exactly zero is a state it is not estimating,
        # not one it knows perfectly, so that sample says nothing here.
        good &= np.isfinite(variance) & (theirs_sigma > 0)
    if not good.any():
        return None, None, None
    d = d[good]
    nd2 = None if variance is None else float(np.mean(d * d / variance[good]))
    return float(np.sqrt(np.mean(d * d))), float(np.max(np.abs(d))), nd2


def change(t, values, start, end):
    """Mean over the last `END_WINDOW` seconds before `end`, less the mean over
    the first after `start`: a start-to-end change, named as one. None for a span
    under two windows, where the two would overlap: `a299e722` runs 61 s."""
    if end - start < 2 * END_WINDOW:
        return None
    t, values = np.asarray(t, float), np.asarray(values, float)
    first = values[(t >= start) & (t < start + END_WINDOW)]
    last = values[(t > end - END_WINDOW) & (t <= end)]
    first, last = first[np.isfinite(first)], last[np.isfinite(last)]
    if not len(first) or not len(last):
        return None
    return float(np.mean(last) - np.mean(first))


def compare(ours, ekf2, rejected, placed, height_reference):
    """Every agreement statistic for one log, as an ordered `{key: value}`.

    `ours` is `(t, {column: array})` from the epoch file; `ekf2` is
    `{"local"|"att"|"states"|"ratio": (t, {column: array})}` from the reference,
    a kind absent where the log carries none, blanks as NaN; `rejected` is
    `{source: (t, rejected_flag)}` from the fusion file; `placed` is the axes,
    of `"ned"`, whose EKF2 position the reference header says is already in this
    filter's frame; `height_reference` is
    EKF2's, as the reference header names it. A value of None is a hole: a
    quantity this log cannot supply, printed as `none` rather than dropped.

    Angles are degrees, positions metres, velocities m/s, biases rad/s and
    m/s^2, rejection times seconds. Each family is its own function, so a caller
    wanting one -- scoring a rejection as correct needs only the last two --
    calls that one.
    """
    return {
        **position_velocity(ours, ekf2.get("local"), ekf2.get("states"), placed),
        **biases(ours, ekf2.get("states")),
        **height(ours, ekf2.get("local"), height_reference),
        **attitude(ours, ekf2.get("att"), ekf2.get("states")),
        **rejections(rejected, ekf2.get("ratio"), height_reference),
        **resets(ekf2.get("local"), ekf2.get("att")),
    }


def position_velocity(ours, local, states, placed):
    """`pos_*`/`vel_*` `_rms`, `_max` and `_nd2` per NED axis. Position is
    compared on the axes in `placed` alone; velocity needs no frame.

    The converter places EKF2's position, since PX4's local frame differs from
    this filter's in scale as well as origin (`ulog2replay.reference_local`), so
    nothing here shifts it.
    """
    t, our = ours
    out = {}
    if local is not None:
        t_ref, ref = local
        keep, pick = paired(t, t_ref)
    if states is not None and local is not None:
        # EKF2's sigma is on its states' timeline and its estimate on its
        # position's: pair the two first, then each with ours.
        t_s, st = states
        keep_s, pick_s = paired(t, t_s)
        here = nearest(t_ref, t_s[keep_s], tolerance_of(t_ref))
        good = here >= 0
    for kind in ("pos", "vel"):
        for axis in "ned":
            name = f"{kind}_{axis}"
            rms = peak = nd2 = None
            if local is not None and (kind == "vel" or axis in placed):
                rms, peak, _ = distance(our[name][pick], ref[name][keep])
                if states is not None:
                    _, _, nd2 = distance(our[name][pick_s[good]], ref[name][here[good]],
                                         our[f"sigma_{name}"][pick_s[good]],
                                         st[f"sigma_{name}"][keep_s][good])
            out[f"{name}_rms"], out[f"{name}_max"], out[f"{name}_nd2"] = rms, peak, nd2
    return out


def biases(ours, states):
    """`ba_*`/`bg_*` `_rms` and `_nd2` per body axis."""
    t, our = ours
    out = {}
    if states is not None:
        t_s, st = states
        keep, pick = paired(t, t_s)
    for kind in ("ba", "bg"):
        for axis in "xyz":
            name = f"{kind}_{axis}"
            rms = nd2 = None
            if states is not None:
                rms, _, nd2 = distance(our[name][pick], st[name][keep],
                                       our[f"sigma_{name}"][pick], st[f"sigma_{name}"][keep])
            out[f"{name}_rms"], out[f"{name}_nd2"] = rms, nd2
    return out


def height(ours, local, height_reference):
    """`climb`, `climb_ekf2` and the reference EKF2 converged to.

    As change, never as a gap: each filter converges to its own height
    reference, so where those disagree (`2c42096b`, EKF2 on its barometer) a gap
    measures the references rather than either estimate. Over the span both
    cover, climb positive.
    """
    t, our = ours
    climb = climb_ekf2 = None
    if local is not None and len(local[0]) and len(t):
        start, end = max(t[0], local[0][0]), min(t[-1], local[0][-1])
        ours_change = change(t, our["pos_d"], start, end)
        theirs_change = change(local[0], local[1]["pos_d"], start, end)
        climb = None if ours_change is None else -ours_change
        climb_ekf2 = None if theirs_change is None else -theirs_change
    return {"climb": climb, "climb_ekf2": climb_ekf2,
            "height_reference_ekf2": height_reference}


def attitude(ours, att, states):
    """`tilt_diff_rms`, `tilt_diff_max`, `heading_diff_med` and `att_nd2`."""
    t, our = ours
    out = {"tilt_diff_rms": None, "tilt_diff_max": None, "heading_diff_med": None,
           "att_nd2": None}
    if att is None:
        return out
    t_a, a = att
    keep, pick = paired(t, t_a)
    q_ours = tuple(our[n][pick] for n in QUATERNION)
    q_ref = tuple(a[n][keep] for n in QUATERNION)
    tilt_o, heading_o = tilt_heading(*q_ours)
    tilt_e, heading_e = tilt_heading(*q_ref)
    out["tilt_diff_rms"], out["tilt_diff_max"], _ = distance(np.degrees(tilt_o),
                                                             np.degrees(tilt_e))
    # Heading after EKF2's first attitude reset once this filter is running:
    # EKF2 aligns yaw in the first seconds of a flight, and a comparison across
    # that step measures its alignment rather than either estimate. Not after its
    # *last*: `093e806a` resets again at 758 s and 0.56 s before the log ends,
    # which would leave nothing to compare.
    after = t_a[keep] > first_change(t_a, a.get("att_reset"), since=t[0])
    gap = np.abs(np.mod(heading_o - heading_e + math.pi, 2 * math.pi) - math.pi)
    gap = gap[after & np.isfinite(gap)]
    out["heading_diff_med"] = float(np.degrees(np.median(gap))) if len(gap) else None
    # The attitude difference in units of both covariances, on the one attitude
    # scalar both files carry in comparable form: a trace, invariant under the
    # rotation between EKF2's NED sigmas and this filter's body ones.
    if states is not None and np.isfinite(states[1]["sigma_att_total"]).any():
        t_s, st = states
        d = np.sqrt(sum(c * c for c in rotation_difference(q_ref, q_ours)))
        at = nearest(t_a[keep], t_s, tolerance_of(t_a))
        good = at >= 0
        ours_var = sum(our[f"sigma_att_{x}"][pick[at[good]]] ** 2 for x in "xyz")
        ratio = d[at[good]] ** 2 / (ours_var + st["sigma_att_total"][good] ** 2)
        ratio = ratio[np.isfinite(ratio)]
        out["att_nd2"] = float(np.mean(ratio)) if len(ratio) else None
    return out


def rejections(rejected, ratio, height_reference):
    """`rej_s_<source>`, `_ekf2` and `_both`: seconds over the gate."""
    out = {}
    for source, column in RATIO_OF.items():
        ours_runs = intervals(*rejected[source]) if source in rejected else None
        theirs_runs = None
        comparable = source != "baro" or height_reference == "baro"
        if ratio is not None and comparable:
            t_r, r = ratio
            values = r.get(column)
            if values is not None and np.isfinite(values).any():
                good = np.isfinite(values)
                theirs_runs = intervals(t_r[good], values[good] > 1.0)
        out[f"rej_s_{source}"] = None if ours_runs is None else total(ours_runs)
        out[f"rej_s_{source}_ekf2"] = None if theirs_runs is None else total(theirs_runs)
        out[f"rej_s_{source}_both"] = (None if ours_runs is None or theirs_runs is None
                                       else overlap(ours_runs, theirs_runs))
    return out


def resets(local, att):
    """`ekf2_<counter>_resets`: how often each of EKF2's reset counters moved."""
    out = {}
    counters = [(local, name) for name in ("xy_reset", "z_reset", "vxy_reset", "vz_reset")]
    for kind, name in counters + [(att, "att_reset")]:
        values = None if kind is None else kind[1].get(name)
        out[f"ekf2_{name}s"] = (None if values is None or not np.isfinite(values).any()
                                else len(change_times(kind[0], values)))
    return out


def change_times(t, counter):
    """When a counter moved: the time of each sample whose value differs from the
    last finite one before it. The report marks EKF2's attitude resets with it."""
    t, counter = np.asarray(t, dtype=float), np.asarray(counter, dtype=float)
    finite = np.isfinite(counter)
    t, counter = t[finite], counter[finite]
    return t[1:][np.diff(counter) != 0]


def first_change(t, counter, since):
    """When a counter first moved at or after `since`, or minus infinity where it
    never did."""
    if counter is None:
        return -math.inf
    moved = change_times(t, counter)
    moved = moved[moved >= since]
    return float(moved[0]) if len(moved) else -math.inf


def format_value(value):
    """How a key prints: `none` for a hole, a count as an integer, and anything
    else to four significant figures in fixed point.

    Fixed point with its trailing zeros, because `data/expect.sh` bands a value by
    its last printed place, reads no exponent, and tells a statistic from a count
    by its decimal point: `.4g` printed 2.000 as `2`, banding it 1..3, and
    9.59e-05 as a word, pinning it exactly. Decimals from the magnitude, so 0.31
    prints `0.3100` rather than the three figures a fixed precision would give.
    """
    if value is None:
        return "none"
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if value == 0 or not math.isfinite(value):
        return "0.000" if value == 0 else str(value)
    # At least one decimal, so a statistic of 1000 or more still reads as one.
    # Rounding can carry into the next decade (0.099996 to 0.1000), so the
    # magnitude is read again from the rounded value.
    decimals = max(1, 3 - math.floor(math.log10(abs(value))))
    rounded = round(value, decimals)
    if rounded:
        decimals = max(1, 3 - math.floor(math.log10(abs(rounded))))
    # `+ 0.0` drops the sign a -0.0 would print with.
    return f"{round(value, decimals) + 0.0:.{decimals}f}"


# -------------------------------------------------------------- self-test


def self_test():
    """Literal fixtures; returns the failures rather than printing them, so
    `tools/replay_report.py --self-test` runs these too."""
    failures = []

    def near(what, got, want, tolerance=1e-9):
        if got is None or want is None:
            if got is not want:
                failures.append(f"{what}: got {got}, want {want}")
            return
        got, want = np.atleast_1d(got), np.atleast_1d(want)
        if got.shape != want.shape:
            failures.append(f"{what}: got {got}, want {want}")
            return
        same = np.isnan(got) == np.isnan(want)
        close = np.isnan(want) | (np.abs(np.nan_to_num(got) - np.nan_to_num(want))
                                  <= tolerance)
        if not (same.all() and close.all()):
            failures.append(f"{what}: got {got}, want {want}")

    d = math.radians
    half = math.sqrt(0.5)

    # ZYX composition, against a hand-derived value: pitched 90 deg is
    # (cos 45, 0, sin 45, 0), and yawed 90 deg (cos 45, 0, 0, sin 45).
    near("euler pitch 90", quaternion_from_euler(0.0, d(90), 0.0), (half, 0, half, 0))
    near("euler yaw 90", quaternion_from_euler(0.0, 0.0, d(90)), (half, 0, 0, half))

    def attitude(roll, pitch, yaw):
        return tilt_heading(*quaternion_from_euler(d(roll), d(pitch), d(yaw)))

    near("level", attitude(0, 0, 30), (0.0, d(30)))
    near("rolled 10", attitude(10, 0, -45), (d(10), d(-45)))
    # Pitched 90 deg and yawed 30: ZYX roll and yaw are inseparable here, and
    # the split is not -- a tailsitter's cruise.
    near("pitched 90", attitude(0, 90, 30), (d(90), d(30)))
    near("pitched 90, rolled", attitude(20, 90, 30)[0], d(90), 1e-9)
    near("inverted", attitude(178, 0, 30), (d(178), np.nan))
    near("-q", tilt_heading(*(-c for c in quaternion_from_euler(0.1, 0.2, 2.9))),
         attitude(math.degrees(0.1), math.degrees(0.2), math.degrees(2.9)))
    # Scalar-last read of a scalar-first quaternion: still a rotation, the
    # wrong one. Level and yawed 30 deg, read as (q1, q2, q3, q0).
    q = quaternion_from_euler(0.0, 0.0, d(30))
    if abs(tilt_heading(q[1], q[2], q[3], q[0])[0] - 0.0) < 1e-6:
        failures.append("a scalar-last read is indistinguishable from the right one")

    # The difference is body-frame: 5 deg about body x on a vehicle pitched
    # 90 deg is (5, 0, 0), where a navigation-frame one would read it on down.
    pitched = quaternion_from_euler(0.0, d(90), d(30))
    nudge = (math.cos(d(2.5)), math.sin(d(2.5)), 0.0, 0.0)
    near("difference in body axes", rotation_difference(pitched, multiply(pitched, nudge)),
         (d(5), 0.0, 0.0))
    # The same 5 deg with the second quaternion negated: q and -q are one
    # rotation, and the long way round is 355 deg about -x.
    near("difference across -q",
         rotation_difference(pitched, tuple(-c for c in multiply(pitched, nudge))),
         (d(5), 0.0, 0.0))
    near("no difference", rotation_difference(pitched, pitched), (0.0, 0.0, 0.0))

    near("nearest", nearest(np.array([0.0, 1.0, 2.0]), np.array([0.4, 1.6, 9.0]), 0.5),
         (0, 2, -1))
    near("nearest of one", nearest(np.array([5.0]), np.array([4.9, 5.1, 9.0]), 0.5),
         (0, 0, -1))

    # A 1 Hz source rejected at 3 and 4 s, then at 9: two runs, the first two
    # seconds long. The last sample holds for `hold` only.
    t = np.arange(10.0)
    flags = np.zeros(10, dtype=bool)
    flags[[3, 4, 9]] = True
    runs = intervals(t, flags, hold=5.0)
    near("intervals", np.array(runs).ravel(), (3.0, 5.0, 9.0, 14.0))
    # A dropout between 4 and 20 s holds the rejection for `hold`, not 16 s.
    runs = intervals(np.array([3.0, 4.0, 20.0]), np.array([False, True, False]), hold=2.0)
    near("hold across a dropout", total(runs), 2.0)
    near("overlap", overlap([(0.0, 2.0), (5.0, 9.0)], [(1.0, 6.0)]), 2.0)
    near("no overlap", overlap([(0.0, 1.0)], [(2.0, 3.0)]), 0.0)
    # A NaN between two 3.0s is a sample with no counter, not a move.
    near("change times", change_times(np.arange(6.0), np.array([2.0, 2.0, 3.0, np.nan, 3.0,
                                                                5.0])), (2.0, 5.0))
    near("first change", first_change(np.arange(4.0), np.array([0.0, 1.0, 1.0, 2.0]), 0.0), 1.0)
    near("first change since", first_change(np.arange(4.0), np.array([0.0, 1.0, 1.0, 2.0]), 2.0),
         3.0)
    near("never changed", first_change(np.arange(3.0), np.array([4.0, 4.0, 4.0]), 0.0),
         -math.inf)

    # Distance: a constant 3 m offset is 3 m RMS and max, and 9 / (4 + 5) = 1 in
    # units of both sigmas.
    near("distance", distance(np.zeros(4), np.full(4, 3.0), np.full(4, 2.0),
                              np.full(4, math.sqrt(5.0))), (3.0, 3.0, 1.0))
    near("distance of nothing", distance(np.array([np.nan]), np.array([1.0]))[0], None)
    # `_max` is the largest magnitude: -3 outranks +1.
    near("max of a negative difference", distance(np.zeros(2), np.array([-3.0, 1.0]))[:2],
         (math.sqrt(5.0), 3.0))
    # A climb of 10 m over 200 s, flat at each end under +/-1 m of alternating
    # noise that each 60 s window averages out, reads 10. Its endpoints read 8.
    t = np.arange(0.0, 200.0, 0.5)
    d = np.where(t < 60, 0.0, np.where(t > 140, 10.0, (t - 60) / 8.0))
    d = d + np.where(np.arange(len(t)) % 2, -1.0, 1.0)
    near("change", change(t, d, 0.0, t[-1]), 10.0)
    # Four figures in every decade, a decimal point on every statistic and none on a
    # count, and a rounding that carries into the next decade reads its new one.
    printed = [format_value(v) for v in (9.59e-05, 0.31, 0.0999996, 2.0, 304.9, 1402.3,
                                         -0.0, 0.0, None, 3)]
    if printed != ["0.00009590", "0.3100", "0.1000", "2.000", "304.9", "1402.3", "0.000",
                   "0.000", "none", "3"]:
        failures.append(f"format_value: got {printed}")
    near("no time is a time", total([]), 0.0)
    if format_value(total([])) != "0.000":
        failures.append(f"total([]) prints {format_value(total([]))}")
    near("no change over a short log", change(t[:200], d[:200], 0.0, t[199]), None)
    # Descending 10 m while EKF2 holds its height: climb is up-positive, so -10 and 0.
    level_d = np.zeros(len(t))
    near("climb is up-positive",
         tuple(height((t, {"pos_d": d}), (t, {"pos_d": level_d}), "gps")[k]
               for k in ("climb", "climb_ekf2")), (-10.0, 0.0))

    # compare(): EKF2 3 m south of this filter, placed on every axis. Its velocity
    # differs by 0.5 m/s east, and each sigma pair sums to a variance of 4, so an
    # nd2 is a squared difference over 4. One of our rejections at 2 s, EKF2 over its gate
    # from 1 to 3 s (held one sample at most).
    t = np.arange(0.0, 10.0, 0.1)
    n = len(t)
    zeros, ones = np.zeros(n), np.ones(n)
    level = {"q0": ones, "q1": zeros, "q2": zeros, "q3": zeros}
    ours = (t, {**{f"pos_{a}": zeros for a in "ned"}, **{f"vel_{a}": zeros for a in "ned"},
                **{f"sigma_pos_{a}": ones for a in "ned"},
                **{f"sigma_vel_{a}": ones for a in "ned"},
                **{f"{k}_{a}": zeros for k in ("ba", "bg") for a in "xyz"},
                **{f"sigma_{k}_{a}": ones for k in ("ba", "bg") for a in "xyz"},
                **{f"sigma_att_{a}": ones for a in "xyz"}, **level})
    t_ref = t[::2]
    m = len(t_ref)
    yawed = quaternion_from_euler(np.zeros(m), np.zeros(m), np.full(m, math.radians(10)))
    ekf2 = {
        "local": (t_ref, {"pos_n": np.full(m, -3.0), "pos_e": np.zeros(m),
                          "pos_d": np.zeros(m), "vel_n": np.zeros(m),
                          "vel_e": np.full(m, 0.5), "vel_d": np.zeros(m),
                          "xy_reset": np.where(t_ref > 5, 3.0, 2.0),
                          "z_reset": np.full(m, np.nan),
                          "vxy_reset": np.where(t_ref > 3, 1.0, 0.0) + (t_ref > 7),
                          "vz_reset": np.zeros(m)}),
        "att": (t_ref, {**dict(zip(QUATERNION, yawed)), "att_reset": np.zeros(m)}),
        "states": (t_ref, {**{f"sigma_{k}_{a}": np.full(m, math.sqrt(3.0))
                              for k in ("pos", "vel") for a in "ned"},
                           **{f"{k}_{a}": np.full(m, 0.1) for k in ("ba", "bg")
                              for a in "xyz"},
                           **{f"sigma_{k}_{a}": np.full(m, math.sqrt(3.0))
                              for k in ("ba", "bg") for a in "xyz"},
                           "sigma_att_total": np.full(m, np.nan)}),
        "ratio": (t_ref, {"r_gnss_pos": np.where((t_ref >= 1) & (t_ref < 3), 2.0, 0.5),
                          "r_baro": np.full(m, 2.0)}),
    }
    rejected = {"gnss_pos": (np.array([1.0, 2.0, 3.0]), np.array([False, True, False]))}
    got = compare(ours, ekf2, rejected, "ned", "gps")
    # 3 m over a variance of 1 + 3 is 9 / 4. A shift applied here as well as in
    # the converter reads 0 on both.
    near("placed north", (got["pos_n_rms"], got["pos_n_nd2"]), (3.0, 2.25))
    near("velocity east", (got["vel_e_rms"], got["vel_e_max"], got["vel_e_nd2"]),
         (0.5, 0.5, 0.0625))
    near("bias", (got["bg_x_rms"], got["bg_x_nd2"]), (0.1, 0.0025))
    # EKF2 reporting a zero sigma is not estimating the state: no nd2 from it.
    unestimated = {**ekf2, "states": (t_ref, {**ekf2["states"][1],
                                              "sigma_bg_x": np.zeros(m)})}
    near("zero EKF2 sigma", compare(ours, unestimated, rejected, "ned",
                                    "gps")["bg_x_nd2"], None)
    near("heading", got["heading_diff_med"], 10.0, 1e-6)
    near("tilt", got["tilt_diff_rms"], 0.0, 1e-6)
    near("no attitude sigma, no att_nd2", got["att_nd2"], None)
    # 10 deg of heading over our three unit sigmas plus EKF2's unit trace:
    # (pi/18)^2 / 4.
    traced = {**ekf2, "states": (t_ref, {**ekf2["states"][1],
                                         "sigma_att_total": np.ones(m)})}
    near("att_nd2", compare(ours, traced, rejected, "ned", "gps")["att_nd2"],
         (math.pi / 18) ** 2 / 4, 1e-9)
    # EKF2 at 90 deg of heading until a reset at 6 s, 10 deg after, and a second
    # reset on the last sample: the median over the whole log would read 90, and
    # after the last reset there is nothing to read. After the first, 10.
    reset = t_ref > 6.0
    stepped = quaternion_from_euler(np.zeros(m), np.zeros(m),
                                    np.radians(np.where(reset, 10.0, 90.0)))
    counter = np.where(reset, 1.0, 0.0)
    counter[-1] = 2.0
    resetting = {**ekf2, "att": (t_ref, {**dict(zip(QUATERNION, stepped)),
                                         "att_reset": counter})}
    near("heading after the first reset",
         compare(ours, resetting, rejected, "ned", "gps")["heading_diff_med"],
         10.0, 1e-6)
    near("our rejection", got["rej_s_gnss_pos"], 1.0)
    near("EKF2 over its gate", got["rej_s_gnss_pos_ekf2"], 2.0, 1e-9)
    near("both", got["rej_s_gnss_pos_both"], 1.0)
    near("baro only against a baro reference", got["rej_s_baro_ekf2"], None)
    near("resets", tuple(got[f"ekf2_{c}_resets"] for c in ("xy", "vxy", "vz", "att")),
         (1, 2, 0, 0))
    near("z resets never logged", got["ekf2_z_resets"], None)
    near("no origin, no position", compare(ours, ekf2, rejected, "", "gps")["pos_n_rms"],
         None)
    near("velocity needs no origin",
         compare(ours, ekf2, rejected, "", "gps")["vel_e_rms"], 0.5)
    horizontal = compare(ours, ekf2, rejected, "ne", "gps")
    near("horizontal placed", horizontal["pos_n_rms"], 3.0)
    near("down not placed", horizontal["pos_d_rms"], None)
    return failures


def main():
    if sys.argv[1:] != ["--self-test"]:
        print(__doc__.strip().splitlines()[0], file=sys.stderr)
        print("usage: tools/agreement.py --self-test", file=sys.stderr)
        return 2
    failures = self_test()
    for failure in failures:
        print(f"FAIL {failure}", file=sys.stderr)
    print(f"agreement self-test: {'FAIL' if failures else 'ok'}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
