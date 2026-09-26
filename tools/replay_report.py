#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["matplotlib==3.10.7", "numpy==2.5.3", "scipy==1.16.3"]
# ///
"""Render one replay into a self-contained HTML validation report.

    uv run tools/replay_report.py <input.csv> <replay.csv> [truth.csv] \
        --reference <reference.csv> --summary <captured-stdout> -o report.html

Reads the replay harness's own files -- the converted input, the per-epoch
output, the per-fusion output beside it, EKF2's reference where one was
converted, and truth where the scenario has it -- and writes one HTML file with
every figure inlined. One file because a report gets mailed and archived, and
PNGs in a directory arrive without their captions; no JavaScript for the same
reason README.md carries no mermaid, so it renders wherever it lands.

This tool computes no statistic the harness already publishes. `nis_`, `acf1_`,
`nu_` and the score keys are read off the captured `summary` and `score` lines
and printed; the innovation and its covariance are read out of the fusion CSV.
Agreement with EKF2, which the harness cannot own because CI has no reference,
is `tools/agreement.py`'s: this tool reads the files and prints what it returns,
for one run beside its figures or, with `--corpus`, for every log in one table.
A second implementation of any of them would eventually disagree with the first,
and the disagreement would be found by somebody chasing a filter bug that does
not exist (GOALS.md, "Harness constraint"; data/README.md, "One statistic, one
implementation").

Sources, their gates and their degrees of freedom are discovered from the files
rather than listed here, so adding a measurement source to the crate needs no
edit in this file.

scipy is a dependency for one reason: the QQ plot needs a chi-square inverse
CDF, and a hand-rolled incomplete-gamma inverse is more risk in a plot than a
wheel is in a tool that never runs in CI.
"""

from __future__ import annotations

import argparse
import array
import base64
import html
import io
import math
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # No display; the figures only ever become PNG bytes.

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402

# A sibling module, found on the path Python puts this script's directory at the
# head of. It owns the quaternion geometry and every agreement-with-EKF2 statistic.
import agreement  # noqa: E402
from agreement import (  # noqa: E402
    QUATERNION,
    attitude_difference,
    quaternion_from_euler,
    tilt_heading,
)


class ReportError(Exception):
    pass


# Status values the harness prints, most severe first, with the shade each gets
# as a plot background. Precedence is the crate's: DeadReckoning > Aligning >
# Degraded > Healthy (src/health.rs).
STATUS_SHADES = {
    "DeadReckoning": "#d94f4f",
    "Aligning": "#e0a030",
    "Degraded": "#e8d44d",
    "Healthy": None,
}

# The vehicle's flight regime, from the reference's `vehicle_mode` rows, drawn as
# a strip along the top of each panel so it never hides the Status shade beneath.
# Multicopter flight is the unshaded default, as Healthy is.
MODE_SHADES = {
    "fw": "#3a7ca5",
    "to_fw": "#8e5ea2",
    "to_mc": "#8e5ea2",
}

# State groups plotted together, as (title, unit, columns, sigma columns). The
# reference's names match the epoch file's for everything except attitude, whose
# frames differ -- see reference_states in tools/ulog2replay.py. Attitude has no
# columns to plot as they stand: it is a quaternion, and attitude_figure draws
# what is derived from it. Its sigmas still belong to the covariance figure.
STATE_GROUPS = [
    ("Position NED", "m", ["pos_n", "pos_e", "pos_d"],
     ["sigma_pos_n", "sigma_pos_e", "sigma_pos_d"]),
    ("Velocity NED", "m/s", ["vel_n", "vel_e", "vel_d"],
     ["sigma_vel_n", "sigma_vel_e", "sigma_vel_d"]),
    ("Attitude", "deg", None,
     ["sigma_att_x", "sigma_att_y", "sigma_att_z"]),
    ("Accelerometer bias", "m/s^2", ["ba_x", "ba_y", "ba_z"],
     ["sigma_ba_x", "sigma_ba_y", "sigma_ba_z"]),
    ("Gyroscope bias", "rad/s", ["bg_x", "bg_y", "bg_z"],
     ["sigma_bg_x", "sigma_bg_y", "sigma_bg_z"]),
]

#: Shorter names for the sigma figure, whose panels are a fifth of a page tall
#: and whose full titles overlap each other down the shared axis.
SHORT_LABELS = {
    "Position NED": "Position",
    "Velocity NED": "Velocity",
    "Accelerometer bias": "Accel bias",
    "Gyroscope bias": "Gyro bias",
}


# ---------------------------------------------------------------- attitude
#
# The quaternion geometry lives in agreement.py, which the comparison with EKF2
# shares; what is left here is decimation for drawing.


def attitude_series(pair, points):
    """Decimated tilt and heading from `(t, {"q0": .., "q3": ..})`, or None."""
    if pair is None:
        return None
    t, q = pair
    tilt, heading = tilt_heading(*(q[n] for n in QUATERNION))
    return decimate(t, {"tilt": tilt, "heading": heading}, points)


# ----------------------------------------------------------------- readers


def read_header_and_rows(path):
    """Yield the `#` note lines, then the column names, then each row's cells."""
    notes, columns = [], None
    with open(path) as handle:
        for line in handle:
            line = line.rstrip("\n")
            if not line:
                continue
            if line.startswith("#"):
                if columns is None:
                    notes.append(line.lstrip("# ").rstrip())
                continue
            if columns is None:
                columns = line.split(",")
                yield notes, columns
                continue
            yield None, line.split(",")
    if columns is None:
        raise ReportError(f"{path}: no header row")


def rows_of(path, source=None, source_column="source"):
    """`(notes, columns, rows)`, the rows restricted to one row kind if `source`
    is given -- the union-schema input and reference files carry several."""
    stream = read_header_and_rows(path)
    notes, columns = next(stream)
    kind = columns.index(source_column) if source_column in columns else None
    if kind is None or source is None:
        return notes, columns, (cells for _, cells in stream)
    return notes, columns, (cells for _, cells in stream if cells[kind] == source)


def count_rows(path, source=None, source_column="source"):
    """Data rows, for choosing a decimation stride before reading values.

    A second pass over the file rather than a guess from its size: the largest
    corpus log is 444 MB and 1.4 M epochs, where a stride off by a factor of two
    is a visibly wrong figure, and counting newlines costs about a second.

    `source` counts only that row kind, and it is not optional for a filtered
    read. Counting the whole file and then reading one source out of it divides
    by every other source's rows as well: on the 2 h log that made the stride for
    the 1 Hz GNSS rows 371x too large, leaving 38 fixes out of 7000 and a gap
    detector that saw an outage nearly everywhere.
    """
    return sum(1 for _ in rows_of(path, source, source_column)[2])


class Decimator:
    """Streaming min/max envelope over a fixed number of buckets.

    Plotting 1.4 M points draws a band of ink and takes minutes; plotting every
    n-th point drops the spike that mattered. Each bucket keeps its extremes and
    the time each occurred at, so a transient survives at any stride and the
    trace stays monotone in time.
    """

    def __init__(self, names, rows, points):
        self.names = list(names)
        self.stride = max(1, math.ceil(rows / points)) if rows else 1
        self.buckets = {name: [] for name in self.names}
        self._pending = {name: None for name in self.names}
        self._index = 0

    def add(self, t, values):
        for name in self.names:
            value = values.get(name)
            if value is None:
                continue
            slot = self._pending[name]
            if slot is None:
                self._pending[name] = [t, value, t, value]
            else:
                if value < slot[1]:
                    slot[0], slot[1] = t, value
                if value > slot[3]:
                    slot[2], slot[3] = t, value
        self._index += 1
        if self._index % self.stride == 0:
            self._flush()

    def _flush(self):
        for name in self.names:
            slot = self._pending[name]
            if slot is None:
                continue
            first, second = ((slot[0], slot[1]), (slot[2], slot[3]))
            if first[0] > second[0]:
                first, second = second, first
            self.buckets[name].extend((first, second))
            self._pending[name] = None

    def done(self):
        self._flush()
        return {
            name: (
                np.array([p[0] for p in points], dtype=float),
                np.array([p[1] for p in points], dtype=float),
            )
            for name, points in self.buckets.items()
        }


def scan(path, wanted, points, source=None, source_column="source", keep=()):
    """One pass: `wanted` decimated, and `keep` undecimated.

    Returns `(notes, {name: (t, values)}, kept)`, where `kept` is
    `(t, {name: array})` over the rows carrying every one of `keep`, or None.
    `keep` is for columns that mean nothing apart -- a quaternion's four
    components -- which an envelope per column would decimate into a rotation
    nobody held, the mistake read_path exists to avoid for a track. Derive from
    them first, then `decimate` what was derived. One pass rather than two
    because on the 2 h log each costs about 5 s.
    """
    rows = count_rows(path, source, source_column) if wanted else 0
    notes, columns, stream = rows_of(path, source, source_column)
    index = {name: columns.index(name) for name in wanted if name in columns}
    kept_index = ([columns.index(name) for name in keep]
                  if keep and all(name in columns for name in keep) else None)
    if not index and kept_index is None:
        return notes, {}, None
    decimator = Decimator(index, rows, points)
    # array('d') rather than lists: on the 2 h log these are 1.4 M rows, and a
    # list of Python floats holds them in four times the memory.
    kept_t, kept = array.array("d"), [array.array("d") for _ in keep]
    for cells in stream:
        values = {}
        for name, position in index.items():
            cell = cells[position]
            if cell:
                values[name] = float(cell)
        if values:
            decimator.add(float(cells[0]), values)
        if kept_index is not None:
            here = [cells[i] for i in kept_index]
            if all(here):
                kept_t.append(float(cells[0]))
                for column_values, cell in zip(kept, here):
                    column_values.append(float(cell))
    result = None
    if kept_t:
        result = np.frombuffer(kept_t), {n: np.frombuffer(v) for n, v in zip(keep, kept)}
    return notes, decimator.done() if index else {}, result


def read_series(path, wanted, points, source=None, source_column="source"):
    """Decimated `{name: (t, values)}` for `wanted`, with the file's notes."""
    notes, series, _ = scan(path, wanted, points, source, source_column)
    return notes, series


def read_columns(path, names, source=None, source_column="source"):
    """Undecimated `(t, {name: array})` for rows carrying every one of `names`."""
    return scan(path, (), 0, source, source_column, keep=names)[2]


def read_table(path, names, source=None, source_column="source"):
    """Undecimated `(t, {name: array})` over every row of the kind, blanks as NaN.

    Where `read_columns` drops a row missing any column, this keeps it: the
    reference leaves `sigma_att_total` blank on a whole era and a reset counter
    blank where the topic lacks it, and dropping those rows would drop the log.
    A name the file does not carry is absent from the result. None for no rows.
    """
    _, columns, stream = rows_of(path, source, source_column)
    present = [name for name in names if name in columns]
    index = [columns.index(name) for name in present]
    t, values = array.array("d"), [array.array("d") for _ in present]
    nan = math.nan
    for cells in stream:
        t.append(float(cells[0]))
        for column, position in zip(values, index):
            cell = cells[position]
            column.append(float(cell) if cell else nan)
    if not t:
        return None
    return np.frombuffer(t), {n: np.frombuffer(v) for n, v in zip(present, values)}


def envelope(t, y, stride):
    """The min/max envelope Decimator keeps, over arrays already in memory.

    Vectorized because the arrays are: fed to Decimator a sample at a time, the
    tilt and heading of the 2 h log alone took 1.9 s. Non-finite samples are
    skipped, so a NaN heading past `agreement.INVERTED` leaves a gap rather than a bucket
    extreme. Buckets are by sample index, as Decimator's are by row.
    """
    index = np.flatnonzero(np.isfinite(y))
    if not len(index):
        return np.array([]), np.array([])
    bucket = index // stride
    # Sorted by bucket, then value: each bucket's first entry is its minimum and
    # its last its maximum.
    order = np.lexsort((y[index], bucket))
    starts = np.flatnonzero(np.r_[True, np.diff(bucket[order]) != 0])
    ends = np.r_[starts[1:], len(order)] - 1
    low, high = index[order[starts]], index[order[ends]]
    first, second = np.minimum(low, high), np.maximum(low, high)
    pairs = np.column_stack((first, second)).ravel()
    return t[pairs], y[pairs]


def decimate(t, series, points):
    """`{name: (t, values)}` for arrays already in memory, at read_series's stride."""
    stride = max(1, math.ceil(len(t) / points)) if len(t) else 1
    return {name: envelope(t, y, stride) for name, y in series.items()}


def read_times(path, source, source_column="source"):
    """Every timestamp for one row kind, undecimated.

    Gaps have to be found before decimation: a decimated series cannot tell an
    outage from its own stride, and reading one column of a 1 Hz source costs a
    few thousand floats even on the 2 h log.
    """
    return [float(cells[0]) for cells in rows_of(path, source, source_column)[2]]


def read_notes(path):
    """Just the `#` header lines of a file, without reading its rows."""
    if not path:
        return []
    notes, _ = next(read_header_and_rows(path))
    return notes


def read_resets(path):
    """Times at which EKF2's `quat_reset_counter` changed.

    Undecimated and cheap, and worth marking: EKF2 resets yaw in the first
    seconds of a flight, and without the marks that step reads as divergence
    from this filter rather than as an event in theirs.
    """
    table = read_table(path, ["att_reset"], source="ekf2_att")
    if table is None or "att_reset" not in table[1]:
        return []
    return list(agreement.change_times(table[0], table[1]["att_reset"]))


def read_path(path, x_name, y_name, points, source=None, source_column="source"):
    """Two columns sampled from the *same* rows, for a parametric plot.

    Not the min/max envelope `read_series` uses, and the distinction is not a
    nicety. An envelope keeps each column's extremes independently, so its point
    k for north and its point k for east come from different epochs: on the 2 h
    log the two disagreed in 3217 of 3994 buckets, and pairing them drew a
    zigzag of positions the vehicle never occupied, over a true track whose
    consecutive steps have a median of 0.59 mm.

    A uniform stride keeps each sample a real position. It can cut a corner
    between samples, which for a path is the honest failure: a reset displaces
    every later sample and still shows.
    """
    rows = count_rows(path, source, source_column)
    stride = max(1, math.ceil(rows / points)) if rows else 1
    _, columns, stream = rows_of(path, source, source_column)
    if x_name not in columns or y_name not in columns:
        return None
    xi, yi = columns.index(x_name), columns.index(y_name)
    x, y, index = [], [], 0
    for cells in stream:
        if index % stride == 0 and cells[xi] and cells[yi]:
            x.append(float(cells[xi]))
            y.append(float(cells[yi]))
        index += 1
    return (np.array(x), np.array(y)) if x else None


def read_runs(path, column, source=None, since=None, until=None,
              source_column="source"):
    """Contiguous `(start, end, value)` runs of one column, for shading a time axis.

    `until` extends the last run to a time the file itself does not reach: the
    reference writes a `vehicle_mode` row per change, so its last regime has no
    row marking where it ends. `since` clips the other end: that first row is
    the first `vehicle_status` sample, which on every corpus log lands 0.11 to
    0.34 s before the replay's first epoch, and a shade there would stretch the
    time axis below zero.
    """
    _, columns, stream = rows_of(path, source, source_column)
    if column not in columns:
        return []
    position = columns.index(column)
    runs, current = [], None
    for cells in stream:
        t, value = float(cells[0]), cells[position]
        if current is None or current[2] != value:
            if current is not None:
                current[1] = t
            current = [t, t, value]
            runs.append(current)
        else:
            current[1] = t
    if runs and until is not None:
        runs[-1][1] = max(runs[-1][1], until)
    if since is not None:
        runs = [[max(start, since), end, value] for start, end, value in runs
                if end > since]
    return runs


def read_fusion(path):
    """Gates from the header, and every fusion row grouped by source.

    Each source's degrees of freedom are how many `nu` columns it fills, not a
    table here, so a new source needs no edit in this file.
    """
    gates, sources = {}, {}
    stream = read_header_and_rows(path)
    notes, columns = next(stream)
    for note in notes:
        for token in note.split():
            if "=" in token and not token.startswith("nu"):
                name, _, value = token.partition("=")
                try:
                    gates[name] = float(value)
                except ValueError:
                    continue
    nu_columns = [c for c in columns if c.startswith("nu")]
    s_columns = [c for c in columns if c.startswith("s") and c[1:].isdigit()]
    positions = {name: columns.index(name) for name in nu_columns + s_columns}
    ratio, outcome = columns.index("ratio"), columns.index("outcome")
    for _, cells in stream:
        entry = sources.setdefault(
            cells[1], {"t": [], "nu": [], "s": [], "ratio": [], "outcome": []}
        )
        entry["t"].append(float(cells[0]))
        entry["outcome"].append(cells[outcome])
        entry["ratio"].append(float(cells[ratio]) if cells[ratio] else math.nan)
        entry["nu"].append([cells[positions[c]] for c in nu_columns])
        entry["s"].append([cells[positions[c]] for c in s_columns])
    return gates, sources


def read_summary(path):
    """The `summary` and `score` keys as one dict, from captured stdout."""
    keys = {}
    with open(path) as handle:
        for line in handle:
            if not (line.startswith("summary ") or line.startswith("score ")):
                continue
            for token in line.split()[1:]:
                name, _, value = token.partition("=")
                if name:
                    keys[name] = value
    if not keys:
        raise ReportError(
            f"{path}: no `summary` or `score` line. Capture the replay's stdout:\n"
            "  cargo run --release --example replay -- in.csv out.csv > summary.txt"
        )
    return keys


# ------------------------------------------------------- provenance checks


def numeric(keys, name):
    try:
        return float(keys[name])
    except (KeyError, ValueError):
        return None


def converted_from(notes):
    """The `.ulg` a `# Converted from <name>.ulg` header names, or None."""
    for note in notes or []:
        found = re.search(r"Converted from (\S+\.ulg)", note)
        if found:
            return found.group(1)
    return None


def scenario_of(notes):
    """The backticked scenario name and seed a simulated file's header carries.

    The same two fields `scenario_of` in examples/replay.rs reads, and for the
    same reason: a truth file from the wrong scenario has timestamps that line
    up often enough that nothing else notices.
    """
    for note in notes or []:
        if "fusion-nav" not in note:
            continue
        name = re.search(r"`([^`]+)`", note)
        seed = re.search(r"seed (\d+)", note)
        if name:
            stem = name.group(1)
            if stem.endswith(".csv"):
                stem = stem[:-4]
            return stem, (seed.group(1) if seed else None)
    return None


def check_pairing(input_notes, reference_notes, truth_notes):
    """Refuse a reference or a truth file that belongs to a different run.

    Kept separate from `check_provenance` because it needs no `--summary`: these
    are claims the files make about themselves, and the report should not plot a
    second flight over the first just because nobody captured a summary line.
    """
    problems = []

    source, reference_source = converted_from(input_notes), converted_from(reference_notes)
    if source and reference_source and source != reference_source:
        problems.append(
            f"the reference was converted from `{reference_source}` but the "
            f"replay input from `{source}`"
        )

    run, truth = scenario_of(input_notes), scenario_of(truth_notes)
    if run and truth and run != truth:
        problems.append(
            f"the truth file is for `{truth[0]}` seed {truth[1]}, but the log is "
            f"`{run[0]}` seed {run[1]}"
        )
    return problems


def check_provenance(keys, epoch_rows, sources, reference_notes):
    """Refuse a set of files that do not describe the same run.

    A captured `summary` has nothing in it tying it to the epoch CSV beside it,
    which is the hazard `Scoring::open` already refuses for a truth file from the
    wrong scenario: the timestamps line up often enough that nothing else
    notices, and the report then publishes one run's statistics beside another
    run's plots. Every check here is a read of a number the harness or the
    converter already wrote, never a recomputation of it.
    """
    problems = []

    epochs = numeric(keys, "epochs")
    if epochs is not None and epoch_rows and int(epochs) != epoch_rows:
        problems.append(
            f"the summary says epochs={int(epochs)} but the epoch CSV has "
            f"{epoch_rows} rows"
        )

    for source, entry in sorted(sources.items()):
        pinned = numeric(keys, f"rejected_{source}")
        if pinned is None:
            continue
        counted = sum(1 for verdict in entry["outcome"] if verdict == "rejected")
        if int(pinned) != counted:
            problems.append(
                f"the summary says rejected_{source}={int(pinned)} but the fusion "
                f"CSV holds {counted} rejected rows for it"
            )

    rate = numeric(keys, "rate")
    period = reference_period(reference_notes)
    if rate and period:
        # The converter states the IMU interval it derived EKF2's mean update
        # period from, and the harness measures the same quantity as `rate=`.
        # Two estimators of one number, and nothing else would notice them
        # drifting apart -- it would move a published bias figure silently.
        used = reference_interval(reference_notes)
        if used and abs(used - 1.0 / rate) > 0.1 / rate:
            problems.append(
                f"the reference derived EKF2's update period from a "
                f"{used * 1e3:.3f} ms IMU interval but the summary reports "
                f"rate={rate:g} ({1e3 / rate:.3f} ms)"
            )

    return problems


def reference_period(notes):
    for note in notes or []:
        if note.startswith("Bias states are delta"):
            return note
    return None


def reference_interval(notes):
    """The IMU interval the converter rounded EKF2's update period up to.

    Anchored on the whole phrase, not on the first `ms` in the line: that note
    also carries `FILTER_UPDATE_PERIOD_MS 10 ms`, and reading the interval off
    that read 10 ms on every legacy log and made this check fire on files that
    matched.
    """
    for note in notes or []:
        found = re.search(r"([0-9]*\.?[0-9]+) ms IMU interval", note)
        if found:
            return float(found.group(1)) * 1e-3
    return None


def reference_origin(notes):
    for note in notes or []:
        if note.startswith("EKF2 origin:"):
            return note.partition(":")[2].strip()
    return None


# ----------------------------------------------------------------- figures


def figure(build, caption, width=11.0, height=4.4):
    """Run `build` on a fresh axes set and return (png bytes, caption)."""
    fig = plt.figure(figsize=(width, height), dpi=110)
    build(fig)
    fig.tight_layout()
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", bbox_inches="tight")
    plt.close(fig)
    return buffer.getvalue(), caption


def shade_runs(axes, runs, palette, alpha, span=(0.0, 1.0)):
    for start, end, value in runs:
        shade = palette.get(value)
        if shade and end > start:
            axes.axvspan(start, end, ymin=span[0], ymax=span[1], color=shade,
                         alpha=alpha, linewidth=0)


class Backdrop:
    """What every time-axis panel shades behind its traces: `Status` over the
    whole height, and the flight regime as a strip along the top."""

    def __init__(self, status_runs, mode_runs=()):
        self.status_runs = status_runs
        self.mode_runs = mode_runs

    def draw(self, axes):
        shade_runs(axes, self.status_runs, STATUS_SHADES, 0.18)
        shade_runs(axes, self.mode_runs, MODE_SHADES, 0.45, span=(0.93, 1.0))


def gap_threshold(times, multiple=agreement.HOLD):
    """How long a silence has to be, for this source, to count as an outage.

    The multiple is the one `agreement.HOLD` holds a rejection for, so a shaded
    outage and a rejection that stops being counted mean the same silence.

    Relative to the source's own cadence rather than an absolute number of
    seconds, which means different things at 1 Hz and at 10 Hz. At a fixed 2 s
    the 2 h log shaded 1219 intervals -- true, its receiver really does miss
    that many fixes, and useless, because the plot became grey.
    """
    intervals = [b - a for a, b in zip(times, times[1:]) if b > a]
    if not intervals:
        return None
    return multiple * float(np.median(intervals))


def find_gaps(times, floor):
    """The `(start, end)` silences longer than `floor`.

    Found once and passed to both the figure and its caption, rather than
    counted as a side effect of drawing: a caption is an argument, so it is
    built before the figure it describes and a count filled in during the draw
    reads zero.
    """
    if not floor or not times:
        return []
    return [(a, b) for a, b in zip(times, times[1:]) if b - a > floor]


def shade_gaps(axes, gaps):
    for start, end in gaps:
        axes.axvspan(start, end, color="#8899aa", alpha=0.25, linewidth=0)


def track_figure(epochs, reference, fixes):
    def build(fig):
        axes = fig.add_subplot(111)
        if fixes is not None:
            axes.plot(fixes[1], fixes[0], ".", color="#bbbbbb", markersize=3,
                      label="GNSS fixes", zorder=1)
        if reference:
            axes.plot(reference[1], reference[0], "-", color="#3a7ca5",
                      linewidth=1.2, label="EKF2", zorder=2)
        axes.plot(epochs[1], epochs[0], "-", color="#1b1b1b", linewidth=1.0,
                  label="fusion-nav", zorder=3)
        axes.set_xlabel("East (m)")
        axes.set_ylabel("North (m)")
        axes.set_aspect("equal", adjustable="datalim")
        axes.grid(alpha=0.25)
        axes.legend(loc="best", fontsize=8)
    return build


def state_figure(group, epochs, reference, truth, backdrop):
    title, unit, columns, sigmas = group

    def build(fig):
        axes = fig.subplots(len(columns), 1, sharex=True)
        for row, (column, sigma) in enumerate(zip(columns, sigmas)):
            plot = axes[row]
            backdrop.draw(plot)
            ours = epochs.get(column)
            if ours is None:
                continue
            t, y = ours
            band = epochs.get(sigma)
            if band is not None and len(band[1]) == len(y):
                spread = 3.0 * band[1]
                plot.fill_between(t, y - spread, y + spread, color="#1b1b1b",
                                  alpha=0.12, linewidth=0, label="+/-3 sigma")
            plot.plot(t, y, "-", color="#1b1b1b", linewidth=0.9, label="fusion-nav")
            for other, colour, label in ((reference, "#3a7ca5", "EKF2"),
                                         (truth, "#c05a2f", "truth")):
                series = (other or {}).get(column)
                if series is not None:
                    plot.plot(series[0], series[1], "-", color=colour,
                              linewidth=0.9, alpha=0.85, label=label)
            plot.set_ylabel(f"{column} ({unit})", fontsize=8)
            plot.grid(alpha=0.25)
            if row == 0:
                plot.legend(loc="upper right", fontsize=7, ncol=4)
        axes[-1].set_xlabel("t (s)")
    return build


def attitude_figure(ours, reference, truth, backdrop, resets):
    """Tilt and heading, each `{"tilt": (t, y), "heading": (t, y)}` in radians.

    No sigma band, and deliberately: the epoch file's attitude sigmas are
    body-axis, tilt and heading are navigation-frame, and turning one into the
    other needs off-diagonals the file does not carry. Near level the body split
    passes for tilt and heading; at 90 deg of pitch it does not (#131). The band
    is on the difference figure, whose frame matches it.

    Heading is drawn as points, not a line: it wraps at +/-180 deg, and a line or
    a min/max bucket across the wrap draws a vertical stroke the vehicle never
    made.
    """
    def build(fig):
        axes = fig.subplots(2, 1, sharex=True)
        for plot, name in zip(axes, ("tilt", "heading")):
            backdrop.draw(plot)
            style = ("-", {"linewidth": 0.9}) if name == "tilt" else (".", {"markersize": 1.2})
            for series, colour, label in ((ours, "#1b1b1b", "fusion-nav"),
                                          (reference, "#3a7ca5", "EKF2"),
                                          (truth, "#c05a2f", "truth")):
                trace = (series or {}).get(name)
                if trace is not None and len(trace[0]):
                    plot.plot(trace[0], np.degrees(trace[1]), style[0], color=colour,
                              alpha=0.85, label=label, **style[1])
            for when in resets:
                plot.axvline(when, color="#3a7ca5", linestyle=":", linewidth=1.0)
            plot.set_ylabel(f"{name} (deg)", fontsize=8)
            plot.grid(alpha=0.25)
        axes[1].set_ylim(-185.0, 185.0)
        axes[0].legend(loc="upper right", fontsize=7, ncol=3, markerscale=6)
        axes[-1].set_xlabel("t (s)")
    return build


def difference_figure(difference, epochs, backdrop, resets):
    """This filter's attitude relative to EKF2's, in body axes, with its own band.

    `difference` is `{"d_x": (t, y), ...}`, radians, from `agreement.rotation_difference`.
    The band is this filter's `sigma_att_x/y/z` around zero -- the same frame as
    the trace it surrounds, at any attitude.
    """
    def build(fig):
        axes = fig.subplots(3, 1, sharex=True)
        for plot, axis in zip(axes, "xyz"):
            backdrop.draw(plot)
            band = epochs.get(f"sigma_att_{axis}")
            if band is not None:
                spread = 3.0 * np.degrees(band[1])
                plot.fill_between(band[0], -spread, spread, color="#1b1b1b",
                                  alpha=0.12, linewidth=0,
                                  label="fusion-nav +/-3 sigma")
            trace = difference.get(f"d_{axis}")
            if trace is not None:
                plot.plot(trace[0], np.degrees(trace[1]), "-", color="#3a7ca5",
                          linewidth=0.9, label="fusion-nav relative to EKF2")
            for when in resets:
                plot.axvline(when, color="#3a7ca5", linestyle=":", linewidth=1.0)
            plot.axhline(0.0, color="#888888", linewidth=0.6)
            plot.set_ylabel(f"body {axis} (deg)", fontsize=8)
            plot.grid(alpha=0.25)
        axes[0].legend(loc="upper right", fontsize=7, ncol=2)
        axes[-1].set_xlabel("t (s)")
    return build


#: EKF2's sigma column for each of ours, where the two are the same quantity.
#: Attitude is absent on purpose: PX4's diagonal is NED and the epoch file's is
#: body, and only `sigma_att_total` compares, which is plotted on its own.
EKF2_SIGMAS = {
    "sigma_pos_n": "sigma_pos_n", "sigma_pos_e": "sigma_pos_e",
    "sigma_pos_d": "sigma_pos_d", "sigma_vel_n": "sigma_vel_n",
    "sigma_vel_e": "sigma_vel_e", "sigma_vel_d": "sigma_vel_d",
    "sigma_ba_x": "sigma_ba_x", "sigma_ba_y": "sigma_ba_y",
    "sigma_ba_z": "sigma_ba_z", "sigma_bg_x": "sigma_bg_x",
    "sigma_bg_y": "sigma_bg_y", "sigma_bg_z": "sigma_bg_z",
}


def positive(series):
    """Drop non-positive samples, which a log axis cannot show.

    EKF2 publishes a zero covariance entry before its own filter has
    initialized, and for states it is not estimating. That is "not reported",
    not "very small", so it is dropped rather than floored -- flooring it to a
    small constant stretched the shared axis over twelve decades and flattened
    every real trace onto one line.
    """
    if series is None:
        return None
    t, y = series
    keep = y > 0.0
    return (t[keep], y[keep]) if keep.any() else None


def sigma_figure(epochs, reference, gaps, backdrop):
    """One panel per state group, ours solid and EKF2's dashed.

    Split by group rather than shared, because a position sigma in metres and a
    gyro-bias sigma in rad/s on one log axis is 27 traces across six decades and
    legible as none of them.
    """
    def build(fig):
        plots = fig.subplots(len(STATE_GROUPS), 1, sharex=True, squeeze=False)
        for row, (title, unit, _columns, sigmas) in enumerate(STATE_GROUPS):
            plot = plots[row][0]
            # Attitude sigmas are radians in both files, while the attitude
            # figures draw in degrees. Converted here too, so every panel reads
            # in one unit and the label is true.
            scale = 180.0 / math.pi if title == "Attitude" else 1.0
            backdrop.draw(plot)
            shade_gaps(plot, gaps)
            for sigma in sigmas:
                series = positive(epochs.get(sigma))
                if series is not None:
                    plot.plot(series[0], series[1] * scale, linewidth=0.8,
                              label=sigma, alpha=0.9)
            # EKF2's own, dashed and unlabelled so the legend stays ours. These
            # are what exercise the layout-keyed covariance map: the bias
            # states sit at one index in both eras, and only these move.
            drawn = False
            for sigma in sigmas:
                series = positive(reference.get(EKF2_SIGMAS.get(sigma)))
                if series is not None:
                    plot.plot(series[0], series[1] * scale, "--", linewidth=0.8,
                              color="#3a7ca5", alpha=0.7,
                              label="EKF2" if not drawn else None)
                    drawn = True
            if title == "Attitude":
                total = positive(reference.get("sigma_att_total"))
                if total is not None:
                    plot.plot(total[0], total[1] * scale, "--", linewidth=0.9,
                              color="#c05a2f", alpha=0.85,
                              label="EKF2 sigma_att_total")
            plot.set_yscale("log")
            plot.set_ylabel(f"{SHORT_LABELS.get(title, title)} ({unit})",
                            fontsize=7)
            plot.grid(alpha=0.25, which="both")
            plot.legend(loc="upper right", fontsize=6, ncol=4)
        plots[-1][0].set_xlabel("t (s)")
    return build


def innovation_axes(entry):
    """A source's dimension: how many `nu` columns it actually fills.

    Read from the rows rather than from a table of sources here, and used for
    the subplot count *and* the figure height. Taking the height from
    `len(row)` instead reads the schema's width, which is always 3, and lays a
    one-axis source out over six inches.
    """
    return max((sum(1 for cell in row if cell) for row in entry["nu"]), default=0)


def innovation_figure(source, entry, gate, backdrop):
    axes_count = innovation_axes(entry)

    def build(fig):
        plots = fig.subplots(max(1, axes_count), 1, sharex=True, squeeze=False)
        for axis in range(max(1, axes_count)):
            plot = plots[axis][0]
            backdrop.draw(plot)
            t, normalized = [], []
            for k, (nu, s) in enumerate(zip(entry["nu"], entry["s"])):
                if axis >= len(nu) or not nu[axis] or not s[axis]:
                    continue
                variance = float(s[axis])
                if variance <= 0.0:
                    continue
                t.append(entry["t"][k])
                normalized.append(float(nu[axis]) / math.sqrt(variance))
            plot.plot(t, normalized, ".", markersize=2, color="#1b1b1b",
                      label=f"nu_{axis} / sqrt(S_{axis}{axis})")
            rejected = [
                entry["t"][k]
                for k, verdict in enumerate(entry["outcome"])
                if verdict == "rejected"
            ]
            for when in rejected:
                plot.axvline(when, color="#d94f4f", alpha=0.35, linewidth=0.6)
            plot.axhline(0.0, color="#888888", linewidth=0.6)
            plot.set_ylabel(f"axis {axis}", fontsize=8)
            plot.grid(alpha=0.25)
            if axis == 0:
                label = f"{source}, gate gamma = {gate:.3f}" if gate else source
                plot.set_title(label + f" ({len(rejected)} rejected)", fontsize=9)
        plots[-1][0].set_xlabel("t (s)")
    return build


#: The epoch file's ratio columns, which the reference file spells alike.
RATIOS = ["r_gnss_pos", "r_gnss_hgt", "r_gnss_vel", "r_baro", "r_mag"]


def ratio_figure(epochs, reference):
    """Test ratios, ours over EKF2's.

    The like-for-like comparison GOALS.md differentiator 6 names: both report
    `r = eps / gamma`, so 1 is each filter's own gate whatever its dimension or
    its thresholds, and the two are directly comparable without knowing either.
    """
    present = [r for r in RATIOS if r in epochs or r in reference]

    def build(fig):
        plots = fig.subplots(max(1, len(present)), 1, sharex=True, squeeze=False)
        for row, name in enumerate(present):
            plot = plots[row][0]
            ours = epochs.get(name)
            if ours is not None:
                plot.plot(ours[0], ours[1], "-", linewidth=0.8, color="#1b1b1b",
                          label="fusion-nav")
            theirs = reference.get(name)
            if theirs is not None:
                plot.plot(theirs[0], theirs[1], "-", linewidth=0.8,
                          color="#3a7ca5", alpha=0.85, label="EKF2")
            plot.axhline(1.0, color="#d94f4f", linewidth=0.8, linestyle="--")
            plot.set_ylabel(name, fontsize=8)
            plot.grid(alpha=0.25)
            if row == 0:
                plot.legend(loc="upper right", fontsize=7, ncol=2)
        plots[-1][0].set_xlabel("t (s)")
    return build


def nis_figure(sources, gates, keys):
    """Normalized innovation squared against its chi-square reference.

    The quantity is `epsilon = ratio * gamma`, recovered from what the filter
    published the way the harness's own
    `nis_is_recovered_with_the_gate_the_filter_was_configured_with` does, not
    rebuilt from nu and diag(S): `epsilon = nu' S^-1 nu` needs the off-diagonals,
    and the fusion CSV carries the diagonal alone, so a histogram built from
    those columns would be a second and wrong implementation of a number the
    harness already defines.

    Each panel is titled with the pinned `nis_<source>` rather than a mean taken
    here. The two are the same quantity at different normalizations -- chi2(dof)
    is the distribution of `epsilon`, while `nis_` is the mean of `epsilon`
    divided by the degrees of freedom, so it is 1 for an honest filter whatever
    the dimension -- and printing the harness's number keeps the plot and the
    manifest reading from one implementation.
    """
    panels = []
    for source, entry in sorted(sources.items()):
        gate = gates.get(source)
        dof = max((sum(1 for cell in row if cell) for row in entry["nu"]), default=0)
        epsilon = np.array(
            [
                r * gate
                for r, verdict in zip(entry["ratio"], entry["outcome"])
                if gate and not math.isnan(r)
            ],
            dtype=float,
        )
        if dof and epsilon.size:
            panels.append((source, dof, epsilon))

    def build(fig):
        if not panels:
            fig.add_subplot(111).text(0.5, 0.5, "no gated fusions",
                                      ha="center", va="center")
            return
        plots = fig.subplots(2, len(panels), squeeze=False)
        for column, (source, dof, epsilon) in enumerate(panels):
            top = plots[0][column]
            limit = float(np.percentile(epsilon, 99.5)) or 1.0
            top.hist(epsilon, bins=60, range=(0.0, limit), density=True,
                     color="#3a7ca5", alpha=0.75)
            grid = np.linspace(1e-6, limit, 400)
            top.plot(grid, stats.chi2.pdf(grid, dof), color="#1b1b1b",
                     linewidth=1.0, label=f"chi2({dof})")
            pinned = keys.get(f"nis_{source}", "not captured")
            top.set_title(f"{source}, dof {dof}, n {epsilon.size}\n"
                          f"nis_{source} = {pinned}", fontsize=8)
            top.set_xlabel("NIS", fontsize=8)
            top.legend(fontsize=7)
            top.grid(alpha=0.25)

            bottom = plots[1][column]
            ordered = np.sort(epsilon)
            quantiles = (np.arange(ordered.size) + 0.5) / ordered.size
            bottom.plot(stats.chi2.ppf(quantiles, dof), ordered, ".",
                        markersize=2, color="#3a7ca5")
            top_right = max(ordered[-1], stats.chi2.ppf(0.999, dof))
            bottom.plot([0, top_right], [0, top_right], color="#1b1b1b",
                        linewidth=0.8)
            bottom.set_xlabel(f"chi2({dof}) quantile", fontsize=8)
            bottom.set_ylabel("observed NIS", fontsize=8)
            bottom.grid(alpha=0.25)
    return build


# ---------------------------------------------------------------- document


PAGE = """<!doctype html>
<meta charset="utf-8">
<title>{title}</title>
<style>
 body {{ font: 15px/1.55 -apple-system, "Segoe UI", system-ui, sans-serif;
        max-width: 60rem; margin: 2.5rem auto; padding: 0 1.25rem; color: #1b1b1b; }}
 h1 {{ font-size: 1.5rem; margin-bottom: 0.2rem; }}
 h2 {{ font-size: 1.1rem; margin-top: 2.2rem; border-bottom: 1px solid #ddd;
       padding-bottom: 0.3rem; }}
 p.caption {{ color: #555; font-size: 0.86rem; margin: 0.4rem 0 1.4rem; }}
 img {{ max-width: 100%; height: auto; }}
 table {{ border-collapse: collapse; font-size: 0.84rem; }}
 td, th {{ border: 1px solid #ddd; padding: 0.18rem 0.5rem; text-align: left; }}
 code {{ background: #f4f4f4; padding: 0.05rem 0.25rem; }}
 .meta {{ color: #555; font-size: 0.86rem; }}
</style>
<h1>{title}</h1>
<p class="meta">{meta}</p>
{body}
"""


def png_block(png, caption):
    encoded = base64.b64encode(png).decode("ascii")
    return (
        f'<img src="data:image/png;base64,{encoded}" alt="">\n'
        f'<p class="caption">{caption}</p>'
    )


def key_table(keys):
    families = {}
    for name, value in sorted(keys.items()):
        prefix = name.split("_")[0]
        families.setdefault(prefix, []).append((name, value))
    rows = []
    for prefix, entries in sorted(families.items()):
        cells = ", ".join(
            f"<code>{html.escape(n)}</code>&nbsp;{html.escape(v)}" for n, v in entries
        )
        rows.append(f"<tr><th>{html.escape(prefix)}</th><td>{cells}</td></tr>")
    return "<table>" + "".join(rows) + "</table>"


# --------------------------------------------------------- agreement with EKF2
#
# The files read here, the statistics in agreement.py. What each key means, and
# what it cannot say, is data/README.md's "Agreement with EKF2".

AGREEMENT_EPOCH_COLUMNS = (
    [f"{k}_{a}" for k in ("pos", "vel") for a in "ned"]
    + [f"sigma_{k}_{a}" for k in ("pos", "vel") for a in "ned"]
    + [f"{k}_{a}" for k in ("ba", "bg") for a in "xyz"]
    + [f"sigma_{k}_{a}" for k in ("ba", "bg", "att") for a in "xyz"]
    + QUATERNION
)

REFERENCE_KINDS = {
    "local": ("ekf2_local", [f"{k}_{a}" for k in ("pos", "vel") for a in "ned"]
              + ["xy_reset", "z_reset", "vxy_reset", "vz_reset"]),
    "att": ("ekf2_att", QUATERNION + ["att_reset"]),
    "states": ("ekf2_states", [f"{k}_{a}" for k in ("ba", "bg") for a in "xyz"]
               + [f"sigma_{k}_{a}" for k in ("pos", "vel") for a in "ned"]
               + [f"sigma_{k}_{a}" for k in ("ba", "bg") for a in "xyz"]
               + ["sigma_att_total"]),
    "ratio": ("ekf2_ratio", ["r_gnss_pos", "r_gnss_vel", "r_baro", "r_mag"]),
}

#: Harness keys printed beside the agreement ones, read off the `summary` line
#: and never recomputed: what each filter fused and how often it stepped. The
#: per-source ones are found by prefix, so a new source needs no edit here.
HARNESS_KEYS = ["r_policy", "alpha0", "resets", "recovered"]
HARNESS_PREFIXES = ("rejected_", "nis_")


def harness_keys(keys):
    """The summary keys printed beside the agreement ones, in the line's order."""
    return [k for k in keys
            if k in HARNESS_KEYS
            or (k.startswith(HARNESS_PREFIXES) and not k.startswith("nis_over95_"))]


#: The corpus table's sections, each a set of key prefixes, in reading order.
AGREEMENT_SECTIONS = [
    ("Position and velocity", ("pos_", "vel_")),
    ("Height", ("climb", "height_reference")),
    ("Attitude", ("tilt_", "heading_", "att_nd2")),
    ("Biases", ("ba_", "bg_")),
    ("Rejections, seconds", ("rej_s_",)),
    ("Resets and harness keys", ("estimator", "ekf2_", *HARNESS_KEYS, *HARNESS_PREFIXES)),
]


def reference_offset(notes):
    """EKF2's origin in the replay frame, `(n, e, d)`, from the reference header."""
    for note in notes:
        found = re.match(r"EKF2 origin in replay frame: (\S+) (\S+) (\S+) m", note)
        if found:
            # `none` down is a fix with no MSL height: horizontal still aligns.
            return tuple(math.nan if v == "none" else float(v) for v in found.groups())
    return None


def reference_estimator(notes):
    """Which PX4 estimator the reference came from, as its header names it."""
    for note in notes:
        found = re.match(r"Estimator: (\w+)", note)
        if found:
            return found.group(1)
    return None


def reference_height(notes):
    """The height reference EKF2 converged to, as the reference header names it."""
    for note in notes:
        found = re.search(r"EKF2 aiding: height reference (\w+)", note)
        if found and found.group(1) != "unknown":
            return found.group(1)
    return None


def agreement_of(ours, reference, sources):
    """Every agreement key for one run, from its epochs as
    `(t, {AGREEMENT_EPOCH_COLUMNS: array})`, the reference beside it, and its
    fusion rows grouped by source (read_fusion)."""
    if ours is None:
        raise ReportError("no epochs to compare")
    notes = read_notes(reference)
    ekf2 = {}
    for kind, (source, names) in REFERENCE_KINDS.items():
        table = read_table(reference, names, source=source)
        if table is not None:
            ekf2[kind] = table
    rejected = {
        source: (np.array(entry["t"]),
                 np.array([verdict == "rejected" for verdict in entry["outcome"]]))
        for source, entry in sources.items()
    }
    values = agreement.compare(ours, ekf2, rejected, reference_offset(notes),
                               reference_height(notes))
    # Not a statistic: which estimator every figure beside it is a distance from.
    return {"estimator": reference_estimator(notes), **values}


def agreement_line(values, keys):
    """`key=value ...`: `r_policy`, the agreement keys, then the harness keys read
    beside them. Every key, always: which of them `data/ekf2.txt` pins is
    `data/fetch.sh`'s decision, since it owns the manifest that pins the rest."""
    pairs = [("r_policy", keys.get("r_policy", "none"))]
    pairs += [(k, agreement.format_value(v)) for k, v in values.items()]
    pairs += [(k, keys[k]) for k in harness_keys(keys) if k != "r_policy"]
    return " ".join(f"{k}={v}" for k, v in pairs)


def section_of(name):
    for title, prefixes in AGREEMENT_SECTIONS:
        if any(name == p or name.startswith(p) for p in prefixes):
            return title
    return AGREEMENT_SECTIONS[-1][0]


def agreement_tables(runs):
    """One HTML table per section, a row per run: `runs` is `[(label, {key: text})]`."""
    blocks = []
    for title, _ in AGREEMENT_SECTIONS:
        names = []
        for _, line in runs:
            names += [n for n in line if section_of(n) == title and n not in names]
        if not names:
            continue
        head = "".join(f"<th><code>{html.escape(n)}</code></th>" for n in names)
        body = "".join(
            f"<tr><th>{html.escape(label)}</th>"
            + "".join(f"<td>{html.escape(line.get(n, '-'))}</td>" for n in names)
            + "</tr>"
            for label, line in runs
        )
        blocks.append(f"<h3>{html.escape(title)}</h3>"
                      f'<div style="overflow-x:auto"><table><tr><th></th>{head}</tr>'
                      f"{body}</table></div>")
    return "\n".join(blocks)


AGREEMENT_CAPTION = (
    "Distance from EKF2's own solution on the same log (or from the PX4 estimator "
    "<code>estimator</code> names): agreement, not accuracy, and <code>none</code> "
    "where the log cannot supply a quantity. What each key is and what it cannot "
    "say: <a href=\"https://github.com/wboayue/fusion-nav/blob/main/data/README.md"
    "#agreement-with-ekf2\">data/README.md, Agreement with EKF2</a>."
)


def build_corpus(directory):
    """`(page, lines)` for every run under `directory`, as `data/fetch.sh --compare`
    lays it out: `<log>/input.csv`, `<log>/reference.csv`, and per `R` policy
    `<log>/<policy>.csv` with its `.fusion.csv` and captured `.summary`, plus a
    `commit` file naming the build that replayed them.

    Each run passes the same pairing and provenance checks a single report
    does, so one stale file in the directory stops the table rather than
    publishing a row from another run.
    """
    directory = Path(directory)
    commit_file = directory / "commit"
    commit = commit_file.read_text().strip() if commit_file.exists() else "unknown"
    runs, lines, notes_rows = [], [], []
    for log in sorted(p for p in directory.iterdir() if p.is_dir()):
        input_path, reference = log / "input.csv", log / "reference.csv"
        reference = reference if reference.exists() else None
        reference_notes = read_notes(reference)
        notes_rows.append((log.name, reference_notes))
        for summary in sorted(log.glob("*.summary")):
            replay = summary.with_suffix(".csv")
            keys = read_summary(summary)
            _, sources = read_fusion(fusion_path(replay))
            # One pass over the epochs serves both the row count the provenance
            # check reads and the comparison.
            ours = read_table(replay, AGREEMENT_EPOCH_COLUMNS)
            epoch_rows = 0 if ours is None else len(ours[0])
            problems = check_pairing(read_notes(input_path), reference_notes, [])
            problems += check_provenance(keys, epoch_rows, sources, reference_notes)
            if keys.get("r_policy") != summary.stem:
                problems.append(f"{summary.name} says r_policy={keys.get('r_policy')}")
            if problems:
                raise ReportError(f"{log.name}/{summary.stem}: these files do not "
                                  "describe the same run:\n  - " + "\n  - ".join(problems))
            values = agreement_of(ours, reference, sources) if reference else {}
            line = agreement_line(values, keys)
            lines.append(f"agreement log={log.name} {line}")
            runs.append((f"{log.name[:8]} {summary.stem}",
                         dict(token.split("=", 1) for token in line.split())))
    if not runs:
        raise ReportError(f"{directory}: no `<log>/<policy>.summary` runs")

    origins = "".join(
        f"<tr><th>{html.escape(name[:8])}</th><td>"
        + "<br>".join(html.escape(n) for n in notes
                      if n.startswith(("Estimator", "EKF2 origin", "EKF2 aiding",
                                       "EKF2 layout")))
        + "</td></tr>"
        for name, notes in notes_rows
    )
    body = (f'<p class="caption">{AGREEMENT_CAPTION}</p>'
            + agreement_tables(runs)
            + "<h3>What each reference says about itself</h3><table>" + origins + "</table>")
    page = PAGE.format(title="Agreement with EKF2",
                       meta=f"commit {html.escape(commit)} &middot; {len(runs)} runs",
                       body=body)
    return page, lines


# -------------------------------------------------------------------- main


def build_report(args):
    epoch_rows = count_rows(args.replay)
    status_runs = read_runs(args.replay, "status")

    epoch_columns = ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d",
                     "ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z"]
    for _title, _unit, _columns, sigmas in STATE_GROUPS:
        epoch_columns.extend(sigmas)
    epoch_columns.extend(RATIOS)
    # The agreement section's columns ride on this pass, undecimated, rather than
    # a pass of their own. Measured on the 2 h log it saves memory rather than
    # time (peak 1.12 GB to 0.95, 32.8 s to 32.5): the cost is parsing the cells,
    # which either way happens once.
    _, epochs, ours_table = scan(args.replay, epoch_columns, args.points,
                                 keep=AGREEMENT_EPOCH_COLUMNS)
    ours_q = None
    if ours_table is not None:
        ours_q = ours_table[0], {n: ours_table[1][n] for n in QUATERNION}

    gates, sources = read_fusion(fusion_path(args.replay))

    reference, reference_notes, resets = {}, [], []
    reference_q, mode_runs = None, []
    if args.reference:
        reference_notes, local = read_series(
            args.reference, ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d"],
            args.points, source="ekf2_local")
        # Every remaining reference column is read here, which is what makes a
        # wrong entry in EKF2_LAYOUTS visible: the bias *states* sit at the same
        # index in both eras, so only these sigmas exercise the era-specific map.
        _, states = read_series(
            args.reference,
            ["ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z",
             "sigma_pos_n", "sigma_pos_e", "sigma_pos_d",
             "sigma_vel_n", "sigma_vel_e", "sigma_vel_d",
             "sigma_att_n", "sigma_att_e", "sigma_att_d", "sigma_att_total",
             "sigma_ba_x", "sigma_ba_y", "sigma_ba_z",
             "sigma_bg_x", "sigma_bg_y", "sigma_bg_z"],
            args.points, source="ekf2_states")
        _, ratios = read_series(args.reference, RATIOS, args.points,
                                source="ekf2_ratio")
        reference = {**local, **states, **ratios}
        resets = read_resets(args.reference)
        reference_q = read_columns(args.reference, QUATERNION, source="ekf2_att")
        start, end = (float(ours_q[0][0]), float(ours_q[0][-1])) if ours_q else (None, None)
        mode_runs = read_runs(args.reference, "mode", source="vehicle_mode",
                              since=start, until=end)

    truth, truth_notes, truth_q = {}, [], None
    if args.truth:
        truth_notes, truth = read_series(args.truth, epoch_columns, args.points)
        euler = read_columns(args.truth, ["roll", "pitch", "yaw"])
        if euler:
            t, angles = euler
            truth_q = t, dict(zip(QUATERNION, quaternion_from_euler(
                angles["roll"], angles["pitch"], angles["yaw"])))

    problems = check_pairing(read_notes(args.input), reference_notes, truth_notes)
    keys = read_summary(args.summary) if args.summary else {}
    if keys:
        problems += check_provenance(keys, epoch_rows, sources, reference_notes)
    if problems:
        raise ReportError(
            "these files do not describe the same run:\n  - " + "\n  - ".join(problems)
        )

    backdrop = Backdrop(status_runs, mode_runs)
    difference = None
    if ours_q and reference_q:
        t, d = attitude_difference(ours_q, reference_q)
        difference = decimate(t, dict(zip(("d_x", "d_y", "d_z"), d)), args.points)

    # `gnss_pos` carries north in v0 and east in v1, the replay input's union
    # schema. Paired from one row, never from two decimated columns.
    fixes = read_path(args.input, "v0", "v1", args.points, source="gnss_pos")
    gaps = read_times(args.input, "gnss_pos") if fixes is not None else None

    blocks = []

    origin = reference_origin(reference_notes)
    period = reference_period(reference_notes)
    blocks.append("<h2>Inputs</h2>")
    inputs = [
        ("input", args.input), ("epochs", args.replay),
        ("fusion", fusion_path(args.replay)), ("reference", args.reference or "-"),
        ("truth", args.truth or "-"), ("summary", args.summary or "-"),
    ]
    blocks.append(
        "<table>"
        + "".join(
            f"<tr><th>{html.escape(name)}</th><td><code>{html.escape(str(value))}"
            f"</code></td></tr>"
            for name, value in inputs
        )
        + (f"<tr><th>EKF2 origin</th><td>{html.escape(origin)}</td></tr>" if origin else "")
        + (f"<tr><th>EKF2 bias scale</th><td>{html.escape(period)}</td></tr>" if period else "")
        + f"<tr><th>decimation</th><td>{epoch_rows} epochs to about "
          f"{args.points} points per trace, min/max envelope per bucket</td></tr>"
        + "</table>"
    )

    ours_track = read_path(args.replay, "pos_n", "pos_e", args.points)
    if ours_track is not None:
        blocks.append("<h2>Horizontal track</h2>")
        reference_track = None
        if args.reference:
            reference_track = read_path(args.reference, "pos_n", "pos_e",
                                        args.points, source="ekf2_local")
        png, caption = figure(
            track_figure(ours_track, reference_track, fixes),
            "Estimate, EKF2's own solution where the log carries one, and the raw "
            "GNSS fixes the filter was offered. EKF2's track is relative to its "
            "own origin, which is not this filter's: two corpus logs report no "
            "origin at all. Every track here is sampled at a uniform stride, so "
            "each point is a position that was actually held &mdash; the "
            "min/max envelope the time series use pairs two columns from "
            "different epochs and draws a path nobody travelled.",
            width=7.5, height=7.0)
        blocks.append(png_block(png, caption))

    shading = ("Background shading is <code>Status</code>: amber Aligning, "
               "yellow Degraded, red DeadReckoning.")
    if mode_runs:
        regimes = sorted({run[2] for run in mode_runs})
        shading += (" The strip along the top of each panel is the vehicle's "
                    "flight regime from the reference: blue fixed-wing, purple "
                    "transition, and unshaded for multicopter or a regime the "
                    "log does not name. This log reads "
                    + ", ".join(f"<code>{html.escape(r)}</code>" for r in regimes)
                    + ".")
    resets_note = (" Blue dotted verticals are EKF2's own "
                   "<code>quat_reset_counter</code> changing &mdash; a step across "
                   "one of those is an event in their filter, not divergence from "
                   "this one.")

    blocks.append("<h2>States</h2>")
    for group in STATE_GROUPS:
        title, _unit, columns, _sigmas = group
        if columns is None:
            if ours_q is None:
                continue
            png, caption = figure(
                attitude_figure(attitude_series(ours_q, args.points),
                                attitude_series(reference_q, args.points),
                                attitude_series(truth_q, args.points), backdrop, resets),
                "Tilt, the angle between body down and navigation down, and "
                "heading, the rotation about navigation down that remains "
                "&mdash; the split <code>Validity</code> reports attitude in, "
                "derived from each file's quaternion (truth's from its Euler "
                "angles). Unlike roll and yaw both stay separate at 90&deg; of "
                "pitch; heading is left undrawn past 170&deg; of tilt, where the "
                "split is singular. No sigma band: the published sigmas are "
                "body-axis, and only near level do they pass for tilt and "
                "heading (#131). " + shading + resets_note,
                height=4.6)
            blocks.append(png_block(png, caption))
            continue
        if columns[0] not in epochs:
            continue
        png, caption = figure(
            state_figure(group, epochs, reference, truth, backdrop),
            f"{html.escape(title)}, with the filter's own +/-3 sigma band. "
            + shading,
            height=6.2)
        blocks.append(png_block(png, caption))

    if difference is not None:
        blocks.append("<h2>Attitude relative to EKF2</h2>")
        png, caption = figure(
            difference_figure(difference, epochs, backdrop, resets),
            "This filter's attitude relative to EKF2's, as the rotation vector "
            "of <code>q<sub>EKF2</sub><sup>-1</sup> q&#770;</code> in body axes, "
            "each EKF2 sample paired with the nearest epoch. The band is this "
            "filter's own +/-3 sigma, in the same body axes as the trace, so the "
            "comparison holds at any attitude. Body x and y are the tilt "
            "difference only while the headings agree: a heading difference "
            "lands partly on them, by the sine of the tilt, and the tilt panel "
            "above compares tilt directly. Heading does not compare at an "
            "instant, for reasons data/README.md, \"What --reference writes, "
            "and what it cannot\", sets out. "
            + shading + resets_note,
            height=6.2)
        blocks.append(png_block(png, caption))

    blocks.append("<h2>Covariance over time</h2>")
    floor = gap_threshold(gaps) if gaps else None
    outages = find_gaps(gaps, floor)
    png, caption = figure(
        sigma_figure(epochs, reference, outages, backdrop),
        "Every published standard deviation, one panel per state group, with "
        "EKF2's own dashed where the two are the same quantity &mdash; attitude "
        "excepted, whose diagonals are in different frames, so only its "
        "frame-invariant <code>sigma_att_total</code> is drawn. A sigma EKF2 "
        "reports as exactly zero is absent rather than floored: that is a state "
        "it is not estimating, not one it knows perfectly."
        + (f" Grey bands are the {len(outages)} GNSS outages longer than "
           f"{floor:.1f} s &mdash; five times this receiver's median fix "
           "interval, so an outage is judged against its own cadence rather than "
           "a fixed number of seconds &mdash; where the position and velocity "
           "sigmas should grow and then collapse on the fix that ends the gap."
           if floor else " This log carries no GNSS, so there is no outage to "
           "shade."))
    blocks.append(png_block(png, caption))

    blocks.append("<h2>Innovations</h2>")
    for source, entry in sorted(sources.items()):
        if not any(any(cell for cell in row) for row in entry["nu"]):
            continue
        png, caption = figure(
            innovation_figure(source, entry, gates.get(source), backdrop),
            f"Per-axis normalized innovation for <code>{html.escape(source)}</code>, "
            "&nu;<sub>i</sub>&nbsp;/&nbsp;&radic;S<sub>ii</sub>. Red verticals are "
            "gate rejections. Note this is <em>not</em> what the <code>nu_*</code> "
            "summary keys report: those average raw &nu; in the observation's own "
            "units, metres or radians, so the printed value and this cloud are "
            "different quantities and will not match.",
            height=2.0 + 1.5 * innovation_axes(entry))
        blocks.append(png_block(png, caption))

    if any(r in epochs or r in reference for r in RATIOS):
        blocks.append("<h2>Test ratios</h2>")
        png, caption = figure(
            ratio_figure(epochs, reference),
            "Gate test ratios, this filter against EKF2's aggregate ones. Both "
            "publish <code>r = &epsilon; / &gamma;</code>, so the red line at 1 "
            "is each filter's own gate and the two compare without knowing "
            "either's thresholds or dimensions &mdash; the like-for-like check "
            "GOALS.md differentiator 6 names. They are not the same statistic "
            "underneath: EKF2's are aggregates over its own aiding, and its "
            "height ratio includes a barometer this filter may not be fusing.",
            height=8.0)
        blocks.append(png_block(png, caption))

    blocks.append("<h2>Innovation distribution</h2>")
    png, caption = figure(
        nis_figure(sources, gates, keys),
        "NIS recovered as <code>&epsilon; = ratio &times; gamma</code>, against "
        "the chi-square it would follow if the filter's own covariance were "
        "right. The pinned <code>nis_</code> key in each title is the same "
        "quantity normalized differently &mdash; the mean of &epsilon; over its "
        "degrees of freedom, so that 1 is honest at any dimension &mdash; and is "
        "read from the summary, not recomputed here. "
        "Two caveats, both measured rather than supposed. <b>No source in this "
        "corpus is white</b> -- <code>acf1_</code> runs 0.6018 to 0.9890 on GNSS "
        "position, 0.1108 to 0.8269 on the barometer and 0.1779 to 0.9855 on the "
        "magnetometer -- because a 1 Hz receiver filters its own solution in time "
        "and a 250 Hz magnetometer is sampled far faster than the field it reads "
        "changes. A departure from the curve is that, not necessarily a filter "
        "fault. And <b>R for the barometer and the magnetometer is a converter "
        "constant</b>, so their distribution tests those constants; only GNSS "
        "tests a receiver's own reported accuracy, unfloored.",
        height=7.0)
    blocks.append(png_block(png, caption))

    if args.reference:
        values = agreement_of(ours_table, args.reference, sources)
        line = dict(token.split("=", 1) for token in agreement_line(values, keys).split())
        blocks.append("<h2>Agreement with EKF2</h2>")
        blocks.append(agreement_tables([(keys.get("r_policy", "?"), line)]))
        blocks.append(f'<p class="caption">{AGREEMENT_CAPTION}</p>')

    if keys:
        blocks.append("<h2>Published keys</h2>")
        blocks.append(key_table(keys))
        blocks.append(
            '<p class="caption">Read from the captured <code>summary</code> and '
            "<code>score</code> lines, never recomputed here. The harness is the "
            "only thing in the repository that computes a statistic.</p>")

    meta = f"{html.escape(Path(args.replay).name)} &middot; {epoch_rows} epochs"
    if keys.get("status"):
        meta += f" &middot; final status {html.escape(keys['status'])}"
    return PAGE.format(
        title=html.escape(args.title or Path(args.input).stem),
        meta=meta,
        body="\n".join(blocks),
    )


def fusion_path(replay):
    """`<out>.fusion.csv`, the way the harness names it beside `<out>`."""
    path = Path(replay)
    return path.with_suffix(".fusion.csv")


def self_test():
    """Literal fixtures for the attitude pictures, at the attitudes that break Euler.

    No corpus log tilts past 7 deg, where tilt and heading read as roll/pitch and
    yaw whatever the convention, so a quaternion read scalar-last would pass on
    every one of them. These pin the convention at 90 deg of pitch.
    """
    failures = []

    def near(what, got, want, tolerance=1e-9):
        got, want = np.atleast_1d(got), np.atleast_1d(want)
        if got.shape != want.shape:
            failures.append(f"{what}: got {got}, want {want}")
            return
        same = np.isnan(got) == np.isnan(want)
        close = np.isnan(want) | (np.abs(np.nan_to_num(got) - np.nan_to_num(want))
                                  <= tolerance)
        if not (same.all() and close.all()):
            failures.append(f"{what}: got {got}, want {want}")

    failures.extend(agreement.self_test())

    t, y = decimate(np.array([0.0, 1.0, 2.0]), {"h": np.array([1.0, np.nan, 3.0])},
                    10)["h"]
    near("decimate skips NaN", np.unique(y), (1.0, 3.0))
    # The envelope keeps each bucket's extremes, in time order: a spike survives
    # a stride that would step over it, and the second bucket's maximum comes
    # before its minimum.
    t, y = envelope(np.arange(6.0), np.array([0.0, 9.0, 1.0, 5.0, 2.0, -4.0]), 3)
    near("envelope values", y, (0.0, 9.0, 5.0, -4.0))
    near("envelope times", t, (0.0, 1.0, 3.0, 5.0))

    # A mode row written before the replay's first epoch is clipped to it, and
    # the last regime runs to the end of the replay.
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as handle:
        handle.write("t_s,source,mode\n-0.2,vehicle_mode,fw\n1.0,ekf2_att,\n"
                     "5.0,vehicle_mode,to_mc\n")
    runs = read_runs(handle.name, "mode", source="vehicle_mode", since=0.5, until=9.0)
    Path(handle.name).unlink()
    if runs != [[0.5, 5.0, "fw"], [5.0, 9.0, "to_mc"]]:
        failures.append(f"read_runs since/until: got {runs}")

    # The reference header's lines, as tools/ulog2replay.py's own self-test writes
    # them. A reworded line must fail here rather than read as "no origin".
    def same(what, got, want):
        if got != want:
            failures.append(f"{what}: got {got!r}, want {want!r}")

    same("offset", reference_offset(["EKF2 origin in replay frame: 110.574 111.320 -9.998 m"]),
         (110.574, 111.32, -9.998))
    down = reference_offset(["EKF2 origin in replay frame: 0.000 0.000 none m"])
    if down is None or down[:2] != (0.0, 0.0) or not math.isnan(down[2]):
        failures.append(f"offset with no MSL height: got {down!r}")
    same("no offset", reference_offset(["EKF2 origin in replay frame: none (EKF2 reports "
                                        "no origin)"]), None)
    same("height reference", reference_height([
        "EKF2 aiding: height reference baro (EKF2_HGT_MODE 0); share of control_mode_flags "
        "samples: gnss_pos 1.00"]), "baro")
    same("no height reference", reference_height([
        "EKF2 aiding: height reference unknown (no EKF2_HGT_REF or EKF2_HGT_MODE)"]), None)
    same("estimator", reference_estimator(["Estimator: lpe (SYS_MC_EST_GROUP 1)"]), "lpe")
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as handle:
        handle.write("# note\nt_s,source,a,b\n0.5,k,1,\n1.5,other,9,9\n2.5,k,,4\n")
    table = read_table(handle.name, ["a", "b", "missing"], source="k")
    Path(handle.name).unlink()
    # A blank is NaN and its row stays; a column the file lacks is absent.
    if (list(table[0]) != [0.5, 2.5] or sorted(table[1]) != ["a", "b"]
            or not (table[1]["a"][0] == 1.0 and math.isnan(table[1]["a"][1]))):
        failures.append(f"read_table: got {table!r}")
    same("agreement line", agreement_line(
        {"pos_n_rms": 0.31, "climb": None},
        {"r_policy": "px4", "rate": "250", "nis_mag": "0.1", "nis_over95_mag": "0.0",
         "resets": "1"}),
        "r_policy=px4 pos_n_rms=0.3100 climb=none nis_mag=0.1 resets=1")

    for failure in failures:
        print(f"FAIL {failure}", file=sys.stderr)
    print(f"replay_report self-test: {'FAIL' if failures else 'ok'}", file=sys.stderr)
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input", type=Path, nargs="?", help="the replay input CSV")
    parser.add_argument("replay", type=Path, nargs="?", help="the per-epoch output CSV")
    parser.add_argument("truth", type=Path, nargs="?", help="truth CSV, when the "
                        "scenario has one")
    parser.add_argument("--reference", type=Path, help="EKF2's reference CSV from "
                        "`ulog2replay.py --reference`")
    parser.add_argument("--summary", type=Path, help="captured replay stdout, for "
                        "the summary and score keys")
    parser.add_argument("-o", "--out", type=Path, help="output HTML")
    parser.add_argument("--points", type=int, default=4000, help="approximate "
                        "points per trace after decimation (default: 4000)")
    parser.add_argument("--title", help="report title (default: the input's stem)")
    parser.add_argument("--corpus", type=Path, help="a directory `data/fetch.sh "
                        "--compare` wrote: one agreement table for every run in it, "
                        "and one `agreement` line per run on stdout")
    parser.add_argument("--self-test", action="store_true",
                        help="run the attitude fixtures no corpus log can check, and exit")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.corpus:
        if args.out is None:
            parser.error("--corpus needs -o/--out")
        try:
            page, lines = build_corpus(args.corpus)
        except ReportError as e:
            print(f"replay_report: {e}", file=sys.stderr)
            return 1
        for line in lines:
            print(line)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(page)
        print(f"{args.out} ({len(page) / 1e6:.2f} MB)", file=sys.stderr)
        return 0
    if args.input is None or args.replay is None or args.out is None:
        parser.error("input, replay and -o/--out are required")

    try:
        page = build_report(args)
    except ReportError as e:
        print(f"replay_report: {e}", file=sys.stderr)
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(page)
    print(f"{args.out} ({len(page) / 1e6:.1f} MB)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
