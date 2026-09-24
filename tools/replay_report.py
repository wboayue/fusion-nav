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

# State groups plotted together, as (title, unit, columns, sigma columns). The
# reference's names match the epoch file's for everything except attitude, whose
# frames differ -- see reference_states in tools/ulog2replay.py.
STATE_GROUPS = [
    ("Position NED", "m", ["pos_n", "pos_e", "pos_d"],
     ["sigma_pos_n", "sigma_pos_e", "sigma_pos_d"]),
    ("Velocity NED", "m/s", ["vel_n", "vel_e", "vel_d"],
     ["sigma_vel_n", "sigma_vel_e", "sigma_vel_d"]),
    ("Attitude", "deg", ["roll", "pitch", "yaw"],
     ["sigma_att_x", "sigma_att_y", "sigma_att_z"]),
    ("Accelerometer bias", "m/s^2", ["ba_x", "ba_y", "ba_z"],
     ["sigma_ba_x", "sigma_ba_y", "sigma_ba_z"]),
    ("Gyroscope bias", "rad/s", ["bg_x", "bg_y", "bg_z"],
     ["sigma_bg_x", "sigma_bg_y", "sigma_bg_z"]),
]

DEGREES = {"roll", "pitch", "yaw"}


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
    total, kind = 0, None
    stream = read_header_and_rows(path)
    _, columns = next(stream)
    if source is not None and source_column in columns:
        kind = columns.index(source_column)
    for _, cells in stream:
        if kind is None or cells[kind] == source:
            total += 1
    return total


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


def read_series(path, wanted, points, source=None, source_column="source"):
    """Decimated `{name: (t, values)}` for `wanted`, streaming the file once.

    `source` restricts to one row kind, which is how the union-schema input and
    reference files are read without holding a blank cell per column per row.
    """
    rows = count_rows(path, source, source_column)
    stream = read_header_and_rows(path)
    notes, columns = next(stream)
    index = {name: columns.index(name) for name in wanted if name in columns}
    if not index:
        return notes, {}
    kind = columns.index(source_column) if source_column in columns else None
    decimator = Decimator(index, rows, points)
    for _, cells in stream:
        if kind is not None and source is not None and cells[kind] != source:
            continue
        values = {}
        for name, position in index.items():
            cell = cells[position]
            if cell:
                values[name] = float(cell)
        if values:
            decimator.add(float(cells[0]), values)
    return notes, decimator.done()


def read_times(path, source, source_column="source"):
    """Every timestamp for one row kind, undecimated.

    Gaps have to be found before decimation: a decimated series cannot tell an
    outage from its own stride, and reading one column of a 1 Hz source costs a
    few thousand floats even on the 2 h log.
    """
    stream = read_header_and_rows(path)
    _, columns = next(stream)
    kind = columns.index(source_column) if source_column in columns else None
    return [
        float(cells[0])
        for _, cells in stream
        if kind is None or cells[kind] == source
    ]


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
    stream = read_header_and_rows(path)
    _, columns = next(stream)
    if x_name not in columns or y_name not in columns:
        return None
    xi, yi = columns.index(x_name), columns.index(y_name)
    kind = columns.index(source_column) if source_column in columns else None
    x, y, index = [], [], 0
    for _, cells in stream:
        if kind is not None and source is not None and cells[kind] != source:
            continue
        if index % stride == 0 and cells[xi] and cells[yi]:
            x.append(float(cells[xi]))
            y.append(float(cells[yi]))
        index += 1
    return (np.array(x), np.array(y)) if x else None


def read_status(path, points):
    """Contiguous `(start, end, status)` runs, for shading a time axis."""
    stream = read_header_and_rows(path)
    _, columns = next(stream)
    if "status" not in columns:
        return []
    position = columns.index("status")
    runs, current = [], None
    for _, cells in stream:
        t, status = float(cells[0]), cells[position]
        if current is None or current[2] != status:
            if current is not None:
                current[1] = t
            current = [t, t, status]
            runs.append(current)
        else:
            current[1] = t
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
        # The converter rounds EKF2's update period up to a whole IMU sample, so
        # the interval it used has to be the one the harness measured. These are
        # two estimators of one quantity and nothing else would notice them
        # drifting apart -- it would move a published bias figure silently.
        used = reference_interval(reference_notes)
        if used and abs(used - 1.0 / rate) > 0.1 / rate:
            problems.append(
                f"the reference scaled biases by a {used * 1e3:.3f} ms IMU "
                f"interval but the summary reports rate={rate:g} "
                f"({1e3 / rate:.3f} ms)"
            )

    if problems:
        raise ReportError(
            "these files do not describe the same run:\n  - " + "\n  - ".join(problems)
        )


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


def shade_status(axes, runs):
    for start, end, status in runs:
        shade = STATUS_SHADES.get(status)
        if shade and end > start:
            axes.axvspan(start, end, color=shade, alpha=0.18, linewidth=0)


def gap_threshold(times, multiple=5.0):
    """How long a silence has to be, for this source, to count as an outage.

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


def state_figure(group, epochs, reference, truth, status_runs):
    title, unit, columns, sigmas = group

    def build(fig):
        axes = fig.subplots(len(columns), 1, sharex=True)
        for row, (column, sigma) in enumerate(zip(columns, sigmas)):
            plot = axes[row]
            shade_status(plot, status_runs)
            scale = 180.0 / math.pi if column in DEGREES else 1.0
            ours = epochs.get(column)
            if ours is None:
                continue
            t, y = ours[0], ours[1] * scale
            band = epochs.get(sigma)
            if band is not None and len(band[1]) == len(y):
                spread = 3.0 * band[1] * scale
                plot.fill_between(t, y - spread, y + spread, color="#1b1b1b",
                                  alpha=0.12, linewidth=0, label="+/-3 sigma")
            plot.plot(t, y, "-", color="#1b1b1b", linewidth=0.9, label="fusion-nav")
            for other, colour, label in ((reference, "#3a7ca5", "EKF2"),
                                         (truth, "#c05a2f", "truth")):
                series = (other or {}).get(column)
                if series is not None:
                    plot.plot(series[0], series[1] * scale, "-", color=colour,
                              linewidth=0.9, alpha=0.85, label=label)
            plot.set_ylabel(f"{column} ({unit})", fontsize=8)
            plot.grid(alpha=0.25)
            if row == 0:
                plot.legend(loc="upper right", fontsize=7, ncol=4)
        axes[-1].set_xlabel("t (s)")
    return build


def sigma_figure(epochs, gaps, status_runs):
    def build(fig):
        axes = fig.add_subplot(111)
        shade_status(axes, status_runs)
        shade_gaps(axes, gaps)
        for title, _unit, _columns, sigmas in STATE_GROUPS:
            for sigma in sigmas:
                series = epochs.get(sigma)
                if series is None:
                    continue
                axes.plot(series[0], np.maximum(series[1], 1e-12), linewidth=0.8,
                          label=sigma, alpha=0.85)
        axes.set_yscale("log")
        axes.set_xlabel("t (s)")
        axes.set_ylabel("sigma (SI, log scale)")
        axes.grid(alpha=0.25, which="both")
        axes.legend(loc="upper right", fontsize=6, ncol=3)
    return build


def innovation_figure(source, entry, gate, status_runs):
    axes_count = max(
        (sum(1 for cell in row if cell) for row in entry["nu"]), default=0
    )

    def build(fig):
        plots = fig.subplots(max(1, axes_count), 1, sharex=True, squeeze=False)
        for axis in range(max(1, axes_count)):
            plot = plots[axis][0]
            shade_status(plot, status_runs)
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


# -------------------------------------------------------------------- main


def build_report(args):
    epoch_rows = count_rows(args.replay)
    status_runs = read_status(args.replay, args.points)

    epoch_columns = ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d",
                     "roll", "pitch", "yaw", "ba_x", "ba_y", "ba_z",
                     "bg_x", "bg_y", "bg_z"]
    for _title, _unit, _columns, sigmas in STATE_GROUPS:
        epoch_columns.extend(sigmas)
    _, epochs = read_series(args.replay, epoch_columns, args.points)

    gates, sources = read_fusion(fusion_path(args.replay))

    reference, reference_notes = {}, []
    if args.reference:
        reference_notes, local = read_series(
            args.reference, ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d"],
            args.points, source="ekf2_local")
        _, attitude = read_series(args.reference, ["roll", "pitch", "yaw"],
                                  args.points, source="ekf2_att")
        _, states = read_series(
            args.reference,
            ["ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z"],
            args.points, source="ekf2_states")
        reference = {**local, **attitude, **states}

    truth = {}
    if args.truth:
        _, truth = read_series(args.truth, epoch_columns, args.points)

    keys = read_summary(args.summary) if args.summary else {}
    if keys:
        check_provenance(keys, epoch_rows, sources, reference_notes)

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

    blocks.append("<h2>States</h2>")
    for group in STATE_GROUPS:
        title = group[0]
        if group[2][0] not in epochs:
            continue
        png, caption = figure(
            state_figure(group, epochs, reference, truth, status_runs),
            f"{html.escape(title)}, with the filter's own +/-3 sigma band. "
            "Background shading is <code>Status</code>: amber Aligning, yellow "
            "Degraded, red DeadReckoning."
            + (" EKF2's attitude is plotted from the same quaternion convention; "
               "its sigma is in NED where this filter's is body-frame, so the "
               "band is this filter's only."
               if title == "Attitude" else ""),
            height=6.2)
        blocks.append(png_block(png, caption))

    blocks.append("<h2>Covariance over time</h2>")
    floor = gap_threshold(gaps) if gaps else None
    outages = find_gaps(gaps, floor)
    png, caption = figure(
        sigma_figure(epochs, outages, status_runs),
        "Every published standard deviation on one log axis."
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
            innovation_figure(source, entry, gates.get(source), status_runs),
            f"Per-axis normalized innovation for <code>{html.escape(source)}</code>. "
            "Red verticals are gate rejections. This is the per-axis quantity the "
            "<code>nu_*</code> summary keys average, not the aggregate NIS below.",
            height=2.0 + 1.5 * max(1, len(entry['nu'][0])))
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
        "corpus is white</b> -- <code>acf1_</code> runs 0.5736 to 0.9885 on GNSS "
        "position, 0.1712 to 0.8511 on the barometer and 0.1793 to 0.9851 on the "
        "magnetometer -- because a 1 Hz receiver filters its own solution in time "
        "and a 250 Hz magnetometer is sampled far faster than the field it reads "
        "changes. A departure from the curve is that, not necessarily a filter "
        "fault. And <b>R for the barometer and the magnetometer is a converter "
        "constant</b>, so their distribution tests those constants; only GNSS "
        "tests a receiver's own reported accuracy, unfloored.",
        height=7.0)
    blocks.append(png_block(png, caption))

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


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input", type=Path, help="the replay input CSV")
    parser.add_argument("replay", type=Path, help="the per-epoch output CSV")
    parser.add_argument("truth", type=Path, nargs="?", help="truth CSV, when the "
                        "scenario has one")
    parser.add_argument("--reference", type=Path, help="EKF2's reference CSV from "
                        "`ulog2replay.py --reference`")
    parser.add_argument("--summary", type=Path, help="captured replay stdout, for "
                        "the summary and score keys")
    parser.add_argument("-o", "--out", type=Path, required=True, help="output HTML")
    parser.add_argument("--points", type=int, default=4000, help="approximate "
                        "points per trace after decimation (default: 4000)")
    parser.add_argument("--title", help="report title (default: the input's stem)")
    args = parser.parse_args()

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
