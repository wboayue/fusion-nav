#!/usr/bin/env python3
"""Average per-epoch NEES across an ensemble of seeds and test it against the chi-square bound.

    tools/anees.py run1.nees.csv run2.nees.csv ...    print one `anees` line
    tools/anees.py --series out.csv run1.nees.csv ... also write eps_bar_k / 3 per epoch
    tools/anees.py --self-test                         run the fixtures

The input is `examples/replay.rs`'s `<out>.nees.csv`: `eps = dx' P^-1 dx` per 3-state block,
one row per epoch, one file per seed of one scenario. The harness computes `eps`; this only
averages it (AGENTS.md, "one statistic, one implementation").

At each epoch k the ensemble mean over N runs is `eps_bar_k = (1/N) sum_i eps_ik`. If the
covariance is honest and the runs independent, `N eps_bar_k` is chi-square with `3N` degrees of
freedom, so the one-sided 95 % bound on `eps_bar_k` is `chi2_{3N}(0.95) / N`. That holds per
epoch and not for a time average: consecutive epochs of one run share their error, so averaging
over a flight first would put one run's correlated samples where the bound assumes independent
ones, and dilute a 7 s lockout into 185 s besides (#89).

Keys, per block (`pos`, `vel`, `att`), all per degree of freedom so 1 is consistent:

    anees_     mean of eps_bar_k / 3 over the epochs
    over_      fraction of epochs whose eps_bar_k exceeds `bound`; 0.05 for an honest filter
    any_       epochs whose eps_bar_k exceeds `bound_any`; 0 for an honest filter, 95 % of the time
    peak_      largest eps_bar_k / 3, the worst epoch the ensemble agreed on
    peak_*_at  its timestamp, which is where to look

`bound=` is the per-epoch one-sided 95 % bound, per degree of freedom. `bound_any=` is the same
quantile at 1 - 0.05/K over K epochs: Bonferroni, so the chance an honest filter crosses it at
*any* epoch is at most 5 % whatever the correlation between epochs, which is what lets one epoch
count as evidence. The two answer different faults. A short overconfident stretch, a lockout or a
transient after an adoption, is a sliver of `over_` and a peak far past `bound_any`; a mild
overconfidence that never lets up is the reverse.

Standard library only, and run by `python3` directly rather than `uv run`: it reads the harness's
own output and nothing else, so it has no dependency to pin and no converter's reason to stay off
the CI path (GOALS.md, "Harness constraint"). The chi-square quantile is computed exactly, from
the regularized incomplete gamma function, rather than by the Wilson-Hilferty approximation.
"""

import math
import re
import sys

BLOCKS = ("pos", "vel", "att")
DOF = 3
PERCENTILE = 0.95
HEADER = re.compile(r"^# fusion-nav nees for `([^`]+)`, seed (\d+)$")


def gamma_p(a, x):
    """Regularized lower incomplete gamma P(a, x): series below a + 1, continued fraction above."""
    if x <= 0.0:
        return 0.0
    log_front = a * math.log(x) - x - math.lgamma(a)
    if x < a + 1.0:
        term = total = 1.0 / a
        n = a
        while abs(term) > abs(total) * 1e-15:
            n += 1.0
            term *= x / n
            total += term
        return total * math.exp(log_front)
    # Lentz's method on the continued fraction for Q(a, x).
    tiny = 1e-300
    b = x + 1.0 - a
    c = 1.0 / tiny
    d = 1.0 / b
    h = d
    i = 1
    while True:
        an = -i * (i - a)
        b += 2.0
        d = an * d + b
        d = tiny if abs(d) < tiny else d
        c = b + an / c
        c = tiny if abs(c) < tiny else c
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < 1e-15:
            break
        i += 1
    return 1.0 - h * math.exp(log_front)


def chi2_quantile(p, k):
    """The p quantile of chi-square with k degrees of freedom, by bisection on the CDF."""
    lo, hi = 0.0, k + 20.0 * math.sqrt(2.0 * k) + 20.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if gamma_p(k / 2.0, mid / 2.0) < p:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def bound(runs, epochs=1):
    """The one-sided bound on eps_bar_k per degree of freedom, family-wise over `epochs`."""
    return chi2_quantile(1.0 - (1.0 - PERCENTILE) / epochs, DOF * runs) / runs / DOF


def read(path):
    """(scenario, seed, [t], {block: [eps]}) from one .nees.csv, or an error naming the file."""
    with open(path) as f:
        lines = f.read().splitlines()
    marker = HEADER.match(lines[0]) if lines else None
    if not marker:
        raise ValueError(f"{path}: no `# fusion-nav nees for` header; not a scenario's run")
    scenario, seed = marker.group(1), int(marker.group(2))
    rows = [line for line in lines if line and not line.startswith("#")]
    if not rows or rows[0] != "t_s,nees_pos,nees_vel,nees_att":
        raise ValueError(f"{path}: columns are not t_s,nees_pos,nees_vel,nees_att")
    times, eps = [], {block: [] for block in BLOCKS}
    for n, row in enumerate(rows[1:], start=2):
        fields = row.split(",")
        # An empty field is a singular block or an epoch with no truth row. Either is a
        # finding, and averaging around it would publish a figure that left it out.
        if len(fields) != 4 or "" in fields:
            raise ValueError(f"{path}: row {n} has no eps for every block: `{row}`")
        times.append(fields[0])
        for block, value in zip(BLOCKS, fields[1:]):
            eps[block].append(float(value))
    return scenario, seed, times, eps


def epoch_means(runs, block):
    """eps_bar_k / 3 for one block at every epoch: the series every key below is read from."""
    n = len(runs)
    return [sum(run[3][block][k] for run in runs) / n / DOF for k in range(len(runs[0][2]))]


def same_ensemble(runs):
    """The scenario a list of read() results is an ensemble of, or ValueError saying why not."""
    scenarios = {run[0] for run in runs}
    if len(scenarios) != 1:
        raise ValueError(f"runs from more than one scenario: {', '.join(sorted(scenarios))}")
    seeds = [run[1] for run in runs]
    if len(set(seeds)) != len(seeds):
        raise ValueError("a seed appears twice; the runs are not independent")
    times = runs[0][2]
    if not times:
        raise ValueError(f"seed {runs[0][1]} has no epochs; the replay never scored")
    for run in runs[1:]:
        if run[2] != times:
            raise ValueError(f"seed {run[1]} has other epochs than seed {runs[0][1]}")
    return scenarios.pop()


def ensemble(runs):
    """The `anees` pairs for a list of read() results, refusing one that is not an ensemble."""
    scenario = same_ensemble(runs)
    times = runs[0][2]
    n = len(runs)
    limit = bound(n)
    limit_any = bound(n, len(times))
    pairs = [
        f"runs={n}",
        f"epochs={len(times)}",
        f"bound={limit:.4f}",
        f"bound_any={limit_any:.4f}",
    ]
    for block in BLOCKS:
        mean = epoch_means(runs, block)
        peak = max(range(len(mean)), key=mean.__getitem__)
        pairs.append(f"anees_{block}={sum(mean) / len(mean):.4f}")
        pairs.append(f"over_{block}={sum(1 for m in mean if m > limit) / len(mean):.4f}")
        pairs.append(f"any_{block}={sum(1 for m in mean if m > limit_any)}")
        pairs.append(f"peak_{block}={mean[peak]:.4f}")
        pairs.append(f"peak_{block}_at={times[peak]}")
    return scenario, pairs


def series(runs):
    """eps_bar_k / 3 per block at every epoch, as CSV text under a header naming the ensemble.

    The series every key of `ensemble` is read from, for a figure to draw against the two
    bounds rather than for a second statistic: `tools/replay_report.py` plots these columns
    and computes nothing from them.
    """
    scenario = same_ensemble(runs)
    times, n = runs[0][2], len(runs)
    means = [epoch_means(runs, block) for block in BLOCKS]
    lines = [
        f"# fusion-nav anees for `{scenario}`, runs {n}, "
        f"bound {bound(n):.4f}, bound_any {bound(n, len(times)):.4f}",
        "t_s," + ",".join(f"anees_{block}" for block in BLOCKS),
    ]
    for k, t in enumerate(times):
        lines.append(t + "," + ",".join(f"{mean[k]:.6f}" for mean in means))
    return "\n".join(lines) + "\n"


def self_test():
    failures = []

    def check(name, got, want):
        if got != want:
            failures.append(f"{name}: got {got!r}, want {want!r}")

    # Quantiles against published tables, to the digits the tables give.
    check("chi2(3) 0.95", f"{chi2_quantile(0.95, 3):.4f}", "7.8147")
    check("chi2(150) 0.95", f"{chi2_quantile(0.95, 150):.2f}", "179.58")
    check("chi2(1) 0.95", f"{chi2_quantile(0.95, 1):.4f}", "3.8415")
    check("bound at N=50", f"{bound(50):.4f}", "1.1972")

    def run(seed, pos, vel=None, att=None, scenario="fixture", times=None):
        k = len(pos)
        return (scenario, seed, times or [f"{i}.0000" for i in range(k)],
                {"pos": pos, "vel": vel or [3.0] * k, "att": att or [3.0] * k})

    # Two seeds, two epochs. pos: epoch means 1.0 and 3.0 per dof, so the second is over any
    # bound and the first under it. The bound at N=2 is chi2_6(0.95)/2/3 = 12.5916/6 = 2.0986.
    _, pairs = ensemble([run(1, [0.0, 6.0]), run(2, [6.0, 12.0])])
    got = dict(p.split("=") for p in pairs)
    check("anees_pos", got["anees_pos"], "2.0000")
    check("over_pos", got["over_pos"], "0.5000")
    check("peak_pos", got["peak_pos"], "3.0000")
    check("peak_pos_at", got["peak_pos_at"], "1.0000")
    # Family-wise over two epochs: chi2_6(0.975)/6 = 14.4494/6. Epoch 1's 3.0 is past it.
    check("bound_any N=2 K=2", got["bound_any"], "2.4082")
    check("any_pos", got["any_pos"], "1")
    check("any_vel", got["any_vel"], "0")
    check("over_vel", got["over_vel"], "0.0000")
    check("bound N=2", got["bound"], "2.0986")

    def refused(name, runs):
        try:
            ensemble(runs)
            failures.append(f"{name}: accepted")
        except ValueError:
            pass

    refused("two scenarios", [run(1, [1.0]), run(2, [1.0], scenario="other")])
    refused("a repeated seed", [run(1, [1.0]), run(1, [1.0])])
    refused("no epochs", [run(1, [])])
    refused("misaligned epochs", [run(1, [1.0]), run(2, [1.0], times=["0.0050"])])

    # The series is the per-epoch mean the keys above read, not a second computation: the same
    # two seeds give pos 1.0 then 3.0 per dof, and vel and att 1.0 throughout.
    text = series([run(1, [0.0, 6.0]), run(2, [6.0, 12.0])]).splitlines()
    check("series header", text[0],
          "# fusion-nav anees for `fixture`, runs 2, bound 2.0986, bound_any 2.4082")
    check("series columns", text[1], "t_s,anees_pos,anees_vel,anees_att")
    check("series epoch 0", text[2], "0.0000,1.000000,1.000000,1.000000")
    check("series epoch 1", text[3], "1.0000,3.000000,1.000000,1.000000")
    try:
        series([run(1, [1.0]), run(1, [1.0])])
        failures.append("series of a repeated seed: accepted")
    except ValueError:
        pass

    for failure in failures:
        print(f"anees self-test: {failure}", file=sys.stderr)
    if failures:
        sys.exit(1)
    print("anees self-test: ok")


def main(argv):
    if argv == ["--self-test"]:
        self_test()
        return
    out = None
    if argv[:1] == ["--series"]:
        if len(argv) < 2:
            sys.exit("anees: --series wants an output path")
        out, argv = argv[1], argv[2:]
    if not argv:
        sys.exit(__doc__.split("\n\n")[1])
    try:
        runs = [read(path) for path in argv]
        scenario, pairs = ensemble(runs)
        if out:
            with open(out, "w") as f:
                f.write(series(runs))
    except ValueError as e:
        sys.exit(f"anees: {e}")
    print(f"anees {scenario} " + " ".join(pairs))


if __name__ == "__main__":
    main(sys.argv[1:])
