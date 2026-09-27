# Honesty of the covariance

**When the filter says "I'm within a metre", is it?** On every simulated scenario but
`gnss_latency` and `correlated`, yes, and by a wide margin: on the baseline the share of axis-epochs where the truth sits inside
the filter's ±3σ is {{score mission in3s}}. It is overconfident on those two, fixes that arrive late
(`gnss_latency`) and aiding errors that persist longer than it assumes (`correlated`), and this
page shows both.

{{stamp}}

## Why this is its own question

An estimate can be accurate and still lie about its accuracy, and an integrator acts on the lie:
`Validity` reads the covariance, so a covariance too small reports a position as usable when it
is not. The [accuracy page](accuracy.md) cannot see this. Its ceilings pass a filter that grew
more accurate and more overconfident at once.

Two measures, both against truth from the simulator:

- **Within 3σ** (`in3s`): the fraction of axis-epochs, over all 15 states, where the error on an axis
  is inside three standard deviations of that axis. Axis by axis, so it reads the diagonal only.
- **NEES**: `δxᵀ P⁻¹ δx` for the position, velocity and attitude blocks, divided by the three
  degrees of freedom. It reads the correlations as well. A covariance that describes its error
  exactly averages 1; below 1 it is pessimistic, above it overconfident.

One flight can be unlucky, so the test that decides is over **{{anees mission runs}} flights**
of each scenario, on seeds 1 to {{anees mission runs}}: `data/anees.sh` averages NEES across them at every epoch (ANEES) and gates it in
CI against the chi-square bound an honest filter stays under 95 % of the time.

## One flight each

The pinned run of each scenario, the one the [accuracy page](accuracy.md) tabulates.

{{table score @scenarios in3s,nees_pos,nees_vel,nees_att,false_valid}}

`false_valid` counts quantity-epochs where `Validity` said a quantity was usable and the truth
error was outside `Config::accuracy`: the lie, counted where an integrator would act on it.

## Over the ensemble

`anees_` is the mean over epochs of the ensemble average, per degree of freedom. `any_` counts
epochs where the ensemble average crosses the bound made family-wise over the whole log (on the
baseline, {{anees mission bound_any}}), which an honest filter crosses anywhere with probability
at most 5 %.

{{table anees @anees anees_pos,anees_vel,anees_att,any_pos,any_vel,any_att}}

### Pessimistic, by design of the test

On the baseline the ensemble reads {{anees mission anees_pos}} on position, well under 1: the
filter believes its error larger than it is. The simulator's IMU is one to two orders
quieter than `ImuNoise::default()`, on purpose, since a filter tuned to the simulator would be
scored against its own assumptions. So a pessimistic reading is expected here, and the gate is
one-sided: it fails on overconfidence only.

{{figure mission anees}}

### Overconfident: fixes that arrive late

`gnss_latency` fuses every GNSS fix 150 ms after the instant it describes, as if it were
current. The position error that leaves is correlated with velocity, which the filter's
measurement noise cannot represent, and its covariance says so wrongly for most of the flight:
ANEES {{anees gnss_latency anees_pos}} on position, past the family-wise bound on
{{anees gnss_latency any_pos}} of {{anees gnss_latency epochs}} epochs. The filter has no
measurement-delay model; whether it gets a state buffer or a documented assumption is #52.

{{figure gnss_latency anees}}

### Overconfident: errors slower than assumed

`correlated` makes every aiding error correlated in time, and slower than the per-source
correlation times in `Config::correlation`. Equation (24′) fuses a correlated error at an
inflated, equivalent white variance; with the wrong time constant the inflation is too small,
and position reads ANEES {{anees correlated anees_pos}}, past the family-wise bound on
{{anees correlated any_pos}} epochs. Measuring each sensor's own correlation time, rather than
assuming the corpus median, is #51.

{{figure correlated anees}}

## What this page cannot say

The real logs have no truth, so their covariance can only be checked against itself: how large
the innovations are against the variance the filter predicted for them (the `nis_` keys in
`data/manifest.txt`). That is self-consistency, a weaker claim than this page's, and it is not
repeated here.
