# Honesty of the reported uncertainty

**When the filter says "I'm within a metre", is it?** Yes, with room to spare, on every
simulated scenario except `gnss_latency` and `correlated`. On the baseline flight, the fraction of checks where the
true value sat inside the filter's ±3σ band was {{score mission in3s}}, where 1 is every check.
In those two scenarios the filter claims more accuracy than it has: when GNSS fixes arrive late, and when
sensor errors persist longer than it assumes. This page shows both.

{{stamp}}

## Why this is its own question

Besides each estimate, the filter reports how uncertain it is, and an application acts on that
report. The filter's `Validity` flags, which tell an application whether a position or a heading
is usable, are read from it. A filter can be accurate and still report the wrong uncertainty,
and if it reports too little, a position gets used when it should not be. The
[accuracy page](accuracy.md) cannot see this: a filter that became more accurate and more
overconfident at the same time would pass it.

## How it is measured

Two tests, both against the simulator's known truth:

- **Within 3σ** (`in3s`): at every moment and on every quantity the filter estimates, is the
  real error inside three standard deviations of the reported uncertainty? The figure is the
  fraction of those checks that pass. An honest filter passes about 99.7 % of them.
- **NEES** (normalized estimation error squared): the size of the real error measured in units
  of the reported uncertainty, for position, velocity and attitude. It reads 1 on average when
  the reported uncertainty is exactly right, below 1 when the filter is pessimistic, and above 1
  when it is overconfident.

One flight can be lucky or unlucky, so the deciding test flies every scenario
{{anees mission runs}} times with different random noise (seeds 1 to {{anees mission runs}}) and
averages NEES across those flights at every moment. That average is **ANEES**. CI fails if it
crosses a statistical bound that an honest filter stays under 95 % of the time
(`data/anees.sh`).

## One flight each

The same flights the [accuracy page](accuracy.md) scores.

{{table score @scenarios in3s,nees_pos,nees_vel,nees_att,false_valid}}

Columns: `in3s` is the fraction of checks inside ±3σ. `nees_pos`, `nees_vel` and `nees_att` are
NEES for position, velocity and attitude, averaged over the flight. `false_valid` counts moments
where the filter told the application a quantity was usable while its real error was worse than
the configured accuracy. It is the count that matters most to an integrator, because it is where
an application would act on a wrong answer.

## Over {{anees mission runs}} flights each

{{table anees @anees anees_pos,anees_vel,anees_att,any_pos,any_vel,any_att}}

Columns: `anees_pos`, `anees_vel` and `anees_att` are ANEES averaged over the flight. `any_pos`,
`any_vel` and `any_att` count moments where ANEES crossed a strict bound
({{anees mission bound_any}} on the baseline), set so that an honest filter would cross it
anywhere in the flight at most 5 % of the time. A count above zero means the filter was
overconfident at those moments.

### Pessimistic, by design of the test

On the baseline, position ANEES is {{anees mission anees_pos}}, well under 1: the filter
believes its error is larger than it is. That is expected here. The simulated IMU is one to two
orders of magnitude quieter than the noise the filter is configured for, on purpose, so that the
filter is not scored against its own assumptions. For that reason CI fails only on
overconfidence, never on pessimism.

In these figures the solid line is ANEES at each moment, on a log scale. Below the thin line
at 1 the filter is pessimistic; above the dashed line it is overconfident at that moment; above
the upper solid line it is overconfident beyond chance.

{{figure mission anees}}

### Overconfident: fixes that arrive late

In `gnss_latency` every GNSS fix arrives 150 ms after the moment it describes, and the filter
uses it as if it were current. The resulting position error depends on how fast the vehicle is
moving, and the filter's model has no way to represent that, so it reports less uncertainty than
it has for most of the flight. Position ANEES is {{anees gnss_latency anees_pos}}, over the
strict bound at {{anees gnss_latency any_pos}} of {{anees gnss_latency epochs}} moments.
Whether the filter gains a model for delayed measurements, or documents the limit, is #52.

{{figure gnss_latency anees}}

### Overconfident: errors that persist

In `correlated`, each sensor's error changes slowly, like a real GNSS receiver's that drifts
over seconds, and more slowly than the filter assumes. The filter compensates for slowly
changing errors by treating each measurement as noisier than reported, by an amount set by an
assumed correlation time (`Config::correlation`, equation (24′)). Here the errors last longer
than assumed, so the compensation is too small. Position ANEES is
{{anees correlated anees_pos}}, over the strict bound at {{anees correlated any_pos}} moments.
Measuring each sensor's correlation time, instead of assuming a typical one, is #51.

{{figure correlated anees}}

## What this page cannot say

The real PX4 logs have no truth, so on them the reported uncertainty can only be checked
against the filter's own predictions: whether each new measurement falls as far from the
prediction as the filter expected (the `nis_` values in `data/manifest.txt`). That is a weaker
test, and it is not repeated here.
