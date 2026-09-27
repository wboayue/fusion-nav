# Validation

**Should you trust this filter?** On simulated flights, where the true answer is known, it holds
horizontal position to {{score mission pos_h}} m RMS on the baseline flight. Its estimate of its
own error is usually larger than the real error, which is the safe direction. The exceptions,
where it claims more accuracy than it has, are named below. On real PX4 flights, where nothing
is known exactly, it mostly agrees with EKF2, the estimator PX4 flies, and each place the two
disagree is shown with its cause, or marked as not yet explained.

{{stamp}}

## How to read these pages

- **Error** is the true value minus the filter's estimate. **RMS** is its root mean square over
  a flight: a typical size, where the **max** is the single worst moment.
- **The band.** Besides its estimate, the filter reports how uncertain it is (its covariance).
  The figures draw that as a grey band of three standard deviations (±3σ) either side of zero.
  An error inside the band is one the filter admitted to; an error outside it is one the filter
  did not see coming.
- **Pessimistic** means the band is wider than the errors need, and **overconfident** means it
  is narrower. Pessimistic costs some usefulness. Overconfident is the dangerous one, because
  an application trusts a number it should not.
- **EKF2** is PX4's own estimator. It is not the truth; on real flights nothing is, so the
  [EKF2 page](validation/ekf2.md) measures agreement, never accuracy.

Four questions, a page each, because each needs different evidence
([GOALS.md, three questions, three kinds of source](GOALS.md#three-questions-three-kinds-of-source)).

## [Accuracy](validation/accuracy.md): how close is it, when the truth is known?

{{count @scenarios}} simulated flights, scored against their exact trajectories. On the baseline
flight: horizontal position {{score mission pos_h}} m RMS, height {{score mission pos_v}} m,
velocity {{score mission vel}} m/s, tilt {{score mission tilt}}° and heading
{{score mission yaw}}°.

{{figure mission error_position}}

## [Honesty](validation/honesty.md): when it says "within a metre", is it?

Tested over {{anees mission runs}} flights of each simulated scenario. The filter is pessimistic
everywhere except in these cases, each with an open issue:
- **GNSS fixes that arrive late** (#52). The filter has no model for measurement delay yet.
- **Sensor errors that persist longer than the filter assumes** (#51).

{{figure gnss_latency anees}}

## [Robustness](validation/robustness.md): what happens when a sensor fails?

A GNSS outage, a magnetic disturbance, a gap in the log and a drifting barometer. In each, the
error stays inside the band and the filter's status reports the problem. Late GNSS fixes are the
fault it does not handle yet.

{{figure gnss_outage error_position}}

## [Agreement with EKF2](validation/ekf2.md): on real flights, does it match PX4?

{{count @corpus/raw}} public PX4 flight logs, from quadrotors to fixed-wing and VTOL aircraft.
On the log with the most precise receiver (RTK), velocity agrees to
{{agreement 89a498ce/raw vel_n_rms}} m/s RMS north and tilt to
{{agreement 89a498ce/raw tilt_diff_rms}}°. The largest disagreement is a hand-launched aircraft
whose attitude this filter gets wrong at startup, a known gap (#59).

{{figure 89a498ce/raw track}}

## Where the numbers come from

Nothing on these pages is typed by hand. Every number is copied from the output of a tool in
this repository, and every figure is drawn from the run it describes, named by scenario and
seed or by log. `tools/validation.sh` regenerates all of it, and
`tools/validation.sh --check` fails if any page differs from what a fresh run produces.
