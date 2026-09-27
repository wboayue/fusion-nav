# Validation

**Should you trust this filter, and where can you check?** On simulated flights, where the true
answer is known, it holds horizontal position to {{score mission pos_h}} m RMS, and its
covariance errs on the pessimistic side, except in the two cases the pages below name. On real
PX4 flights, where nothing is known exactly, it agrees with EKF2 wherever both filters are fed
the same trust in the receiver, and each disagreement is shown with its cause, or marked as open.

{{stamp}}

Every number on these pages is copied off a line a tool in this repository printed, and every
figure is drawn from the run it describes. `tools/validation.sh` regenerates all of it, and
`tools/validation.sh --check` fails if any page is not what a fresh run renders. Nothing is
typed by hand, and each figure names the scenario and seed, or the log, it came from.

The questions need different evidence, so they get a page each (`GOALS.md`,
[three questions, three kinds of source](GOALS.md#three-questions-three-kinds-of-source)).

## [Accuracy](validation/accuracy.md): how close, when the truth is known

Eleven simulated scenarios scored against analytic truth. On the baseline circuit: position
{{score mission pos_h}} m RMS horizontal and {{score mission pos_v}} m vertical, velocity
{{score mission vel}} m/s, tilt {{score mission tilt}}° and heading {{score mission yaw}}°.

{{figure mission error_position}}

## [Honesty](validation/honesty.md): when it says "within a metre", is it

The covariance tested against truth over 50 flights per scenario. It is pessimistic everywhere
except two scenarios, where it is overconfident: GNSS fixes arriving late (position ANEES
{{anees gnss_latency anees_pos}}, #52) and aiding errors slower than it assumes
({{anees correlated anees_pos}}, #51).

{{figure gnss_latency anees}}

## [Robustness](validation/robustness.md): when a sensor lies or drops out

A GNSS outage, a magnetic disturbance, a logging dropout and a drifting barometer. Each stays
inside the band the filter claims, with `Status` saying so. Late GNSS fixes are the fault it
does not yet handle.

{{figure gnss_outage error_position}}

## [Agreement with EKF2](validation/ekf2.md): on real flights, beside what PX4 flies

Twelve PX4 logs, each replayed twice: once with the receiver's reported noise, and once at
EKF2's own floors. EKF2 is not truth, so this page measures agreement, not accuracy. On the RTK
baseline, velocity agrees to {{agreement 89a498ce/raw vel_n_rms}} m/s RMS north and tilt to
{{agreement 89a498ce/raw tilt_diff_rms}}°. The largest disagreement is a hand launch this
filter levels wrong, a loss with a known cause (#59).

{{figure 89a498ce/raw track}}
