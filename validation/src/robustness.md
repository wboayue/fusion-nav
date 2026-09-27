# Robustness to faults

**What happens when a sensor lies or goes quiet?** In each simulated fault below, the error stays
inside the band the filter claims. During the fault, the filter either widens that band or turns
the bad measurement down, and `Status` reports that aiding stopped. The exception is GNSS fixes
arriving late. The filter has no model for that yet, and it pays in both accuracy and honesty.

{{stamp}}

Each scenario here is the baseline `mission` circuit with one fault injected, flown at the seed
`data/scenarios.txt` pins. Each figure is the truth error inside the filter's own ±3σ band. The
background shading is the filter's `Status`: amber while aligning, yellow `Degraded`, red
`DeadReckoning`. For the whole-flight numbers side by side, see the
[accuracy page](accuracy.md#every-scenario).

## GNSS outage

GNSS is gone from 60 s to 80 s. With no aiding, position is dead reckoning: the accelerometer
integrated twice. The error grows to {{score gnss_outage pos_h_max}} m at worst, against
{{score mission pos_h_max}} m on the baseline. The band grows faster than the error does, and
it collapses on the first fix after the gap, which is accepted rather than rejected: the
filter's reported uncertainty covered where it had drifted to. `Status` reads `Degraded`
through the gap, not `DeadReckoning`: the barometer and magnetometer are
still being accepted, and any accepted source counts as aiding. That position is being dead
reckoned all the same is what `Validity` reports per quantity, and whether `Status` should
weigh sources differently is #56.

{{figure gnss_outage error_position}}

## A magnetic disturbance

From 60 s to 70 s the magnetometer reads a heading 30° wrong, as it would beside a steel
structure or a power line. The heading gate turns down {{summary mag_disturbance rejected_mag}}
readings, and the heading error over the whole flight is {{score mag_disturbance yaw}}° RMS
against the baseline's {{score mission yaw}}°. While magnetometer readings are refused the
heading is held by the gyroscopes, and the band widens until readings are accepted again.

{{figure mag_disturbance error_attitude}}

## A logging dropout at speed

1.2 s of the log is lost from 60 s, mid-turn. The IMU step across the gap is too long to
integrate from one sample, so the filter coasts it on the estimated velocity by equation (22′)
and grows its covariance by the acceleration and rotation it could not see (`coasted=`
{{summary logging_dropout coasted}}). The first fix after the gap falls inside that grown
uncertainty and is accepted: the worst horizontal error is {{score logging_dropout pos_h_max}} m,
and nothing had to be recovered (`recovered=` {{summary logging_dropout recovered}}).

{{figure logging_dropout error_position}}

## A drifting barometer

The barometer's reference drifts 2 cm/s away from the one fixed at startup, for the whole
flight. The filter estimates that offset as it flies, equation (30′), and hands the low
frequencies of height to GNSS. So height is off by {{score baro_drift pos_v}} m RMS where the
baseline reads {{score mission pos_v}}, and the covariance stays honest about it
(`nees_pos` {{score baro_drift nees_pos}}). A filter that holds the reference constant does
far worse, both in accuracy and in honesty. Why the offset is estimated rather than held or
treated as a consider state, and what each choice measured, is in
[GOALS.md, "Barometric reference as an estimated offset"](../GOALS.md#barometric-reference-as-an-estimated-offset).

{{figure baro_drift error_position}}

## GNSS fixes 150 ms late

This is the fault the filter does not handle. It fuses each fix as if it described the present
instant, so the position error follows velocity, reaching {{score gnss_latency pos_h}} m RMS
against the baseline's {{score mission pos_h}}. `Validity` reported
{{score gnss_latency false_valid}} quantity-epochs as usable while the truth error was past
`Config::accuracy`. The [honesty page](honesty.md#overconfident-fixes-that-arrive-late) shows the
covariance failing its ensemble test here. Whether this gets a delayed-state buffer or a
documented assumption is #52.

{{figure gnss_latency error_position}}

## What this page cannot say

Every fault here is simulated, and each is one fault at a time on one flight. Real hostile
measurements, such as urban multipath and reflections, arrive with the UrbanNav corpus (#60),
and will be scored here against truth when they do. The real PX4 logs show faults too, a
logging dropout at 30 m/s among them, but without truth. They are on the
[EKF2 page](ekf2.md), where the only question they can answer is agreement.
