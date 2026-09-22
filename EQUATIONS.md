# Equations

> **Status: the estimation mathematics is partly implemented.** (9)–(15) propagate the nominal
> state and (16)–(22) propagate `P`, so `predict` moves position, velocity and attitude and says
> how little it knows about them. GNSS corrects them: the update (23)–(27), the observation
> models for position (28) and velocity (29), the gate (37)–(38) and the injection and reset
> (39)–(41) are built, and so is the barometric altitude of (30). The remaining observation
> models, (31)–(36), are not, and `fuse_mag_heading` accepts with a zero test ratio.
> Also built: the attitude and biases of (5)–(7), the
> initial covariance (8) and the bound (8′) a coarse window earns, the barometric reference `α₀` of
> (30), the window's own acceleration
> `ā_n` of (5′) — measured and reported though nothing levels with it yet — the geodetic origin
> (43)–(44), the angle wrap of (35), and the symmetry enforcement of (42), called after every
> covariance operation; the [equation-to-code mapping](#equation-to-code-mapping) marks the functions
> that do not exist yet.

This document is the normative mathematical description of `fusion-nav`. Equations are numbered
so that the implementation can cite them directly; see
[Readable mathematics](GOALS.md#3-readable-mathematics) for why that matters.

See [DESIGN.md](DESIGN.md) for the architecture and [GOALS.md](GOALS.md) for positioning.

## Notation and conventions

### Frames

Frame labels appear only as subscripts on vectors: $`a_b`$ is a body-frame vector, $`a_n`$ a
navigation-frame vector.

| symbol | meaning |
| ------ | ------- |
| $`n`$ | navigation frame, North-East-Down |
| $`b`$ | body frame |

The navigation frame is down-positive, so gravity has a **positive** $`z`$ component.

The attitude error is defined as a **local** (body-frame) perturbation, $`q = \hat{q} \otimes \delta q`$.
Every Jacobian below follows from that choice. The global alternative
$`q = \delta q \otimes \hat{q}`$ does not flip signs — it moves $`R`$: Solà (295) and (311) against
his (238) and (270) give $`-[\,R a_b\,]_\times`$ for $`-R[\,a_b\,]_\times`$ in (17) and (20),
$`I`$ for $`R\{\omega\Delta t\}^\mathsf{T}`$ in the attitude block, and $`-R\Delta t`$ for
$`-I\Delta t`$ where the gyroscope bias enters. Solà numbers are v1's throughout; see
[Correspondence with Solà](#correspondence-with-solà).

### States and measurements

| symbol | meaning | units |
| ------ | ------- | ----- |
| $`p, v`$ | position and velocity, navigation frame | m, m s⁻¹ |
| $`q`$ | unit quaternion rotating body to navigation, Hamilton convention, scalar first | — |
| $`R(q)`$ | rotation matrix equivalent of $`q`$, body to navigation | — |
| $`\beta_a, \beta_g`$ | accelerometer and gyroscope bias, body frame | m s⁻², rad s⁻¹ |
| $`a_m, \omega_m`$ | raw accelerometer and gyroscope measurements, body frame | m s⁻², rad s⁻¹ |
| $`g`$ | gravity vector in the navigation frame | m s⁻² |
| $`m_n`$ | reference magnetic field, navigation frame | normalized |
| $`D_m`$ | magnetic declination | rad |
| $`\alpha`$ | barometric altitude above its own reference, positive **up** | m |

Biases are written $`\beta`$ rather than $`b`$ so that they never collide with the body-frame
subscript.

### Filter quantities

| symbol | meaning | dimension |
| ------ | ------- | --------- |
| $`\delta x`$ | error state | 15 |
| $`P`$ | error covariance | 15 × 15 |
| $`F`$ | discrete state transition matrix | 15 × 15 |
| $`Q`$ | discrete process noise | 15 × 15 |
| $`H`$ | measurement Jacobian, $`H = \left.\partial h / \partial \delta x\right\rvert_{\hat{x}}`$ | dim(z) × 15 |
| $`R_m`$ | measurement noise covariance | dim(z) × dim(z) |
| $`y`$ | innovation | dim(z) |
| $`S`$ | innovation covariance | dim(z) × dim(z) |
| $`K`$ | Kalman gain | 15 × dim(z) |
| $`G`$ | reset Jacobian | 15 × 15 |
| $`w_a, w_g`$ | accelerometer and gyroscope white noise | — |
| $`w_{\beta a}, w_{\beta g}`$ | bias random-walk driving noise | — |
| $`\sigma_a, \sigma_g, \sigma_{\beta a}, \sigma_{\beta g}`$ | the spectral densities of those four, as `ImuNoise` states them | per $`\sqrt{\mathrm{Hz}}`$ |

### Operators

| symbol | meaning |
| ------ | ------- |
| $`\hat{x}`$ | estimated (nominal) quantity |
| $`[\,u\,]_\times`$ | skew-symmetric matrix of $`u`$, so that $`[\,u\,]_\times v = u \times v`$ |
| $`\otimes`$ | quaternion product |
| $`\mathrm{Exp}(\phi)`$ | rotation vector to quaternion, $`\mathrm{Exp}(\phi) = [\cos\tfrac{\lVert\phi\rVert}{2},\ \tfrac{\phi}{\lVert\phi\rVert}\sin\tfrac{\lVert\phi\rVert}{2}]`$ |
| $`R\{\phi\}`$ | rotation **matrix** of the rotation vector $`\phi`$, equal to $`R(\mathrm{Exp}(\phi))`$ |
| $`\mathrm{wrap}(\cdot)`$ | angle wrapped to $`(-\pi, \pi]`$ |
| $`e_3`$ | $`[0, 0, 1]^\mathsf{T}`$ |

### Gravity

$`g = [0, 0, \gamma]^\mathsf{T}`$ with $`\gamma`$ the local gravity magnitude, entering the
propagation of (11). `config::GRAVITY` is the WGS-84 standard value 9.80665 m s⁻² and stays a
constant: $`\gamma`$ varies by roughly 0.5 % between the equator and the poles and the filter
holds a geodetic origin, but that origin is placed by the first fix, which can arrive after
propagation has begun. Deriving it there would change a propagation constant mid-flight. It is
derived offline instead, by the tool that prints a `Config` from a log (#51); see
[GOALS.md](GOALS.md#local-gravity-as-a-constant-derived-offline) for the decision.

## State definitions

The nominal state has 16 components:

**(1)**

```math
x = \begin{bmatrix} p & v & q & \beta_a & \beta_g \end{bmatrix}^\mathsf{T}
```

The error state has 15:

**(2)**

```math
\delta x = \begin{bmatrix} \delta p & \delta v & \delta\theta & \delta\beta_a & \delta\beta_g \end{bmatrix}^\mathsf{T} \in \mathbb{R}^{15}
```

True state as nominal composed with error:

**(3)**

```math
p = \hat{p} + \delta p, \quad v = \hat{v} + \delta v, \quad q = \hat{q} \otimes \delta q, \quad \beta_a = \hat{\beta}_a + \delta\beta_a, \quad \beta_g = \hat{\beta}_g + \delta\beta_g
```

with the small-angle approximation

**(4)**

```math
\delta q \approx \begin{bmatrix} 1 & \tfrac{1}{2}\delta\theta \end{bmatrix}^\mathsf{T}
```

Using a three-component $`\delta\theta`$ rather than four quaternion states keeps the covariance
non-singular and preserves the unit-norm constraint on $`q`$ automatically.

## Initialization

The filter is initialized from a quasi-static interval: the vehicle stationary, with gravity the
only specific force. Initialization quality dominates early-flight performance, so the static
assumption must be validated rather than assumed.

Roll and pitch follow from the accelerometer. With $`f = a_m`$ averaged over the interval:

**(5)**

```math
\phi_0 = \mathrm{atan2}(-f_y,\ -f_z), \qquad \theta_0 = \mathrm{atan2}\left(f_x,\ \sqrt{f_y^2 + f_z^2}\right)
```

The signs follow from the down-positive convention: a level, stationary accelerometer reads
$`f = [0, 0, -\gamma]^\mathsf{T}`$.

### Levelling a window taken in motion

Stationarity is what lets (5) read $`f`$ as gravity alone, and a vehicle that is accelerating
breaks it. Equation (11) read backwards says by how much: $`R^\mathsf{T}(a_n - g) = f`$, so the
vector to level is the averaged specific force with the vehicle's own acceleration taken out of
it,

**(5′)**

```math
\bar{f}' = \bar{f} - R^\mathsf{T} \bar{a}_n, \qquad
\bar{a}_n = \frac{v_n(t_1) - v_n(t_0)}{t_1 - t_0}
```

with $`v_n`$ the GNSS velocity at the first and the last sample of the window carrying one. At
rest $`\bar a_n = 0`$ and (5′) is (5). Applying (5) to $`\bar f'`$ rather than to $`\bar f`$ is
the whole of in-motion levelling
([GOALS.md, alignment option 4](GOALS.md#alignment-beyond-the-static-window)).

Two properties keep it a *coarse* alignment, and neither improves with care:

* $`\bar a_n`$ is a difference of two noisy velocities over the span between them. A receiver
  reporting $`\sigma_v`$ = 0.28 m s⁻¹ at 1 Hz, differenced over 1 s, puts 0.39 m s⁻² of noise into
  a term whose whole purpose is to be subtracted from 9.8 — so the corrected tilt inherits an
  error the static case does not have, and the result must not promote to `Alignment::Static`.
  The window mean is taken from the endpoints for this reason: the mean of a derivative is its
  endpoint difference, and differencing the samples in between would add their noise back.
* $`R^\mathsf{T}`$ is part of what is being solved for. The correction splits by axis, which
  decides how much of it is available: the third column of $`R^\mathsf{T}`$ is the body-frame
  down direction, so the **vertical** part of $`\bar a_n`$ needs only the tilt and can be taken by
  fixed-point iteration from the uncorrected (5) — one pass is worth
  $`\lVert\bar a_n\rVert / \gamma`$, and a fixed count rather than a convergence test, for the
  reason `geodetic.rs` fixes its own. The **horizontal** part needs the yaw as well, so it is
  available only where (6) supplies one, iterating (5) and (6) together. A bare vehicle
  accelerating horizontally with no heading source is the case option 4 does not reach on its own;
  that is what [option 5](GOALS.md#alignment-beyond-the-static-window) is for.

Solving instead for the rotation that carries $`\bar f`$ onto the known navigation vector
$`\bar a_n - g`$ would need no iteration and would observe part of the yaw, but it degenerates as
$`\bar a_n \to 0`$ — the regime most launches sit in — and it couples heading into a step whose
job is tilt.

Yaw follows from the magnetometer, levelled by the roll and pitch just computed. With
$`R_0 = R_y(\theta_0) R_x(\phi_0)`$ and $`m_b`$ the averaged magnetometer reading:

**(6)**

```math
\tilde{m} = R_0\, m_b, \qquad \psi_0 = D_m - \mathrm{atan2}(\tilde{m}_E,\ \tilde{m}_N)
```

The nominal state is then initialized as

**(7)**

```math
\hat{q}_0 = q_{ZYX}(\psi_0, \theta_0, \phi_0), \qquad \hat{p}_0 = 0, \qquad \hat{v}_0 = 0, \qquad \hat{\beta}_{a,0} = 0, \qquad \hat{\beta}_{g,0} = \overline{\omega_m}
```

with $`q_{ZYX}`$ the quaternion of the yaw-pitch-roll sequence
$`R = R_z(\psi) R_y(\theta) R_x(\phi)`$, matching $`R_0`$ in (6).

The gyroscope bias is observable at rest and is initialized to the measured average. The
accelerometer bias is not separable from attitude error at rest and is initialized to zero.

`β̂_{g,0} = \overline{\omega_m}` holds only where the window was taken **at rest**, which is what
makes the bias observable; a window taken in motion offers the vehicle's own rotation under the
same name, so it starts at zero instead. That test is `init::at_rest`, not the `Alignment` — the
same distinction `α₀` draws below. Neither production estimator averages at all: ArduPilot zeroes
the bias at bootstrap (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:546`, `368dc0c4`) and PX4 refuses
to initialize outside 0.8–1.2 g and 15°/s (`src/modules/ekf2/EKF/ekf.cpp:213-227`, `c4e4ef98`).

The initial covariance is diagonal:

**(8)**

```math
P_0 = \mathrm{diag}\left( \sigma_{p,0}^2 I,\quad \sigma_{v,0}^2 I,\quad \mathrm{diag}(\sigma_{\text{tilt},0}^2, \sigma_{\text{tilt},0}^2, \sigma_{\psi,0}^2),\quad \sigma_{\beta a,0}^2 I,\quad \sigma_{\beta g,0}^2 I \right)
```

Yaw uncertainty $`\sigma_{\psi,0}`$ is set much larger than tilt uncertainty
$`\sigma_{\text{tilt},0}`$: roll and pitch come from gravity and are well determined, whereas yaw
comes from the magnetometer and inherits its calibration error.

A window that is *not* a static interval keeps that shape and reads both figures off what it
measured. (5)–(6) level **averages**, so what bounds the attitude they yield is how far those
averages sit from what a still vehicle reads — not how far the worst sample in the window was:

**(8′)**

```math
\sigma_{\text{tilt},0} = \max\left( \sigma_{\text{tilt}},\quad
\frac{\bigl|\, \lVert \bar f \rVert - \gamma \,\bigr|}{\gamma},\quad
\bigl\lVert (I - \hat d\, \hat d^\mathsf{T})\, \bar\omega_r\, T \bigr\rVert,\quad
\angle\bigl( \bar f_1,\, \bar f_2 \bigr) \right)
```

```math
\sigma_{\psi,0} = \min\left( \max\bigl( \sigma_\psi,\; \tan\delta \cdot \sigma_{\text{tilt},0},\; \bigl| \hat d \cdot \bar\omega_r\, T \bigr|,\; \lvert \psi_2 - \psi_1 \rvert \bigr),\; \frac{\pi}{\sqrt 3} \right),
\qquad
\tan\delta = \frac{\bigl| \bar m \cdot \hat d \bigr|}{\bigl\lVert (I - \hat d\, \hat d^\mathsf{T})\, \bar m \bigr\rVert}
```

with $`T`$ the window's span, $`\hat d = -\bar f / \lVert \bar f \rVert`$ the direction (5)
levelled to, and the subscripts 1 and 2 the same averages taken over the first and the second
**half** of the window — $`\psi_i`$ being the heading (6) yields from that half alone, so the
declination cancels. The rotation charged is what (7) did *not* take,
$`\bar\omega_r = \bar\omega - \hat\beta_{g,0}`$: a window at rest commits the whole average as
gyroscope bias, and the same quantity cannot be both removed from the state and charged to the
prior around it. Where (7) left the bias at zero, $`\bar\omega_r = \bar\omega`$.

Past the configured floor and the specific force that is not gravity, the attitude is bounded by
two witnesses to how far it moved while it was being averaged, and **both** are needed because
each is blind to what the other sees. The gyroscope's net rotation splits about $`\hat d`$:
across it spoils the tilt, since $`\dot{\hat g} = -\omega \times \hat g`$ has magnitude
$`\lVert \omega_\perp \rVert`$, while about it turns the vehicle without moving that vector
in body axes and spoils the heading instead. But a net rotation cancels for a vehicle that swings
out and comes back, charging nothing where the attitude (5) commits is the middle of an arc
already left. The disagreement between the window's two halves never cancels — and cannot see a
*coordinated* turn, where the specific force stays put in body axes while the vehicle banks, so
every average agrees and the tilt is wrong by the bank angle. Neither alone is a bound; the
evidence for that is in `data/scenarios.txt`.

$`\sigma_{\psi,0} = \pi/\sqrt 3`$, a heading uniform on the circle, where no magnetometer
observed the window, where its field has no horizontal part, or where the bound above would claim
more spread than a circle holds. The dip $`\delta`$ is read off the same averaged field (6) takes
its heading from, and is the rate at which a tilt error turns that heading.

## Nominal state propagation

Bias-corrected IMU measurements:

**(9)**

```math
\omega = \omega_m - \hat{\beta}_g
```

**(10)**

```math
a_b = a_m - \hat{\beta}_a
```

Specific force rotated into the navigation frame and gravity added:

**(11)**

```math
a_n = R(\hat{q})\, a_b + g
```

Continuous-time kinematics:

**(12)**

```math
\dot{p} = v, \qquad \dot{v} = a_n, \qquad \dot{q} = \tfrac{1}{2}\, q \otimes \begin{bmatrix} 0 \\ \omega \end{bmatrix}, \qquad \dot{\beta}_a = 0, \qquad \dot{\beta}_g = 0
```

Discrete integration over $`\Delta t`$:

**(13)**

```math
\hat{p} \leftarrow \hat{p} + \hat{v}\,\Delta t + \tfrac{1}{2} a_n \Delta t^2
```

**(14)**

```math
\hat{v} \leftarrow \hat{v} + a_n \Delta t
```

**(15)**

```math
\hat{q} \leftarrow \hat{q} \otimes \mathrm{Exp}(\omega\,\Delta t)
```

> **Order matters.** Equation (13) uses the **pre-update** velocity. Applying (14) first and then
> (13) adds a spurious $`a_n \Delta t^2`$ to position on every propagation step — a bias that
> integrates without bound. Implementations must evaluate (13) before (14), or compute both from
> a saved copy of $`\hat{v}`$.

Biases are modelled as random walks and are unchanged by propagation. The quaternion is
renormalized after (15).

## Error-state dynamics

Linearized continuous-time error dynamics, local attitude error:

**(16)**

```math
\delta\dot{p} = \delta v
```

**(17)**

```math
\delta\dot{v} = -R(\hat{q})\,[\,a_b\,]_\times\,\delta\theta \;-\; R(\hat{q})\,\delta\beta_a \;-\; R(\hat{q})\,w_a
```

**(18)**

```math
\delta\dot{\theta} = -[\,\omega\,]_\times\,\delta\theta \;-\; \delta\beta_g \;-\; w_g
```

**(19)**

```math
\delta\dot{\beta}_a = w_{\beta a}, \qquad \delta\dot{\beta}_g = w_{\beta g}
```

Equation (17) is the coupling that motivates the whole filter: an attitude error rotates the
measured specific force incorrectly, which integrates into velocity and then position.

## Covariance propagation

Discrete state transition matrix. Every block is first order in $`\Delta t`$ except the attitude
block, which is exact:

**(20)**

```math
F = \begin{bmatrix}
I & I\Delta t & 0 & 0 & 0 \\
0 & I & -R(\hat{q})[\,a_b\,]_\times \Delta t & -R(\hat{q})\Delta t & 0 \\
0 & 0 & R\{\omega \Delta t\}^\mathsf{T} & 0 & -I\Delta t \\
0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & I
\end{bmatrix}
```

The attitude block $`R\{\omega\Delta t\}^\mathsf{T}`$ may be approximated as
$`I - [\,\omega\,]_\times \Delta t`$ where the cost of the exact form is not justified; that
approximation is the usual source of small attitude-covariance error at high rotation rates.

Discrete process noise, impulse form:

**(21)**

```math
Q = \mathrm{diag}\left( 0,\quad \sigma_a^2 \Delta t\, I,\quad \sigma_g^2 \Delta t\, I,\quad \sigma_{\beta a}^2 \Delta t\, I,\quad \sigma_{\beta g}^2 \Delta t\, I \right)
```

Every block carries $`\Delta t`$, and the four $`\sigma`$ are **spectral densities**:
`ImuNoise`'s fields are stated per $`\sqrt{\mathrm{Hz}}`$ and `examples/simulate.rs` draws its
per-sample noise as $`\sigma / \sqrt{\Delta t}`$, so a density's contribution to variance over a
step is $`\sigma^2 \Delta t`$ — for the white-noise blocks exactly as for the two random walks.

Solà writes the white-noise blocks with $`\Delta t^2`$ (262)–(265), and so do PX4 and ArduPilot,
because the $`\sigma`$ each of them names is one sample's increment rather than a density: Solà
states $`\sigma_{\tilde a}`$ in m s⁻² (452) and holds it constant across the step (427), (444);
PX4 has `sq(dt) * accel_var` with `accel_var = sq(ekf2_acc_noise)`
(`src/modules/ekf2/EKF/python/ekf_derivation/generated/predict_covariance.h:161-164`,
`EKF/covariance.cpp:119-133`, at `c4e4ef98e9`); ArduPilot has `dvxVar = sq(dt * _accNoise)`
(`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1177` at `368dc0c428`). The two forms are one
equation, since $`\sigma_{\text{sample}} = \sigma / \sqrt{\Delta t}`$ carries
$`\sigma_{\text{sample}}^2 \Delta t^2`$ into $`\sigma^2 \Delta t`$; what differs is which of the
two is held fixed when the rate changes. Holding the per-sample $`\sigma`$ fixed ties $`Q`$ to
the sample rate — the variance it adds over $`T`$ seconds is $`\sigma^2 \Delta t\, T`$, so the
same airframe logged at 400 Hz is given eight times less process noise than at 50 Hz — and the
logs in `data/manifest.txt` run from 50 Hz to 250 Hz against one `Config`. Holding the density
fixed is rate-independent, and `propagate::process_noise` implements that.

The velocity block of (21) is the rotated accelerometer noise $`R \Sigma_a R^\mathsf{T}`$. Writing
it as $`\sigma_a^2 I`$ is exact only when the accelerometer noise is **isotropic**, since
$`R (\sigma_a^2 I) R^\mathsf{T} = \sigma_a^2 I`$ for orthogonal $`R`$. Real IMUs are not
isotropic — the z axis is typically noisier. Either use a per-axis $`\Sigma_a`$ and carry the
rotation, or set $`\sigma_a`$ to the worst axis and document the conservatism; the code takes the
second, which is also what lets $`Q`$ be built as a diagonal rather than a matrix.

Covariance propagation:

**(22)**

```math
P \leftarrow F P F^\mathsf{T} + Q
```

## Measurement update

For an observation $`z`$ with model $`h(x)`$, Jacobian $`H`$, and noise covariance $`R_m`$:

**(23)**

```math
y = z - h(\hat{x})
```

**(24)**

```math
S = H P H^\mathsf{T} + R_m
```

**(25)**

```math
K = P H^\mathsf{T} S^{-1}
```

**(26)**

```math
\delta\hat{x} = K y
```

Because the error state is reset to zero after every update, its prior is always zero and (26)
has no prior term.

Covariance update in Joseph form, which preserves symmetry and positive definiteness under
finite precision:

**(27)**

```math
P \leftarrow (I - KH)\,P\,(I - KH)^\mathsf{T} + K R_m K^\mathsf{T}
```

Joseph form costs more than $`P \leftarrow (I - KH)P`$ but is the appropriate default for `f32`
arithmetic on an embedded target.

## Observation models

### GNSS position

**(28)**

```math
z = p_{\text{GNSS}}, \qquad h(x) = p, \qquad H = \begin{bmatrix} I_3 & 0 & 0 & 0 & 0 \end{bmatrix}
```

Where the filter has no position estimate at all — a coarse initialization, with the vehicle
moving through somewhere it cannot name — the first fix is adopted rather than fused:

```math
\hat{p} \leftarrow z, \qquad P_{pp} \leftarrow R, \qquad P_{p\ast} \leftarrow 0
```

which is $`\lim_{P_{pp} \to \infty}`$ of the update above, taken exactly. The correlations go to
zero because the new error is the measurement's and has nothing to do with what preceded it.
Applies once, to a quantity never established; it is not a recovery mechanism. See
[gate lockout](#gate-lockout).

### GNSS velocity

**(29)**

```math
z = v_{\text{GNSS}}, \qquad h(x) = v, \qquad H = \begin{bmatrix} 0 & I_3 & 0 & 0 & 0 \end{bmatrix}
```

Velocity observations matter disproportionately: velocity error grows linearly from accelerometer
and attitude error, so constraining it directly also constrains those states through the
covariance.

The same adoption applies to the first velocity solution after a coarse start, and matters more
there: a static initialization knows the vehicle is at rest, a coarse one knows nothing at all.

### Barometric altitude

The barometer reports altitude $`\alpha`$ above its own reference, positive up, while the
navigation frame is down-positive:

**(30)**

```math
z = -(\alpha - \alpha_0), \qquad h(x) = p_D, \qquad H = \begin{bmatrix} e_3^\mathsf{T} & 0 & 0 & 0 & 0 \end{bmatrix}
```

$`\alpha_0`$ is the barometric altitude recorded during initialization, which fixes the barometer
reference to the navigation origin. **It is a constant, not a state.** The 15-state vector (2) has
no barometer bias, so slow drift in the barometric reference — from weather, from ground effect,
from sensor warm-up — is not estimated and appears directly as vertical position error. Where
that matters, the options are to re-establish $`\alpha_0`$ on the ground, to lean on GNSS height
for the low-frequency component, or to extend the state vector, which is out of scope for this
filter.

The barometer measures height, and the navigation frame is a plane. Written as above, (30)
treats $`-p_D`$ as height, which is off by the plane's rise above the surface, $`d^2 / 2R`$ at a
horizontal distance $`d`$ from the origin (see [geodetic origin](#geodetic-origin)): 8 cm at 1 km,
7.8 m at 10 km. GNSS positions converted by (43) carry that rise and the barometer does not, so
beyond a few kilometres the two disagree about height by exactly that amount. Removing it means
writing $`h(x)`$ as minus the height of $`\hat{p}`$ above $`h_0`$, by the inverse of (43) —
$`p_D - (p_N^2 + p_E^2) / 2R`$ to second order — so both sides are heights; $`H`$ is unchanged to
first order.

`altitude_observation` does not write it, and its doc comment carries the measurement that
decided so: neither the corpus nor the simulator travels far enough from an origin for the term
to be worth a millimetre against a barometer's own noise, and the simulator generates its reading
from $`-p_D`$ on a flat plane, so it could not score the correction even where it mattered. That
comment owns the numbers and the condition for revisiting them.

### Magnetometer, three-axis

With $`m_n`$ the reference field in the navigation frame, the predicted body-frame measurement is

**(31)**

```math
\hat{m}_b = R(\hat{q})^\mathsf{T} m_n
```

Perturbing with a local attitude error, $`R = R(\hat{q})\,\mathrm{Exp}(\delta\theta)`$, and using
$`[\,u\,]_\times v = -[\,v\,]_\times u`$:

**(32)**

```math
m_b \approx \hat{m}_b + [\,\hat{m}_b\,]_\times\, \delta\theta
```

so

**(33)**

```math
z = m_b^{\text{meas}}, \qquad h(x) = \hat{m}_b, \qquad H = \begin{bmatrix} 0 & 0 & [\,\hat{m}_b\,]_\times & 0 & 0 \end{bmatrix}
```

Three-axis fusion constrains all three attitude components from the magnetometer. That is a
liability rather than a benefit here: hard- and soft-iron errors and local field anomalies
corrupt roll and pitch, which the accelerometer already determines well, and `fusion-nav` carries
no magnetic-field states to absorb them. See
[Magnetometer without magnetic-field states](GOALS.md#magnetometer-without-magnetic-field-states).

### Magnetometer, heading only

This is the **default**. It constrains yaw alone, leaving roll and pitch to gravity.

Rotate the measurement into the navigation frame with the current attitude estimate:

**(34)**

```math
\tilde{m}_n = R(\hat{q})\, m_b^{\text{meas}}
```

If the attitude estimate were exact, the horizontal part of $`\tilde{m}_n`$ would make an angle
$`D_m`$ with North. Any residual is yaw error, so the innovation is formed directly rather than
as a difference of two angles:

**(35)**

```math
y = -\,\mathrm{wrap}\big(\mathrm{atan2}(\tilde{m}_{n,E},\ \tilde{m}_{n,N}) - D_m\big)
```

A local body-frame error $`\delta\theta`$ corresponds to the navigation-frame rotation vector
$`R(\hat{q})\,\delta\theta`$, and yaw is rotation about the navigation down axis, so

**(36)**

```math
H = \begin{bmatrix} 0 & 0 & e_3^\mathsf{T} R(\hat{q}) & 0 & 0 \end{bmatrix}
```

with $`R_m = \sigma_\psi^2`$ scalar. Equation (36) treats yaw as the down-axis component of the
rotation vector, which is exact at zero tilt and degrades as $`1/\cos\theta`$; at the tilt angles
flight controllers operate at, the error is not significant, and the approximation avoids a
singularity at 90° pitch.

Note that (35) wraps the angle **before** it is used, so the innovation is always in
$`(-\pi, \pi]`$ and a heading near ±180° does not produce a spurious 2π innovation.

## Innovation gating

The normalized innovation squared

**(37)**

```math
\epsilon = y^\mathsf{T} S^{-1} y
```

is compared against a threshold $`\gamma`$ from the chi-square distribution with
$`\dim(z)`$ degrees of freedom. The measurement is rejected when $`\epsilon > \gamma`$.

| dim(z) | observation | 95 % | 99 % | 99.9 % |
| ------ | ----------- | ---- | ---- | ------ |
| 1 | barometric altitude, magnetic heading | 3.8415 | 6.6349 | 10.8276 |
| 3 | GNSS position, GNSS velocity, three-axis magnetometer | 7.8147 | 11.3449 | 16.2662 |

`Gate::at` holds these, typed by $`\dim(z)`$, and `Gates::at` builds one per source from a single
percentile. The test is joint over every component of $`z`$ rather than per axis as PX4 and
ArduPilot gate; why, and what it costs, is on `Gates`.

Rather than reporting $`\epsilon`$ directly, the filter exposes the dimensionless **test ratio**

**(38)**

```math
r = \frac{\epsilon}{\gamma}
```

so that $`r > 1`$ means rejected regardless of the degrees of freedom of the observation. One
number is then comparable across GNSS position, barometric altitude, and magnetic heading, and
directly comparable with the innovation test ratios PX4 publishes in its logs — which is what
makes replay comparison against EKF2 a like-for-like check rather than an approximate one.

This single mechanism covers GNSS glitches, barometer transients, and magnetic interference.

### Gate lockout

Gating is self-sealing. If the **filter** is wrong rather than the measurement — a poor
initialization, an unmodelled bias, a divergence — then correct measurements are inconsistent
with the state, every one of them is rejected, and the filter locks itself out of the very data
that would correct it. It then dead-reckons on the IMU alone while continuing to report a
solution whose covariance says it is confident.

A filter that gates without a route out of this state is more dangerous than one that does not
gate at all.

`fusion-nav` therefore tracks, per observation source, the time since a measurement was last
accepted and the number of consecutive rejections, and reports an aggregate status alongside the
state estimate. It does **not** reset itself: recovery policy belongs to the application, which
is the only layer that knows whether to reset the affected states, degrade the flight mode, or
alert the operator. PX4, for comparison, resets its states to the measurement after 7 s
of horizontal inertial dead reckoning or 5 s of failed height fusion (`reset_timeout_max` and
`hgt_fusion_timeout_max`, `src/modules/ekf2/EKF/common.h:515-517` at PX4 `c4e4ef98e9`).

The design obligation is that the degraded condition cannot be missed, not that the filter hides
it by recovering silently.

## Error injection and reset

The estimated error is composed into the nominal state:

**(39)**

```math
\hat{p} \leftarrow \hat{p} + \delta\hat{p}, \qquad \hat{v} \leftarrow \hat{v} + \delta\hat{v}, \qquad \hat{q} \leftarrow \hat{q} \otimes \mathrm{Exp}(\delta\hat{\theta})
```

**(40)**

```math
\hat{\beta}_a \leftarrow \hat{\beta}_a + \delta\hat{\beta}_a, \qquad \hat{\beta}_g \leftarrow \hat{\beta}_g + \delta\hat{\beta}_g
```

The error state is then reset to zero and the covariance transformed by the reset Jacobian:

**(41)**

```math
\delta x \leftarrow 0, \qquad P \leftarrow G P G^\mathsf{T}, \qquad G = \mathrm{diag}\left(I, I, I - [\tfrac{1}{2}\delta\hat{\theta}]_\times, I, I\right)
```

The attitude block of $`G`$ is frequently approximated as $`I`$. That is acceptable for small
corrections and should be an explicit, documented choice rather than an omission. `update.rs`
keeps the exact block, and `reset` says why.

## Numerical conditioning

Symmetry is enforced after every covariance operation:

**(42)**

```math
P \leftarrow \tfrac{1}{2}\left(P + P^\mathsf{T}\right)
```

Diagonal variances are floored at a small positive value to prevent a state from becoming
unobservably certain and then unrecoverable. With `f32` these are not optional refinements;
they are what keeps a 15-state filter stable over a long flight.

## Geodetic origin

The navigation frame is the plane tangent to the WGS84 ellipsoid at an origin
$`(\varphi_0, \lambda_0, h_0)`$, which the filter holds so that a geodetic fix and the estimate
are relative to the same point. A geodetic position goes to Earth-centered, Earth-fixed
coordinates (Groves (2.112)), with $`N`$ the radius of curvature in the prime vertical,

```math
r^e = \begin{bmatrix} (N + h)\cos\varphi\cos\lambda \\ (N + h)\cos\varphi\sin\lambda \\ \left(N(1 - e^2) + h\right)\sin\varphi \end{bmatrix},
\qquad N = \frac{a}{\sqrt{1 - e^2\sin^2\varphi}}
```

and a fix is its ECEF offset from the origin, rotated onto the origin's north, east and down:

**(43)**

```math
p = C_e^n \left(r^e - r^e_0\right), \qquad
C_e^n = \begin{bmatrix}
-\sin\varphi_0\cos\lambda_0 & -\sin\varphi_0\sin\lambda_0 & \cos\varphi_0 \\
-\sin\lambda_0 & \cos\lambda_0 & 0 \\
-\cos\varphi_0\cos\lambda_0 & -\cos\varphi_0\sin\lambda_0 & -\sin\varphi_0
\end{bmatrix}
```

Exact at every range, including the poles; the only rounding is the final narrowing to `f32`,
1 mm at 10 km. The inverse is $`r^e = r^e_0 + (C_e^n)^\mathsf{T} p`$ followed by ECEF to geodetic,
whose latitude is the fixed point of $`\varphi = \operatorname{atan2}(z + e^2 N(\varphi)\sin\varphi,\ p_{xy})`$:
each pass shrinks the error by about $`e^2`$, so five passes from the geocentric latitude reach
below a micrometre. Height is $`h = p_{xy}\cos\varphi + z\sin\varphi - a\sqrt{1 - e^2\sin^2\varphi}`$,
which stays finite at the poles.

Exact conversion does not make the plane follow the Earth. At a horizontal distance $`d`$ from the
origin, the plane sits $`d^2 / 2R`$ above the surface — 8 cm at 1 km, 7.8 m at 10 km — so
$`-p_D`$ there is not height above the origin. A GNSS fix converted by (43) carries that
curvature; a barometer does not, which is what the [barometric model](#barometric-altitude) has
to account for.

The first geodetic fix $`z_g`$ places the origin. A filter that already has a position estimate
$`\hat{p}`$ — it has been navigating relative to its own start — places it so the fix lands on the
estimate:

**(44)**

```math
r^e_0 = r^e(z_g) - (C_e^n)^\mathsf{T}\,\hat{p}
```

where $`C_e^n`$ is itself the origin's, so the equation is solved by iteration: start at the fix,
and take each pass's axes from the previous guess. The error shrinks by $`|\hat{p}| / R`$ a pass —
from 10 km, three passes leave 40 µm — and `LocalOrigin::placing` runs five, then checks that the
fix lands on the estimate rather than assuming it did, because near a pole the passes need not
converge and an origin putting the fix at the estimate need not exist at all. The first fix then carries no information about position,
which is correct: before it, the filter's absolute position was unknown, not wrong. It is spent
placing the origin and is not fused as well, which would count it twice.

It does fix the position uncertainty. With $`e_g`$ the fix's error, the origin sits $`e_g`$
from where it should, so the position error about it is $`\delta p = -e_g`$ — whatever $`P`$
said about position relative to the start no longer applies, and $`e_g`$ is independent of
every other error state:

```math
P_{pp} \leftarrow R_g, \qquad P_{px} \leftarrow 0
```

the covariance half of `Eskf::reset_position_to`, with $`\hat{p}`$ unchanged. Fusing the fix
instead, at zero innovation, would give $`(P_{pp}^{-1} + R_g^{-1})^{-1}`$: after a static start,
where $`P_{pp}`$ is small, an estimate claiming centimeters about an origin placed to meters.

Without an estimate (a coarse start) the origin is the fix and the fix is adopted as
$`\hat{p} = 0`$, per (28).

## Equation-to-code mapping

Intended layout. Each implementing function cites its equation numbers in a doc comment.

| equations | concept | module | function |
| --------- | ------- | ------ | -------- |
| (1)–(4) | state definitions | `state.rs` | `State`, `ErrorState` |
| (5)–(8) | static initialization | `init.rs` | `measure`, `level_from_accel`, `heading_from_mag`, `nominal_state`, `classify`, `attitude_sigmas`, `initial_covariance` |
| (8′) | what a coarse window supports | `init.rs` | `coarse_sigmas`, `window_drift`, `heading_sensitivity` |
| (5′) `ā_n` | in-motion levelling | `init.rs` | `inertial_acceleration`; the correction itself is unbuilt |
| (30) `α₀` | barometric reference | `init.rs` | `baro_reference` |
| (9)–(11) | bias correction, gravity | `propagate.rs` | `ImuSample`, `corrected_imu` |
| (12)–(15) | nominal propagation | `propagate.rs` | `propagate_nominal` |
| (16)–(19) | error dynamics | `propagate.rs` | carried as the derivation on `transition_matrix`; (20) is what the filter computes |
| (20) | state transition matrix | `propagate.rs` | `transition_matrix` |
| (21) | discrete process noise | `propagate.rs` | `process_noise` |
| (22) | covariance propagation | `propagate.rs` | `propagate_covariance`, called with (9)–(15) by `propagate` |
| (23)–(27) | generic update, Joseph form | `update.rs` | `update` |
| (28) | GNSS position | `observation/gnss.rs` | `position_jacobian`, `position_observation` |
| (29) | GNSS velocity | `observation/gnss.rs` | `velocity_jacobian`, `velocity_observation` |
| (30) | barometric altitude | `observation/baro.rs` | `altitude_jacobian`, `altitude_observation` |
| (31)–(33) | magnetometer, three-axis | `observation/mag.rs` | `field_jacobian`, unbuilt and out of scope |
| (34)–(36) | magnetometer, heading only | `observation/mag.rs` | `heading_innovation`, `heading_jacobian`, unbuilt |
| (37) `γ` | gate thresholds | `config.rs` | `Gate::at`, `Gate::new`, `Gates::at` |
| (37)–(38) | innovation gating, test ratio | `update.rs` | `nis`, `test_ratio`, called by `update` |
| — | per-source health tracking | `health.rs` | `SourceHealth`, `Status` |
| (39)–(41) | injection and reset | `update.rs` | `inject`, `reset`, called by `update` |
| (42) | symmetry enforcement | `math.rs` | `enforce_symmetry` |
| (43) | local tangent plane | `geodetic.rs` | `LocalOrigin::to_ned`, `to_geodetic` |
| (44) | origin placement | `geodetic.rs` | `LocalOrigin::placing`; committed by `Eskf::fuse_gnss_geodetic` |
| — | skew, quaternion exponential, angle wrap | `math.rs` | `skew`, `exp_quat`, `wrap_pi` |

## Deferred

Measurement latency is not modelled. Every observation above is fused as though it were
simultaneous with the current state, which is not true of GNSS. See
[Measurement latency](GOALS.md#measurement-latency).

## References

* J. Solà, *Quaternion kinematics for the error-state Kalman filter*, [arXiv:1711.02508](https://arxiv.org/abs/1711.02508) — the primary source for the error-state formulation and the Jacobians above; [Correspondence with Solà](#correspondence-with-solà) says which equation here is which of his
* P. D. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. — (2.112), the geodetic-to-ECEF conversion of (43), and the one citation here that needs a book. The ellipsoid constants it is evaluated with are cited in `src/geodetic.rs` to NGA.STND.0036, which is free and in [`reference/`](reference/README.md)
* [PX4 EKF2](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) — reference for practical behavior, not for derivations

### Correspondence with Solà

A bare number in parentheses elsewhere in this document is an equation of *this* document; a
number introduced by "Solà" is his, from arXiv v1, the only version arXiv holds, which
[`reference/README.md`](reference/README.md) says how to fetch. This document follows his
Section 5, the locally-defined angular error; Section 7 is the global alternative noted under
[Frames](#frames).

| here | Solà v1 | |
| ---- | ------- | --- |
| (2) | (266) | same ordering, without his gravity state |
| (3), (4) | Table 3, §6.1.1 | $`q_t = q \otimes \delta q`$, and $`\delta q \to [1, \tfrac{1}{2}\delta\theta]`$ |
| (12) | (237) | |
| (13)–(15) | (260a)–(260c) | (260a) takes the pre-update velocity, as (13) does |
| (16)–(19) | (238) | without $`\delta g`$, which this filter does not estimate |
| (20) | (270) | block for block |
| (21) | (262)–(265) | densities rather than per-sample $`\sigma`$; see [above](#covariance-propagation) |
| (22) | (269) | |
| (23)–(26) | (274), (275) | |
| (27) | footnote 26 | Joseph form, which he recommends over his own (276) |
| (39), (40) | (283) | |
| (41) | (285)–(288) | the exact $`G`$; he gives $`G = I`$ as the usual approximation, which `reset` does not take |
| $`\mathrm{Exp}(\phi)`$ | (101) | |

The rest is not his. Initialization (5)–(8) follows the practice of PX4 and ArduPilot cited at
(7); (43) is Groves (2.112); the observation models (28)–(36), the gating of (37)–(38) and the
origin placement of (44) are derived here.
