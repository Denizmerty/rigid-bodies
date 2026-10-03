# Analytical and convergence validation

Two headless physics suites provide analytical and convergence checks: `analytical_validation` and `convergence`.
Every experiment advances the production `World::step` path, including force
sampling, collision detection, contact or joint solving, and integration. The tests use the real contact manifolds and integrators. Sleeping is disabled,
worker count is one, and any motion-limit event fails the experiment. These
experiments use fixed parameter grids, so no random seed or external data is
required. Failures include the method and physical parameters; convergence runs
also print all three measured errors and both observed orders to the test log.

Run the registered suites from an existing build with:

```powershell
ctest --test-dir build/cmake -R "Physics.(analytical_validation|convergence).Tests" --output-on-failure
```

Use `-V` to display the convergence measurements even when the tests pass. The
full CTest matrix includes these suites in both desktop and headless builds.

## Closed-form experiments

### Projectile motion

The exact center-of-mass trajectory in a uniform field is

```text
v(T) = v0 + g T
x(T) = x0 + v0 T + g T² / 2.
```

For `N = T/h` semi-implicit Euler steps, summing the arithmetic velocity sequence
gives an additional displacement `g T h / 2`. Velocity Verlet and RK4 integrate
this constant acceleration exactly in real arithmetic. The test checks both
components against the expected result for each method, including Euler's first-order displacement error. It also checks mechanical
energy `K - m g·x`: Euler's exact energy defect is `-m |g|² T h / 2`.

The grid is all three methods, masses 0.2, 1, and 9 kg, and 15, 30, and 60 steps
over 0.5 s: **27 experiments**. Gravity is `(1.25, -9.81)` m/s², initial velocity
is `(2.3, 4.1)` m/s, and the collider is offset from the body origin so the center
of mass must be used. Position and velocity tolerances are `2e-12` in SI units;
the energy tolerance is `2e-11 × mass` joules. These tolerances cover floating-point roundoff for the short polynomial calculation.
The expected result already includes the method's discretization error.

### Elastic collision

Two circles begin in contact with normal velocities `u1 = 3` and `u2 = -1` m/s.
Conservation of normal momentum and reversal of relative normal velocity give

```text
v1 = ((m1 - m2) u1 + 2 m2 u2) / (m1 + m2)
v2 = (2 m1 u1 + (m2 - m1) u2) / (m1 + m2).
```

Each mass ranges over 0.25, 1, and 4 kg and the contact normal rotates through
0, 0.37, and 1.2 radians: **27 experiments**. A common drift of 0.7 m/s along the
normal and 0.3 m/s along its tangent tests the moving-frame result. Friction is
zero and restitution is one. The test requires an actual detected manifold,
checks both normal and tangent velocities, checks both momentum components and
kinetic energy, and excludes spurious spin. Velocity, spin, and momentum
tolerances are `2e-10`; energy tolerance is `2e-9` J. This is an isolated, centered
collision; multi-contact restitution is a different numerical problem.

### Block on a ramp

A 2 kg rectangular block sits on a rotated static rectangle. Rotation is fixed
to isolate translation, and both materials have the same specified friction.
The normal reaction cancels `m g cos(theta)`. Therefore:

```text
frictionless: a = g sin(theta)
static:      v = 0 when tan(theta) <= mu_s
sliding:     a = g [sin(theta) - mu_k cos(theta)].
```

The **10 experiments** cover frictionless slopes 0.15, 0.5, and 0.85 radians;
static slopes 0.1, 0.3, and 0.45 with `mu_s = 0.5`, `mu_k = 0.3`; and sliding
slopes 0.45 and 0.6 with `mu_s = 0.3`, `mu_k = 0.2`, starting at either 0 or
0.7 m/s downhill. All run 120 steps at `h = 1/240` s. Sliding samples have
positive downhill acceleration throughout, so no sign change or stick/slide
transition invalidates the formula.

Velocity is compared with `v0 + a T`; position is compared with Euler's exact
discrete trajectory `v0 T + a T (T + h)/2`. Downhill speed, travel, normal speed,
and supported height use `2e-8` SI-unit budgets, covering accumulated contact
roundoff over 120 steps. Orientation uses `1e-12` rad. These checks validate
normal support and friction together, including the change from static to
kinetic friction when the static capacity is exceeded.

### Small-angle pendulum

A centered distance joint connects a fixed anchor and a fixed-rotation bob.
The bob behaves as a point mass at distance `L`, since the rod attaches at its
center of mass and does not couple the bob's rotational inertia. With
`omega = sqrt(g/L)`, the linear model is

```text
theta_linear(t) = A cos(omega t).
```

This formula is an approximation to the nonlinear pendulum, so the test keeps
model mismatch separate from numerical error. The nonlinear equation is
`theta'' + omega² theta = omega² (theta - sin(theta))`. Energy conservation
implies `|theta| <= A` for the ideal undamped libration. Duhamel's formula and
`|theta - sin(theta)| <= |theta|³/6` then bound the linearization error by

```text
|theta_nonlinear(t) - theta_linear(t)| <= omega t A³ / 6.
```

The **four experiments** use lengths 0.7 and 1.3 m and amplitudes 0.02 and
0.04 rad, with 4096 steps per small-angle period. Every 64 steps the angular
error must remain below the model bound plus `2 A omega h`. The latter is a
first-order tolerance used to detect numerical regressions in these tests.
Rod-length error must stay below `2e-6` m. The independent pendulum refinement study below verifies that the numerical error
actually falls at first order before the model mismatch becomes significant.

## Three-grid convergence studies

For error `E(h)` the observed order is `p = log2(E(h)/E(h/2))`. Each smooth-force
case runs three grids and checks **both** consecutive orders within 0.15 of the
method's expected order. Errors must remain above `1e-13`, preventing an
apparently favorable ratio formed from floating-point noise. Parameters are
scaled by the natural frequency or decay rate so each grid has the same
dimensionless step across physical scales.

| Experiment | Exact solution and error norm | Grids | Expected order |
| --- | --- | --- | --- |
| Harmonic oscillator | `x = A cos(omega T)`, `v = -A omega sin(omega T)`; Euclidean norm of `dx/A` and `dv/(A omega)` | 24, 48, 96 steps, `omega T = 1.3` | Euler 1, Verlet 2, RK4 4 |
| Linear viscous transient | `v = v_inf + (v0-v_inf) exp(-lambda T)`; `x = v_inf T + (v0-v_inf)(1-exp(-lambda T))/lambda`; norm of `lambda dx/(v_inf-v0)` and `dv/(v_inf-v0)` | 12, 24, 48 steps, `lambda T = 1.1` | Euler 1, Verlet 2, RK4 4 |
| Constrained pendulum | Angular error at one quarter-period against `A cos(omega T)` | 128, 256, 512 steps | Euler plus joint solve 1 |

The oscillator uses `A = 0.7` m and frequencies 0.8, 2, and 4 rad/s. The viscous
case uses decay rates 0.5, 1.5, and 3 per second, `v0 = -0.8` m/s and
`v_inf = 1.1` m/s. Its body force is `-m lambda v` with constant acceleration
`lambda v_inf`; this tests force sampling
at the trial velocity as well as the integration method. Together these are 54 smooth-force runs.

The constrained pendulum uses `A = 0.002` rad and lengths 0.7 and 1.3 m, giving
six runs. Every measured error must exceed 100 times the independently bounded
small-angle model error, so the refinement study cannot mistake model mismatch
for discretization error.

Windows x64 MSVC Release measurements against the current physics
library, obtained while introducing these tests, were:

| Experiment | Euler observed orders | Verlet observed orders | RK4 observed orders |
| --- | --- | --- | --- |
| Oscillator | 1.0023–1.0048 | 2.0001–2.0003 | 4.0000 (rounded) |
| Viscous transient | 1.0084–1.0171 | 2.0250–2.0506 | 4.0276–4.0552 |
| Constrained pendulum | 1.0008–1.0013 | Not asserted | Not asserted |

These values record the measurements from that run. The tests calculate fresh measurements
for each compiler and build and write them to the test log.

## Nonsmooth impact refinement

Fourth-order smooth-force convergence does not imply fourth-order collision
timing. A circle approaching a fixed frictionless wall at 3 m/s hits when its
center reaches `x = -0.25` m. Starting from `-1.337` m puts the analytical event
at `t_c = 1.087/3` s, off every tested grid. The exact reflected trajectory after
impact is `x(T) = -0.25 - 3(T-t_c)` with final velocity `-3` m/s.

All three integrators run 32, 64, 128, and 256 steps over 0.8 s: **12 experiments**.
Each must produce one contact-begin event, preserve the elastic speed and energy,
and place the final center within

```text
2 × incoming_speed × h + 2 × contact_margin
```

of the analytical reflected position. A one-step shift of the discontinuity
changes displacement by `|v_before-v_after| h = 2 speed h`; the remaining term
accounts for the finite contact margin. This is a shrinking first-order
bound on event-location error, limited by the contact margin. No monotonic ratio or
smooth-force order is required across impacts, because the fractional grid
location of a collision changes under refinement. Post-impact velocity uses
`2e-10` m/s and energy uses `2e-9` J tolerances. This regression bound applies to the isolated impact described here. Multi-body CCD needs
separate tests.
