# Testing Strategy — Algorithm Metrics

The library's test strategy: every algorithm is validated against the **mathematical invariants of
its family**, not golden output. This is the reference for the `unit-tester` agent (and humans) when
writing **unit tests** for `robotics/`. It answers one question per algorithm: *which mathematical
properties must a correct implementation satisfy, and how do we assert them?*

## Rationale

A numerical algorithm is not validated by "it compiles and doesn't crash." Every family has a small
set of **characteristic invariants** — properties that hold for any correct implementation
regardless of parameters (a rotation must preserve lengths; inverse dynamics composed with forward
dynamics must be the identity; a mass matrix must stay symmetric positive-definite). A unit test earns its place by pinning one such invariant against a **known
ground truth**, not by re-running the implementation and trusting its own output.

This file groups the library by **metric family** so a test author picks the right invariants fast:
kinematics needs geometric/round-trip metrics, dynamics needs cross-method and conservation metrics,
trajectories need boundary/limit metrics, controllers need closed-loop metrics. Without this, tests
drift toward shallow "golden output" snapshots that pass while the math is wrong.

Canonical rules still apply ([AGENTS.md](AGENTS.md), [testing.instructions.md](.github/instructions/testing.instructions.md)):
`TEST_F` on `float`, one behaviour per test, no redundant cases, **no heap in tests**, assert with
`EXPECT_NEAR` + `math::Tolerance<float>()` against reference values.

## How the agent uses this

1. Identify the target algorithm's **family** (section below).
2. From that family's row, take the **applicable metric types** and author **one `TEST_F` per
   distinct property** — not per parameter permutation.
3. Prefer **analytic ground truth** (closed-form response, known transform pair, hand-solved system)
   over self-consistency. Fall back to a cross-method check (e.g. RNEA vs an Euler-Lagrange model,
   ABA ∘ RNEA = identity) only when no closed form exists.
4. Always include the cross-cutting metrics (accuracy, boundary, determinism/reset) plus the
   family-specific ones. Keep reference data on the stack (`std::array`, bounded buffers).

## Metric types (the vocabulary)

| #  | Metric type                   | What it asserts                                              | Typical assertion                                                       |
|----|-------------------------------|--------------------------------------------------------------|-------------------------------------------------------------------------|
| M1 | **Numerical accuracy**        | output matches a closed-form / reference value               | `EXPECT_NEAR(out, ref, tol)`; ULP error for math funcs                  |
| M3 | **Time / transient response** | step & impulse behaviour                                     | rise time, settling time, % overshoot, steady-state error               |
| M4 | **Stability**                 | poles/eigenvalues inside unit circle; BIBO; Riccati/Lyapunov | pole radius < 1; bounded long run; residual of Lyapunov/Riccati eq      |
| M5 | **Convergence**               | iterative process reaches the answer                         | iterations-to-tolerance; monotonic objective/residual; contraction rate |
| M6 | **Boundary / edge**           | zero, saturation, extreme magnitude, min sizes               | clamp limits; zero-in→zero-out; no NaN/Inf at extremes                  |
| M7 | **Invariants & conservation** | energy, norm, orthogonality, symmetry / definiteness         | ‖q‖=1; RᵀR=I; energy drift bound; M = Mᵀ ≻ 0                            |
| M8 | **Statistical consistency**   | estimator bias / error / covariance sanity                   | RMSE vs truth; NEES/NIS in χ² band; R²; unbiasedness                    |
| M9 | **Conditioning / robustness** | behaviour near singularities & under ill-conditioning        | bounded steps near singular Jacobians; residual vs condition number     |

## Family → metric-type matrix

| Family                                | M1 | M3 | M4 | M5 | M6 | M7 | M8 | M9 |
|---------------------------------------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| Kinematics                            | ●  |    |    | ●  | ●  | ●  |    | ○  |
| Dynamics                              | ●  | ○  |    |    | ●  | ●  |    | ○  |
| Trajectory generation (planned)       | ●  | ●  |    |    | ●  | ●  |    |    |
| Manipulator control (planned)         | ●  | ●  | ●  | ○  | ●  |    |    |    |
| Estimation & identification (planned) | ●  | ●  | ●  | ●  | ●  |    | ●  | ●  |

● primary   ○ situational

---

## Per-family detail

### 1. Kinematics — `kinematics/`
`ForwardKinematics`, `InverseKinematics`; planned: SE(3) transforms, DH, chain Jacobian, pose IK,
manipulability, redundancy resolution, PoE, analytical IK, parallel/mobile/continuum kinematics.

- **M1 known geometry** — joint and tool positions for canonical angles (0, ±90°, 180°), including a
  **base offset** and a **tool offset** that differs from the center of mass.
- **M1 conventions** — right-hand rule on a non-z axis (`R_y(+90°)` maps `x → −z`) and rotation
  **composition order** with mixed axes at non-zero angles (parallel-axis tests cannot detect it).
- **M5 inverse kinematics round-trip** — `FK(IK(FK(q_ref))) ≈ FK(q_ref)`; converges within the
  iteration budget; an initial guess at the solution returns without iterating.
- **M6 boundary** — unreachable target reports non-convergence after exactly the iteration budget;
  near-full-extension and near-base targets still converge.
- **M7 invariants** — rotations stay orthonormal; Jacobian columns match a finite difference of FK;
  manipulability vanishes exactly at the known singular postures.

### 2. Dynamics — `dynamics/`
`RecursiveNewtonEuler`, `ArticulatedBodyAlgorithm`, `EulerLagrangeSolver`, `NewtonEulerSolver`;
planned: CRBA, chain-dynamics model, regressor, friction, generic joints.

- **M1 analytic references** — single rod: holding torque `−mgl/2`, `τ = (ml²/3)·q̈`, free fall
  `q̈ = 3g/2l`; 2-link uniform rods: closed-form `M(q)`, `C(q,q̇)q̇`, `g(q)` and the horizontal
  release `q̈ = g·[9/7, −12/7]`.
- **M1 cross-method consistency** — `RNEA` equals an independent Euler-Lagrange model under gravity
  and motion; `ABA(RNEA(q̈)) = q̈` on a **non-planar** chain (skewed axes, full inertia tensors,
  off-axis centers of mass) with gravity active. Gravity must act across the joint axes — a chain
  whose axes are parallel to gravity does not exercise it.
- **M6 boundary** — zero input gives zero output (no gravity, no motion); constant spin about the
  joint axis needs no torque.
- **M7 energy / passivity** — mass matrix symmetric positive definite; energy conserved in free
  motion up to the integrator's error; `Ṁ − 2C` skew-symmetric for the Christoffel form.

### 3. Trajectory generation — `trajectory/` (planned)
Polynomial, trapezoidal, S-curve, Cartesian SLERP, TOPP.

- **M1 boundary conditions** — endpoint position/velocity/acceleration matched exactly.
- **M6 limits** — velocity, acceleration and jerk never exceed their bounds; degenerate short moves
  collapse phases without negative durations.
- **M7 continuity** — position and velocity continuous; acceleration continuous for jerk-limited
  profiles; SLERP output stays unit-norm.
- **M3 timing** — phase durations and total time match the closed-form formulas.

### 4. Manipulator control — `controllers/` (planned)
PD + gravity, computed torque, impedance, operational space, hybrid force/position, Slotine–Li.

- **M1 control law** — the computed torque equals the documented law term by term (StrictMock
  dynamics/Jacobian providers).
- **M3/M4 closed loop** — on a simulated plant the tracking error decays as the designed error
  dynamics predict; equilibrium at the set-point; rendered stiffness/compliance matches the design.
- **M6 boundary** — singular Jacobians stay finite (transpose-based laws, damped inverses).

### 5. Estimation & identification (planned)
Momentum observer, dynamic parameter identification.

- **M3 observer response** — the residual follows a step external torque with the designed
  first-order dynamics.
- **M8 identification** — recovers known base parameters from noiseless excitation; bounded bias
  under noise.
- **M9 conditioning** — regressor conditioning reported for the excitation trajectory.

---

## Anti-patterns

- Golden-output snapshots with no independent reference ("the output is whatever it printed").
- One test per parameter value instead of one per property (violates *no redundant tests*).
- Asserting only "no NaN / no crash" without a numerical reference.
- Heap-allocated buffers in tests — use `std::array` / bounded buffers.
- Planar-only fixtures for spatial algorithms (all joint axes parallel, or parallel to gravity) — they
  cannot detect rotation-composition or gravity-propagation errors.
- Re-deriving the algorithm inside the test as the "reference" — the reference must be independent
  (closed form, hand computation, or a distinct method).
