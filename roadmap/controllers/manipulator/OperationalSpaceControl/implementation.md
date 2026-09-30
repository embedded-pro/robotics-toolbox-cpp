# Operational-Space Control — Implementation Pseudocode

> Roadmap ref: #M18 (Tier 4) · Target: `robotics/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof, std::size_t TaskDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class OperationalSpaceControl:                                # TaskDim = 6 (pose) or ≤ 3 (position; 2 = planar xy)
    const dynamics::EulerLagrangeDynamics<T, Dof>&       model      # M, C q̇, g — typically ChainDynamicsModel (M29)
    const kinematics::JacobianProvider<T, TaskDim, Dof>& jacobian   # J, J̇q̇, tool pose — ChainTaskJacobian (M8)
    math::SquareMatrix<T, TaskDim> Kp        # task-space position gain
    math::SquareMatrix<T, TaskDim> Kd        # task-space damping gain
    T sigma                                  # singularity damping σ (0 ⇒ exact Λ)
```

## Interface

```
OperationalSpaceControl(const dynamics::EulerLagrangeDynamics<T,Dof>& model,
                        const kinematics::JacobianProvider<T,TaskDim,Dof>& jacobian,
                        const SquareMatrix& Kp, const SquareMatrix& Kd, T sigma)

JointVector ComputeTorque(const JointVector& q, const JointVector& qDot,
                          const kinematics::SE3Transform<T>& desiredPose,
                          const TaskVector& xdDot, const TaskVector& xdDdot,
                          const JointVector& tauSecondary)       # hot path
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, desiredPose, xdDot, xdDdot, tauSecondary):   # OPTIMIZE_FOR_SPEED
    J  = jacobian.Jacobian(q)                                   # TaskDim×Dof
    M  = model.ComputeMassMatrix(q)                             # Dof×Dof (SPD)
    # --- task command (TaskError.hpp, see ImpedanceControl: PoseError for 6, position difference for ≤ 3) ---
    eX    = TaskError(desiredPose, jacobian.ToolPose(q))
    eXDot = xdDot − J·qDot
    aX    = xdDdot + Kd·eXDot + Kp·eX
    # --- task-space inertia Λ = (J M⁻¹ Jᵀ + σ²I)⁻¹ ---
    X      = solvers::SolveSystem(M, Jᵀ)                        # M⁻¹Jᵀ, no explicit inverse
    Lambda = solvers::SolveSystem(J·X + σ²·I, I)                # TaskDim×TaskDim (small)
    # --- dynamically-consistent null-space projector ---
    JbarT = Lambda·Xᵀ                                           # J̄ᵀ = Λ J M⁻¹  (M symmetric)
    N     = I − Jᵀ·JbarT                                        # Dof×Dof
    # --- torque: task term + FULL joint-space C q̇ + g + projected secondary torque ---
    return Jᵀ·Lambda·(aX − jacobian.BiasAcceleration(q, qDot))  # J̇q̇
           + model.ComputeCoriolisTerms(q, qDot)
           + model.ComputeGravityTerms(q)
           + N·tauSecondary
```

## Complexity & memory

- Time: `O(TaskDim·Dof³)` for the `M⁻¹Jᵀ` solve with `SolveSystem` (it eliminates per column; factoring
  `M` once with `math::CholeskyDecomposition` gives `O(Dof³ + TaskDim·Dof²)`); `O(TaskDim³)` for `Λ`;
  `O(Dof²·TaskDim)` for the projector.
- Memory: `O(Dof²)` working matrices; all stack/static, no heap.

## Numerical / embedded notes

- **Task dynamics.** With `σ = 0`: `ẍ = J q̈ + J̇q̇ = aX` exactly, and `J·M⁻¹·N = 0`, so `tauSecondary`
  produces **no** task-space acceleration.
- **Why `C q̇ + g` in joint space.** The classic form `F = Λ(aX − J̇q̇) + J̄ᵀC q̇ + J̄ᵀg`, `τ = JᵀF + Nτ₀`
  compensates only `JᵀJ̄ᵀ(C q̇ + g)`; on a redundant arm (`Dof > TaskDim`) the null-space part
  `N·(C q̇ + g)` is left uncompensated and the self-motion sags. The law above equals the classic form
  plus `N·(C q̇ + g)`; task behaviour is unchanged, and at rest with zero error `τ = g` exactly.
- Solve `M·X = Jᵀ` with `solvers::SolveSystem` (multi-column Gaussian elimination) rather than forming
  `M⁻¹` — cheaper and better-conditioned.
- `Λ = (J M⁻¹ Jᵀ)⁻¹` blows up at kinematic singularities (`J` loses rank); near
  `det(J M⁻¹ Jᵀ) → 0` use σ > 0 (adds `σ²I` inside the inverse). With σ > 0 the task is tracked only
  approximately and `J·M⁻¹·N ≈ 0`; with σ = 0 a rank-deficient `J` violates `SolveSystem`'s pivot assert.
- For `TaskDim = 6` the rotation-vector error rate equals `ωd − ω` only to first order about `eR = 0`.
- `Λ` and `J̄` are small (`TaskDim`-wide) — cache them when the task dimension ≪ `Dof`.
- The injected interfaces are a virtual seam for StrictMock tests; hard real-time builds may template
  the controller on the concrete `ChainDynamicsModel` / `ChainTaskJacobian` to avoid virtual calls
  (AGENTS.md: no virtual calls in real-time paths).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/controllers/manipulator/OperationalSpaceControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class OperationalSpaceControl<float, 2, 2>;` / `<float, 3, 2>` / `<float, 7, 6>`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/controllers/manipulator/OperationalSpaceControl.cpp` →
  `template class OperationalSpaceControl<float, 2, 2>;` / `<float, 3, 2>` / `<float, 7, 6>`
- Test: `robotics/controllers/manipulator/test/TestOperationalSpaceControl.cpp`
- Doc: `doc/controllers/manipulator/OperationalSpaceControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestOperationalSpaceControl.cpp` → the `_test` target.
- New module: create `robotics/controllers/manipulator/CMakeLists.txt` via `robotics_add_header_library(...)`,
  add a `test/` subdir, register it in `robotics/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Depends on: M6 (`SE3Transform`), M8 (`JacobianProvider`), M29 (`ChainDynamicsModel`); `TaskError.hpp`
  (specified in M17, deployed with whichever of M17/M18/M19 lands first); `solvers::SolveSystem` from
  [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
