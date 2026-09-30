# Impedance Control — Implementation Pseudocode

> Roadmap ref: #M17 (Tier 3) · Target: `robotics/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

Sign convention: `fExternal` is the wrench exerted **by the environment on the tool**, base frame,
`(f; n)` about the tool point (M6 ordering), so the arm obeys `M q̈ + C q̇ + g = τ + Jᵀ·fExternal`.
Task error `e = xd ⊖ x` (`TaskError`, below); `x̃ = x − xd = −e`.

## Data structures

```
template<typename T, std::size_t Dof, std::size_t TaskDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class ImpedanceControl:                                       # TaskDim = 6 (pose) or ≤ 3 (position; 2 = planar xy)
    const dynamics::EulerLagrangeDynamics<T, Dof>&       model      # M, C q̇, g — typically ChainDynamicsModel (M29)
    const kinematics::JacobianProvider<T, TaskDim, Dof>& jacobian   # J, J̇q̇, tool pose — ChainTaskJacobian (M8)
    math::SquareMatrix<T, TaskDim> K          # stiffness (SPD)
    math::SquareMatrix<T, TaskDim> D          # damping (SPD) — the target Dd in (b)
    math::SquareMatrix<T, TaskDim> MdInverse  # target inertia Md⁻¹, precomputed once: SolveSystem(Md, I)

# shared by M17/M18/M19 — robotics/controllers/manipulator/TaskError.hpp
template<typename T, std::size_t TaskDim>
math::Vector<T, TaskDim> TaskError(const kinematics::SE3Transform<T>& desired,
                                   const kinematics::SE3Transform<T>& current)
```

## Interface

```
ImpedanceControl(const dynamics::EulerLagrangeDynamics<T,Dof>& model,
                 const kinematics::JacobianProvider<T,TaskDim,Dof>& jacobian,
                 const SquareMatrix& K, const SquareMatrix& D, const SquareMatrix& Md)

# (a) stiffness/damping impedance — no force sensor, no mass matrix:
JointVector ComputeTorque(const JointVector& q, const JointVector& qDot,
                          const kinematics::SE3Transform<T>& desiredPose,
                          const TaskVector& xdDot)                               # hot path

# (b) full impedance with inertia shaping — needs M, J̇q̇ and the measured fExternal:
JointVector ComputeTorqueWithInertiaShaping(const JointVector& q, const JointVector& qDot,
                          const kinematics::SE3Transform<T>& desiredPose,
                          const TaskVector& xdDot, const TaskVector& xdDdot,
                          const TaskVector& fExternal)                           # hot path
```

## Algorithm (pseudocode)

```
function TaskError(desired, current):                    # never subtracts orientations
    if TaskDim == 6: return kinematics::SE3Transform<T>::PoseError(desired, current)   # (Δp; rotation vector)
    return first TaskDim entries of (desired.p − current.p)

function ComputeTorque(q, qDot, desiredPose, xdDot):     # (a) OPTIMIZE_FOR_SPEED
    J    = jacobian.Jacobian(q)
    e    = TaskError(desiredPose, jacobian.ToolPose(q))
    eDot = xdDot − J·qDot
    return Jᵀ·(K·e + D·eDot) + model.ComputeGravityTerms(q)     # C q̇ deliberately NOT added

function ComputeTorqueWithInertiaShaping(q, qDot, desiredPose, xdDot, xdDdot, fExt):   # (b) OPTIMIZE_FOR_SPEED
    J    = jacobian.Jacobian(q)
    e    = TaskError(desiredPose, jacobian.ToolPose(q))
    eDot = xdDot − J·qDot
    # commanded task acceleration realising  Md·x̃̈ + D·x̃̇ + K·x̃ = fExt
    aX   = xdDdot + MdInverse·(D·eDot + K·e + fExt)
    # Λ = (J M⁻¹ Jᵀ)⁻¹ applied through solves — M⁻¹ and Λ never formed explicitly
    X    = solvers::SolveSystem(model.ComputeMassMatrix(q), Jᵀ)                  # M⁻¹Jᵀ (Dof×TaskDim)
    F    = solvers::SolveSystem(J·X, aX − jacobian.BiasAcceleration(q, qDot)) − fExt   # Λ(aX − J̇q̇) − fExt
    return Jᵀ·F + model.ComputeCoriolisTerms(q, qDot) + model.ComputeGravityTerms(q)
```

## Complexity & memory

- (a): `O(TaskDim·Dof)` for `J·q̇` and `Jᵀ·F`, plus one `O(Dof)` gravity pass.
- (b): adds `M` (`O(Dof²)`, CRBA), the `M⁻¹Jᵀ` solve (`O(TaskDim·Dof³)` with `SolveSystem`, which eliminates
  per column; factor `M` once with `math::CholeskyDecomposition` for `O(Dof³ + TaskDim·Dof²)`) and a
  `TaskDim×TaskDim` solve.
- Memory: `O(TaskDim²)` gains, `O(Dof·TaskDim)` workspace; no dynamic state, no heap.

## Numerical / embedded notes

- **(a) contact equilibrium.** Closed loop `M q̈ + C q̇ = Jᵀ(K·e + D·ė + fExt)`. At rest with `J` full rank:
  `K·e = −fExt` ⇒ `x − xd = K⁻¹·fExt` — the tool yields *along* the external force with the programmed
  compliance. The inertia felt at the tool is the arm's own `Λ(q)`; `Md` is unused by (a).
- **(a) passivity.** `C q̇` is not added: with `q̇ᵀ(Ṁ − 2C)q̇ = 0`, `V = ½q̇ᵀMq̇ + ½eᵀKe` gives
  `V̇ = −ẋᵀD ẋ + ẋᵀfExt` for constant `xd` (position tasks) ⇒ passive port `(ẋ, fExt)`, stable against
  any passive environment. Cancelling `C q̇` would leave the indefinite term `½q̇ᵀṀq̇`.
- **(b) rendered impedance.** `ẍ = J q̈ + J̇q̇ = Λ⁻¹(F + fExt) + J̇q̇ = aX` ⇒ `Md·x̃̈ + D·x̃̇ + K·x̃ = fExt`
  exactly for position tasks. For `TaskDim = 6`, `ωd − ω` is the rate of the rotation-vector error only
  to first order, so the rotational impedance is exact to first order about `eR = 0`.
- **(b) equivalent classic form.** Equal to `F = Λ(aX − J̇q̇) + J̄ᵀC q̇ + J̄ᵀg − fExt`, `τ = JᵀF`
  (`J̄ = M⁻¹JᵀΛ`) plus `N·(C q̇ + g)`, `N = I − JᵀJ̄ᵀ`: that extra term is zero for a non-redundant arm and
  compensates null-space gravity/Coriolis on a redundant one (as in `OperationalSpaceControl`).
- **(b) with `Md = Λ(q)`** the `fExt` terms cancel: `τ = Jᵀ(K·e + D·ė) + g + [JᵀΛ(ẍd − J̇q̇) + C q̇]` —
  law (a) plus feed-forward, no force sensor needed. Shaping `Md ≠ Λ` requires a measured `fExt`
  (low-pass it and match `D` to the sensor bandwidth); sensor bias shifts the rendered equilibrium.
- (b) needs `Λ`, so it inherits the singularity caveat of `OperationalSpaceControl` (damp
  `J M⁻¹ Jᵀ + σ²I`); (a) uses only `Jᵀ` and passes through singularities safely.
- **Admittance** is the dual: measure `fExt`, integrate to a motion command, feed a position loop —
  better on stiff/non-backdrivable robots; impedance is better on backdrivable ones.
- Diagonal `K`/`D` are chosen per task axis (e.g. stiff normal, compliant tangential for insertion).
- The injected interfaces are a virtual seam for StrictMock tests; hard real-time builds may template
  the controller on the concrete `ChainDynamicsModel` / `ChainTaskJacobian` to avoid virtual calls
  (AGENTS.md: no virtual calls in real-time paths).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/controllers/manipulator/ImpedanceControl.hpp` (+ `TaskError.hpp`, shared with M18/M19;
  deploy it with whichever lands first) — `#pragma once` → `#pragma GCC optimize("O3","fast-math")`,
  `OPTIMIZE_FOR_SPEED` on both torque methods, and `extern template class ImpedanceControl<float, 2, 2>;` /
  `<float, 6, 6>` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/controllers/manipulator/ImpedanceControl.cpp` →
  `template class ImpedanceControl<float, 2, 2>;` / `<float, 6, 6>`
- Test: `robotics/controllers/manipulator/test/TestImpedanceControl.cpp`
- Doc: `doc/controllers/manipulator/ImpedanceControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestImpedanceControl.cpp` → the `_test` target.
- New module: create `robotics/controllers/manipulator/CMakeLists.txt` via `robotics_add_header_library(...)`,
  add a `test/` subdir, register it in `robotics/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Depends on: M6 (`SE3Transform`), M8 (`JacobianProvider`), M29 (`ChainDynamicsModel`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
