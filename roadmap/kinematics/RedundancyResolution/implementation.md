# Redundancy Resolution (Null-Space Projection) — Implementation Pseudocode

> Roadmap ref: #M14 (Tier 2) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

`q̇ = J⁺_λ·ẋ + (I − J⁺J)·q̇₀` for a redundant arm. The linear algebra — damped pseudo-inverse and exact
null-space projector from one SVD, tall or wide — is upstream `solvers::DampedPseudoInverse` (NT62); this
item binds it to the arm's task Jacobian.

## Data structures

```cpp
template<typename T, std::size_t TaskDim, std::size_t Dof>   # static_assert(std::is_floating_point_v<T>); static_assert(Dof > TaskDim)
class RedundancyResolution:                                    # instantiated for float
    const JacobianProvider<T, TaskDim, Dof>& jacobian         # M8 seam; TaskDim 3 (position) or 6 (pose)
    solvers::DampedPseudoInverse<T, TaskDim, Dof> inverse     # NT62: damping λ (primary only), rank tolerance
```

## Interface

```text
RedundancyResolution(const JacobianProvider<T, TaskDim, Dof>& jacobian, T damping, T rankTolerance = 1e-4)
JointVector              Resolve(const JointVector& q, const TaskVector& xDot,
                                 const JointVector& qDot0)                 # hot path
Matrix<T, Dof, TaskDim>  DampedPseudoInverse(const JointVector& q)         # J⁺_λ (λ = 0 ⇒ rank-thresholded J⁺)
Matrix<T, Dof, Dof>      NullSpaceProjector(const JointVector& q)          # P = I − J⁺J, undamped
```

## Algorithm (pseudocode)

```cpp
function Resolve(q, ẋ, q̇₀):                    # OPTIMIZE_FOR_SPEED
    inverse.Factor(jacobian.Jacobian(q))        # one SVD of the TaskDim × Dof Jacobian
    return inverse.Solve(ẋ) + inverse.Project(q̇₀)   # primary (row space) + secondary (null space)

function DampedPseudoInverse(q):   inverse.Factor(jacobian.Jacobian(q));  return inverse.Inverse()
function NullSpaceProjector(q):    inverse.Factor(jacobian.Jacobian(q));  return inverse.NullSpaceProjector()
```

## Complexity & memory

- One SVD of the `TaskDim × Dof` Jacobian per call plus `O(Dof·TaskDim)` (upstream); the diagnostic
  accessors add `O(Dof²·r)`; `Resolve` never forms `P`.
- Memory: the upstream SVD factors; stack only.

## Numerical / embedded notes

- **What the tool sees:** the secondary term adds nothing to the task velocity regardless of `λ` or
  `‖q̇₀‖`, because the projector is built undamped from the thresholded SVD (the damped `I − J⁺_λJ` would
  leak: 7-DOF fixture, `λ = 0.01`: `‖J·P_λ·q̇₀‖ = 2.8e-4`). With `λ > 0` the primary task is met only
  approximately: `‖J·q̇ − ẋ‖ ≤ λ²/(σ_min² + λ²)·‖ẋ‖`.
- Near a singularity `σ_min → 0` the damped gain stays bounded (`≤ 1/(2λ)`) and the rank threshold grows
  the null space (the lost task direction is not "protected" — by design).
- Common `q̇₀`: gradient of manipulability (M11), distance to joint limits, obstacle potentials.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/RedundancyResolution.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Resolve`, and
  `extern template class RedundancyResolution<float, 6, 7>;` / `<float, 3, 7>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/RedundancyResolution.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestRedundancyResolution.cpp`
- Doc: `doc/kinematics/RedundancyResolution.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestRedundancyResolution.cpp` → the `_test` target.
- Depends on: M8 (`JacobianProvider`); NT62 (`solvers::DampedPseudoInverse`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
