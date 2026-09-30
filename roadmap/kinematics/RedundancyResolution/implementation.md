# Redundancy Resolution (Null-Space Projection) — Implementation Pseudocode

> Roadmap ref: #M14 (Tier 3) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```cpp
template<typename T, std::size_t TaskDim, std::size_t Dof>   # static_assert(std::is_floating_point_v<T>); static_assert(Dof > TaskDim)
class RedundancyResolution:                                    # instantiated for float
    const JacobianProvider<T, TaskDim, Dof>& jacobian         # M8 seam; TaskDim 3 (position) or 6 (pose)
    T  damping                                                 # λ — primary term only
    T  rankTolerance                                           # relative: σᵢ > rankTolerance·σ₀ counts toward rank
    solvers::SingularValueDecomposition<T, Dof, TaskDim> svd   # of Jᵀ (upstream needs Rows ≥ Cols)
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
function Factor(q):
    J = jacobian.Jacobian(q)                    # TaskDim × Dof
    svd.Decompose(Jᵀ)                           # Jᵀ = U·Σ·Vᵀ ⇒ J = V·Σ·Uᵀ
    # uᵢ = column i of U (Dof): right singular vectors of J;  vᵢ = column i of V (TaskDim)
    τ = rankTolerance · σ₀;  r = #{ σᵢ > τ }

function Gain(σ):                               # damped inverse of one singular value
    λ > 0 ? σ / (σ² + λ²) : (σ > τ ? 1/σ : 0)

function Resolve(q, ẋ, q̇₀):                    # OPTIMIZE_FOR_SPEED
    Factor(q)
    q̇ = Σᵢ Gain(σᵢ)·(vᵢ·ẋ)·uᵢ                  # primary: J⁺_λ·ẋ, lies in the row space of J
    q̇ = q̇ + q̇₀ − Σ_{i<r} (uᵢ·q̇₀)·uᵢ           # secondary: P·q̇₀ without forming P, O(Dof·r)
    return q̇

function DampedPseudoInverse(q):   Factor(q);  return Σᵢ Gain(σᵢ)·uᵢ·vᵢᵀ
function NullSpaceProjector(q):    Factor(q);  return I_Dof − Σ_{i<r} uᵢ·uᵢᵀ
                                   # = I − J⁺J with J⁺ = Transpose(svd.PseudoInverse(τ))
```

## Complexity & memory

- One Golub–Kahan SVD of the `Dof × TaskDim` matrix `Jᵀ` per call, then `O(Dof·TaskDim)`.
- `NullSpaceProjector` (diagnostics/tests) adds `O(Dof²·r)`; `Resolve` never forms it.
- Memory: SVD factors (`Dof×TaskDim`, `TaskDim×TaskDim`) + vectors; stack only.

## Numerical / embedded notes

- **Why the projector is undamped:** with the damped inverse, `J·(I − J⁺_λJ) = λ²(JJᵀ + λ²I)⁻¹J ≠ 0`, so
  `I − J⁺_λJ` is neither idempotent nor task-invisible (7-DOF fixture, `λ = 0.01`: `‖J·P_λ·q̇₀‖ = 2.8e-4`,
  `‖P_λ² − P_λ‖ = 2.0e-3`). Built from the thresholded SVD, `P` is an exact orthogonal projector:
  `P² = P`, `Pᵀ = P`, `J·P = Σ_{σᵢ ≤ τ} σᵢvᵢuᵢᵀ` (zero for exact rank loss, rounding-level at full rank).
- **What the tool sees:** `J·q̇ = Σᵢ σᵢ·Gain(σᵢ)·vᵢvᵢᵀ·ẋ`. With `λ > 0` the primary task is met only
  approximately: `‖J·q̇ − ẋ‖ ≤ λ²/(σ_min² + λ²)·‖ẋ‖` for `ẋ` in the range of `J`. The secondary term
  adds nothing to the task velocity regardless of `λ` or `‖q̇₀‖`.
- The primary term lies in the row space (`P·J⁺_λ·ẋ = 0`), so it is the minimum-norm solution.
- Near a singularity `σ_min → 0`: the damped gain stays bounded (`≤ 1/(2λ)`), and the rank threshold
  grows the null space (the lost task direction is not "protected" — by design).
- Common `q̇₀`: gradient of manipulability (M11), distance to joint limits, obstacle potentials.
- SVD: upstream `numerical/solvers/SingularValueDecomposition.hpp`
  ([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/RedundancyResolution.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Resolve`, and
  `extern template class RedundancyResolution<float, 6, 7>;` / `<float, 3, 7>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/RedundancyResolution.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestRedundancyResolution.cpp`
- Doc: `doc/kinematics/RedundancyResolution.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestRedundancyResolution.cpp` → the `_test` target.
- Depends on: M8 (`JacobianProvider`); upstream `solvers::SingularValueDecomposition`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
