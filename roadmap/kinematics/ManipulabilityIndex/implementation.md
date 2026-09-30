# Manipulability Index (Yoshikawa) — Implementation Pseudocode

> Roadmap ref: #M11 (Tier 2) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```cpp
template<typename T, std::size_t TaskDim, std::size_t Dof>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class ManipulabilityIndex:
    const JacobianProvider<T, TaskDim, Dof>& jacobian        # M8 seam, injected
    # measured blocks share one physical unit:
    static constexpr std::size_t Rows = (TaskDim == 6) ? 3 : TaskDim   # TaskDim = 6 → linear / angular blocks
    static constexpr std::size_t Axes = min(Rows, Dof)
```

## Interface

```cpp
explicit ManipulabilityIndex(const JacobianProvider<T, TaskDim, Dof>& jacobian)
T                    Compute(const JointVector& q) const            # w of rows 0..Rows−1 (translational for TaskDim 3/6); hot path
T                    ComputeRotational(const JointVector& q) const requires (TaskDim == 6)   # rows 3–5
std::array<T, Axes>  EllipsoidAxes(const JointVector& q) const      # singular values of the Compute block, descending
T                    ConditionNumber(const JointVector& q) const    # σ_max / σ_min of that block; +∞ when σ_min ≤ ε
bool                 NearSingular(const JointVector& q, T eps) const   # Compute(q) < eps
```

## Algorithm (pseudocode)

```text
function GramRoot(B):                           # B: Rows × Dof, one unit
    if Rows ≤ Dof:  G = B·Bᵀ                    # Rows × Rows: task-space ellipsoid volume
    else:           G = Bᵀ·B                    # Dof × Dof: B·Bᵀ would be rank ≤ Dof ⇒ det ≡ 0
    lu = solvers::LuDecomposition<T, dim(G)>
    if !lu.Decompose(G): return 0               # upstream flags pivot < 1e-6·max ⇒ σ_min/σ_max ≲ 1e-3
    return sqrt(max(0, lu.Determinant()))       # clamp −0 from rounding

function Compute(q):                            # OPTIMIZE_FOR_SPEED
    J = jacobian.Jacobian(q)                    # TaskDim × Dof
    return GramRoot(J.rows(0 .. Rows−1))

function ComputeRotational(q):                  # TaskDim = 6
    return GramRoot(jacobian.Jacobian(q).rows(3 .. 5))

function EllipsoidAxes(q):
    B = jacobian.Jacobian(q).rows(0 .. Rows−1)
    svd = solvers::SingularValueDecomposition<T, max(Rows, Dof), min(Rows, Dof)>
    svd.Decompose(Rows ≥ Dof ? B : Bᵀ)           # upstream requires Rows ≥ Cols; σ(Bᵀ) = σ(B)
    return svd.SingularValues()                  # descending; ∏ σᵢ = GramRoot(B)

function ConditionNumber(q):
    σ = EllipsoidAxes(q)
    return σ_min ≤ ε·σ_max ? +∞ : σ_max / σ_min  # upstream ConditionNumber() returns 0 when singular — map to +∞

function NearSingular(q, eps):
    return Compute(q) < eps
```

## Complexity & memory

- `Compute`: `O(Rows·Dof·min(Rows, Dof))` for the Gram product + `O(min(Rows, Dof)³)` LU.
- `EllipsoidAxes` / `ConditionNumber`: one Golub–Kahan SVD of a `max × min` matrix.
- Memory: one `TaskDim×Dof` Jacobian, one Gram matrix or SVD factors; stack only.

## Numerical / embedded notes

- **Branch choice matters:** with fewer joints than measured rows (`Dof < Rows`), `det(B·Bᵀ)` is
  identically zero; `√det(Bᵀ·B)` is the volume of the `Dof`-dimensional ellipsoid actually reachable.
  For the 2-link planar arm with position rows this gives `|sin q₂|`.
- **Units:** linear rows are in m/s per unit rate, angular rows in rad/s. For `TaskDim = 6` the two
  blocks are measured separately; a single 6-row determinant mixes units and depends on the length unit.
- A planar arm with `TaskDim = 3` and `Dof ≥ 3` has a zero `z` row ⇒ `w ≡ 0`; use a `TaskDim = 2`
  provider (planar position rows) for planar tasks.
- `det(G) = (∏ σᵢ)²`; when conditioning or axes are needed use the SVD (upstream
  `numerical/solvers/SingularValueDecomposition.hpp`,
  [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)) — it avoids the
  squaring of `G` (the Gram/LU path reports `0` once `σ_min/σ_max ≲ 1e-3`; `∏ EllipsoidAxes` stays
  accurate below that). For `Rows = Dof`, `w = |det B|` directly.
- Use `NearSingular` as a guard *before* any Jacobian inverse.
- `ComputeRotational` is constrained with `requires` (C++20), not `static_assert`, so explicit
  coverage instantiations with `TaskDim ≠ 6` still compile.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/ManipulabilityIndex.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class ManipulabilityIndex<float, 3, 2>;` / `<float, 2, 3>` / `<float, 6, 2>` /
  `<float, 6, 6>` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/ManipulabilityIndex.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestManipulabilityIndex.cpp`
- Doc: `doc/kinematics/ManipulabilityIndex.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestManipulabilityIndex.cpp` → the `_test` target.
- Depends on: M8 (`JacobianProvider`); upstream `solvers::SingularValueDecomposition`, `solvers::LuDecomposition`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
