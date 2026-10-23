# Dynamic (Base-Parameter) Identification — Implementation Pseudocode

> Roadmap ref: #M22 (Tier 3) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

Estimates the inertial parameters of a real arm from measured joint positions, velocities,
accelerations and torques recorded along an exciting trajectory. The rigid-body model is linear in
the parameters, `τ = Y(q, q̇, q̈)·π` (M31 regressor with `q̇r = q̇`, `q̈r = q̈`), so this is a linear
least-squares problem — but only certain linear combinations of `π` (the **base parameters**) are
identifiable, so the solver must handle a rank-deficient regressor. The streaming, rank-revealing least
squares is upstream `estimators::IncrementalQrLeastSquares` (NT63); this item feeds it the regressor and
interprets the result.

## Data structures

```cpp
template<typename T, std::size_t Dof>                   # static_assert(std::is_floating_point_v<T>); instantiated for float
class DynamicParameterIdentification:
    static constexpr std::size_t P = 10 * Dof           # full parameter count (M31 ordering)
    const InertialRegressor<T, Dof, P>& regressor       # ChainInertialRegressor (M31)
    estimators::IncrementalQrLeastSquares<T, P> leastSquares   # NT63 (STATE: P×P factor, no samples stored)

struct IdentificationResult:
    math::Vector<T, P>  parameters          # minimum-norm solution (identifiable projection of π)
    std::size_t         rank                # number of base parameters
    math::Matrix<T, P, P> baseDirections    # first `rank` rows span the identifiable combinations
    T                   residualRms
```

## Interface

```cpp
void AddSample(const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
               const JointVector& tauMeasured)          # streaming, O(Dof·P²)
std::optional<IdentificationResult> Solve(T relativeTolerance) const   # nullopt if no samples
JointVector Predict(const IdentificationResult&, q, q̇, q̈) const       # Y·π̂ for validation
void Reset()
```

## Algorithm (pseudocode)

```text
function AddSample(q, q̇, q̈, τ):
    leastSquares.AddRows(regressor.Compute(q, q̇, q̇, q̈), τ)     # Dof rows of the P-column regressor

function Solve(tol):
    r = leastSquares.Solve(tol);  if !r: return nullopt
    return { r.parameters, r.rank, r.rowSpace, r.residualRms }    # rowSpace rows < rank = base directions

function Predict(result, q, q̇, q̈):
    return regressor.Compute(q, q̇, q̇, q̈) · result.parameters
```

## Complexity & memory

- `AddSample`: `O(Dof·P²)` Givens rotations; `Solve`: one `P×P` SVD (upstream).
- Memory: `P² + P` scalars regardless of the number of samples (e.g. 6-DOF: `P = 60`, about 14 KB in
  `float`); no heap. This is an offline / commissioning routine, not an ISR path.

## Numerical / embedded notes

- Excitation matters more than the solver: use a periodic (finite Fourier series) trajectory designed
  to minimise the condition number of the stacked regressor; report `σ_max/σ_min` of the kept part.
- Filter `q̇`, `q̈` and `τ` with the same zero-phase low-pass filter (offline) before `AddSample`.
- The regressor columns mix units (mass, first and second moments): scale them before accumulating so the
  relative rank tolerance is meaningful.
- The minimum-norm estimate is not guaranteed physically consistent (positive mass, valid inertia);
  enforcing that needs a constrained (LMI) fit — out of scope, noted for future work.
- Friction (NT57) parameters can be appended to the regressor as extra columns (`sign(q̇)`, `q̇`).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/dynamics/DynamicParameterIdentification.hpp` — `#pragma once`,
  `extern template class DynamicParameterIdentification<float, 2>;`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/dynamics/DynamicParameterIdentification.cpp` → the same instantiation.
- Test: `robotics/dynamics/test/TestDynamicParameterIdentification.cpp`
- Doc: `doc/dynamics/DynamicParameterIdentification.md` (per `doc/TEMPLATE.md`)
- Depends on: M31 (regressor); NT63 (`estimators::IncrementalQrLeastSquares`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
