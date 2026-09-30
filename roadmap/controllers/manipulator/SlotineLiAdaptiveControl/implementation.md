# Slotine-Li Adaptive Control — Implementation Pseudocode

> Roadmap ref: #M20 (Tier 4) · Target: `robotics/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```cpp
template<typename T, std::size_t Dof, std::size_t NumParams>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class SlotineLiAdaptiveControl:
    const dynamics::InertialRegressor<T, Dof, NumParams>& regressor  # Y(q,q̇,q̇r,q̈r) — M31
    math::SquareMatrix<T, Dof>       Lambda   # sliding-surface slope (SPD)
    math::SquareMatrix<T, Dof>       Kd       # sliding-variable gain  (SPD)
    math::SquareMatrix<T, NumParams> Gamma    # adaptation gain        (SPD)
    math::Vector<T, NumParams>       aHat     # inertial-parameter estimate (STATE)
    T dt
```

## Interface

```text
# Regressor injected (DIP): supplies Y such that  M(q)q̈r + C(q,q̇)q̇r + g(q) = Y·a,
# C the Christoffel-consistent Coriolis matrix (M31, dynamics::InertialRegressor)
SlotineLiAdaptiveControl(const dynamics::InertialRegressor<T,Dof,NumParams>& regressor, const SquareMatrix& Lambda,
                         const SquareMatrix& Kd, const SquareMatrix& Gamma,
                         const Vector& aHat0, T dt)

Vector<T,Dof> ComputeTorque(const StateVector& q,   const StateVector& qDot,
                            const StateVector& qd,   const StateVector& qdDot,
                            const StateVector& qdDdot)         # hot path, updates aHat
const Vector<T,NumParams>& ParameterEstimate() const
void Reset(const Vector& aHat0)
```

## Algorithm (pseudocode)

```text
function ComputeTorque(q, qDot, qd, qdDot, qdDdot):    # OPTIMIZE_FOR_SPEED
    qTilde    = q    - qd
    qTildeDot = qDot - qdDot
    # sliding variable and reference (not measured) motion — no q̈ measurement needed:
    s      = qTildeDot + Lambda * qTilde
    qrDot  = qdDot  - Lambda * qTilde
    qrDdot = qdDdot - Lambda * qTildeDot
    # linearity-in-parameters:  M q̈r + C q̇r + g = Y·a
    Y = regressor.Compute(q, qDot, qrDot, qrDdot)      # Dof×NumParams
    # control: model feedforward on the current estimate + sliding-variable damping
    tau = Y * aHat - Kd * s
    # passivity-based parameter update  (integrate  â̇ = −Γ Yᵀ s):
    aHat = aHat - (Gamma * transpose(Y) * s) * dt
    return tau
```

## Complexity & memory

- Time: `O(Dof·NumParams)` for `Y·aHat` and `Yᵀ·s`; regressor build is `O(Dof·NumParams)`
  (`ChainInertialRegressor`, M31, `NumParams = 10·N`).
- Memory: `O(NumParams)` parameter state + `O(Dof²)` gains; no heap.

## Numerical / embedded notes

- **No acceleration measurement:** `Y` is evaluated at the *reference* `q̈r`, not measured `q̈` — the
  key robustness advantage over direct inverse-dynamics identification.
- Global tracking convergence via Lyapunov `V = ½sᵀM(q)s + ½ãᵀΓ⁻¹ã` (SPD `M`, `ã = âHat − a`):
  `s → 0` hence `q̃ → 0`; parameters stay **bounded** but converge only under persistent excitation.
- **The proof needs `Ṁ − 2C` skew-symmetric**, so `Y` must use the *Christoffel-consistent* `C(q,q̇)`
  applied to the reference velocity, `C(q,q̇)q̇r`. Plain RNEA (`ChainDynamicsModel`, M29) only returns the
  vector `C(q,q̇)q̇` for a single velocity and cannot provide it; use the regressor of
  `roadmap/dynamics/CoriolisMatrixAndRegressor` (M31).
- Add a **projection** (or dead-zone) so `aHat` stays in a physically valid set (positive masses)
  and does not drift under sensor noise or unmodelled dynamics.
- `Λ` sets the error-manifold bandwidth, `Kd` damps `s`, `Γ` sets adaptation speed — too large `Γ`
  causes estimate oscillation and can excite unmodelled modes.
- Sibling of `ComputedTorqueControl` (M12, non-adaptive). The interface is a virtual seam for StrictMock
  tests; hard real-time builds may template the controller on the concrete `ChainInertialRegressor`
  to avoid the virtual call (AGENTS.md: no virtual calls in real-time paths).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/controllers/manipulator/SlotineLiAdaptiveControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class SlotineLiAdaptiveControl<float, 2, 3>;` / `<float, 2, 20>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD` (`20 = 10·N` matches `ChainInertialRegressor<float, 2>`).
- Coverage: `robotics/controllers/manipulator/SlotineLiAdaptiveControl.cpp` →
  `template class SlotineLiAdaptiveControl<float, 2, 3>;` / `<float, 2, 20>`
- Test: `robotics/controllers/manipulator/test/TestSlotineLiAdaptiveControl.cpp`
- Doc: `doc/controllers/manipulator/SlotineLiAdaptiveControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestSlotineLiAdaptiveControl.cpp` → the `_test` target.
- New module: create `robotics/controllers/manipulator/CMakeLists.txt` via `robotics_add_header_library(...)`,
  add a `test/` subdir, register it in `robotics/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Depends on: M31 (`InertialRegressor`, `ChainInertialRegressor`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
