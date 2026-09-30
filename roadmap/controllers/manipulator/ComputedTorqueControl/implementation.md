# Computed-Torque Control — Implementation Pseudocode

> Roadmap ref: #M12 (Tier 3) · Target: `robotics/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```cpp
template<typename T, std::size_t Dof>    # static_assert(std::is_floating_point_v<T>); instantiated for float
class ComputedTorqueControl:
    const dynamics::InverseDynamicsModel<T, Dof>& model   # τ = M(q)q̈ + C(q,q̇)q̇ + g(q) in one call (M29)
    math::SquareMatrix<T, Dof> Kp        # position-error gain
    math::SquareMatrix<T, Dof> Kd        # velocity-error gain
```

## Interface

```cpp
# Inverse-dynamics model injected (DIP); gains chosen for the resulting double integrator:
ComputedTorqueControl(const dynamics::InverseDynamicsModel<T,Dof>& model,
                      const SquareMatrix& Kp, const SquareMatrix& Kd)

Vector<T,Dof> ComputeTorque(const StateVector& q,      const StateVector& qDot,
                            const StateVector& qd,     const StateVector& qdDot,
                            const StateVector& qdDdot)          # hot path
```

## Algorithm (pseudocode)

```text
function ComputeTorque(q, qDot, qd, qdDot, qdDdot):     # OPTIMIZE_FOR_SPEED
    e    = qd    - q
    eDot = qdDot - qDot
    # inner-loop joint-space acceleration command (feedforward + PD):
    aq   = qdDdot + Kd * eDot + Kp * e
    # inverse dynamics at the measured state and the commanded acceleration:
    #   τ = M(q)·aq + C(q,q̇)q̇ + g(q)   — ONE O(n) RNEA pass, M is never formed
    return model.ComputeInverseDynamics(q, qDot, aq)
```

## Complexity & memory

- Time: one `ComputeInverseDynamics` call — `O(Dof)` with `ChainDynamicsModel` (single RNEA pass) —
  plus `O(Dof²)` for the gain products (`O(Dof)` when the gains are diagonal). No `Dof×Dof` mass matrix
  is built or multiplied.
- Memory: `O(Dof²)` for the two gains; no dynamic state, no heap.

## Numerical / embedded notes

- Substituting `τ` into `M q̈ + Cq̇ + g = τ` gives the **decoupled** linear error dynamics
  `ë + Kd·ė + Kp·e = 0` — every joint becomes an independent, tunable second-order system.
- `M(q)` is neither formed nor inverted: RNEA evaluated with `q̈ = aq` returns `M·aq + Cq̇ + g`
  directly, so the hot path is `O(n)` and well-conditioned (unlike forward dynamics).
- The model is `dynamics::InverseDynamicsModel` (M29), implemented by `ChainDynamicsModel`. The interface
  is a virtual seam for StrictMock tests; hard real-time builds may template the controller on the
  concrete `ChainDynamicsModel` to avoid the virtual call (AGENTS.md: no virtual calls in real-time paths).
- Model error leaves a residual (`ë + Kd·ė + Kp·e = M⁻¹Δ`); pair with an integral, robust
  (sliding-mode), or adaptive (`SlotineLiAdaptiveControl`) term to reject it.
- Choose `Kd = 2√Kp` per channel for a critically-damped response.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/controllers/manipulator/ComputedTorqueControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class ComputedTorqueControl<float, 2>;` / `<float, 3>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/controllers/manipulator/ComputedTorqueControl.cpp` →
  `template class ComputedTorqueControl<float, 2>;` / `<float, 3>`
- Test: `robotics/controllers/manipulator/test/TestComputedTorqueControl.cpp`
- Doc: `doc/controllers/manipulator/ComputedTorqueControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestComputedTorqueControl.cpp` → the `_test` target.
- New module: create `robotics/controllers/manipulator/CMakeLists.txt` via `robotics_add_header_library(...)`,
  add a `test/` subdir, register it in `robotics/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Depends on: M29 (`InverseDynamicsModel`, `ChainDynamicsModel`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
