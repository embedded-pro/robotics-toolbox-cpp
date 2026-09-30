# Generalized-Momentum Observer (Collision Detection) — Implementation Pseudocode

> Roadmap ref: #M16 (Tier 3) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

Estimates the external joint torque from commanded torque and joint state only — no joint-torque
sensors and no acceleration measurement — so contacts and collisions can be detected on-line.

## Data structures

```
template<typename T, std::size_t Dof>              # static_assert(std::is_floating_point_v<T>); instantiated for float
class MomentumObserver:
    const EulerLagrangeDynamics<T, Dof>& model     # M(q), g(q) — typically ChainDynamicsModel (M29)
    const CoriolisMatrix<T, Dof>&        coriolis  # Cᵀ(q, q̇) q̇ (M31); the RNEA vector C q̇ is NOT enough
    const LinkArray&                     links
    math::Vector<T, Dof>                 gain      # K_O (diagonal), 1/s
    math::Vector<T, Dof>                 initialMomentum, integral, residual     # STATE
```

## Interface

```
MomentumObserver(model, coriolis, links, gain)
void                 Reset(const JointVector& q, const JointVector& qDot)                       # p(0)
const JointVector&   Update(const JointVector& q, const JointVector& qDot,
                            const JointVector& tauCommanded, T dt)                               # hot path
bool                 Collision(const JointVector& threshold) const                              # |r_i| > threshold_i
```

## Algorithm (pseudocode)

```
# dynamics: ṗ = τ + Cᵀ(q,q̇) q̇ − g(q) + τ_ext,  p = M(q) q̇   (uses Ṁ = C + Cᵀ)
function Reset(q, q̇):
    initialMomentum = M(q)·q̇;  integral = 0;  residual = 0

function Update(q, q̇, τ, dt):                     # OPTIMIZE_FOR_SPEED
    β = g(q) − coriolis.TransposeTimesVelocity(links, q, q̇)
    integral = integral + (τ − β + residual)·dt
    residual = gain ⊙ (M(q)·q̇ − initialMomentum − integral)
    return residual                                 # ṙ = K_O (τ_ext − r): first-order estimate of τ_ext
```

## Complexity & memory

- `Update`: `O(N²)` (mass matrix and `Cᵀq̇`); memory `O(N)` state + `O(N²)` scratch; no heap.

## Numerical / embedded notes

- Subtract a friction model (M4) from `τ` if friction is not part of `M, C, g`; otherwise friction
  shows up as a (false) external torque.
- The residual is a first-order low-pass of `τ_ext` with bandwidth `K_O`; larger gains detect faster
  but amplify model error and velocity noise. Thresholds are set above the residual seen in
  collision-free runs.
- Forward-Euler integration of the observer at the control rate is standard; keep `K_O·dt ≪ 1`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/dynamics/MomentumObserver.hpp` — `#pragma once` → `#pragma GCC optimize("O3","fast-math")`,
  `OPTIMIZE_FOR_SPEED` on `Update`, `extern template class MomentumObserver<float, 2>;` / `<float, 3>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/dynamics/MomentumObserver.cpp` → the same instantiations.
- Test: `robotics/dynamics/test/TestMomentumObserver.cpp`
- Doc: `doc/dynamics/MomentumObserver.md` (per `doc/TEMPLATE.md`)
- Depends on: M28/M29 (M, g), M31 (`Cᵀq̇`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
