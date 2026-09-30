# Forward-Dynamics Integrator (Simulation Step) — Implementation Pseudocode

> Roadmap ref: #M34 (Tier 2) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

A fixed-step simulator for a link chain: state `(q, q̇)`, dynamics `q̈ = ABA(q, q̇, τ)`. The simulator
application currently hand-rolls this; a library version makes model-in-the-loop tests of the
controllers (M5, M12, M16–M20) reproducible.

## Data structures

```
enum class IntegrationMethod : uint8_t { SemiImplicitEuler, RungeKutta4 }

template<typename T, std::size_t NumLinks>        # static_assert(std::is_floating_point_v<T>); instantiated for float
struct ChainState:
    JointVector q, qDot, qDDot                    # qDDot of the last evaluation (for logging / RNEA checks)

template<typename T, std::size_t NumLinks, IntegrationMethod Method>
class ForwardDynamicsIntegrator:
    const LinkArray& links
    Vector3 gravity
    ArticulatedBodyAlgorithm<T, NumLinks> aba
```

## Interface

```
ForwardDynamicsIntegrator(const LinkArray& links, const Vector3& gravity)
ChainState Step(const ChainState& state, const JointVector& tau, T dt) const     # hot path, τ held over dt
```

## Algorithm (pseudocode)

```
function Step(x, τ, dt):                         # OPTIMIZE_FOR_SPEED
    if Method == SemiImplicitEuler:              # symplectic: bounded energy error
        a = aba.ForwardDynamics(links, x.q, x.q̇, τ, gravity)
        q̇ = x.q̇ + a·dt
        q  = x.q + q̇·dt
    else:  # RungeKutta4 on (q, q̇), zero-order-hold τ
        f(q, q̇) = (q̇, aba.ForwardDynamics(links, q, q̇, τ, gravity))
        k1 = f(x);  k2 = f(x + k1·dt/2);  k3 = f(x + k2·dt/2);  k4 = f(x + k3·dt)
        (q, q̇) = x + (k1 + 2k2 + 2k3 + k4)·dt/6
        a = k1.acceleration                      # acceleration at the step start
    return { q, q̇, a }
```

## Complexity & memory

- Semi-implicit Euler: one ABA pass (`O(N)`) per step; RK4: four.
- Memory: two state copies; no heap.

## Numerical / embedded notes

- RK4 can reuse the upstream `solvers::RungeKutta4<T, 2N, N>` with an `OdeSystem` adapter whose
  `Derivative` evaluates ABA (numerical-toolbox-cpp); the virtual call is acceptable off the real-time path.
- Semi-implicit Euler keeps energy error bounded (no secular drift) at first-order accuracy; RK4 is
  fourth-order but slowly dissipative/drifting over long runs — choose by use case.
- Joint friction or damping belongs in `τ` (e.g. `τ − b·q̇`), not as ad-hoc velocity scaling.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/dynamics/ForwardDynamicsIntegrator.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Step`, `extern template` for
  `<float, 2, IntegrationMethod::SemiImplicitEuler>` and `<float, 2, IntegrationMethod::RungeKutta4>`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/dynamics/ForwardDynamicsIntegrator.cpp` → the same instantiations.
- Test: `robotics/dynamics/test/TestForwardDynamicsIntegrator.cpp`
- Doc: `doc/dynamics/ForwardDynamicsIntegrator.md` (per `doc/TEMPLATE.md`)
- The simulator's `RobotArmSimulator::StepChain` should switch to this integrator once it exists.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
