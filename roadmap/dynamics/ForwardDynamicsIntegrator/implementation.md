# Forward-Dynamics Integrator (Simulation Step) — Implementation Pseudocode

> Roadmap ref: #M34 (Tier 2) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

A fixed-step simulator for a link chain: state `(q, q̇)`, dynamics `q̈ = ABA(q, q̇, τ)`. The simulator
application currently hand-rolls this; a library version makes model-in-the-loop tests of the
controllers (M5, M12, M16–M20) reproducible. The integrators are upstream — `solvers::SemiImplicitEuler`
(NT58) and `solvers::RungeKutta4` — and this item is the adapter that exposes ABA as their system.

## Data structures

```cpp
enum class IntegrationMethod : uint8_t { SemiImplicitEuler, RungeKutta4 }

template<typename T, std::size_t NumLinks>        # static_assert(std::is_floating_point_v<T>); instantiated for float
struct ChainState:
    JointVector q, qDot, qDDot                    # qDDot of the last evaluation (for logging / RNEA checks)

template<typename T, std::size_t NumLinks>   # implements solvers::SecondOrderSystem<T, NumLinks, NumLinks> (NT58)
class ChainAccelerationSystem:                 # and solvers::OdeSystem<T, 2·NumLinks, NumLinks> for RK4
    const LinkArray& links
    Vector3 gravity
    ArticulatedBodyAlgorithm<T, NumLinks> aba
    Acceleration(q, q̇, τ) = aba.ForwardDynamics(links, q, q̇, τ, gravity)
    Derivative(x = (q; q̇), τ, t) = (q̇; Acceleration(q, q̇, τ))

template<typename T, std::size_t NumLinks, IntegrationMethod Method>
class ForwardDynamicsIntegrator:
    ChainAccelerationSystem<T, NumLinks> system
    solvers::SemiImplicitEuler<T, NumLinks, NumLinks>  or  solvers::RungeKutta4<T, 2·NumLinks, NumLinks>   # by Method
```

## Interface

```text
ForwardDynamicsIntegrator(const LinkArray& links, const Vector3& gravity, T dt)
ChainState Step(const ChainState& state, const JointVector& tau)     # hot path, τ held over dt
```

## Algorithm (pseudocode)

```cpp
function Step(x, τ):                             # OPTIMIZE_FOR_SPEED
    a = system.Acceleration(x.q, x.q̇, τ)         # acceleration at the step start (logged)
    if Method == SemiImplicitEuler:              # symplectic (NT58): bounded energy error
        (q, q̇) = semiImplicitEuler.Step({ x.q, x.q̇ }, τ)
    else:                                        # RungeKutta4 on (q; q̇), zero-order-hold τ
        (q, q̇) = rungeKutta4.Step((x.q; x.q̇), τ, 0)     # time-invariant system
    return { q, q̇, a }
```

## Complexity & memory

- Semi-implicit Euler: one ABA pass (`O(N)`) per step plus the logged one; RK4: four plus one.
- Memory: two state copies; no heap.

## Numerical / embedded notes

- Both integrators call the system through a virtual interface; acceptable off the real-time path
  (simulation, MIL tests).
- The logged `qDDot` costs one extra ABA pass; drop it (or reuse the integrator's first evaluation) where
  cycles matter.
- Semi-implicit Euler keeps energy error bounded (no secular drift) at first-order accuracy; RK4 is
  fourth-order but slowly dissipative/drifting over long runs — choose by use case.
- Joint friction or damping belongs in `τ` (e.g. `τ − b·q̇`), not as ad-hoc velocity scaling.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/dynamics/ForwardDynamicsIntegrator.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Step`, `extern template` for
  `<float, 2, IntegrationMethod::SemiImplicitEuler>` and `<float, 2, IntegrationMethod::RungeKutta4>`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/dynamics/ForwardDynamicsIntegrator.cpp` → the same instantiations.
- Depends on: NT58 (`solvers::SemiImplicitEuler`, `SecondOrderSystem`); upstream `solvers::RungeKutta4`.
- Test: `robotics/dynamics/test/TestForwardDynamicsIntegrator.cpp`
- Doc: `doc/dynamics/ForwardDynamicsIntegrator.md` (per `doc/TEMPLATE.md`)
- The simulator's `RobotArmSimulator::StepChain` should switch to this integrator once it exists.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
