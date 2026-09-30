# Chain Dynamics Model (Link Chain → Manipulator Equation) — Implementation Pseudocode

> Roadmap ref: #M29 (Tier 2) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

The manipulator controllers (M5, M12, M16–M20) consume dynamics through interfaces, but nothing in the
library turns a link description into those interfaces. This adapter does: it implements the shipped
`EulerLagrangeDynamics` interface (`M`, `C q̇`, `g`) with CRBA (M28) and RNEA, and a new one-call
`InverseDynamicsModel` interface so computed-torque control can run in `O(n)`.

## Data structures

```cpp
template<typename T, std::size_t Dof>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class InverseDynamicsModel:                           # new interface, virtual ~InverseDynamicsModel() = default
    virtual StateVector ComputeInverseDynamics(const StateVector& q, const StateVector& qDot,
                                               const StateVector& qDDot) const = 0

template<typename T, std::size_t NumLinks>
class ChainDynamicsModel
    : public EulerLagrangeDynamics<T, NumLinks>       # shipped interface
    , public InverseDynamicsModel<T, NumLinks>:
    LinkArray                                  links      # owned copy (payload updates via SetLinks)
    Vector3                                    gravity    # base-frame gravity, e.g. (0, 0, −9.81)
    RecursiveNewtonEuler<T, NumLinks>          rnea
    CompositeRigidBodyAlgorithm<T, NumLinks>   crba       # M28
```

## Interface

```text
ChainDynamicsModel(const LinkArray& links, const Vector3& gravity)
void        SetLinks(const LinkArray& links)                             # e.g. payload change
MassMatrix  ComputeMassMatrix(const StateVector& q) const override
StateVector ComputeCoriolisTerms(const StateVector& q, const StateVector& qDot) const override
StateVector ComputeGravityTerms(const StateVector& q) const override
StateVector ComputeInverseDynamics(const StateVector& q, const StateVector& qDot,
                                   const StateVector& qDDot) const override     # hot path
```

## Algorithm (pseudocode)

```text
ComputeMassMatrix(q)          = crba.Compute(links, q)
ComputeCoriolisTerms(q, q̇)    = rnea.InverseDynamics(links, q, q̇, 0, 0)        # no gravity, no q̈
ComputeGravityTerms(q)        = rnea.InverseDynamics(links, q, 0, 0, gravity)
ComputeInverseDynamics(q,q̇,q̈) = rnea.InverseDynamics(links, q, q̇, q̈, gravity) # one O(n) pass
# with M1/M4 fields present: + armature ⊙ q̈ + friction(q̇) in ComputeInverseDynamics,
# + diag(armature) in ComputeMassMatrix, friction reported separately (it is not part of C q̇)
```

## Complexity & memory

- `ComputeInverseDynamics`, `ComputeCoriolisTerms`, `ComputeGravityTerms`: `O(n)` each (one RNEA pass).
- `ComputeMassMatrix`: `O(n²)` (CRBA).
- Memory: one copy of the link array; no heap.

## Numerical / embedded notes

- `C(q,q̇)q̇` obtained from RNEA is the correct Coriolis/centrifugal *vector*; controllers that need
  the Coriolis *matrix* (Slotine–Li, momentum observer) must use M31 instead.
- The interfaces are virtual so controllers can be tested with StrictMock models. On a hard real-time
  path, template the controller on `ChainDynamicsModel` directly to devirtualize (AGENTS.md: no virtual
  calls in real-time paths).
- Must satisfy `ComputeInverseDynamics(q, q̇, q̈) = M q̈ + C q̇ + g` exactly (up to rounding) — the
  cross-check test pins it.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Headers: `robotics/dynamics/InverseDynamicsModel.hpp` (pure interface) and
  `robotics/dynamics/ChainDynamicsModel.hpp` — `#pragma once` → `#pragma GCC optimize("O3","fast-math")`,
  `OPTIMIZE_FOR_SPEED` on `ComputeInverseDynamics`, and `extern template class ChainDynamicsModel<float, 2>;`
  / `<float, 3>` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/dynamics/ChainDynamicsModel.cpp` → the same instantiations.
- Test: `robotics/dynamics/test/TestChainDynamicsModel.cpp`
- Doc: `doc/dynamics/ChainDynamicsModel.md` (per `doc/TEMPLATE.md`)
- Depends on: M28.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
