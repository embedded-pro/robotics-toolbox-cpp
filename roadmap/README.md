# Roadmap — Pseudocode Specifications

Design-level pseudocode for every proposed component in [../ROADMAP.md](../ROADMAP.md).
These are **specifications, not compilable code** — they describe *what* to build and *how*
the algorithm works, so an implementer can produce the real templated C++ afterwards.

## Convention

The tree mirrors the `robotics/` layout. Each algorithm gets its own folder with **three files**:

```
roadmap/<domain>/<AlgorithmName>/
├── implementation.md   # data structures, interface, algorithm pseudocode, complexity, float notes, deployment
├── tests.md            # GoogleTest test plan in pseudocode (TEST_F on float, StrictMock, no heap)
└── explanation.md      # short plain-language overview + the reference paper
```

All pseudocode respects the library constraints: **no heap**, bounded containers / `std::array`,
`OPTIMIZE_FOR_SPEED` on hot paths, and — per the current **float-only** decision — a generic
`template<typename T>` interface that is validated and instantiated for **`float`** only
(no `Q15`/`Q31`). The generic signature keeps a future fixed-point specialisation cheap to add.

The canonical worked example is
[trajectory/PolynomialTrajectory](trajectory/PolynomialTrajectory/implementation.md).

Conventions shared by every spec: 6-vectors are ordered linear part first — twists `(v; ω)`, wrenches
`(f; n)` — as fixed by [kinematics/SE3Transform](kinematics/SE3Transform/implementation.md); dynamics
and Jacobians reach controllers through interfaces implemented by
[dynamics/ChainDynamicsModel](dynamics/ChainDynamicsModel/implementation.md) and
[kinematics/GeometricJacobian](kinematics/GeometricJacobian/implementation.md).

## Deployment shape (float-only)

Each spec maps to this concrete artifact set when deployed into `robotics/`:

| Spec file           | Deploys to                                                             |
|---------------------|------------------------------------------------------------------------|
| `implementation.md` | `robotics/<domain>/<Name>.hpp` + `<Name>.cpp` (coverage instantiation) |
| `tests.md`          | `robotics/<domain>/test/Test<Name>.cpp`                                |
| `explanation.md`    | `doc/<domain>/<Name>.md` (expanded to follow `doc/TEMPLATE.md`)        |

Header shape (mirroring existing components such as `ForwardKinematics.hpp`):

```cpp
#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"

namespace <ns>
{
    template<typename T /*, std::size_t sizes... */>
    class <Name>
    {
        static_assert(std::is_floating_point_v<T>, "<Name> supports floating-point types");
    public:
        // OPTIMIZE_FOR_SPEED on hot paths (Filter/Compute/Update/Solve/Step)
    };

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class <Name><float /*, sizes... */>;
#endif
}
```

Coverage `.cpp`:

```cpp
#include "robotics/<domain>/<Name>.hpp"
namespace <ns> { template class <Name><float /*, sizes... */>; }
```

CMake wiring:
- add `<Name>.hpp` to `target_sources(robotics.<target> PRIVATE ...)`
- add `<Name>.cpp` to `robotics_add_coverage_sources(robotics.<target> ...)`
- add `Test<Name>.cpp` to the `_test` target's `target_sources`

Tests: single-type **`TEST_F` on `float`**, `StrictMock` only, no heap; validate reference vectors
with `EXPECT_NEAR` and `math::Tolerance<float>()` (or an explicit tolerance).

> **Float-only.** The `template<typename T>` signature is retained so `Q15`/`Q31` could be enabled
> per algorithm later by relaxing the `static_assert` and adding instantiations; none is planned.

## Index

### `kinematics`
`SE3Transform` (M6) · `DenavitHartenberg` (M7) · `GeometricJacobian` (M8) · `ManipulabilityIndex` (M11) · `PoseInverseKinematics` (M13) · `RedundancyResolution` (M14) · `ProductOfExponentials` (M15) · `AnalyticalIkOpw` (M21) · `ParallelManipulatorKinematics` (M23) · `MobileManipulatorKinematics` (M24) · `ContinuumKinematics` (M26) · `ChainPoseKinematics` (M30)

### `dynamics`
`GenericJointLink` (M1) · `FrictionCompensation` (M4) · `MomentumObserver` (M16) · `DynamicParameterIdentification` (M22) · `CompositeRigidBodyAlgorithm` (M28) · `ChainDynamicsModel` (M29) · `CoriolisMatrixAndRegressor` (M31) · `RneaExternalWrench` (M32) · `ForwardDynamicsIntegrator` (M34)

### `trajectory`
`PolynomialTrajectory` (M2) · `TrapezoidalProfile` (M3) · `SCurveProfile` (M9) · `CartesianSlerpInterpolation` (M10) · `TimeOptimalPathParameterization` (M27) · `CubicSplineTrajectory` (M33)

### `controllers/manipulator`
`PdGravityCompensation` (M5) · `ComputedTorqueControl` (M12) · `ImpedanceControl` (M17) · `OperationalSpaceControl` (M18) · `HybridPositionForceControl` (M19) · `SlotineLiAdaptiveControl` (M20) · `CableTensionDistribution` (M25)
