# Polynomial Point-to-Point Trajectory — Implementation Pseudocode

> Roadmap ref: #M2 (Tier 1) · Target: `robotics/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

Shared trajectory types — **canonical definition**, created once in
`robotics/trajectory/TrajectoryTypes.hpp` by the first trajectory item deployed; every other trajectory
spec includes it and never redefines these names:

```cpp
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct TrajectoryState:            # one scalar axis
    T position{}, velocity{}, acceleration{}, jerk{}   # jerk = 0 where a profile leaves it undefined

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct MotionLimits:
    T vMax{}, aMax{}               # > 0 for every limit-driven profile
    T jMax{}                       # > 0 required only by SCurveProfile

template<typename T, std::size_t Dof>   # static_assert(std::is_floating_point_v<T>); multi-joint samplers (M27, M33)
struct JointTrajectoryState:
    math::Vector<T, Dof> position, velocity, acceleration
```

This item:

```cpp
enum class Degree : uint8_t { Cubic = 3, Quintic = 5 }

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct BoundaryConditions:         # per joint
    T q0, qf                       # start / end position
    T v0 = 0, vf = 0               # start / end velocity
    T a0 = 0, af = 0               # start / end acceleration (quintic only)

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class PolynomialTrajectory:
    array<T, 6>  coeff             # a0..a5 (cubic uses a0..a3, a4 = a5 = 0)
    T            duration          # tf > 0
    Degree       degree
```

## Interface

```text
PolynomialTrajectory(BoundaryConditions<T> bc, T tf, Degree degree)
TrajectoryState<T> Sample(T t)      # hot path
T    Duration()
void Reset(BoundaryConditions<T> bc, T tf)   # keeps the degree
```

## Algorithm (pseudocode)

```text
function solveCubic(bc, tf):            # closed form, no linear solve
    a0 = bc.q0
    a1 = bc.v0
    a2 = ( 3*(bc.qf-bc.q0) - (2*bc.v0 + bc.vf)*tf) / tf^2
    a3 = (-2*(bc.qf-bc.q0) + (  bc.v0 + bc.vf)*tf) / tf^3
    a4 = a5 = 0

function solveQuintic(bc, tf):          # six matched boundary conditions
    a0 = bc.q0;  a1 = bc.v0;  a2 = bc.a0 / 2
    a3 = ( 20*Δq - (8*bc.vf + 12*bc.v0)*tf - (3*bc.a0 - bc.af)*tf^2) / (2*tf^3)
    a4 = (-30*Δq + (14*bc.vf + 16*bc.v0)*tf + (3*bc.a0 - 2*bc.af)*tf^2) / (2*tf^4)
    a5 = ( 12*Δq - ( 6*bc.vf +  6*bc.v0)*tf - (  bc.a0 - bc.af)*tf^2) / (2*tf^5)
    # Δq = qf - q0

constructor / Reset:
    degree == Degree::Cubic ? solveCubic(bc, tf) : solveQuintic(bc, tf)

function Sample(t):                     # OPTIMIZE_FOR_SPEED
    t = clamp(t, 0, duration)
    pos  = Horner(coeff, t)             # a0 + a1 t + ... + a5 t^5
    vel  = Horner(derivative(coeff), t)
    acc  = Horner(secondDerivative(coeff), t)
    jerk = Horner(thirdDerivative(coeff), t)   # 6 a3 + 24 a4 t + 60 a5 t^2
    return { pos, vel, acc, jerk }
```

## Complexity & memory

- Coefficient solve: `O(1)` once at construction (closed-form expressions).
- `Sample`: `O(degree)` — four Horner evaluations, ≤ 14 multiply-adds.
- Memory: `O(1)` — six coefficients, a duration and the degree; all stack-resident, no heap.

## Numerical / embedded notes

- Solve coefficients **once** at construction; `Sample` is then pure Horner — deterministic cycles.
- Multi-joint moves: hold one instance per joint; for many via points use `CubicSplineTrajectory` (M33).
- Guard `tf > 0`; the `1/tf^k` terms blow up for a zero-duration segment.
- Quintic gives continuous acceleration (zero jerk endpoints); prefer it when actuators are jerk-sensitive.
- A 0 → 1 instance (e.g. `v0 = vf = 1`, `tf = 1` cubic ⇒ `s = t`) is a valid `TimeLaw` for
  `CartesianSlerpInterpolation`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/trajectory/PolynomialTrajectory.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class PolynomialTrajectory<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Uses `robotics/trajectory/TrajectoryTypes.hpp` (create it from the block above if this is the first
  trajectory item; header-only aggregates, no coverage `.cpp`).
- Coverage: `robotics/trajectory/PolynomialTrajectory.cpp` → `template class PolynomialTrajectory<float>;`
- Test: `robotics/trajectory/test/TestPolynomialTrajectory.cpp`
- Doc: `doc/trajectory/PolynomialTrajectory.md` (per `doc/TEMPLATE.md`) + row in `doc/trajectory/README.md`.
- CMake: `.hpp` → `target_sources(robotics.trajectory …)`;
  `robotics_add_coverage_sources(robotics.trajectory PolynomialTrajectory.cpp)`;
  `TestPolynomialTrajectory.cpp` → `robotics.trajectory_test`.
- New module (first trajectory item only): `robotics/trajectory/CMakeLists.txt` with
  `robotics_add_header_library(robotics.trajectory)` (links `numerical.math`), `TrajectoryTypes.hpp` in
  `target_sources`, a `test/` subdir, `add_subdirectory(trajectory)` in `robotics/CMakeLists.txt`,
  and a `doc/trajectory/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
