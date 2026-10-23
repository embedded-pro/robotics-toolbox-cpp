# Multi-Joint Cubic Spline Trajectory — Implementation Pseudocode

> Roadmap ref: #M33 (Tier 1) · Target: `robotics/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

Joint-space trajectory through timed via points. The interpolation itself — the C² vector-valued spline,
its tridiagonal solve and the clamped/natural ends — is upstream `math::CubicSpline` (NT61); this item
adds the trajectory semantics: knot times as the abscissa, rest-to-rest by default, joint-space state.

## Data structures

Shared joint-space trajectory types — **canonical definition**, created once in
`robotics/trajectory/TrajectoryTypes.hpp` by the first of M33/M27 deployed; every other trajectory spec
includes it and never redefines these names (scalar `TrajectoryState`/`MotionLimits` are upstream, NT55):

```cpp
template<typename T, std::size_t Dof>   # static_assert(std::is_floating_point_v<T>); multi-joint samplers (M27, M33)
struct JointTrajectoryState:
    math::Vector<T, Dof> position, velocity, acceleration
```

This item:

```cpp
template<typename T, std::size_t Dof, std::size_t MaxPoints>   # static_assert(std::is_floating_point_v<T>);
class CubicSplineTrajectory:                                   # static_assert(MaxPoints >= 2)
    using JointVector = math::Vector<T, Dof>
    math::CubicSpline<T, Dof, MaxPoints> spline                 # upstream NT61; abscissa = time
```

`SplineBoundary<T, Dof>` / `SplineEnd` (upstream) are used as is: `Clamped` with zero slopes — the
default — is a rest-to-rest move.

## Interface

```cpp
bool Plan(const std::array<T, MaxPoints>& times, const std::array<JointVector, MaxPoints>& positions,
          std::size_t count, const math::SplineBoundary<T, Dof>& boundary = {})
                                    # false if count ∉ [2, MaxPoints] or times not strictly increasing
JointTrajectoryState<T, Dof> Sample(T t) const   # hot path; absolute time, precondition: Plan() == true
T StartTime() const;  T Duration() const         # t_0, t_{K−1} − t_0
```

## Algorithm (pseudocode)

```text
function Plan(times, positions, count, boundary):
    return spline.Fit(times, positions, count, boundary)       # end slopes = end joint velocities

function Sample(t):                                             # OPTIMIZE_FOR_SPEED
    p = spline.Evaluate(t)                                      # clamps t to [t_0, t_{K−1}]
    return { p.value, p.firstDerivative, p.secondDerivative }   # position, velocity, acceleration

StartTime() = spline.First();   Duration() = spline.Last() − spline.First()
```

## Complexity & memory

- As upstream: `Plan` `O(Dof·K)`, `Sample` `O(log K + Dof)`; storage `MaxPoints·(2 + 2·Dof)` scalars. No heap.

## Numerical / embedded notes

- Plan once (planning time), then `Sample` is deterministic: one binary search plus fixed arithmetic.
- Clamped zero velocities (default) give a rest-to-rest move, but end accelerations are generally non-zero
  (an acceleration step at start/stop); Natural ends zero the accelerations but leave the end velocities
  free — pick per application.
- The spline does not enforce joint limits and may overshoot between knots; verify with `Sample`, or
  stretch all knot times by `λ ≥ 1` (velocities `/λ`, accelerations `/λ²`).
- Good default knot times: proportional to the largest joint displacement per segment (or chord length).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/trajectory/CubicSplineTrajectory.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class CubicSplineTrajectory<float, 3, 8>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Uses `robotics/trajectory/TrajectoryTypes.hpp` (create it from the block above if this is the first of
  M33/M27) and `numerical/math/CubicSpline.hpp` (NT61).
- Coverage: `robotics/trajectory/CubicSplineTrajectory.cpp` → `template class CubicSplineTrajectory<float, 3, 8>;`
- Test: `robotics/trajectory/test/TestCubicSplineTrajectory.cpp`
- Doc: `doc/trajectory/CubicSplineTrajectory.md` (per `doc/TEMPLATE.md`) + row in `doc/trajectory/README.md`.
- CMake: `.hpp` → `target_sources(robotics.trajectory …)`;
  `robotics_add_coverage_sources(robotics.trajectory CubicSplineTrajectory.cpp)`;
  `TestCubicSplineTrajectory.cpp` → `robotics.trajectory_test`.
- New module (first trajectory item only): `robotics/trajectory/CMakeLists.txt` with
  `robotics_add_header_library(robotics.trajectory)`, `TrajectoryTypes.hpp` in `target_sources`, a
  `test/` subdir, `add_subdirectory(trajectory)` in `robotics/CMakeLists.txt`, and a `doc/trajectory/` folder.
- Depends on: NT61 (`math::CubicSpline`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
