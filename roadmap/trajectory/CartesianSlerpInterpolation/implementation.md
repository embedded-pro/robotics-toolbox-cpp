# Cartesian Path + Orientation (SLERP) Interpolation — Implementation Pseudocode

> Roadmap ref: #M10 (Tier 2) · Target: `robotics/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

Poses are `kinematics::SE3Transform<T>` (M6); twists are `kinematics::Vector6<T>` ordered linear-first
`(v; ω)`. Orientation is interpolated with the upstream `math::Quaternion<T>`
(`numerical/math/Quaternion.hpp`, [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp):
`FromRotationMatrix`, `Slerp`, `ToRotationMatrix`, `Conjugate`). `TrajectoryState<T>` comes from
`robotics/trajectory/TrajectoryTypes.hpp`.

```
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct CartesianState:
    kinematics::SE3Transform<T> pose
    kinematics::Vector6<T>      twist          # (v; ω), base frame — feed-forward for task-space control

# TimeLaw requirement (static polymorphism, no virtual call on the hot path):
#   TrajectoryState<T> Sample(T t) const  — position = progress s ∈ [0, 1], velocity = ṡ
#   T Duration() const                    — s(0) = 0, s(Duration()) = 1
#   e.g. TrapezoidalProfile<T>{0, 1, limits} or PolynomialTrajectory<T>{{.q0 = 0, .qf = 1}, tf, Degree::Quintic}

template<typename T, typename TimeLaw>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class CartesianSlerpInterpolation:
    math::Vector<T, 3>   p0, deltaP          # start position, goal − start
    math::Quaternion<T>  q0, q1              # start / goal orientation, q1 sign-fixed (q0·q1 ≥ 0)
    math::Vector<T, 3>   rotationVector      # φ·k of R1·R0ᵀ (base frame), |φ| ≤ π
    TimeLaw              timeLaw
```

## Interface

```
CartesianSlerpInterpolation(const kinematics::SE3Transform<T>& start,
                            const kinematics::SE3Transform<T>& goal, const TimeLaw& timeLaw)
CartesianState<T> Sample(T t)       # hot path
T                 Duration()        # = timeLaw.Duration()
```

## Algorithm (pseudocode)

```
function plan(start, goal):                   # orientation geometry once
    p0 = start.p;  deltaP = goal.p - start.p
    q0 = Quaternion::FromRotationMatrix(start.R)
    q1 = Quaternion::FromRotationMatrix(goal.R)
    if dot4(q0, q1) < 0: q1 = -q1             # shortest of the two arcs (same rotation)
    d = q1 * q0.Conjugate()                   # R1·R0ᵀ, base frame; d.w = dot4(q0, q1) ≥ 0
    n = |(d.x, d.y, d.z)|
    if n < ε: rotationVector = 2·(d.x, d.y, d.z)            # small angle, no division
    else:     rotationVector = 2·atan2(n, d.w)·(d.x, d.y, d.z)/n   # φ·k, φ = 2·acos(d.w)

function Sample(t):                           # OPTIMIZE_FOR_SPEED
    law = timeLaw.Sample(clamp(t, 0, Duration()))
    s = clamp(law.position, 0, 1);  sDot = law.velocity
    p = p0 + s*deltaP                         # straight line
    q = Quaternion::Slerp(q0, q1, s)          # great-circle arc; nlerp fallback for dot > 0.9995 upstream
    v = sDot * deltaP                         # linear velocity
    ω = sDot * rotationVector                 # angular velocity ṡ·φ·k, constant axis
    return { SE3Transform{ q.ToRotationMatrix(), p }, (v; ω) }
```

Verified (python3, finite differences of `R(s(t))`): `vee(Ṙ·Rᵀ) = ṡ·φ·k` for arbitrary start/goal,
including the sign-flipped (obtuse) case.

## Complexity & memory

- Planning: `O(1)` — two matrix → quaternion conversions, one quaternion product, one `atan2`.
- `Sample`: `O(1)` — one time-law sample, a 3-vector lerp, one upstream `Slerp` (`acos` + 3 `sin`, or an
  nlerp) and one quaternion → matrix conversion.
- Memory: `O(1)` — two quaternions, three 3-vectors and the time law; stack-resident, no heap.

## Numerical / embedded notes

- Reuse upstream `math::Quaternion::Slerp` (shortest-path flip + nlerp fallback built in) rather than
  re-deriving the blend.
- **Shortest-path fix:** negate `q1` when `dot < 0` *before* computing `rotationVector`, so the twist
  matches the arc `Slerp` actually takes.
- `rotationVector` uses `φ = 2·atan2(n, w)` — well-conditioned near `φ → 0` (where `acos` loses `float`
  precision) and finite at `φ = π`; the angular part of `SE3Transform::PoseError(goal, start)` equals it.
- Position and orientation are **coupled** by sharing one scalar `s(t)` from the time law, so both reach
  the goal simultaneously; limits of the time law (on `s`) bound `|v| = ṡ·|Δp|` and `|ω| = ṡ·|φ|`.
- `TimeLaw` is a template parameter, not an interface: no virtual call on the hot path.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/trajectory/CartesianSlerpInterpolation.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class CartesianSlerpInterpolation<float, PolynomialTrajectory<float>>;` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Uses `robotics/trajectory/TrajectoryTypes.hpp`, `robotics/kinematics/SE3Transform.hpp` (M6) and
  `numerical/math/Quaternion.hpp`; `robotics.trajectory` links `robotics.kinematics`.
- Coverage: `robotics/trajectory/CartesianSlerpInterpolation.cpp` →
  `template class CartesianSlerpInterpolation<float, PolynomialTrajectory<float>>;`
- Test: `robotics/trajectory/test/TestCartesianSlerpInterpolation.cpp`
- Doc: `doc/trajectory/CartesianSlerpInterpolation.md` (per `doc/TEMPLATE.md`) + row in `doc/trajectory/README.md`.
- CMake: `.hpp` → `target_sources(robotics.trajectory …)`;
  `robotics_add_coverage_sources(robotics.trajectory CartesianSlerpInterpolation.cpp)`;
  `TestCartesianSlerpInterpolation.cpp` → `robotics.trajectory_test`.
- New module (first trajectory item only): `robotics/trajectory/CMakeLists.txt` with
  `robotics_add_header_library(robotics.trajectory)`, `TrajectoryTypes.hpp` in `target_sources`, a
  `test/` subdir, `add_subdirectory(trajectory)` in `robotics/CMakeLists.txt`, and a `doc/trajectory/` folder.
- Depends on: M6, M2 (any other `TimeLaw`, e.g. M3/M9, works without extra code).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
