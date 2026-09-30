# Multi-Joint Cubic Spline Trajectory — Implementation Pseudocode

> Roadmap ref: #M33 (Tier 2) · Target: `robotics/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

`JointTrajectoryState<T, Dof>` comes from `robotics/trajectory/TrajectoryTypes.hpp` (canonical
definition: PolynomialTrajectory spec → Data structures).

```
enum class EndCondition : uint8_t { Clamped, Natural }

template<typename T, std::size_t Dof>        # static_assert(std::is_floating_point_v<T>); instantiated for float
struct SplineBoundary:
    EndCondition condition = EndCondition::Clamped
    math::Vector<T, Dof> startVelocity{}, endVelocity{}   # Clamped only; default zero (rest-to-rest)

template<typename T, std::size_t Dof, std::size_t MaxPoints>   # static_assert(std::is_floating_point_v<T>);
class CubicSplineTrajectory:                                   # static_assert(MaxPoints >= 2)
    using JointVector = math::Vector<T, Dof>
    array<T, MaxPoints>            knotTimes          # t_0 < t_1 < … < t_{K−1}
    array<JointVector, MaxPoints>  knotPositions      # q_k
    array<JointVector, MaxPoints>  knotAccelerations  # M_k = q̈(t_k), solved by Plan
    array<T, MaxPoints>            inverseSpan        # 1/h_k, h_k = t_{k+1} − t_k
    std::size_t                    count              # K, 2 ≤ K ≤ MaxPoints
```

## Interface

```
bool Plan(const array<T, MaxPoints>& times, const array<JointVector, MaxPoints>& positions,
          std::size_t count, const SplineBoundary<T, Dof>& boundary = {})
                                    # false if count ∉ [2, MaxPoints] or times not strictly increasing
JointTrajectoryState<T, Dof> Sample(T t) const   # hot path; absolute time, precondition: Plan() == true
T StartTime() const;  T Duration() const         # t_0, t_{K−1} − t_0
```

## Algorithm (pseudocode)

```
# Per joint, unknowns M_k = q̈(t_k). On interval k (τ = t − t_k ∈ [0, h_k]):
#   q(τ) = q_k + β_k·τ + (M_k/2)·τ² + d_k·τ³
#   β_k  = (q_{k+1} − q_k)/h_k − h_k·(2M_k + M_{k+1})/6        # velocity at t_k
#   d_k  = (M_{k+1} − M_k)/(6h_k)
# Position interpolates the knots and acceleration is shared at knots ⇒ C⁰ and C² by construction.
# Velocity continuity at interior knots k = 1..K−2 gives the tridiagonal rows
#   h_{k−1}·M_{k−1} + 2(h_{k−1} + h_k)·M_k + h_k·M_{k+1} = 6·[(q_{k+1} − q_k)/h_k − (q_k − q_{k−1})/h_{k−1}]
# End rows:
#   Clamped:  2h_0·M_0 + h_0·M_1                   = 6·[(q_1 − q_0)/h_0 − v_start]
#             h_{K−2}·M_{K−2} + 2h_{K−2}·M_{K−1}   = 6·[v_end − (q_{K−1} − q_{K−2})/h_{K−2}]
#   Natural:  M_0 = 0,  M_{K−1} = 0
# Every row is strictly diagonally dominant (|diag| = 2·Σ|off-diag|, or 1 vs 0) ⇒ Thomas, no pivoting.

function Plan(times, positions, count, boundary):
    if count < 2 or count > MaxPoints: return false
    for k in 0..count−2: if times[k+1] − times[k] ≤ ε: return false
    store times, positions, count;  inverseSpan[k] = 1/h_k
    build sub[k], diag[k], sup[k] from h and boundary.condition     # stack arrays of MaxPoints
    # forward elimination factors depend only on the knot times ⇒ computed once for all joints
    invPivot[0] = 1/diag[0];  supPrime[0] = sup[0]·invPivot[0]
    for k = 1..K−1:
        invPivot[k] = 1/(diag[k] − sub[k]·supPrime[k−1]);  supPrime[k] = sup[k]·invPivot[k]
    for j in 0..Dof−1:                                              # O(K) per joint
        r = right-hand side of joint j (rows above, v_start_j / v_end_j when Clamped, 0 when Natural)
        y[0] = r[0]·invPivot[0]
        for k = 1..K−1:          y[k] = (r[k] − sub[k]·y[k−1])·invPivot[k]
        M[K−1][j] = y[K−1]
        for k = K−2 down to 0:   M[k][j] = y[k] − supPrime[k]·M[k+1][j]
    return true

function Sample(t):                                                 # OPTIMIZE_FOR_SPEED
    t = clamp(t, t_0, t_{K−1})
    k = (upper_bound(knotTimes[1 .. K−1], t) index) clamped to [0, K−2]   # binary search, O(log K)
    τ = t − t_k;  h = t_{k+1} − t_k;  invH = inverseSpan[k]
    for j in 0..Dof−1:
        d = (M_{k+1,j} − M_{k,j})·invH/6
        β = (q_{k+1,j} − q_{k,j})·invH − h·(2M_{k,j} + M_{k+1,j})/6
        position_j     = q_{k,j} + τ·(β + τ·(M_{k,j}/2 + τ·d))
        velocity_j     = β + τ·(M_{k,j} + 3d·τ)
        acceleration_j = M_{k,j} + 6d·τ
    return { position, velocity, acceleration }
```

Verified with python3: knots reproduced to `1e-15`, velocity/acceleration continuous at interior knots,
clamped end velocities exact, natural end accelerations zero, collinear knots ⇒ `M ≡ 0`, two clamped
knots ⇒ exactly the `PolynomialTrajectory` cubic.

## Complexity & memory

- `Plan`: `O(K)` shared factorization + `O(K)` per joint ⇒ `O(Dof·K)`; no pivoting, no iteration.
- `Sample`: `O(log K)` interval search + `O(Dof)` Horner evaluations.
- Memory: `MaxPoints·(2 + 2·Dof)` scalars in bounded `std::array`s; `Plan` uses `O(MaxPoints)` stack
  scratch (tridiagonal bands, pivots, `y`). No heap.

## Numerical / embedded notes

- Plan once (planning time), then `Sample` is deterministic: one binary search plus fixed arithmetic.
- Reject `h_k ≤ ε`: near-coincident knot times make the rows ill-conditioned and the accelerations huge.
- Clamped zero velocities (default) give a rest-to-rest move, but end accelerations `M_0`, `M_{K−1}` are
  generally non-zero (an acceleration step at start/stop); Natural ends zero the accelerations but leave the
  end velocities free — pick per application.
- The spline does not enforce joint limits and may overshoot between knots; verify with `Sample`, or
  stretch all knot times by `λ ≥ 1` (velocities `/λ`, accelerations `/λ²`).
- Good default knot times: proportional to the largest joint displacement per segment (or chord length).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/trajectory/CubicSplineTrajectory.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class CubicSplineTrajectory<float, 3, 8>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Uses `robotics/trajectory/TrajectoryTypes.hpp` (create it if this is the first trajectory item).
- Coverage: `robotics/trajectory/CubicSplineTrajectory.cpp` → `template class CubicSplineTrajectory<float, 3, 8>;`
- Test: `robotics/trajectory/test/TestCubicSplineTrajectory.cpp`
- Doc: `doc/trajectory/CubicSplineTrajectory.md` (per `doc/TEMPLATE.md`) + row in `doc/trajectory/README.md`.
- CMake: `.hpp` → `target_sources(robotics.trajectory …)`;
  `robotics_add_coverage_sources(robotics.trajectory CubicSplineTrajectory.cpp)`;
  `TestCubicSplineTrajectory.cpp` → `robotics.trajectory_test`.
- New module (first trajectory item only): `robotics/trajectory/CMakeLists.txt` with
  `robotics_add_header_library(robotics.trajectory)`, `TrajectoryTypes.hpp` in `target_sources`, a
  `test/` subdir, `add_subdirectory(trajectory)` in `robotics/CMakeLists.txt`, and a `doc/trajectory/` folder.
- Roadmap bookkeeping: add the M33 row to `ROADMAP.md` and the `roadmap/README.md` trajectory index.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
