# Dynamic Window Approach (Local Obstacle Avoidance) — Implementation Pseudocode

> Roadmap ref: #W9 (★★★☆☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Reactive velocity selection for unicycle-type bases (W1, W4). Every control cycle it samples the
`(v, ω)` pairs reachable within one period under the acceleration limits (the *dynamic window*). It
drops those that could not stop before the nearest obstacle on their arc, and scores the rest on goal
heading, clearance and speed. Obstacles are a bounded set of points in the body frame, typically one
laser scan. The circular-arc clearance is computed in closed form, so no collision library is needed.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct DwaParameters:
    T minSpeed, maxSpeed                        # 0 ≤ minSpeed ≤ maxSpeed (forward only)
    T maxTurnRate                               # |ω| ≤ maxTurnRate
    T maxAcceleration, maxTurnAcceleration      # also the braking deceleration
    T controlPeriod                             # window half-widths = acceleration · period
    T predictionTime                            # horizon of the heading term
    T robotRadius                               # circular footprint (inflate by a safety margin)
    T maxClearance                              # clearance cap, metres
    T headingWeight, clearanceWeight, speedWeight

template<typename T>
struct VelocityWindow:
    T vMin, vMax, omegaMin, omegaMax

template<typename T>
struct DwaCommand:
    T v, omega, clearance

template<typename T, std::size_t NumV, std::size_t NumW>   # static_assert(NumV >= 2 && NumW >= 2)
class DynamicWindowApproach:
    DwaParameters<T> parameters
```

## Interface

```cpp
explicit DynamicWindowApproach(const DwaParameters<T>& parameters)
VelocityWindow<T> Window(T v, T omega) const
std::optional<DwaCommand<T>> Compute(T v, T omega, const math::Vector2<T>& goal,
                                     std::span<const math::Vector2<T>> obstacles) const   # body frame; hot path
static T ArcClearance(T v, T omega, const math::Vector2<T>& obstacle, T robotRadius, T maxClearance)
```

## Algorithm (pseudocode)

```text
function Window(v, ω):
    return { max(v_min, v − a·Δt), min(v_max, v + a·Δt), max(−ω_max, ω − α·Δt), min(ω_max, ω + α·Δt) }

function ArcClearance(v, ω, p, r, cap):         # arc length the robot centre travels before its disc touches p
    if |p| ≤ r: return 0                        # already in contact
    if v ≤ 0: return cap                        # rotating in place: a disc sweeps no new area
    κ = ω / v
    if |κ| < 1e-6:                              # straight ahead
        return (|p.y| ≤ r and p.x > 0) ? min(cap, p.x − √(r² − p.y²)) : cap
    R = 1/κ;  c = (0, R);  d = |p − c|          # the centre moves on the circle of radius |R| about c
    if |d − |R|| > r: return cap                # the obstacle is never within r of that circle
    Δ = acos(clamp((R² + d² − r²) / (2|R|·d), −1, 1))     # half-angle of the contact interval
    a0 = atan2(−R, 0);  ap = atan2(p.y − R, p.x)          # start and obstacle bearings about c
    sweep = (κ > 0 ? ap − a0 : a0 − ap) mod 2π            # obstacle angle along the direction of travel
    return min(cap, |R|·((sweep − Δ) mod 2π))

function Compute(v0, ω0, goal, obstacles):      # OPTIMIZE_FOR_SPEED
    w = Window(v0, ω0)
    for i in [0, NumV), j in [0, NumW):          # inclusive grid over the window
        v = w.vMin + i·(w.vMax − w.vMin)/(NumV − 1);  ω = w.omegaMin + j·(w.omegaMax − w.omegaMin)/(NumW − 1)
        dist = min(cap, min over obstacles of ArcClearance(v, ω, p, r, cap))
        admissible[i][j] = v ≤ √(2·a·dist)       # can brake to a stop within the free arc
        P = SE2Transform::Exp((v, 0, ω), predictionTime)  # NT66
        heading[i][j] = π − |AngleDifference(atan2(goal − P.p), P.Heading())|   # NT54
        clearance[i][j] = dist
    if no admissible sample: return nullopt
    normalise heading, clearance and speed (v) by their maxima over the admissible samples (skip a zero maximum)
    G = w_h·heading_n + w_c·clearance_n + w_v·v_n;  pick the admissible sample with the largest G (ties: first in grid order)
    return { v, ω, clearance }
```

## Complexity & memory

- `Compute`: `O(NumV·NumW·M)` clearance evaluations for `M` obstacle points (one `acos` and two `atan2`
  each, at most), plus one `Exp` per sample. 11 × 21 samples × 360 beams ≈ 83 k arc tests. Budget for it,
  or decimate the scan to the nearest points per sector.
- Stack: three `NumV·NumW` arrays of `T` and a flag array. No heap.

## Numerical / embedded notes

- **Admissibility.** Fox et al. also bound `ω` by `√(2·α·dist)`. That bound is dimensionally inconsistent
  (it mixes metres and radians); for a disc robot that brakes `v` and `ω` together along the same arc,
  the `v` bound is the binding one when `α ≥ |κ|·a`. Only the `v` bound is used here.
- **Standstill is always admissible** (`v = 0` has clearance `cap`), so `nullopt` only happens while
  moving too fast to stop. The caller then brakes at `maxAcceleration` and calls again.
- **Normalisation.** Each term is divided by its maximum over the admissible set, as Fox et al. normalise,
  so the weights stay meaningful when the window shrinks near obstacles.
- **Local minima.** DWA is reactive and can stall in U-shaped traps. Feed it intermediate goals from a
  global planner (P3) or a path (W5) rather than the final goal.
- Obstacles are points in the **body frame**; transform world-frame maps with the current pose (NT66)
  before the call. Forward motion only (`minSpeed ≥ 0`).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/DynamicWindowApproach.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Compute`, and `extern template class DynamicWindowApproach<float, 11, 21>;` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/DynamicWindowApproach.cpp` → `template class DynamicWindowApproach<float, 11, 21>;`
- Test: `robotics/planning/test/TestDynamicWindowApproach.cpp`
- Doc: `doc/planning/DynamicWindowApproach.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test`.
- Depends on: W1 (unicycle command model), NT66 (`Exp`), NT54 (`AngleDifference`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
