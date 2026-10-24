# Differential-Drive Kinematics — Implementation Pseudocode

> Roadmap ref: #W1 (★☆☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

Two independently driven wheels on one axle (plus casters): the base of most indoor robots and the base
model of M24. The body twist is `(v, 0, ω)` at the axle midpoint; the wheels cannot slide sideways.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct WheelSpeeds:                             # wheel angular speeds in rad/s; positive drives the robot forward
    T left
    T right

template<typename T>
class DifferentialDrive:
    T wheelRadius                               # r
    T trackWidth                                # B, distance between the wheel contact points
    T maxWheelSpeed                             # ω_max, rad/s
```

## Interface

```cpp
DifferentialDrive(T wheelRadius, T trackWidth, T maxWheelSpeed)    # really_assert all > 0
math::Vector3<T> Forward(const WheelSpeeds<T>& wheels) const       # body twist (v, 0, ω); hot path
WheelSpeeds<T>   Inverse(T v, T omega) const                       # hot path
WheelSpeeds<T>   Limit(const WheelSpeeds<T>& wheels) const
math::Vector3<T> Achievable(T v, T omega) const                    # Forward(Limit(Inverse(v, ω)))
T                MaxLinearSpeed() const                            # r·ω_max
T                MaxAngularSpeed() const                           # 2·r·ω_max / B

template<typename T>
WheelSpeeds<T> ScaleToLimit(const WheelSpeeds<T>& wheels, T maxWheelSpeed)   # free function, shared with W4
```

## Algorithm (pseudocode)

```text
function Forward(w):                            # OPTIMIZE_FOR_SPEED
    return ( r·(w.left + w.right)/2,  0,  r·(w.right − w.left)/B )

function Inverse(v, ω):                         # OPTIMIZE_FOR_SPEED
    return { (v − ω·B/2)/r,  (v + ω·B/2)/r }

function ScaleToLimit(w, ω_max):                # one factor for both wheels keeps the curvature ω/v
    m = max(|w.left|, |w.right|)
    if m ≤ ω_max: return w
    k = ω_max / m
    return { w.left·k, w.right·k }

function Limit(w):  return ScaleToLimit(w, maxWheelSpeed)
```

## Complexity & memory

- `O(1)`: a handful of multiplies, no trigonometry. Three scalars of state. No heap.

## Numerical / embedded notes

- **Limit keeps the path.** Scaling both wheels by one factor keeps `ω/v`, so a saturated command still
  follows the planned curve, only more slowly. Clipping each wheel on its own changes the curvature and
  makes trackers (W5, W7) overshoot.
- The feasible commands form the diamond `|v|/v_max + |ω|/ω_body,max ≤ 1`; `Achievable` projects onto it
  along the ray through the origin.
- `Forward` is linear, so applied to wheel-angle increments it gives the body displacement for F1's
  `Integrate(Δ, 1)`.
- Odometry accuracy is set by `r` (left and right separately, if needed) and `B`; calibrate them, for
  example with UMBmark (Borenstein & Feng, 1996).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/mobile/DifferentialDrive.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Forward`/`Inverse`, and `extern template class DifferentialDrive<float>;` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/DifferentialDrive.cpp` → `template class DifferentialDrive<float>;` and
  `template WheelSpeeds<float> ScaleToLimit<float>(const WheelSpeeds<float>&, float);`
- Test: `robotics/mobile/test/TestDifferentialDrive.cpp`
- Doc: `doc/mobile/DifferentialDrive.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.mobile ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.mobile_test`.
- Depends on: NT66 (`math::Vector3` twist convention).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
