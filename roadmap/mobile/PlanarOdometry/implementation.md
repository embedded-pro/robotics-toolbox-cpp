# Planar Odometry on SE(2) — Implementation Pseudocode

> Roadmap ref: #F1 (★★☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

Dead-reckoning integrator shared by every wheeled model (W1–W4) and the localization filters (E1, E4).
A body twist held over one sample period moves the robot along a circular arc. Integrating it with the
SE(2) exponential (NT66) lands on that arc exactly, so heading changes inside the step do not bias the
position, as they do with Euler or Runge-Kutta integration of `(ẋ, ẏ, θ̇)`.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
class PlanarOdometry:
    math::SE2Transform<T> pose                  # body frame in the odometry frame (NT66)

template<typename T>
struct OdometryNoise:                           # increment noise shared by the localization filters (E3, E4)
    T rotationFromRotation, rotationFromTranslation        # α₁, α₂: σ_θ² = α₁Δθ² + α₂Δs²
    T translationFromTranslation, translationFromRotation  # α₃, α₄: σ_xy² = α₃Δs² + α₄Δθ²
```

## Interface

```cpp
PlanarOdometry()                                              # starts at Identity
explicit PlanarOdometry(const math::SE2Transform<T>& initial)
void Integrate(const math::Vector3<T>& twist, T dt)           # body twist (vx, vy, ω); hot path
void Reset(const math::SE2Transform<T>& pose)
const math::SE2Transform<T>& Pose() const
static math::Vector3<T> Increment(const math::SE2Transform<T>& from, const math::SE2Transform<T>& to)

template<typename T>
math::Vector3<T> IncrementDeviation(const math::SE2Transform<T>& increment, const OdometryNoise<T>& noise)
                                                              # (σ_xy, σ_xy, σ_θ) of the body-frame increment
```

## Algorithm (pseudocode)

```text
function Integrate(ξ, dt):                      # OPTIMIZE_FOR_SPEED
    pose = (pose * SE2Transform::Exp(ξ, dt)).Normalized()

function Increment(from, to):                   # body-frame motion between two odometry readings
    return (from.Inverse() * to).Log()          # (Δx, Δy, Δθ) as a twist held for unit time

function IncrementDeviation(Δ, α):              # noise grows with distance and rotation; same for reverse
    (dx, dy, dθ) = Δ.Pose();  Δs² = dx² + dy²
    σ_xy = √(α₃Δs² + α₄dθ²);  σ_θ = √(α₁dθ² + α₂Δs²)
    return (σ_xy, σ_xy, σ_θ)
```

## Complexity & memory

- `Integrate`: one `Exp` (one `sin`/`cos` pair), one composition and one `Hypot`.
- Memory: one `SE2Transform` (4 scalars). No heap.

## Numerical / embedded notes

- **Exact for a constant twist.** For a twist that changes during the period, pass the mean of the twists
  at both ends of the step (second order), not the newest sample.
- **Encoder increments.** The wheel models' `Forward` maps are linear, so applying them to wheel-angle
  increments gives the body displacement `Δ = ξ·dt` directly. Integrate it with `Integrate(Δ, 1)`; no
  division by a jittery `dt` is needed.
- `Normalized` every step keeps `cos² + sin² = 1` for arbitrarily long runs (one `Hypot` per step).
- `Increment` is what odometry motion models consume (E1, E4). It goes through `Log`, so it needs the
  heading change between the two readings to stay below `π`; sample faster than half a turn.
- Odometry error grows without bound. Wheel-radius and track-width errors dominate it, which is the
  wheel models' job to calibrate (UMBmark), not this integrator's.
- `IncrementDeviation` is Thrun's odometry noise model applied to the body-frame increment instead of
  the rotate–translate–rotate split, whose first rotation jumps by π when the robot reverses.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/mobile/PlanarOdometry.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Integrate`, and
  `extern template class PlanarOdometry<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/PlanarOdometry.cpp` → `template class PlanarOdometry<float>;` and
  `template math::Vector3<float> IncrementDeviation<float>(...);`
- Test: `robotics/mobile/test/TestPlanarOdometry.cpp`
- Doc: `doc/mobile/PlanarOdometry.md` (per `doc/TEMPLATE.md`)
- CMake: new module `robotics/mobile` (`robotics.mobile`, test target `robotics.mobile_test`), created by
  the first `mobile` item deployed; `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`.
- Depends on: NT66 (`math::SE2Transform`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
