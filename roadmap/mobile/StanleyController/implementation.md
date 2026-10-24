# Stanley Path Tracking — Implementation Pseudocode

> Roadmap ref: #W6 (★★☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

The steering law of Stanford's DARPA Grand Challenge winner, for front-steered vehicles. It measures the
error at the front axle and adds the heading error to a speed-normalised arctangent of the cross-track
error. For small errors this gives exponential convergence `ė ≈ −k·e`, at a rate independent of speed.
It reuses W5's `PlanarPath` and W2's steering limits.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct StanleyParameters:
    T gain                                      # k, 1/s
    T softening                                 # k_s, m/s: keeps the law finite at low speed
    std::size_t searchWindow

template<typename T>
struct StanleyError:
    T crossTrack                                # front axle, positive when left of the path
    T headingError                              # path heading − vehicle heading, wrapped (NT54)

template<typename T, std::size_t MaxPoints>
class StanleyController:
    const PlanarPath<T, MaxPoints>& path        # W5
    const CarLikeKinematics<T>&     vehicle     # W2: wheelbase, steering limits
    StanleyParameters<T>            parameters
    T                               progress{ 0 }
    std::size_t                     segment{ 0 }
```

## Interface

```cpp
StanleyController(const PlanarPath<T, MaxPoints>& path, const CarLikeKinematics<T>& vehicle, const StanleyParameters<T>& parameters)
T               Compute(const math::SE2Transform<T>& rearAxlePose, T speed, T previousSteer, T dt)   # steering angle; hot path
StanleyError<T> Error(const math::SE2Transform<T>& rearAxlePose)          # updates progress like Compute
void            Restart()
```

## Algorithm (pseudocode)

```text
function Error(pose):
    front = pose * SE2Transform::FromPose(L, 0, 0)
    proj  = path.Project(front.translation, segment, searchWindow)
    if proj.arcLength ≥ progress: progress = proj.arcLength;  segment = proj.segment   # monotone, as in W5
    return { proj.crossTrack,  math::AngleDifference(proj.heading, pose.Heading()) }

function Compute(pose, v, δ_prev, dt):          # OPTIMIZE_FOR_SPEED
    e = Error(pose)
    δ = e.headingError − atan2(k·e.crossTrack, k_s + v)    # left of the path ⇒ steer right
    return vehicle.LimitSteer(δ, δ_prev, dt)               # ±δ_max, then ±δ̇_max·dt (W2)
```

## Complexity & memory

- `O(searchWindow)` for the projection plus one `atan2`; state is a reference pair and 4 scalars. No heap.

## Numerical / embedded notes

- **Front-axle reference.** Measuring at the front axle makes the cross-track term act directly on the
  steered wheels. With the rear axle the law is not stable at speed.
- **Softening `k_s`.** Without it, `atan2(k·e, v)` jumps to `±π/2` at `v → 0` for any `e`. A value near
  1 m/s keeps low-speed steering gentle.
- **Rate limit.** The steering actuator's slew rate dominates behaviour at speed; feed the applied steer
  back as `previousSteer` every cycle.
- Forward driving only (`v ≥ 0`). Reversing uses the rear axle as reference with a mirrored law (or pure
  pursuit, W5).
- The original paper adds yaw-rate damping and steering-servo delay terms for high speed; the kinematic
  law here is the low-to-moderate-speed core.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/mobile/StanleyController.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class StanleyController<float, 32>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/StanleyController.cpp` → `template class StanleyController<float, 32>;`
- Test: `robotics/mobile/test/TestStanleyController.cpp`
- Doc: `doc/mobile/StanleyController.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.mobile ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.mobile_test`.
- Depends on: W2 (`CarLikeKinematics`), W5 (`PlanarPath`), NT66, NT54 (`AngleDifference`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
