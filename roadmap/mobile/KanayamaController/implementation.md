# Kanayama Lyapunov Trajectory Tracking — Implementation Pseudocode

> Roadmap ref: #W7 (★★☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

Time-parameterised trajectory tracking for unicycle-type bases (differential drive, skid steer). The
reference supplies a pose and its feedforward `(v_r, ω_r)` at every tick. The error is expressed in the
robot frame, and a Lyapunov-derived law returns `(v, ω)` that converge to the reference for any initial
error while `v_r > 0`.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct KanayamaGains:
    T kx                                        # 1/s, along-track
    T ky                                        # 1/m², cross-track
    T ktheta                                    # 1/m, heading

template<typename T>
struct UnicycleCommand:
    T v
    T omega

template<typename T>
class KanayamaController:
    KanayamaGains<T> gains                      # really_assert all > 0
```

## Interface

```cpp
explicit KanayamaController(const KanayamaGains<T>& gains)
UnicycleCommand<T> Compute(const math::SE2Transform<T>& pose, const math::SE2Transform<T>& reference,
                           T referenceSpeed, T referenceTurnRate) const               # hot path
static math::Vector3<T> TrackingError(const math::SE2Transform<T>& pose, const math::SE2Transform<T>& reference)
static KanayamaGains<T> CriticallyDamped(T kx, T ky)                                  # ktheta = 2·√ky
```

## Algorithm (pseudocode)

```text
function TrackingError(pose, ref):              # reference seen from the robot
    e = pose.Inverse() * ref
    return (e.translation.x, e.translation.y, e.Heading())        # (ex, ey, eθ), eθ ∈ (−π, π]

function Compute(pose, ref, v_r, ω_r):          # OPTIMIZE_FOR_SPEED
    (ex, ey, eθ) = TrackingError(pose, ref)
    v = v_r·cos eθ + kx·ex
    ω = ω_r + v_r·(ky·ey + kθ·sin eθ)
    return { v, ω }
```

## Complexity & memory

- `O(1)`: one composition, one `atan2` (heading) and one `sin`/`cos` pair. Three scalars of state. No heap.

## Numerical / embedded notes

- **Stability.** `V = ex² + ey² + 2(1 − cos eθ)/ky` gives `V̇ = −kx·ex² − v_r·kθ·sin²eθ/ky ≤ 0` for
  `v_r > 0` and positive gains (Kanayama et al., 1990). The test checks that `V` never increases along a
  simulated run.
- **Tuning.** Linearised about a straight reference, the lateral loop has natural frequency
  `v_r·√ky` and damping `kθ/(2√ky)`. `CriticallyDamped` sets `kθ = 2√ky`.
- **`v_r = 0`.** The lateral error is uncontrollable in place (the nonholonomic constraint), so a
  stationary reference only regulates `ex` and `eθ`. For point stabilisation use a time-varying or
  discontinuous law instead.
- **Saturation.** Pass the output through W1's `Achievable`, which scales `v` and `ω` together. The
  Lyapunov argument assumes the command is applied as computed, so keep the reference within about 70 %
  of the actuator limits.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/mobile/KanayamaController.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class KanayamaController<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/KanayamaController.cpp` → `template class KanayamaController<float>;`
- Test: `robotics/mobile/test/TestKanayamaController.cpp`
- Doc: `doc/mobile/KanayamaController.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.mobile ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.mobile_test`.
- Depends on: NT66 (`math::SE2Transform`); W1 for saturation (caller side).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
