# Car-Like (Kinematic Bicycle / Ackermann) Kinematics — Implementation Pseudocode

> Roadmap ref: #W2 (★☆☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

Front-steered vehicles (cars, tractors, forklifts driven from the rear) reduced to the kinematic bicycle:
one virtual front wheel at steering angle `δ`, reference point at the rear-axle midpoint, no lateral
slip. Valid at low lateral acceleration (below roughly 0.4 g); beyond that tyre slip needs a dynamic
model.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct CarLikeParameters:
    T wheelBase                                 # L, rear to front axle
    T trackWidth                                # W, between the two front wheels (Ackermann only)
    T maxSteer                                  # δ_max > 0, rad; really_assert δ_max < atan(2L/W)
    T maxSteerRate                              # δ̇_max > 0, rad/s

template<typename T>
struct AckermannAngles:
    T left
    T right

template<typename T>
class CarLikeKinematics:
    CarLikeParameters<T> parameters
```

## Interface

```cpp
explicit CarLikeKinematics(const CarLikeParameters<T>& parameters)
math::Vector3<T>   Forward(T v, T steer) const                     # rear-axle twist (v, 0, v·tan δ / L); hot path
math::Vector3<T>   TwistAt(T v, T steer, T offset) const           # twist of a point `offset` ahead of the rear axle
T                  Curvature(T steer) const                        # tan δ / L
T                  SteerForCurvature(T curvature) const            # atan(L·κ), clamped to ±δ_max
T                  LimitSteer(T requested, T previous, T dt) const # angle limit, then rate limit
AckermannAngles<T> Ackermann(T steer) const
T                  MinTurningRadius() const                        # L / tan δ_max
const CarLikeParameters<T>& Parameters() const
```

`v` is the rear-axle speed (signed; negative reverses).

## Algorithm (pseudocode)

```text
function Forward(v, δ):                         # OPTIMIZE_FOR_SPEED
    return (v, 0, v·tan δ / L)

function TwistAt(v, δ, a):                      # = FromPose(−a, 0, 0).TransformTwist(Forward(v, δ))  (NT66 adjoint)
    ω = v·tan δ / L
    return (v, a·ω, ω)

function SteerForCurvature(κ):  return clamp(atan(L·κ), −δ_max, δ_max)

function LimitSteer(δ_req, δ_prev, dt):
    δ = clamp(δ_req, −δ_max, δ_max)
    return clamp(δ, δ_prev − δ̇_max·dt, δ_prev + δ̇_max·dt)

function Ackermann(δ):                          # both front-wheel normals meet the rear-axle line at R = L / tan δ
    t = tan δ
    return { atan(L·t / (L − (W/2)·t)),  atan(L·t / (L + (W/2)·t)) }   # finite at δ = 0, no division by tan δ
```

## Complexity & memory

- `O(1)`: one `tan` or `atan` per call. Four scalars of state. No heap.

## Numerical / embedded notes

- **Reference point.** Pure pursuit (W5) uses the rear axle, the point whose velocity has no lateral
  part. Stanley (W6) uses the front axle (`TwistAt(v, δ, L)`), and Rajamani's centre-of-gravity model
  uses `TwistAt(v, δ, l_r)`. `TwistAt` is the NT66 adjoint, so it agrees with SE(2) composition.
- `LimitSteer` assumes `δ_prev` is already within `±δ_max`, so the result is too. The steering actuator's
  rate limit is what makes Stanley and pure pursuit overshoot at speed, so trackers apply it every cycle.
- `Ackermann` needs the inner wheel below 90°, i.e. `|δ| < atan(2L/W)`; the constructor asserts it for
  `δ_max`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/mobile/CarLikeKinematics.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Forward`,
  and `extern template class CarLikeKinematics<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/CarLikeKinematics.cpp` → `template class CarLikeKinematics<float>;`
- Test: `robotics/mobile/test/TestCarLikeKinematics.cpp`
- Doc: `doc/mobile/CarLikeKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.mobile ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.mobile_test`.
- Depends on: NT66 (twist convention; the test uses its adjoint).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
