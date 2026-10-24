# Artificial Potential Fields — Implementation Pseudocode

> Roadmap ref: #P2 (★★☆☆☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Reactive motion as gradient descent on an attractive goal potential plus repulsive obstacle potentials
(Khatib, 1986). The attractive term is quadratic near the goal and conic (bounded) farther away. The
repulsive term acts only inside an influence distance and grows without bound at contact. Obstacles come
in as proximity readings (distance plus a direction away from the obstacle), typically from P1/P5
queries against the robot's control point. The field's well-known weakness, local minima, is detected and
reported rather than hidden.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct PotentialFieldParameters:
    T attractiveGain                            # k_a, 1/s
    T conicDistance                             # d*: quadratic inside, conic outside (bounds the pull)
    T repulsiveGain                             # k_r, m³/s
    T influenceDistance                         # ρ₀: obstacles farther than this are ignored
    T maxSpeed                                  # output clamp, m/s
    T stallSpeed                                # |F| below this away from the goal ⇒ local minimum
    T goalTolerance

template<typename T>
struct ObstacleProximity:
    math::Vector3<T> direction                  # unit, from the obstacle's closest point towards the robot point
    T distance                                  # surface distance ρ > 0 (P1/P5)

enum class FieldStatus : std::uint8_t { moving, reached, stalled }

template<typename T>
struct FieldCommand:
    math::Vector3<T> velocity                   # clamped gradient step
    T attractiveMagnitude, repulsiveMagnitude
    FieldStatus status

template<typename T>
class PotentialField:
    PotentialFieldParameters<T> parameters
```

## Interface

```cpp
explicit PotentialField(const PotentialFieldParameters<T>& parameters)
FieldCommand<T> Compute(const math::Vector3<T>& position, const math::Vector3<T>& goal,
                        std::span<const ObstacleProximity<T>> obstacles) const          # hot path
```

Planar robots pass `z = 0`. For a manipulator, evaluate the field at one or more control points and map
each velocity through the transposed point Jacobian (M8); the field itself is task-space only.

## Algorithm (pseudocode)

```text
function Compute(p, g, obstacles):              # OPTIMIZE_FOR_SPEED
    e = g − p;  d = ‖e‖
    F_att = (d ≤ d*) ? k_a·e : k_a·d*·e/d       # −∇U_att: quadratic well, conic beyond d*
    F_rep = 0
    for (n, ρ) in obstacles with ρ < ρ₀:
        F_rep += k_r·(1/ρ − 1/ρ₀)·(1/ρ²)·n      # −∇ of ½k_r(1/ρ − 1/ρ₀)²
    F = F_att + F_rep;  m = ‖F‖
    v = (m ≤ maxSpeed) ? F : F·maxSpeed/m
    status = d ≤ goalTolerance ? reached : (m < stallSpeed ? stalled : moving)
    return { v, ‖F_att‖, ‖F_rep‖, status }
```

## Complexity & memory

- `O(M)` for `M` obstacle readings: one division chain per active obstacle. Five scalars of state. No heap.

## Numerical / embedded notes

- **Conic far field.** A purely quadratic attraction makes far goals pull arbitrarily hard and swamps
  the repulsion. Above `d*` the pull is capped at `k_a·d*`.
- **Local minima.** An obstacle straight between the robot and the goal creates an equilibrium where the
  forces cancel. `stalled` reports it (test: at `ρ = 0.393003` the two magnitudes are both 1). Escaping is
  the caller's job: a random walk, a wall-following behaviour, or a global plan from P3/P6 with
  intermediate goals.
- `ρ → 0` makes the repulsion unbounded. The speed clamp keeps the command finite, and the caller keeps
  `ρ > 0` by stopping at contact (P1 returns signed distances).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/PotentialField.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class PotentialField<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/PotentialField.cpp` → `template class PotentialField<float>;`
- Test: `robotics/planning/test/TestPotentialField.cpp`
- Doc: `doc/planning/PotentialField.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test`.
- Depends on: P1 (proximity readings, caller side).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
