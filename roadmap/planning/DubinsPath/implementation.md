# Dubins Shortest Paths (+ Car Path Type) — Implementation Pseudocode

> Roadmap ref: #W8 (★★☆☆☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Shortest forward-only path between two planar poses for a vehicle with a minimum turning radius `ρ`.
Dubins (1957) proved it is one of six words of at most three segments: `CSC` (LSL, RSR, LSR, RSL) or
`CCC` (RLR, LRL), with `C` an arc of radius `ρ` and `S` a straight segment. Each word has a closed form,
so the planner evaluates all six and keeps the shortest. The same header defines `CarPath`, which the
Reeds–Shepp planner (W10) reuses.

## Data structures

```cpp
enum class Steering : std::uint8_t { left, straight, right }

template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct CarPathSegment:
    Steering steering
    T        length                             # metres; negative = reversing (W10 only)

template<typename T>
struct CarPath:
    std::array<CarPathSegment<T>, 5> segments   # Dubins uses 3, Reeds–Shepp up to 5
    std::size_t                      count{ 0 }
    T                                turningRadius

template<typename T>
class DubinsPlanner:
    T turningRadius                             # ρ > 0 (really_assert)
```

## Interface

```cpp
# CarPath
T                     Length() const                                        # Σ |segment length|
math::SE2Transform<T> Sample(const math::SE2Transform<T>& start, T s) const # pose after arc length s, clamped to [0, Length]

# DubinsPlanner
explicit DubinsPlanner(T turningRadius)
CarPath<T> Shortest(const math::SE2Transform<T>& start, const math::SE2Transform<T>& goal) const
```

## Algorithm (pseudocode)

```text
function CarPath::Sample(start, s):
    pose = start;  remaining = clamp(s, 0, Length())
    for each segment (steering, ℓ) while remaining > 0:
        take = min(remaining, |ℓ|);  dir = sign(ℓ)
        κ = { left: 1/ρ, straight: 0, right: −1/ρ }[steering]
        pose = pose * SE2Transform::Exp(dir·(1, 0, κ), take)     # NT66; reversing flips the yaw rate too
        remaining −= take
    return pose

function Mod2Pi(x):  y = x mod 2π in [0, 2π);  return (y > 2π − ε) ? 0 : y      # ε = 1e-5: no spurious full loops

function DubinsPlanner::Shortest(start, goal):
    (Δx, Δy) = goal.p − start.p;  D = |Δ|;  d = D/ρ
    θ = (D > 0) ? atan2(Δy, Δx) : 0
    α = Mod2Pi(start.Heading() − θ);  β = Mod2Pi(goal.Heading() − θ)
    sa, ca, sb, cb = sin α, cos α, sin β, cos β;  cab = cos(α − β)
    words (t, p, q), each kept only if its existence test passes:
      LSL: p² = 2 + d² − 2·cab + 2d(sa − sb);  τ = atan2(cb − ca, d + sa − sb)
           (Mod2Pi(τ − α), √p², Mod2Pi(β − τ))
      RSR: p² = 2 + d² − 2·cab + 2d(sb − sa);  τ = atan2(ca − cb, d − sa + sb)
           (Mod2Pi(α − τ), √p², Mod2Pi(τ − β))
      LSR: p² = −2 + d² + 2·cab + 2d(sa + sb);  τ = atan2(−ca − cb, d + sa + sb) − atan2(−2, √p²)
           (Mod2Pi(τ − α), √p², Mod2Pi(τ − β))
      RSL: p² = −2 + d² + 2·cab − 2d(sa + sb);  τ = atan2(ca + cb, d − sa − sb) − atan2(2, √p²)
           (Mod2Pi(α − τ), √p², Mod2Pi(β − τ))
      RLR: c = (6 − d² + 2·cab + 2d(sa − sb))/8;  φ = atan2(ca − cb, d − sa + sb)
           p = Mod2Pi(2π − acos c);  t = Mod2Pi(α − φ + p/2);  (t, p, Mod2Pi(α − β − t + p))
      LRL: c = (6 − d² + 2·cab + 2d(sb − sa))/8;  φ = atan2(ca − cb, d + sa − sb)
           p = Mod2Pi(2π − acos c);  t = Mod2Pi(−α − φ + p/2);  (t, p, Mod2Pi(β − α − t + p))
    existence: CSC needs p² ≥ −ε (then p² = max(p², 0)); CCC needs |c| ≤ 1 + ε (then clamp c to [−1, 1])
    keep the smallest t + p + q; ties keep the first word in the order above
    return CarPath{ word's steering letters, lengths (t·ρ, p·ρ, q·ρ), count 3, ρ }
```

## Complexity & memory

- `Shortest`: `O(1)`, six closed forms (about 20 trigonometric calls). `Sample`: up to 3 `Exp` calls.
- `CarPath`: 11 scalars plus steering tags. No heap.

## Numerical / embedded notes

- **Float boundary cases (normative).** Poses reachable by a single arc (for example `(ρ, ρ, π/2)`) put
  the CSC discriminants at exactly 0. In `float` they come out around `−1e-7`, and a strict `p² ≥ 0` test
  then drops every CSC word and returns a path five times too long. The tolerance `ε = 1e-5` (in units of
  `ρ`) keeps them, and the test covers it. `Mod2Pi` snaps `2π − ε` to 0 for the same reason.
- Lengths are computed in units of `ρ` and scaled at the end, so one planner serves any radius.
- The Dubins distance is discontinuous in the goal pose (a slightly rotated goal can need a full loop);
  planners that need a continuous metric should use Reeds–Shepp (W10).
- Formulas as in A. M. Shkel and V. Lumelsky (2001) and A. Walker's reference implementation; every word
  in the spec's tests was checked by integrating its segments to the goal (worst error `3e-11`, double).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Headers: `robotics/planning/CarPath.hpp`, `robotics/planning/DubinsPath.hpp` — `#pragma once`, and
  `extern template struct CarPath<float>;` / `extern template class DubinsPlanner<float>;` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/CarPath.cpp`, `robotics/planning/DubinsPath.cpp` → the same instantiations.
- Test: `robotics/planning/test/TestDubinsPath.cpp` (also covers `CarPath::Sample`)
- Doc: `doc/planning/DubinsPath.md` (per `doc/TEMPLATE.md`)
- CMake: new module `robotics/planning` (`robotics.planning`, test target `robotics.planning_test`),
  created by the first `planning` item deployed; `.hpp` → `target_sources`; `.cpp` →
  `robotics_add_coverage_sources`.
- Depends on: NT66 (`math::SE2Transform`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
