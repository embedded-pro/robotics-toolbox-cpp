# Reeds–Shepp Shortest Paths — Implementation Pseudocode

> Roadmap ref: #W10 (★★★☆☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Shortest path between two planar poses for a car with minimum turning radius `ρ` that may also reverse.
Reeds and Shepp (1990) showed the optimum is one of 48 words of at most five arcs and straights with at
most two direction changes (cusps). They are generated from 8 closed-form base formulas and three
symmetries, 44 evaluations in all, as in OMPL's `ReedsSheppStateSpace`. Unlike Dubins (W8), the
resulting distance is continuous in the goal pose, which makes it the metric of choice for parking and
car-like sampling planners.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
class ReedsSheppPlanner:
    T turningRadius                             # ρ > 0 (really_assert)
# result type: W8's CarPath<T> (signed segment lengths, up to 5 segments)
```

## Interface

```cpp
explicit ReedsSheppPlanner(T turningRadius)
CarPath<T> Shortest(const math::SE2Transform<T>& start, const math::SE2Transform<T>& goal) const
```

## Algorithm (pseudocode)

```text
# Normalised problem: start at the origin heading +x, ρ = 1. W(x) = WrapToPi(x) (NT54). ζ = 1e-5.
function Shortest(start, goal):
    rel = start.Inverse() * goal
    (x, y) = rel.translation / ρ;  φ = rel.Heading()
    best = none
    evaluate every family below; keep the candidate with the smallest Σ|ℓᵢ| (ties keep the first found)
    return CarPath{ best letters, best lengths · ρ, count, ρ }

# Symmetries of a base formula f with letters P and length builder B(t, u, v):
#   direct              f(x, y, φ)      → P,          B(t, u, v)
#   time-flip           f(−x, y, −φ)    → P,          −B(t, u, v)        (drive it backwards)
#   reflect             f(x, −y, −φ)    → mirror(P),  B(t, u, v)         (swap L and R)
#   time-flip + reflect f(−x, −y, φ)    → mirror(P),  −B(t, u, v)
#   backwards (CCC, CCSC only): the same four on (xb, yb) = (x cos φ + y sin φ, x sin φ − y cos φ),
#   with the letters and the lengths in reverse order.

polar(a, b) = (Hypot(a, b), atan2(b, a))

function τω(u, v, ξ, η, φ):
    δ = W(u − v);  A = sin u − sin δ;  B = cos u − cos δ − 1
    t1 = atan2(η·A − ξ·B, ξ·A + η·B);  t2 = 2(cos δ − cos v − cos u) + 3
    τ = (t2 < 0) ? W(t1 + π) : W(t1)
    return (τ, W(τ − u + v − φ))

# Base formulas (Reeds–Shepp eq. numbers); each returns (t, u, v) or fails.
LpSpLp (8.1):      (u, t) = polar(x − sin φ, y − 1 + cos φ);  t ≥ −ζ;  v = W(φ − t);  v ≥ −ζ
LpSpRp (8.2):      (u1, t1) = polar(x + sin φ, y − 1 − cos φ);  u1² ≥ 4
                   u = √(u1² − 4);  t = W(t1 + atan2(2, u));  v = W(t − φ);  t ≥ −ζ, v ≥ −ζ
LpRmL (8.3):       ξ = x − sin φ;  η = y − 1 + cos φ;  (u1, θ) = polar(ξ, η);  u1 ≤ 4
                   u = −2·asin(u1/4);  t = W(θ + u/2 + π);  v = W(φ − t + u);  t ≥ −ζ, u ≤ ζ
LpRupLumRm (8.7):  ξ = x + sin φ;  η = y − 1 − cos φ;  r = (2 + Hypot(ξ, η))/4;  r ≤ 1
                   u = acos r;  (t, v) = τω(u, −u, ξ, η, φ);  t ≥ −ζ, v ≤ ζ
LpRumLumRp (8.8):  ξ = x + sin φ;  η = y − 1 − cos φ;  r = (20 − ξ² − η²)/16;  0 ≤ r ≤ 1
                   u = −acos r;  u ≥ −π/2;  (t, v) = τω(u, u, ξ, η, φ);  t ≥ −ζ, v ≥ −ζ
LpRmSmLm (8.9):    ξ = x − sin φ;  η = y − 1 + cos φ;  (r, θ) = polar(ξ, η);  r ≥ 2
                   s = √(r² − 4);  u = 2 − s;  t = W(θ + atan2(s, −2));  v = W(φ − π/2 − t);  t ≥ −ζ, u ≤ ζ, v ≤ ζ
LpRmSmRm (8.10):   ξ = x + sin φ;  η = y − 1 − cos φ;  (r, θ) = polar(−η, ξ);  r ≥ 2
                   t = θ;  u = 2 − r;  v = W(t + π/2 − φ);  t ≥ −ζ, u ≤ ζ, v ≤ ζ
LpRmSLmRp (8.11):  ξ = x + sin φ;  η = y − 1 − cos φ;  (r, θ) = polar(ξ, η);  r ≥ 2
                   u = 4 − √(r² − 4);  u ≤ ζ;  t = W(atan2((4 − u)ξ − 2η, −2ξ + (u − 4)η));  v = W(t − φ);  t ≥ −ζ, v ≥ −ζ

# Families (letters, length builder); "+ backwards" adds the backwards variants
CSC:    LpSpLp → LSL (t, u, v);   LpSpRp → LSR (t, u, v)
CCC:    LpRmL  → LRL (t, u, v)                           + backwards → LRL (v, u, t)
CCCC:   LpRupLumRm → LRLR (t, u, −u, v);   LpRumLumRp → LRLR (t, u, u, v)
CCSC:   LpRmSmLm → LRSL (t, −π/2, u, v)                  + backwards → LSRL (v, u, −π/2, t)
        LpRmSmRm → LRSR (t, −π/2, u, v)                  + backwards → RSRL (v, u, −π/2, t)
CCSCC:  LpRmSLmRp → LRSLR (t, −π/2, u, −π/2, v)
```

## Complexity & memory

- `Shortest`: `O(1)`, 44 closed-form evaluations (about 150 trigonometric calls). Use it as a heuristic or
  local connector, not inside an inner loop that runs millions of times per cycle.
- Stack only: one `CarPath` candidate and the running best. No heap.

## Numerical / embedded notes

- **Corrected formulas.** Equations 8.3/8.4 and 8.11 contain typos in the original paper. The forms
  above are OMPL's corrected ones, and each was verified here by integrating every candidate to its goal:
  worst endpoint error `4.8e-11` over 20 000 random cases, double precision.
- **Float.** The shortest length matches double to `3.3e-7` in `float` on boundary and random goals,
  whether `ζ` is 1e-6 or 1e-4. `ζ = 1e-5` is used.
- **Sanity relations (tested).** `RS ≤ Dubins` (W8) for every pair, and `RS(a → b) = RS(b → a)`. The
  distance is continuous: perturbing the goal by 1e-4 changes the length by at most 4.4e-4 in the
  validation sweep.
- Signed lengths: a negative `ℓ` means reversing. `CarPath::Sample` (W8) applies `dir·(1, 0, κ)`, so a
  reversing left arc turns the heading clockwise, as a real car does.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/ReedsSheppPath.hpp` — `#pragma once`, and
  `extern template class ReedsSheppPlanner<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/ReedsSheppPath.cpp` → `template class ReedsSheppPlanner<float>;`
- Test: `robotics/planning/test/TestReedsSheppPath.cpp`
- Doc: `doc/planning/ReedsSheppPath.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test`.
- Depends on: W8 (`CarPath`, and `DubinsPlanner` in the test), NT66, NT54 (`WrapToPi`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
