# Pure-Pursuit Path Tracking (+ Planar Polyline Path) — Implementation Pseudocode

> Roadmap ref: #W5 (★★☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

Geometric path tracker: aim at the point one look-ahead distance ahead on the path and drive the
circular arc that reaches it. It outputs a curvature, which fits both a unicycle (W1: `ω = v·κ`) and a
car (W2: `δ = SteerForCurvature(κ)`). It also defines `PlanarPath`, the bounded polyline that W6 and the
planners share.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct PathProjection:
    T                arcLength                  # s of the closest point
    math::Vector2<T> point
    T                heading                    # direction of the segment there
    T                crossTrack                 # signed distance, positive when the query point is left of the path
    std::size_t      segment

template<typename T, std::size_t MaxPoints>     # static_assert(MaxPoints >= 2)
class PlanarPath:
    std::array<math::Vector2<T>, MaxPoints> points
    std::array<T, MaxPoints>                arcLength      # cumulative, arcLength[0] = 0
    std::size_t                             count{ 0 }

template<typename T>
struct PurePursuitParameters:
    T minLookahead, maxLookahead                # L_min ≤ L_max, metres
    T lookaheadGain                             # seconds: L = clamp(L_min + gain·|v|, L_min, L_max)
    T goalTolerance
    std::size_t searchWindow                    # segments scanned from the current one

template<typename T>
struct PursuitCommand:
    T                curvature
    math::Vector2<T> goal
    bool             reached

template<typename T, std::size_t MaxPoints>
class PurePursuit:
    const PlanarPath<T, MaxPoints>& path
    PurePursuitParameters<T>        parameters
    T                               progress{ 0 }     # arc length reached so far; never decreases
    std::size_t                     segment{ 0 }
```

## Interface

```cpp
# PlanarPath
bool        Assign(std::span<const math::Vector2<T>> waypoints)  # false, path unchanged, if size ∉ [2, MaxPoints]
                                                                  # or a segment is shorter than 1e-6
std::size_t Size() const;  std::size_t Segments() const;  T Length() const
const math::Vector2<T>& Point(std::size_t i) const;  T ArcLength(std::size_t i) const
PathProjection<T> Project(const math::Vector2<T>& p, std::size_t firstSegment, std::size_t segmentWindow) const
math::Vector2<T>  PointAt(T s) const                             # s clamped to [0, Length]
T                 HeadingAt(T s) const

# PurePursuit
PurePursuit(const PlanarPath<T, MaxPoints>& path, const PurePursuitParameters<T>& parameters)
PursuitCommand<T> Compute(const math::SE2Transform<T>& pose, T speed)   # hot path
void Restart()                                                          # progress and segment back to 0
T    Progress() const
```

## Algorithm (pseudocode)

```text
function PlanarPath::Project(p, first, window):          # nearest point over a bounded run of segments
    best = none
    for k in [first, min(first + window, Segments())):
        a = points[k];  d = points[k+1] − a;  ℓ = |d|;  u = d/ℓ
        t = clamp((p − a)·u, 0, ℓ);  c = a + t·u
        if best is none or |p − c| < best.distance:      # ties keep the earlier segment
            best = { arcLength[k] + t, c, atan2(u.y, u.x), u.x·(p − a).y − u.y·(p − a).x, k }
    return best

function PlanarPath::PointAt(s):                         # binary search over arcLength, then interpolate
    s = clamp(s, 0, Length());  k = last index with arcLength[k] ≤ s (k ≤ Segments() − 1)
    return points[k] + (s − arcLength[k])·(points[k+1] − points[k]) / (arcLength[k+1] − arcLength[k])

function PurePursuit::Compute(pose, v):                  # OPTIMIZE_FOR_SPEED
    L = clamp(L_min + gain·|v|, L_min, L_max)
    q = pose.translation
    proj = path.Project(q, segment, searchWindow)
    if proj.arcLength ≥ progress: progress = proj.arcLength;  segment = proj.segment

    goal = none
    for k in [segment, path.Segments()):                 # first exit point of the circle |x − q| = L ahead
        a = path.Point(k);  d = path.Point(k+1) − a;  f = a − q
        A = d·d;  B = 2·f·d;  C = f·f − L²;  Δ = B² − 4AC
        if Δ < 0: continue
        t = (−B + √Δ) / (2A)                             # larger root: where the path leaves the circle
        if 0 ≤ t ≤ 1 and path.ArcLength(k) + t·√A ≥ progress: goal = a + t·d;  break
    if goal is none: goal = path.PointAt(min(progress + L, path.Length()))   # off the path or near its end

    b = pose.Inverse().Apply(goal)                       # goal in the body frame
    κ = (b·b < 1e-12) ? 0 : 2·b.y / (b·b)                # arc tangent to the heading through the goal
    reached = |q − path.Point(path.Size() − 1)| ≤ goalTolerance
    return { κ, goal, reached }
```

## Complexity & memory

- `Project`: `O(searchWindow)`; goal search `O(K)` worst case, normally one or two segments; `PointAt`
  `O(log K)`.
- Memory: `PlanarPath` stores `3·MaxPoints` scalars; `PurePursuit` holds a reference and 7 scalars. No heap.

## Numerical / embedded notes

- **Monotone progress.** The closest point is searched only from the current segment over `searchWindow`
  segments, and `progress` never decreases. A path that doubles back (U-turns, parking loops) therefore
  cannot capture the robot onto its return leg; the test shows the difference.
- **Look-ahead.** A short look-ahead tracks tightly but oscillates at speed; a long one cuts corners. The
  speed-scheduled `L_min + gain·|v|` is Coulter's recommendation. The regulated variant (Macenski et al.,
  2023) also slows down on high curvature, which the caller can do from `PursuitCommand::curvature`.
- **Reversing.** List the waypoints in driving order and pass a negative speed. The arc formula is the
  same, because the circle through the goal tangent to the heading does not depend on the direction of
  travel.
- Off the path by more than `L`, the goal falls back to `PointAt(progress + L)`, which steers back
  towards the path instead of stopping.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Headers: `robotics/mobile/PlanarPath.hpp` and `robotics/mobile/PurePursuit.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Compute`, and `extern template class PlanarPath<float, 32>;` /
  `extern template class PurePursuit<float, 32>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/PlanarPath.cpp`, `robotics/mobile/PurePursuit.cpp` → the same instantiations.
- Tests: `robotics/mobile/test/TestPlanarPath.cpp`, `robotics/mobile/test/TestPurePursuit.cpp`
- Doc: `doc/mobile/PurePursuit.md` (per `doc/TEMPLATE.md`; covers the path type)
- CMake: `.hpp` → `target_sources(robotics.mobile ...)`; `.cpp` → `robotics_add_coverage_sources`;
  tests → `robotics.mobile_test`.
- Depends on: NT66 (`math::SE2Transform`); W1 or W2 to turn the curvature into a command (caller side).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
