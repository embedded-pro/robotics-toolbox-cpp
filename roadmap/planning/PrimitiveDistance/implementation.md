# Primitive Distance Queries (Point, Segment, Sphere, Capsule, Box) — Implementation Pseudocode

> Roadmap ref: #P1 (★★☆☆☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Closed-form closest points and signed distances between the convex primitives that robot links,
grippers, people and obstacles are usually wrapped in. Everything is reduced to the core geometry
(point and segment) plus a radius, the "rounded" representation that keeps capsule queries closed-form.
P4 (self-collision), P2 (potential fields) and H2 (speed and separation monitoring) build on these. Pairs
without a closed form (capsule–box, box–box, convex hulls) go to GJK/EPA (P5).

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct Segment:  math::Vector3<T> start, end
struct Sphere:   math::Vector3<T> center;  T radius
struct Capsule:  Segment<T> axis;  T radius     # a sphere swept along the segment
struct Box:      math::SE3Transform<T> pose;  math::Vector3<T> halfExtents        # oriented box (NT53)

template<typename T>
struct ClosestPoints:
    math::Vector3<T> onFirst, onSecond          # on the surfaces (cores for points and segments)
    T distance                                  # signed: < 0 is penetration depth

template<typename T>
struct SegmentPair:
    math::Vector3<T> onFirst, onSecond
    T s, t                                      # parameters along the two segments, in [0, 1]
    T distance
```

## Interface

```cpp
template<typename T> class PrimitiveDistance            # stateless; static members only
    static ClosestPoints<T> PointSegment(const math::Vector3<T>& p, const Segment<T>& segment)
    static SegmentPair<T>   SegmentSegment(const Segment<T>& a, const Segment<T>& b)                 # hot path
    static ClosestPoints<T> PointBox(const math::Vector3<T>& p, const Box<T>& box)                   # signed
    static ClosestPoints<T> SphereSphere(const Sphere<T>& a, const Sphere<T>& b)
    static ClosestPoints<T> SphereCapsule(const Sphere<T>& a, const Capsule<T>& b)
    static ClosestPoints<T> CapsuleCapsule(const Capsule<T>& a, const Capsule<T>& b)                 # hot path
    static ClosestPoints<T> SphereBox(const Sphere<T>& a, const Box<T>& b)
```

## Algorithm (pseudocode)

```text
ε = 1e-12 (squared lengths); Dot/Norm from Geometry3D

function PointSegment(p, [a, b]):
    d = b − a;  L = d·d;  t = (L ≤ ε) ? 0 : clamp((p − a)·d / L, 0, 1)
    c = a + t·d;  return { p, c, ‖p − c‖ }

function SegmentSegment([p₁, q₁], [p₂, q₂]):       # Ericson (2004) §5.1.9, handles degenerate segments
    d₁ = q₁ − p₁;  d₂ = q₂ − p₂;  r = p₁ − p₂;  a = d₁·d₁;  e = d₂·d₂;  f = d₂·r
    if a ≤ ε and e ≤ ε: s = t = 0
    elif a ≤ ε: s = 0;  t = clamp(f/e, 0, 1)
    else:
        c = d₁·r
        if e ≤ ε: t = 0;  s = clamp(−c/a, 0, 1)
        else:
            b = d₁·d₂;  den = a·e − b²
            s = (den > ε) ? clamp((b·f − c·e)/den, 0, 1) : 0          # parallel: any s, pick 0
            t = (b·s + f)/e
            if t < 0:   t = 0;  s = clamp(−c/a, 0, 1)
            elif t > 1: t = 1;  s = clamp((b − c)/a, 0, 1)
    c₁ = p₁ + s·d₁;  c₂ = p₂ + t·d₂;  return { c₁, c₂, s, t, ‖c₁ − c₂‖ }

function PointBox(p, box):                         # Ericson §5.1.4, signed
    l = Rᵀ·(p − c)                                 # box frame
    if |lᵢ| ≤ hᵢ for all i:                         # inside: push to the nearest face
        k = argminᵢ (hᵢ − |lᵢ|);  q = l with q_k = sign(l_k)·h_k (sign(0) = +1)
        return { p, c + R·q, −(h_k − |l_k|) }
    q = clamp(l, −h, h);  return { p, c + R·q, ‖l − q‖ }

function Rounded(core₁, core₂, coreDistance, r₁, r₂):   # shared by the sphere and capsule pairs
    n = (coreDistance > √ε) ? (core₂ − core₁)/coreDistance : (0, 0, 1)   # coincident cores: any normal
    return { core₁ + r₁·n, core₂ − r₂·n, coreDistance − r₁ − r₂ }

SphereSphere:   Rounded(a.center, b.center, ‖b.center − a.center‖, a.r, b.r)
SphereCapsule:  q = PointSegment(a.center, b.axis);   Rounded(a.center, q.onSecond, q.distance, a.r, b.r)
CapsuleCapsule: s = SegmentSegment(a.axis, b.axis);    Rounded(s.onFirst, s.onSecond, s.distance, a.r, b.r)
SphereBox:      q = PointBox(a.center, b)
                outside: Rounded(a.center, q.onSecond, q.distance, a.r, 0)
                inside:  { a.center − a.r·n_face, q.onSecond, q.distance − a.r }   # n_face = outward normal of face k
```

## Complexity & memory

- Every query is `O(1)`: at most two divisions and one square root. No heap, no iteration.

## Numerical / embedded notes

- **Signed distances.** Negative values are penetration depths along the reported points, which is what
  repulsive fields (P2) and safety monitors (H2) need near contact. For capsule pairs the depth is exact
  while the cores stay apart; with intersecting cores it is a lower bound (use P5 if that matters).
- **Degenerate inputs.** Zero-length segments (spheres as capsules) and parallel segments take explicit
  branches. The parallel branch returns one valid pair of closest points out of a continuum.
- **Box frame.** The box pose is an SE(3) transform, so `Rᵀ` comes from the transform's rotation, and no
  matrix inverse is formed.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/PrimitiveDistance.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `SegmentSegment`/`CapsuleCapsule`, and `extern template class PrimitiveDistance<float>;` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/PrimitiveDistance.cpp` → `template class PrimitiveDistance<float>;`
- Test: `robotics/planning/test/TestPrimitiveDistance.cpp`
- Doc: `doc/planning/PrimitiveDistance.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test` (the `planning` module is created by the first planning item deployed).
- Depends on: NT53 (`SE3Transform`); upstream `Geometry3D`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
