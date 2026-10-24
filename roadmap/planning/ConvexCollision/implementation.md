# GJK + EPA Convex Distance and Penetration — Implementation Pseudocode

> Roadmap ref: #P5 (★★★★☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Distance, closest points, penetration depth and contact normal between any two convex shapes described by
a support function: spheres, capsules, oriented boxes and bounded convex hulls. It covers the pairs P1 has
no closed form for (capsule–box, box–box, hull–anything).
- **Separated:** Gilbert–Johnson–Keerthi on the shape cores, with the radii subtracted afterwards (the
  "core + margin" representation). Rounded shapes stay exact, and the iteration count stays small.
- **Overlapping:** the expanding polytope algorithm on the Minkowski difference, with bounded face and
  edge storage.

## Data structures

```cpp
enum class ShapeKind : std::uint8_t { sphere, capsule, box, hull }

template<typename T, std::size_t MaxHullVertices = 16>      # static_assert(std::is_floating_point_v<T>); instantiated for float
struct ConvexShape:
    ShapeKind kind
    math::SE3Transform<T> pose                  # shape frame in the world (NT53)
    T radius                                    # margin: sphere / capsule radius (0 for box and hull)
    math::Vector3<T> halfExtents                # box
    T halfLength                                # capsule core along the local z axis
    std::array<math::Vector3<T>, MaxHullVertices> vertices;  std::size_t vertexCount   # hull, shape frame
    static ConvexShape Sphere(const math::Vector3<T>& centre, T radius)
    static ConvexShape Capsule(const math::Vector3<T>& a, const math::Vector3<T>& b, T radius)
    static ConvexShape Box(const math::SE3Transform<T>& pose, const math::Vector3<T>& halfExtents)
    static ConvexShape Hull(const math::SE3Transform<T>& pose, std::span<const math::Vector3<T>> vertices)   # really_assert ≤ MaxHullVertices
    math::Vector3<T> CoreSupport(const math::Vector3<T>& direction) const   # farthest core point along direction
    math::Vector3<T> Support(const math::Vector3<T>& direction) const       # core + radius·direction/‖direction‖

template<typename T>
struct ConvexProximity:
    T distance                                  # ≥ 0 separated; < 0 penetration depth
    math::Vector3<T> onFirst, onSecond          # witness points (contact points when overlapping)
    math::Vector3<T> normal                     # unit, from the first shape to the second
    bool converged                              # false if an iteration or face budget ran out
    std::size_t iterations

template<typename T, std::size_t MaxHullVertices = 16, std::size_t MaxFaces = 64>
class ConvexCollision                           # stateless; static members only
```

## Interface

```cpp
static ConvexProximity<T> Query(const ConvexShape<T, MaxHullVertices>& a, const ConvexShape<T, MaxHullVertices>& b)   # hot path
```

## Algorithm (pseudocode)

```text
CoreSupport(d), world frame (l = Rᵀd in the shape frame):
    sphere: centre;  capsule: centre ± halfLength·ẑ_world (sign of d·ẑ_world)
    box: c + R·(sign(l) ⊙ h) with sign(0) = +1;  hull: c + R·argmax_v (v·l)

function Gjk(a, b, support):                     # support = CoreSupport (distance) or Support (EPA seed)
    v = centre(a) − centre(b) (any non-zero point of A ⊖ B);  simplex W = ∅ with witnesses (aᵢ, bᵢ)
    repeat at most 32 times:
        aᵢ = support_a(−v);  bᵢ = support_b(v);  w = aᵢ − bᵢ
        if ‖v‖² − v·w ≤ 1e-6·‖v‖² or w ∈ W: stop                      # no further progress
        W ∪= { w };  (v, λ, W) = ClosestOnSimplex(W)                  # keep only the support subset
        if |W| == 4 or ‖v‖² ≤ 1e-12: return intersecting(W)
    return separated: distance ‖v‖, onA = Σ λᵢ aᵢ, onB = Σ λᵢ bᵢ

function ClosestOnSimplex(W):                    # Johnson's sub-algorithm by enumeration (≤ 15 subsets)
    for every non-empty subset S of W (k = |S| ≤ 4):
        if k == 1: λ = (1) else solve G·μ = −Dᵀ·s₀ with D = [s₁ − s₀ … s_{k−1} − s₀], G = DᵀD (Cholesky, skip if singular)
                   λ = (1 − Σμ, μ)
        if all λ ≥ 0: candidate x = Σ λᵢ sᵢ
    return the candidate with the smallest ‖x‖ (the closest point of the hull lies in one face's relative interior)

function Query(a, b):
    core = Gjk(a, b, CoreSupport)
    if core separated:                           # exact whether or not the radii overlap
        n = (onB − onA)/‖·‖
        return { core.distance − r_a − r_b, onA + r_a·n, onB − r_b·n, n, true }   # negative ⇒ rounded overlap
    return Epa(a, b, Gjk(a, b, Support).simplex) # cores intersect

function Epa(a, b, W):                           # cores overlap: expand a polytope of A ⊖ B
    drop duplicate points of W; then add supports along the 6 axes and 8 diagonals, keeping a point only if
    it raises the rank, until four points span a volume
    c = centroid of the tetrahedron              # orient faces away from c, NOT from the origin:
    faces = 4 triangles with outward normals     # the origin may lie on the boundary (touching, axis-aligned boxes)
    repeat at most MaxFaces times:
        f = face with the smallest distance n_f·v_f to the origin;  w = Support_a(n_f) − Support_b(−n_f)
        if w·n_f − dist_f < 1e-6: break                                   # converged
        remove faces that see w, coplanar ones included (n·(w − v) > −1e-6·scale); collect the horizon edges
        (edges seen once, kept in order)
        add one face (edge, w) per horizon edge;  if faces > MaxFaces: converged = false; break
    depth = dist_f;  normal = n_f;  λ = barycentric of depth·n_f in f
    return { −depth, Σλ·a_witness, Σλ·b_witness, normal, converged }
```

## Complexity & memory

- GJK: typically 2–5 iterations for polytopes, each up to 15 subset solves (≤ 3×3). EPA: up to 21 iterations
  and 44 faces for deep random box overlaps in the validation, so `MaxFaces = 64` leaves headroom.
- Memory: `MaxFaces` triangles (3 indices plus a normal and a distance each), `MaxFaces/2 + 2` polytope
  vertices with witness points (a closed triangle mesh has `F = 2V − 4`), a horizon list of at most
  `MaxFaces` edges. All on the stack; with
  `MaxFaces = 64` that is about 6 KB in `float`. No heap.

## Numerical / embedded notes

- **Core + margin.** Running GJK on the cores (point, segment) of rounded shapes and subtracting the radii
  is exact and converges in a couple of iterations. GJK on the full sphere would only approach the answer
  asymptotically. EPA on a smooth shape converges slowly (5e-6 depth error for spheres in the validation),
  so it runs only when the cores themselves intersect.
- **Interior-point orientation.** A blown-up simplex can have the origin exactly on a face (axis-aligned
  boxes, touching shapes). Orienting the faces from the origin then gives inconsistent normals and wrong
  depths. Orienting them away from the initial tetrahedron's centroid fixed exactly that case in the
  validation, where the axis-aligned box pair came out as 0.150756 instead of 0.5.
- **Coplanar faces count as visible.** Box supports with tied coordinates return points inside the faces of
  A ⊖ B. With a strict `> 0` visibility test, a new support point can be collinear with a horizon edge, and
  the zero-area face it creates poisons EPA with NaNs (identical boxes in the validation). Removing coplanar
  faces as well prevents it: 634 axis-aligned overlaps full of ties then match the separating-axis depth
  exactly.
- Johnson's sub-algorithm by explicit enumeration trades a few extra small solves for robustness. Signed
  volumes (Montanari et al., 2017) are a faster drop-in if profiling demands it.
- Float tolerances (`1e-6` relative) bound the iterations; `converged = false` reports budget exhaustion
  instead of looping.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/ConvexCollision.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Query`, and
  `extern template class ConvexCollision<float, 16, 64>;` / `extern template struct ConvexShape<float, 16>;`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/ConvexCollision.cpp` → the same instantiations.
- Test: `robotics/planning/test/TestConvexCollision.cpp`
- Doc: `doc/planning/ConvexCollision.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test`.
- Depends on: NT53 (`SE3Transform`); upstream `Geometry3D`, `CholeskyDecomposition`; P1 in the tests (closed-form references).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
