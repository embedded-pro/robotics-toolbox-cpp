# Self-Collision Checking for a Link Chain (Capsule Model) — Implementation Pseudocode

> Roadmap ref: #P4 (★★★☆☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Distance-based self-collision checking for a serial arm. Link `i` is a capsule around the segment from
joint `i`'s origin to joint `i+1`'s (the last one ends at the tool point), all taken from M30's
`FrameChain`. Every pair not marked allowed is checked with P1's capsule–capsule distance; adjacent links
are allowed by default because they share a joint. The result is the minimum signed distance and the pair
that attains it, which serves as a binary collision test, a safety margin for P6 and a repulsive input
for P2.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct SelfCollisionResult:
    T minimumDistance                           # signed (P1); +∞ when every pair is allowed
    std::array<std::uint8_t, 2> pair            # links attaining it (i < j)
    bool colliding                              # minimumDistance < margin
    std::size_t checkedPairs

template<typename T, std::size_t NumLinks>      # static_assert(NumLinks ≥ 2 and NumLinks ≤ 32)
class SelfCollision:
    std::array<T, NumLinks>             radii
    std::array<std::uint32_t, NumLinks> allowed  # bit j of allowed[i] ⇒ pair (i, j) skipped; kept symmetric
```

## Interface

```cpp
explicit SelfCollision(const std::array<T, NumLinks>& radii)   # allows (i, i+1) for every i
void Allow(std::size_t i, std::size_t j);  void Disallow(std::size_t i, std::size_t j)
bool IsAllowed(std::size_t i, std::size_t j) const
SelfCollisionResult<T> Check(const kinematics::FrameChain<T, NumLinks>& frames, T margin) const   # hot path
Capsule<T> LinkCapsule(const kinematics::FrameChain<T, NumLinks>& frames, std::size_t link) const
```

## Algorithm (pseudocode)

```text
function LinkCapsule(frames, i):
    end = (i + 1 < N) ? frames.jointOrigins[i + 1] : frames.tool.translation
    return { [frames.jointOrigins[i], end], radii[i] }

function Check(frames, margin):                  # OPTIMIZE_FOR_SPEED
    best = { +∞, (0, 0), false, 0 }
    for i in [0, N), j in (i, N):
        if IsAllowed(i, j): continue
        d = PrimitiveDistance::CapsuleCapsule(LinkCapsule(frames, i), LinkCapsule(frames, j)).distance   # P1
        best.checkedPairs += 1
        if d < best.minimumDistance: best.minimumDistance = d;  best.pair = (i, j)
    best.colliding = best.minimumDistance < margin
    return best
```

## Complexity & memory

- `O(N²)` capsule pairs (`N(N−1)/2` minus allowed pairs). A 7-link arm has 15 non-adjacent pairs, each one
  segment–segment solve.
- Memory: `N` radii and `N` 32-bit masks. No heap.

## Numerical / embedded notes

- **Allowed pairs.** Beyond adjacent links, allow pairs that can never meet because of joint limits, or
  that always overlap in the capsule model (a wrist housing over the forearm). Then a real self-collision
  never hides behind a permanent false positive.
- **Margin.** Checking against a positive margin (a few centimetres) instead of 0 absorbs model error and
  the motion between control cycles; at speed `v` and period `Δt`, use at least `v·Δt`.
- **Coarse model.** Capsules on the joint-to-joint segments under-represent offset links (elbow housings,
  tools). Add radius, or model those parts as extra links in the frame chain.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/SelfCollision.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Check`, and
  `extern template class SelfCollision<float, 6>;` / `<float, 7>` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/SelfCollision.cpp` → the same instantiations.
- Test: `robotics/planning/test/TestSelfCollision.cpp`
- Doc: `doc/planning/SelfCollision.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test`.
- Depends on: P1 (`PrimitiveDistance`, `Capsule`), M30 (`FrameChain`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
