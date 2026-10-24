# 2-D Scan Matching (Point-to-Point / Point-to-Line ICP) — Implementation Pseudocode

> Roadmap ref: #E5 (★★★★☆) · Target: `robotics/localization` · Namespace `localization` · Type: `float` (templated on `T`, instantiated for `float` only)

Iterative closest point between a reference scan (the previous scan, or points from a map) and the
current scan. It finds the transform from the current sensor frame to the reference frame. Each iteration
associates every current point with its nearest reference point within a distance bound, then solves for
the transform:
- **point-to-point:** weighted Kabsch in 2-D (NT67);
- **point-to-line:** a few Levenberg–Marquardt steps on the normal distances to the local reference
  segments (NT64), with SE(2) retraction (NT66).

Point-to-line converges in fewer iterations and has no sampling bias when the two scans sample the walls
at different spots; the tests show both.

## Data structures

```cpp
enum class IcpMetric : std::uint8_t { pointToPoint, pointToLine }

template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct ScanMatcherParameters:
    IcpMetric   metric
    T           maxCorrespondenceDistance       # pairs farther apart are outliers
    std::size_t maxIterations
    T           translationTolerance, rotationTolerance   # convergence on the per-iteration increment
    std::size_t minCorrespondences              # fail below this (≥ 3)
    std::size_t refinementIterations            # LM steps per ICP iteration (point-to-line), e.g. 3

template<typename T>
struct ScanMatchResult:
    math::SE2Transform<T> transform             # current sensor frame → reference frame
    std::size_t iterations, correspondences
    T rms                                       # over accepted pairs, in the chosen metric
    bool converged

template<typename T, std::size_t MaxPoints>
class ScanMatcher:
    ScanMatcherParameters<T> parameters
    # per-call work arrays on the stack: transformed points, matched reference points, normals, weights (MaxPoints each)
```

## Interface

```cpp
explicit ScanMatcher(const ScanMatcherParameters<T>& parameters)
std::optional<ScanMatchResult<T>> Match(std::span<const math::Vector2<T>> reference,   # ordered by bearing (scan order)
                                        std::span<const math::Vector2<T>> current,
                                        const math::SE2Transform<T>& initialGuess) const
```

`nullopt` when either span exceeds `MaxPoints`, when an iteration has fewer than `minCorrespondences`
pairs, or when an LM step fails (degenerate geometry).

## Algorithm (pseudocode)

```text
function Match(ref, cur, T):
    for it in [1, maxIterations]:
        yᵢ = T.Apply(curᵢ)
        for each i:                              # brute-force nearest neighbour, O(N·M)
            j = argmin_j ‖ref_j − yᵢ‖;  accept if ‖ref_j − yᵢ‖ ≤ maxCorrespondenceDistance (weight 1, else 0)
            if pointToLine:                      # local segment: j and its closer scan-order neighbour
                k = closer of j − 1, j + 1 to yᵢ;  t = ref_k − ref_j;  nᵢ = (−t.y, t.x)/‖t‖
        if accepted < minCorrespondences: return nullopt
        if pointToPoint:
            a = PointSetAlignment<T, 2>::Align(y, matched ref, false, weights)        # NT67
            if not a: return nullopt
            ΔT = SE2 from a.rotation, a.translation
        else:
            N = JᵀJ of PointToLineProblem at Identity                  # 3×3
            if λ_min(N) ≤ 1e-6·λ_max(N): return nullopt                # JacobiEigenSolver<T, 3>: unobservable direction
            ΔT = LevenbergMarquardt over PointToLineProblem (Identity start, refinementIterations steps)  # NT64
        T = ΔT * T
        if ‖ΔT.translation‖ < translationTolerance and |ΔT.Heading()| < rotationTolerance:
            return { T, it, accepted, rms(T), true }
    return { T, maxIterations, accepted, rms(T), false }

class PointToLineProblem : optimization::LeastSquaresProblem<T, SE2Transform<T>, MaxPoints, 3>:
    Residual(D):  eᵢ = wᵢ · nᵢ·(D.Apply(yᵢ) − qᵢ)                    # unused rows are 0
    Jacobian(D):  rowᵢ = wᵢ · [nᵢ.x, nᵢ.y, nᵢ.y·y′ᵢ.x − nᵢ.x·y′ᵢ.y]  with y′ᵢ = D.Apply(yᵢ)
    Retract(D, δ) = FromPose(δ) * D                                   # left perturbation
```

## Complexity & memory

- Association: `O(N·M)` per iteration (66 × 68 points ≈ 4.5 k distances). For full 360-beam scans, use
  scan-order (projective) association or a bounded k-d tree; the matcher's interface does not change.
- Point-to-line: an LM step forms a `MaxPoints × 3` Jacobian and a 3×3 normal system.
- Stack: about `8·MaxPoints` scalars of work arrays plus the LM Jacobian (`3·MaxPoints`). No heap.

## Numerical / embedded notes

- **Sampling bias.** Two scans rarely hit the same wall spots. Point-to-point then pulls the points
  towards the reference samples and settles 3 cm off (test). Point-to-line measures only the distance
  across the wall, so it recovers the transform exactly.
- **Initial guess.** ICP converges locally. Seed it with odometry (F1, E1); with a 0.15 m / 0.1 rad guess
  error the test still converges with point-to-line.
- **Degeneracy.** A corridor (parallel walls only) leaves the motion along the walls unobservable: the
  point-to-line normal matrix `JᵀJ` has a zero eigenvalue. The eigenvalue-ratio check makes `Match` return
  `nullopt` rather than a drifted answer, and the caller falls back to odometry along that axis.
- Outlier points (people, glass) beyond `maxCorrespondenceDistance` get weight 0 and do not move the
  solution (test).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/localization/ScanMatcher.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Match`, and
  `extern template class ScanMatcher<float, 128>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/localization/ScanMatcher.cpp` → `template class ScanMatcher<float, 128>;`
- Test: `robotics/localization/test/TestScanMatcher.cpp`
- Doc: `doc/localization/ScanMatcher.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.localization ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.localization_test`.
- Depends on: NT66 (`SE2Transform`), NT67 (`PointSetAlignment`), NT64 (`LevenbergMarquardt`, `ConstantDamping`);
  upstream `JacobiEigenSolver`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
