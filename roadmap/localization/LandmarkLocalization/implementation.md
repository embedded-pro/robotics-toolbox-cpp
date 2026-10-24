# EKF Landmark Localization (Known Map, Range–Bearing) — Implementation Pseudocode

> Roadmap ref: #E3 (★★★☆☆) · Target: `robotics/localization` · Namespace `localization` · Type: `float` (templated on `T`, instantiated for `float` only)

Pose tracking against a known map of point landmarks (reflectors, fiducials, poles) seen as range and
bearing. The motion step composes the odometry increment (F1) with noise proportional to translation
and rotation. Each observation is either labelled with a landmark id or associated by minimum
Mahalanobis distance. It is χ²-gated and applied with upstream `ResidualKalmanUpdate` (NT74), with the
bearing residual wrapped.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct Landmark:
    math::Vector2<T> position                   # world frame
    std::uint16_t    id

template<typename T>
struct RangeBearing:
    T range, bearing                            # sensor = robot frame
    std::optional<std::uint16_t> id             # known identity (fiducial) or nullopt (association needed)

template<typename T>
struct LandmarkLocalizationNoise:
    mobile::OdometryNoise<T> motion             # F1: α₁…α₄
    T rangeNoise, bearingNoise                             # σ_r (m), σ_b (rad)
    T gate                                                 # χ²(2), e.g. 9.21 (99 %)

template<typename T>
struct LandmarkUpdate:
    filters::KalmanUpdateResult<T> result
    std::optional<std::size_t>     landmark     # index used (nullopt if none in the map)

template<typename T, std::size_t MaxLandmarks>
class LandmarkLocalization:
    std::array<Landmark<T>, MaxLandmarks> map;  std::size_t count
    math::Vector3<T> state                      # (x, y, θ)
    math::SquareMatrix<T, 3> covariance
    LandmarkLocalizationNoise<T> noise
```

## Interface

```cpp
LandmarkLocalization(const math::SE2Transform<T>& pose, const math::SquareMatrix<T, 3>& covariance, const LandmarkLocalizationNoise<T>& noise)
bool SetMap(std::span<const Landmark<T>> landmarks)                 # false if more than MaxLandmarks
void Predict(const math::SE2Transform<T>& increment)                # body-frame motion since the last call; hot path
LandmarkUpdate<T> Update(const RangeBearing<T>& observation)        # hot path
std::optional<T>  Nis(const RangeBearing<T>& observation, std::size_t landmark) const
math::SE2Transform<T> Pose() const;  const math::SquareMatrix<T, 3>& Covariance() const
```

`increment` is `previousOdometry.Inverse() * currentOdometry`, i.e. `Exp(F1::Increment(...), 1)`.

## Algorithm (pseudocode)

```text
function Predict(Δ):                            # OPTIMIZE_FOR_SPEED
    (dx, dy, dθ) = Δ.Pose();  c, s = cos θ, sin θ;  Δs² = dx² + dy²
    F = [[1, 0, −s·dx − c·dy], [0, 1, c·dx − s·dy], [0, 0, 1]]       # ∂(pose ⊕ Δ)/∂pose
    J = blockdiag(R(θ), 1)                                           # ∂(pose ⊕ Δ)/∂Δ
    σ = mobile::IncrementDeviation(Δ, motion)                       # F1: (σ_xy, σ_xy, σ_θ)
    P = CongruenceTransform(F, P) + CongruenceTransform(J, diag(σ²))
    state = (Pose() * Δ).Pose()

function Expected(m):                           # predicted measurement and Jacobian for landmark m
    d = m.position − (x, y);  q = d·d;  ρ = √q
    ẑ = (ρ, AngleDifference(atan2(d.y, d.x), θ))
    H = [[−d.x/ρ, −d.y/ρ, 0], [d.y/q, −d.x/q, −1]]
    return ẑ, H

function Residual(z, m):  (ẑ, H) = Expected(m);  return ((z.range − ẑ.range, AngleDifference(z.bearing, ẑ.bearing)), H)

function Update(z):
    k = z.id ? index of the landmark with that id : argmin over map of Nis(z, k)   # nearest neighbour in Mahalanobis distance
    if no such landmark: return { singular, nullopt }
    (r, H) = Residual(z, map[k]);  R = diag(σ_r², σ_b²)
    result = ResidualKalmanUpdate<T, 3, 2>::Apply(state, P, r, H, R, gate);  θ = WrapToPi(θ)
    return { result, k }
```

## Complexity & memory

- `Predict`: two 3×3 congruences. `Update`: `O(MaxLandmarks)` NIS evaluations (2×2 Cholesky each) for an
  unlabelled observation, then one NT74 update.
- Memory: `3·MaxLandmarks` scalars of map plus 12 of state. No heap.

## Numerical / embedded notes

- **Increment noise in the body frame.** F1's `IncrementDeviation` adds the noise to the body-frame
  increment, which `J` rotates into the world. Forward and backward motion are treated alike (test: a
  reverse step gives the same covariance as a forward one), unlike the rotate–translate–rotate split.
- **Bearing wrap.** A landmark straight behind the robot sits on the ±π seam. The wrapped residual is
  0.026665 where the raw difference is −6.256520; the test checks the update stays small.
- **Association.** Nearest neighbour by NIS with a gate is the classic low-cost choice. Where two
  landmarks are inside the gate together, prefer labelled observations, or space landmarks wider than
  the 3σ position uncertainty.
- Observations taken at one time are applied one after another; each update refines `P` for the next.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/localization/LandmarkLocalization.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Predict`/`Update`, and `extern template class LandmarkLocalization<float, 16>;` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/localization/LandmarkLocalization.cpp` → `template class LandmarkLocalization<float, 16>;`
- Test: `robotics/localization/test/TestLandmarkLocalization.cpp`
- Doc: `doc/localization/LandmarkLocalization.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.localization ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.localization_test`.
- Depends on: F1 (`OdometryNoise`, `IncrementDeviation`, increment convention), NT66, NT54, NT74 (`ResidualKalmanUpdate`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
