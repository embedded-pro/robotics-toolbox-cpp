# Monte Carlo Localization (Likelihood Field) — Implementation Pseudocode

> Roadmap ref: #E4 (★★★★☆) · Target: `robotics/localization` · Namespace `localization` · Type: `float` (templated on `T`, instantiated for `float` only)

Particle-filter localization in an occupancy grid (E2). Upstream `ParticleFilter` (NT70) carries the
particles. The odometry motion model samples the same body-frame increment noise as E3. The
likelihood-field sensor model scores each beam end point by its distance to the nearest occupied cell,
precomputed once per map with a two-pass chamfer transform. Because the filter is multimodal, it can
localize globally (from a spread prior) as well as track. All randomness comes from caller-injected
samplers, so runs are deterministic under a fixed seed.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct MclParameters:
    mobile::OdometryNoise<T> motion             # F1: α₁…α₄
    T hitSigma                                  # σ_hit, m
    T hitWeight, randomWeight                   # z_hit, z_rand (z_hit + z_rand = 1)
    T maxRange                                  # beams ≥ maxRange are skipped
    T resampleFraction                          # ESS threshold as a fraction of N, e.g. 0.5
    std::size_t beamStride                      # use every k-th beam

template<typename T, std::size_t Width, std::size_t Height>
class LikelihoodField:
    std::array<T, Width·Height> distance        # metres to the nearest occupied cell, capped at maxDistance
    OccupancyGridParameters<T>  grid            # resolution and origin of the map it was built from
    T maxDistance

template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
class MonteCarloLocalization:
    filters::ParticleFilter<T, 3, NumParticles> filter      # state (x, y, θ); NT70
    const LikelihoodField<T, Width, Height>&    field
    math::SE2Transform<T>                       sensorMount # sensor in the body frame
    MclParameters<T>                            parameters
```

## Interface

```cpp
LikelihoodField(const OccupancyGridMap<T, Width, Height>& map, T occupiedThreshold, T maxDistance)
T LikelihoodField::Distance(const math::Vector2<T>& world) const                  # maxDistance outside the map

MonteCarloLocalization(const LikelihoodField<T, Width, Height>& field, const math::SE2Transform<T>& sensorMount, const MclParameters<T>& parameters)
template<class Draw>    void Initialize(Draw&& draw)                              # draw(i) ⇒ SE2Transform: global or local prior
template<class Normal>  void InitializeAround(const math::SE2Transform<T>& pose, const math::Vector3<T>& sigma, Normal&& normal)
template<class Normal>  void Predict(const math::SE2Transform<T>& increment, Normal&& normal)              # hot path
template<class Uniform> bool Update(std::span<const RangeBeam<T>> scan, Uniform&& uniform)               # true if it resampled
math::SE2Transform<T> Estimate() const                                            # weighted mean, circular in θ
const filters::ParticleFilter<T, 3, NumParticles>& Particles() const
```

`normal()` returns a standard normal draw and `uniform()` a draw in `[0, 1)`, both of type `T`.

## Algorithm (pseudocode)

```text
function LikelihoodField(map, threshold, cap):   # 8-neighbour chamfer with weights (1, √2), two raster passes
    d[c] = map.LogOdds(c) > threshold ? 0 : ∞
    forward pass  (rows ↑, columns ↑): d[c] = min(d[c], d[W] + 1, d[S] + 1, d[SW] + √2, d[SE] + √2)
    backward pass (rows ↓, columns ↓): d[c] = min(d[c], d[E] + 1, d[N] + 1, d[NE] + √2, d[NW] + √2)
    distance[c] = min(d[c]·resolution, cap)

function Predict(Δ, normal):                     # OPTIMIZE_FOR_SPEED
    (dx, dy, dθ) = Δ.Pose();  σ = mobile::IncrementDeviation(Δ, motion)     # F1
    filter.Predict([&](x, i) {
        noisy = FromPose(dx + σ₀·normal(), dy + σ₁·normal(), dθ + σ₂·normal())
        x = (FromPose(x) * noisy).Pose() })     # heading stays wrapped

function Update(scan, uniform):                  # OPTIMIZE_FOR_SPEED
    filter.Update([&](x) {
        sensor = FromPose(x) * sensorMount;  ℓ = 0
        for every beamStride-th beam (r, b) with r < maxRange:
            d = field.Distance(sensor.Apply((r cos b, r sin b)))
            ℓ += log(z_hit·exp(−d²/(2σ_hit²)) + z_rand/maxRange)
        return ℓ })
    return filter.ResampleIfNeeded(resampleFraction, uniform())

function Estimate():
    (x̄, ȳ) = Σ wᵢ (xᵢ, yᵢ);  θ̄ = atan2(Σ wᵢ sin θᵢ, Σ wᵢ cos θᵢ)
    return FromPose(x̄, ȳ, θ̄)
```

## Complexity & memory

- `Update`: `NumParticles · (beams / stride)` field lookups and `exp`/`log` pairs; 500 particles × 60 beams =
  30 k per scan. `Predict`: `3·NumParticles` normal draws.
- Memory: the field has `Width·Height` scalars (the same as the map); the filter has `8·NumParticles`
  scalars (NT70). No heap.

## Numerical / embedded notes

- **Likelihood field, not ray casting.** Scoring end points against a precomputed distance map costs one
  lookup per beam, against tens of cell steps for a ray-cast beam model. It is also smooth in the pose,
  which keeps particle weights from collapsing. The two-pass chamfer overestimates the Euclidean distance
  by at most 8 % (test: 0.524264 against 0.5), well inside `σ_hit`.
- **`z_rand` floor.** The random-measurement term keeps one unexplained beam (a person, glass) from zeroing
  a particle; with the log-domain filter (NT70) the remaining beams decide.
- **Circular mean.** Averaging headings component-wise fails across ±π, so `Estimate` averages unit
  vectors.
- **Kidnapping.** For recovery from a wrong estimate, inject a few uniformly drawn particles through
  `Particles()` when the average likelihood drops (augmented MCL, Thrun et al. Ch. 8.3).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/localization/MonteCarloLocalization.hpp` (with `LikelihoodField`) — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Predict`/`Update`, and `extern template class LikelihoodField<float, 64, 64>;` /
  `extern template class MonteCarloLocalization<float, 128, 64, 64>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/localization/MonteCarloLocalization.cpp` → the same instantiations.
- Test: `robotics/localization/test/TestMonteCarloLocalization.cpp`
- Doc: `doc/localization/MonteCarloLocalization.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.localization ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.localization_test`.
- Depends on: E2 (`OccupancyGridMap`, `RangeBeam`), F1 (`OdometryNoise`, `IncrementDeviation`, increment
  convention), NT70 (`ParticleFilter`), NT66.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
