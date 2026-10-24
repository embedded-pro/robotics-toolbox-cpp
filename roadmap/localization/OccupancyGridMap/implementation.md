# Occupancy-Grid Mapping (Log-Odds, Ray Casting) — Implementation Pseudocode

> Roadmap ref: #E2 (★★★☆☆) · Target: `robotics/localization` · Namespace `localization` · Type: `float` (templated on `T`, instantiated for `float` only)

Bounded 2-D occupancy grid updated from range scans with known sensor poses. Each beam marks the cells
it crosses as freer and its end cell as more occupied, adding clamped log-odds along a Bresenham ray.
Readings at maximum range only clear space. The grid also serves as the map for Monte Carlo
localization (E4) and as the obstacle source for grid planners (P3).

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct RangeBeam:
    T range                                     # metres
    T bearing                                   # radians, sensor frame

template<typename T>
struct OccupancyGridParameters:
    T resolution                                # metres per cell
    math::Vector2<T> origin                     # world position of the corner of cell (0, 0)
    T logOddsOccupied                           # added at a hit, e.g. 0.85 (p = 0.70)
    T logOddsFree                               # added on a pass-through, e.g. −0.4 (p = 0.40)
    T logOddsMin, logOddsMax                    # clamp, e.g. [−2, 3.5]: keeps the map able to change
    T maxRange                                  # readings ≥ maxRange mark free space only

struct GridCell { int x; int y; }

template<typename T, std::size_t Width, std::size_t Height>
class OccupancyGridMap:
    std::array<T, Width·Height> logOdds{}       # row-major, 0 = unknown (p = 0.5)
    OccupancyGridParameters<T>  parameters
```

## Interface

```cpp
explicit OccupancyGridMap(const OccupancyGridParameters<T>& parameters)
void Integrate(const math::SE2Transform<T>& sensorPose, std::span<const RangeBeam<T>> beams)   # hot path
GridCell WorldToCell(const math::Vector2<T>& point) const                    # floor((p − origin)/resolution)
math::Vector2<T> CellCenter(GridCell cell) const
bool Contains(GridCell cell) const
T    LogOdds(GridCell cell) const;  void SetLogOdds(GridCell cell, T value)  # clamped; maps loaded from storage
T    Probability(GridCell cell) const                                       # 1 − 1/(1 + e^l)
bool IsOccupied(GridCell cell, T threshold) const                           # l > threshold (0 ⇔ p > 0.5)
const OccupancyGridParameters<T>& Parameters() const
```

## Algorithm (pseudocode)

```text
function Integrate(pose, beams):                 # OPTIMIZE_FOR_SPEED
    start = WorldToCell(pose.translation)
    for each (r, b) in beams:
        hit = r < maxRange;  ρ = min(r, maxRange)
        end = WorldToCell(pose.Apply((ρ cos b, ρ sin b)))
        for each cell c on Bresenham(start, end), in order:
            if not Contains(c): break           # leaves the map: stop the ray
            Δ = (c == end and hit) ? logOddsOccupied : logOddsFree
            logOdds[c] = clamp(logOdds[c] + Δ, logOddsMin, logOddsMax)

function Bresenham(a, b):                        # all octants, integer only
    dx = |b.x − a.x|;  dy = −|b.y − a.y|;  sx = sign(b.x − a.x);  sy = sign(b.y − a.y);  e = dx + dy
    loop: emit a;  if a == b: stop
          e2 = 2e;  if e2 ≥ dy: e += dy, a.x += sx;  if e2 ≤ dx: e += dx, a.y += sy
```

## Complexity & memory

- `Integrate`: `O(Σ beam length / resolution)` cell updates, integer stepping only. A 360-beam scan at
  4 m and 5 cm resolution is about 29 k additions.
- Memory: `Width·Height` scalars. 200 × 200 cells (10 m at 5 cm) take 160 KB in `float`; size the grid to
  the target. No heap.

## Numerical / embedded notes

- **Log odds** turn the Bayesian cell update into an addition. The clamp keeps a cell that has been
  observed many times able to change when the world changes (doors, people).
- **Max-range readings** have no reliable hit. Marking their end cell occupied would paint phantom walls,
  so they only clear.
- The start cell is the sensor's own cell, which is also cleared: a sensor cannot sit inside an obstacle.
- Bresenham visits one cell per major-axis step; diagonal rays skip corner-touching cells. That is the
  standard trade-off; an exact traversal (Amanatides–Woo) is a drop-in replacement for the cell walker.
- Rays that leave the grid stop at its border, and their outside end point is never written.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/localization/OccupancyGridMap.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Integrate`, and `extern template class OccupancyGridMap<float, 64, 64>;` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/localization/OccupancyGridMap.cpp` → `template class OccupancyGridMap<float, 64, 64>;`
- Test: `robotics/localization/test/TestOccupancyGridMap.cpp`
- Doc: `doc/localization/OccupancyGridMap.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.localization ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.localization_test`.
- Depends on: NT66 (`SE2Transform`, `Vector2`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
