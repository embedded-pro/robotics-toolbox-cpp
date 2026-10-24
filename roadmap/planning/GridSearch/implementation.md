# A* / Dijkstra on Bounded Occupancy Grids — Implementation Pseudocode

> Roadmap ref: #P3 (★★★☆☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Optimal shortest paths on 2-D or 3-D grids with per-cell traversal costs, all in compile-time storage. It
uses an indexed binary heap with decrease-key, so each cell sits in the heap at most once. Index types
shrink to 16 bits when the grid has at most 65 535 cells, and the heuristic is the admissible octile
distance. Cell costs come from a callable, so the same search runs on E2's occupancy grid, a costmap, or a
voxel map without copying.

## Data structures

```cpp
enum class Connectivity : std::uint8_t { faces, full }      # 4 / 8 neighbours in 2-D, 6 / 26 in 3-D
enum class SearchMode : std::uint8_t { aStar, dijkstra }

struct GridIndex { std::uint16_t x, y, z; }

template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct GridSearchParameters:
    Connectivity connectivity
    SearchMode   mode
    bool         allowCornerCutting{ false }    # diagonal moves past a blocked side neighbour
    T            minimumCellCost{ 1 }           # lower bound of the cost callable: keeps the heuristic admissible

template<typename T, std::size_t Width, std::size_t Height, std::size_t Depth = 1>
class GridSearch:
    static constexpr std::size_t Cells = Width·Height·Depth
    using Index = std::conditional_t<Cells < 65535, std::uint16_t, std::uint32_t>    # max value = "none"
    std::array<T, Cells>            costFromStart        # g
    std::array<Index, Cells>        parent
    std::array<Index, Cells>        heap                 # cell indices ordered by f = g + h
    std::array<Index, Cells>        heapSlot             # position of each cell in the heap
    std::array<std::uint8_t, Cells> state                # unseen / open / closed
    std::size_t heapSize, expanded
    GridSearchParameters<T> parameters
```

## Interface

```cpp
explicit GridSearch(const GridSearchParameters<T>& parameters)
template<class CellCost>
std::optional<T> Search(GridIndex start, GridIndex goal, CellCost&& cost)   # cost(GridIndex) ⇒ std::optional<T>; nullopt = blocked
std::size_t ExtractPath(std::span<GridIndex> out) const                     # start → goal; 0 if `out` is too small or no path
std::size_t Expanded() const
```

## Algorithm (pseudocode)

```text
function Heuristic(c):                           # octile distance × cheapest cell
    sort |Δx|, |Δy|, |Δz| descending into (d₁, d₂, d₃)
    return mode == aStar ? minimumCellCost·(d₁ + (√2 − 1)·d₂ + (√3 − √2)·d₃) : 0
    # faces connectivity: Manhattan d₁ + d₂ + d₃

function Search(s, g, cost):                     # OPTIMIZE_FOR_SPEED
    if cost(s) or cost(g) is nullopt: return nullopt
    reset state/heap;  g[s] = 0;  push s with f = Heuristic(s)
    while heap not empty:
        c = pop min f (ties: larger g first, so the search dives towards the goal)
        state[c] = closed;  expanded += 1
        if c == g: return g[c]
        for each neighbour n of c (connectivity, inside the grid):
            if state[n] == closed: continue
            k = cost(n);  if k is nullopt: continue
            if diagonal and not allowCornerCutting and a side neighbour sharing the move is blocked: continue
            candidate = g[c] + stepLength(c, n)·k      # stepLength ∈ {1, √2, √3}
            if state[n] == unseen: g[n] = candidate, parent[n] = c, push n
            elif candidate < g[n]:  g[n] = candidate, parent[n] = c, decrease-key n
    return nullopt                               # goal unreachable

function ExtractPath(out):                       # walk parents from the goal, then reverse in place
```

## Complexity & memory

- `O(C log C)` time for `C` reachable cells. Each cell is pushed once and sifted on decrease-key.
- Memory per cell: `sizeof(T) + 3·sizeof(Index) + 1`. A 100 × 100 `float` grid uses 16-bit indices:
  11 bytes per cell, 110 KB. A 32³ voxel grid is 360 KB. Size the grid to the platform. No heap.

## Numerical / embedded notes

- **Admissibility.** The octile heuristic is exact on an empty grid of unit cost. Scaled by the minimum
  cell cost it never overestimates, so A* returns the same optimal cost as Dijkstra (tested on a weighted
  map) while expanding fewer cells.
- **Corner cutting** is off by default: a diagonal step between two blocked side cells would clip the
  obstacle corners with any real footprint. Inflate obstacles by the robot radius before searching.
- **Ties.** Equal-cost paths are common on grids; which one is returned depends on tie-breaking. Tests
  check the cost, the path length and its validity, not the exact cell sequence.
- `GridIndex` uses 16-bit coordinates, so each grid dimension must be ≤ 65 535.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/GridSearch.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Search`, and
  `extern template class GridSearch<float, 64, 64>;` / `GridSearch<float, 16, 16, 16>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/planning/GridSearch.cpp` → the same instantiations.
- Test: `robotics/planning/test/TestGridSearch.cpp`
- Doc: `doc/planning/GridSearch.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test`.
- Depends on: nothing in the library (E2's map plugs in through the cost callable).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
