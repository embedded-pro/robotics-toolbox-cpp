# Grid Search (A* and Dijkstra)

## Overview & Motivation

Mobile robots, drones and manipulators often plan on a **grid**: an occupancy map from a laser scanner, a costmap with inflated obstacles, or a voxel map of the workspace. Shortest paths on such grids are the backbone of global planners. They feed local controllers with waypoints and give reference routes for sampling planners.

**Dijkstra's algorithm** expands cells in order of their cost from the start and finds optimal paths. **A\*** adds an admissible heuristic, an optimistic estimate of the remaining cost, so it expands cells towards the goal first. It returns the same optimal cost while visiting far fewer cells.
This implementation keeps all its storage in fixed-size arrays sized by the grid. It uses an indexed binary heap with decrease-key, and reads cell costs through a callable, so any map representation plugs in without copying.

## Mathematical Theory

### Graph Model

Each cell is a node. Neighbours share a face (4-connected in 2-D, 6-connected in 3-D) or, with full connectivity, also an edge or a corner (8- and 26-connected). Moving into cell $n$ along a step that changes $k$ coordinates costs

$$c(m \to n) = \sqrt{k}\,\kappa(n), \qquad k \in \{1, 2, 3\}$$

where $\kappa(n) \ge \kappa_{\min} > 0$ is the cell's traversal cost and blocked cells have no cost.

### Best-First Expansion

Each open cell carries $g(n)$, the best known cost from the start, and a priority $f(n) = g(n) + h(n)$. Cells are expanded in increasing $f$:

1. Pop the open cell with the smallest $f$ and close it. Stop if it is the goal.
2. For each neighbour that is free and not closed, compute $g(m) + c(m \to n)$. If it improves $g(n)$, record $m$ as the parent of $n$ and push $n$, or decrease its key if it is already open.

Ties in $f$ are broken towards larger $g$, which dives towards the goal. Dijkstra is the special case $h \equiv 0$.

### Admissible Heuristic

With $|\Delta x|, |\Delta y|, |\Delta z|$ sorted into $d_1 \ge d_2 \ge d_3$:

$$h_{\text{full}} = \kappa_{\min}\left(d_1 + (\sqrt{2} - 1)\,d_2 + (\sqrt{3} - \sqrt{2})\,d_3\right), \qquad h_{\text{faces}} = \kappa_{\min}\,(d_1 + d_2 + d_3)$$

$h_{\text{full}}$ is the exact octile distance on an empty grid of unit cost, and $h_{\text{faces}}$ the Manhattan distance. Neither ever overestimates the true remaining cost, so A\* stays optimal. Since $h$ is also consistent, every cell is closed once.

### Corner Cutting

By default a diagonal step is rejected when any cell it sweeps past is blocked: the two side cells of a planar diagonal, or the six intermediate cells of a 3-D diagonal. Otherwise the path would clip obstacle corners with any real robot footprint.

## Complexity Analysis

| Case                  | Time          | Space                              | Notes                                               |
|-----------------------|---------------|------------------------------------|-----------------------------------------------------|
| $C$ reachable cells   | $O(C \log C)$ | $O(C)$                             | Each cell pushed once, sifted on decrease-key       |
| Per cell              | —             | $4 + 3\cdot\text{index} + 1$ bytes | 16-bit indices up to 65 534 cells, otherwise 32-bit |
| $100 \times 100$ grid | —             | 110 KB                             | 11 bytes per cell with `float` costs                |
| $32^3$ voxel grid     | —             | 360 KB                             | Still 16-bit indices: 32 768 cells                  |

## Step-by-Step Walkthrough

An $8 \times 6$ map has a wall at $x = 4$ for $y = 0$ to $4$, leaving a gap at $y = 5$. All free cells cost $1$, and the query runs from $(1,1)$ to $(6,1)$.

1. **8-connected A\*.** The path climbs diagonally to $(3,3)$, runs up to the gap, crosses at $(4,5)$, steps diagonally down to $(6,4)$ and descends to the goal: 11 cells, cost $7 + 3\sqrt{2} = 11.243$. Dijkstra returns the same cost.
2. **Effort.** A\* expands 29 cells, Dijkstra 39. With 4-connectivity, the cost is $13$ over 14 cells, with 27 versus 40 expansions.
3. **Weighted cells.** On a free map where cells $x \in \{3, 4\}$, $y \le 2$ cost $5$, the straight route would cost $13$. The optimal path detours through $y = 3$ instead: $(1,1), (2,2), (3,3), (4,3), (5,2), (6,1)$, for $4\sqrt{2} + 1 = 6.657$. A\* expands 10 cells and Dijkstra 37.
4. **No path.** Closing the gap, or placing the goal on the wall, returns no result, and path extraction reports zero cells.

## Pitfalls & Edge Cases

- **Inadmissible heuristic**: a minimum cell cost set above the true minimum makes A\* faster but no longer optimal. Use the smallest traversal cost the callable can return.
- **Footprint**: the search treats the robot as a point. Inflate obstacles by the robot radius before searching, or the path grazes walls.
- **Ties**: grids have many equal-cost paths, and which one is returned depends on tie-breaking. Compare costs and validity, not exact cell sequences.
- **Memory**: storage scales with the full grid, not the explored part. Size the grid to the platform's RAM.
- **Path buffer**: extraction returns zero when the output buffer is shorter than the path. Size it for the worst case or retry with a larger one.
- **Coordinate range**: each dimension is limited to 65 535 cells by the 16-bit coordinates.

## Variants & Generalizations

- **Weighted A\***: inflate the heuristic by $\varepsilon > 1$ for faster, $\varepsilon$-suboptimal paths.
- **Jump point search**: prunes symmetric paths on uniform-cost grids for large speed-ups.
- **D\* Lite and LPA\***: incremental replanning when map cells change.
- **Theta\* and any-angle planners**: allow line-of-sight shortcuts instead of grid-aligned steps.
- **Hybrid A\***: searches over continuous headings for car-like robots.

## Applications

- Global path planning for mobile robots on occupancy grids and costmaps.
- 3-D route planning for drones in voxel maps.
- Coarse workspace routing that seeds sampling-based planners.
- Coverage and exploration planners that need cost-to-go fields.

## Connections to Other Algorithms

- [Artificial Potential Fields](PotentialField.md): a reactive local planner that can follow the waypoints produced here.
- [Primitive Distance Queries](PrimitiveDistance.md): build costmaps by inflating geometric obstacles.
- [Self-Collision Checking](SelfCollision.md): the configuration-space validity test for manipulator planners.

## References & Further Reading

- Dijkstra, E.W. (1959). "A note on two problems in connexion with graphs." *Numerische Mathematik*, 1, 269–271.
- Hart, P.E., Nilsson, N.J. and Raphael, B. (1968). "A formal basis for the heuristic determination of minimum cost paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.
- LaValle, S.M. (2006). *Planning Algorithms*. Cambridge University Press. Chapter 2.
- Koenig, S. and Likhachev, M. (2002). "D\* Lite." *AAAI Conference on Artificial Intelligence*, 476–483.
