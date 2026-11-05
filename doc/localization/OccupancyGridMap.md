# Occupancy-Grid Mapping

## Overview & Motivation

A mobile robot needs a map to plan collision-free motion and to localise itself. The **occupancy grid** (Moravec and Elfes, 1985) is the simplest and most widely used representation: the plane is divided into square cells, and each cell holds the probability that it is occupied.

The grid is built from range scans taken at known sensor poses. Every beam passes through free space and ends on an obstacle, so the cells it crosses become more likely free and its end cell more likely occupied. Storing probabilities as **log odds** turns each Bayesian update into a single addition, and walking each beam with **Bresenham's line algorithm** needs only integer arithmetic.
That makes the update cheap enough for a microcontroller.

## Mathematical Theory

### Log-Odds Representation

A cell with occupancy probability $p$ stores

$$l = \ln\frac{p}{1 - p}, \qquad p = 1 - \frac{1}{1 + e^{l}}$$

An unknown cell has $l = 0$ ($p = 0.5$). With a static world and independent measurements, Bayes' rule for a new observation $z$ becomes an addition of the inverse sensor model:

$$l_k = l_{k-1} + \ln\frac{p(\text{occ} \mid z)}{1 - p(\text{occ} \mid z)}$$

### Inverse Sensor Model

Each beam adds $l_{\text{occ}} > 0$ to the cell containing its end point and $l_{\text{free}} < 0$ to every cell it crosses before. With $l_{\text{occ}} = 0.85$ ($p = 0.70$) and $l_{\text{free}} = -0.4$ ($p = 0.40$), one hit outweighs about two pass-throughs.

Readings at or beyond the maximum range have no reliable end point. They only clear the cells along the beam up to the maximum range, so they never paint phantom walls.

### Clamping

Each cell is clamped to $[l_{\min}, l_{\max}]$. Without the clamp, a cell observed thousands of times would need thousands of contrary observations to change, so the map could not follow doors opening or people moving.

### Ray Traversal

The sensor's cell and the beam's end cell are found by $\lfloor (x - o)/r \rfloor$ for the grid origin $o$ and resolution $r$. Bresenham's algorithm steps one cell at a time along the major axis, deciding with an integer error term when to step along the minor axis. It visits one cell per major-axis step in every octant. A beam that leaves the grid stops at the border.

## Complexity Analysis

| Operation      | Time                 | Space    | Notes                                             |
|----------------|----------------------|----------|---------------------------------------------------|
| Integrate scan | $O(\sum_i \rho_i/r)$ | $O(1)$   | Integer stepping, one addition and clamp per cell |
| Cell query     | $O(1)$               | $O(1)$   | Probability needs one exponential                 |
| Map storage    | —                    | $O(W H)$ | One scalar per cell, no heap                      |

A 360-beam scan at 4 m and 5 cm resolution updates about 29 000 cells. A 10 m by 10 m map at 5 cm takes 160 KB in single precision.

## Step-by-Step Walkthrough

A 1 m by 1 m grid at 10 cm resolution, with the sensor at the centre of cell $(0, 0)$, $l_{\text{occ}} = 0.85$, $l_{\text{free}} = -0.4$, a clamp of $[-2, 3.5]$ and a maximum range of 4 m:

1. **Axis beam**: a 0.5 m reading straight ahead clears cells $(0, 0)$ to $(4, 0)$ and marks $(5, 0)$ occupied.
2. **Shallow beam**: a reading ending at $(0.75, 0.35)$ crosses $(0,0)$, $(1,0)$, $(2,1)$, $(3,1)$, $(4,2)$, $(5,2)$ and $(6,3)$, and hits $(7,3)$.
3. **Maximum range**: with the maximum range lowered to 0.5 m, the same axis reading clears six cells and marks none.
4. **Leaving the map**: a 2 m reading clears the whole first row and stops at the border.
5. **Clamping**: after ten identical scans, the hit cell saturates at $3.5$ ($p = 0.971$) and the sensor cell at $-2$ ($p = 0.119$).
6. **Origin**: with the grid corner at $(-0.5, -0.5)$, the world origin falls in cell $(5, 5)$, whose centre is $(0.05, 0.05)$.

## Pitfalls & Edge Cases

- **Pose errors**: the update assumes the sensor pose is exact. Odometry drift smears walls, so map with a localised pose or with SLAM.
- **Corner cells**: Bresenham skips cells a diagonal beam only touches at a corner. An exact traversal (Amanatides and Woo) visits them at a small extra cost.
- **Invalid readings**: zero or negative ranges mark the sensor's own cell. Filter them before integrating.
- **Dynamic obstacles**: people walking through a scan leave traces. The clamp limits how long they persist.
- **Memory**: the grid size is fixed at compile time. Size it to the area and resolution the target can afford.

## Variants & Generalizations

- **Exact ray traversal**: Amanatides–Woo visits every cell a beam crosses.
- **Beam-endpoint and likelihood fields**: precomputed distance maps for fast scan matching and Monte Carlo localization.
- **Multi-resolution and sparse maps**: quadtrees and OctoMap for large or 3-D environments.
- **Occupancy grid SLAM**: Rao–Blackwellised particle filters and graph optimisation that estimate the poses as well.

## Applications

- Maps for indoor service robots, warehouse vehicles and robot vacuums.
- Obstacle maps for grid planners such as A*.
- The map against which Monte Carlo localization tracks the robot.

## Connections to Other Algorithms

- [Grid Search (A* and Dijkstra)](../planning/GridSearch.md): plans shortest paths on the thresholded grid.
- [Planar Odometry](../mobile/PlanarOdometry.md): provides sensor poses for mapping when no better pose is available.
- The SE(2) pose used to place each beam comes from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Moravec, H. and Elfes, A. (1985). "High resolution maps from wide angle sonar." *IEEE International Conference on Robotics and Automation*, 116–121.
- Elfes, A. (1989). "Using occupancy grids for mobile robot perception and navigation." *Computer*, 22(6), 46–57.
- Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapter 9.
- Bresenham, J.E. (1965). "Algorithm for computer control of a digital plotter." *IBM Systems Journal*, 4(1), 25–30.
- Amanatides, J. and Woo, A. (1987). "A fast voxel traversal algorithm for ray tracing." *Eurographics*, 3–10.
