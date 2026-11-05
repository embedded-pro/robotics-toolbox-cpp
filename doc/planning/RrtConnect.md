# RRT-Connect

## Overview & Motivation

A manipulator moving among obstacles needs a collision-free path in **joint space**, where every point is a full arm configuration. Grids do not scale beyond three or four dimensions: a six-joint arm at 100 cells per joint has $10^{12}$ cells. Potential fields get stuck in local minima.

**Sampling-based planners** avoid both problems. They explore the space by random sampling and only need a yes/no validity test per configuration. **RRT-Connect** (Kuffner and LaValle, 2000) grows two rapidly-exploring random trees, one from the start and one from the goal, and tries to join them greedily after every step.
It is one of the fastest single-query planners for arms, and it is **probabilistically complete**: if a path exists, the probability of finding it tends to one as the iterations grow.

This implementation stores both trees in fixed-size pools, injects the sampler and the validity test, and adds random shortcutting to smooth the result.

## Mathematical Theory

### Extend

To extend a tree towards a sample $x$:

1. Find the nearest node $q_{\text{near}} = \arg\min_j \|q_j - x\|$.
2. Step towards $x$ by at most $\delta$: $q_{\text{new}} = x$ if $\|x - q_{\text{near}}\| \le \delta$, otherwise $q_{\text{near}} + \delta\,\frac{x - q_{\text{near}}}{\|x - q_{\text{near}}\|}$.
3. Accept $q_{\text{new}}$ if the edge from $q_{\text{near}}$ is collision-free.

The result is **trapped** (edge blocked), **advanced** (one step taken) or **reached** ($x$ itself added).

### Connect

Connect repeats Extend towards the same target until the result is no longer "advanced". It either reaches the target or gets trapped.

### Bidirectional Loop

Each iteration extends tree $A$ towards a random sample. If that succeeds, it connects tree $B$ to $A$'s new node. When $B$ reaches it, the two branches joined at that node form the path. Otherwise the trees swap roles.

The Voronoi bias of nearest-neighbour extension pulls the trees into unexplored regions, and the greedy connect closes the gap quickly once the trees see each other.

### Edge Validity

An edge from $a$ to $b$ is checked at $n = \lceil \|b - a\| / r \rceil$ points $a + (b - a)\,k/n$ for $k = 1, \ldots, n$, where $r$ is the collision resolution.

### Shortcutting

Raw paths are jagged. Random shortcutting repeatedly picks two waypoints $i < j - 1$. If the straight edge between them is free, it removes the waypoints in between. The path never gets longer and stays valid.

## Complexity Analysis

| Operation        | Time                   | Space                       | Notes                                                |
|------------------|------------------------|-----------------------------|------------------------------------------------------|
| Extend           | $O(N + L/r)$           | $O(1)$                      | Linear nearest-neighbour scan plus edge checks       |
| Planning         | $O(I \cdot (N + L/r))$ | $O(N \cdot n)$              | $I$ iterations, $N$ nodes in a fixed pool            |
| Shortcut attempt | $O(L/r + W)$           | $O(1)$                      | One edge check and an in-place removal of $W$ points |
| Storage          | —                      | $2N(n \cdot s + 2) + 2Nn s$ | Two trees and the path buffer; $s$ = scalar size     |

## Step-by-Step Walkthrough

A two-joint configuration space $[-\pi, \pi]^2$ contains a wall, $|q_0| \le 0.3$ for $q_1 \le 1$, open above $q_1 = 1$. The planner uses step $0.3$ rad, collision resolution $0.05$ rad, a 256-node pool per tree and a seeded xorshift32 sampler.

1. **Query** from $(-2, -2)$ to $(2, -2)$: the straight edge crosses the wall, so the trees are grown.
2. **Seed 1**: found after 78 iterations with 70 nodes. The path has 40 waypoints and length 11.53, against the shortest possible 7.50 over the wall top.
3. **Shortcutting**: 100 attempts reduce it to 4 waypoints and length 8.12.
4. **Other seeds**: seed 7 gives 9.83, then 7.84 after shortcutting; seed 12345 gives 11.83, then 8.05.
5. **Trivial query**: from $(-2, 2)$ to $(2, 2)$ the straight edge is free, so the two-point path returns with zero iterations.
6. **Small pool**: with 10 nodes per tree, the pool fills before the trees meet, and the planner reports exhaustion.

## Pitfalls & Edge Cases

- **Correlated samplers**: the trees alternate every iteration, so consecutive samples must be independent. A base-2 Halton sequence consumed in order sends every other sample to the same half of the space, and in the validation the trees stalled at 19 nodes. Use a seeded PRNG, or a separate low-discrepancy sequence per tree.
- **Collision resolution**: discrete edge checks can step over thin obstacles. Choose the resolution below the thinnest obstacle as seen in joint space, scaled by the longest lever arm.
- **Path quality**: raw paths are far from optimal. Shortcut them, then time-parameterize them before execution.
- **Metric**: plain Euclidean distance in joint space treats every joint alike. Weight joints by their lever arm when one dominates Cartesian motion.
- **Pool size**: when the pool is exhausted, the query fails. Size the pools for the hardest expected query, or raise the step size.

## Variants & Generalizations

- **RRT\***: asymptotically optimal rewiring at higher cost per iteration.
- **BiTRRT and T-RRT**: cost-aware trees for low-cost paths in cost maps.
- **PRM**: multi-query roadmaps built once and reused.
- **KPIECE and EST**: exploration guided by coverage estimates for kinodynamic problems.
- **Informed sampling**: restricting samples to an ellipsoid once a solution is known.

## Applications

- Collision-free motion planning for industrial and service manipulators.
- Mobile-manipulator and humanoid planning in high-dimensional spaces.
- Grasp approach and retreat planning.

## Connections to Other Algorithms

- [Self-Collision Checking](SelfCollision.md): a typical component of the validity test.
- [Primitive Distance Queries](PrimitiveDistance.md): environment checks inside the validity test.
- [Grid Search (A* and Dijkstra)](GridSearch.md): the resolution-complete alternative for low-dimensional spaces.
- [Cubic Spline Trajectory](../trajectory/CubicSplineTrajectory.md): turns the waypoints into a smooth joint trajectory.

## References & Further Reading

- Kuffner, J.J. and LaValle, S.M. (2000). "RRT-Connect: an efficient approach to single-query path planning." *IEEE International Conference on Robotics and Automation*, 995–1001.
- LaValle, S.M. (2006). *Planning Algorithms*. Cambridge University Press. Chapter 5.
- Geraerts, R. and Overmars, M.H. (2007). "Creating high-quality paths for motion planning." *International Journal of Robotics Research*, 26(8), 845–863.
- Karaman, S. and Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *International Journal of Robotics Research*, 30(7), 846–894.
