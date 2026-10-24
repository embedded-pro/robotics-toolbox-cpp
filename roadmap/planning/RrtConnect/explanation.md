# RRT-Connect — Overview

## What it is
A motion planner that finds a collision-free path for a robot arm in joint space by growing two random
trees, one from the start and one from the goal, until they meet. It does not need a map of free space,
only a test that says whether a given configuration collides.

## Why it matters (embedded)
Arms with six or seven joints have configuration spaces far too big for grids. Sampling planners like
RRT-Connect find a path in milliseconds to seconds for typical pick-and-place scenes. With fixed node pools
and an injected random generator they run without allocation and give the same answer every time for a
given seed, which is what an embedded controller and its tests need.

## How it works (intuition)
Each step picks a random configuration and grows one tree a short step towards it, if that step is
collision-free. The other tree then grows towards the new node as far as it can in a straight line. If it
reaches it, the trees are connected and the path is read off by walking both back to their roots. The trees
take turns, which makes the search reach out from both ends at once. The raw path zig-zags, so a
shortcutting pass repeatedly tries to replace a stretch of waypoints with one straight, collision-free
segment.

## Key parameters
- **step size** and **collision resolution** — growth per step and how finely edges are checked.
- **node pool size** (compile-time) and **iteration limit**.
- **sampler** and **validity test** — supplied by the caller; the sampler must not correlate consecutive draws.

## Reference
J. J. Kuffner, S. M. LaValle, "RRT-Connect: An efficient approach to single-query path planning," *ICRA*
2000; R. Geraerts, M. H. Overmars, "Creating high-quality paths for motion planning," *IJRR*, 26(8), 2007
(shortcutting).

## See also
`SelfCollision` (P4) and `ConvexCollision` (P5), the usual validity test; `TimeOptimalPathParameterization` (M27),
time-scales the result.
