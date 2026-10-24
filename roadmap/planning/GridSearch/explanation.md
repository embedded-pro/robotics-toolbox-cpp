# A* / Dijkstra on Bounded Grids — Overview

## What it is
The classic shortest-path search on a grid map: from a start cell to a goal cell, through free cells,
minimising the total travel cost, which can be higher in cells the robot should avoid. Dijkstra explores
evenly in all directions. A* uses an optimistic estimate of the remaining distance to explore towards the
goal first, and returns the same optimal path.

## Why it matters (embedded)
Grid search is the global planner of most mobile robots, from vacuum cleaners to warehouse fleets, and in
3-D it plans for drones in voxel maps. Here all bookkeeping lives in fixed arrays sized at compile time:
no allocation, a known worst-case memory footprint, and small index types when the grid is small.

## How it works (intuition)
The search keeps a priority queue of cells ordered by cost so far plus estimated cost to go. It
repeatedly takes the most promising cell, marks it final and offers its neighbours a cheaper route through
it. When a cheaper route to a queued cell appears, that cell moves forward in the queue instead of being
added twice. The estimate is the "octile" distance, the length of the best path on an empty grid. It never
overestimates, which is what guarantees the result is optimal. When the goal is taken from the queue,
following the stored parents back to the start gives the path.

## Key parameters
- **connectivity** — 4/8 neighbours in 2-D, 6/26 in 3-D.
- **mode** — A-star or Dijkstra; **minimum cell cost** — keeps the A-star estimate admissible.
- **corner cutting** — whether diagonal moves may brush past obstacle corners.

## Reference
P. E. Hart, N. J. Nilsson, B. Raphael, "A formal basis for the heuristic determination of minimum cost
paths," *IEEE Trans. Systems Science and Cybernetics*, 4(2), 1968; E. W. Dijkstra, "A note on two problems
in connexion with graphs," *Numerische Mathematik*, 1, 1959.

## See also
`OccupancyGridMap` (E2, a cost source), `PotentialField` (P2), `PurePursuit` (W5, follows the resulting path).
