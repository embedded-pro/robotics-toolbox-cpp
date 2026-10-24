# 2-D Scan Matching (ICP) — Overview

## What it is
A method to work out how a robot moved between two laser scans, or where a scan sits on a map, by
overlaying the scans. It repeatedly pairs each new point with the nearest old point and solves for the
shift and rotation that line the pairs up, until the overlay stops improving.

## Why it matters (embedded)
Scan matching turns a 2-D lidar into an odometry sensor that does not slip, corrects wheel-odometry
drift, and is the front end of most 2-D SLAM systems. On a small processor it has to run with bounded
memory and a bounded number of iterations, and it must notice when the geometry, such as a featureless
corridor, cannot pin the motion down.

## How it works (intuition)
Starting from a guess (usually odometry), the current scan is moved onto the reference and each point is
matched to its closest reference point. Pairs that are too far apart are treated as clutter. In the
simplest variant the best rigid motion for these pairs is computed directly, and the process repeats
with new pairs. Because two scans rarely hit a wall at exactly the same spots, a better variant measures
only how far each point is from the wall (the line through its two nearest reference points). It then
solves for the motion that removes those across-wall distances, which converges faster and lands exactly
on the wall.

## Key parameters
- **metric** — point-to-point or point-to-line.
- **maximum correspondence distance** — the outlier bound.
- **iteration limit** and **convergence tolerances**; **refinement steps** per iteration.

## Reference
P. J. Besl, N. D. McKay, "A method for registration of 3-D shapes," *IEEE T-PAMI*, 14(2), 1992;
A. Censi, "An ICP variant using a point-to-line metric," *ICRA* 2008.

## See also
`OdometryFusion` (E1, consumes the result as a position update), `MonteCarloLocalization` (E4); upstream
`PointSetAlignment` (NT67), `LevenbergMarquardt` (NT64), `SE2Transform` (NT66).
