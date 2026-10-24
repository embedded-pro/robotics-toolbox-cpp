# GJK + EPA Convex Collision — Overview

## What it is
A general method for measuring the distance between two convex shapes, or how deeply they overlap,
that works for any convex shape that can answer one question: "which of your points lies farthest in this
direction?" That covers spheres, capsules, boxes and convex hulls of a few points.

## Why it matters (embedded)
Real robots and their surroundings need more than spheres and capsules: boxes for tables and shelves,
hulls for grippers and tools. GJK gives exact distances for every combination with one compact algorithm.
EPA adds the penetration depth and direction that contact handling and collision recovery need. Both run
in bounded time and fixed memory.

## How it works (intuition)
Two shapes overlap exactly when their "difference" set, every point of one minus every point of the
other, contains the origin; their distance is how far that set is from the origin. GJK never builds the set.
It walks towards the origin with a small simplex of up to four points, each found by asking the shapes
for their farthest points in a direction. Rounded shapes are handled as a core plus a radius, so their
distance is exact. When the cores overlap, EPA starts from GJK's final simplex and grows a polytope inside
the difference set until its face nearest the origin stops moving. That face gives the depth and the
contact normal.

## Key parameters
- **shape descriptions** — kind, pose, radius, box half extents, hull vertices (bounded count).
- **face budget** — compile-time cap on the EPA polytope.

## Reference
E. G. Gilbert, D. W. Johnson, S. S. Keerthi, "A fast procedure for computing the distance between complex
objects in three-dimensional space," *IEEE J. Robotics and Automation*, 4(2), 1988; G. van den Bergen,
*Collision Detection in Interactive 3D Environments* (2003), Ch. 4.

## See also
`PrimitiveDistance` (P1, closed forms and test references), `SelfCollision` (P4), `math::SE3Transform` (NT53).
