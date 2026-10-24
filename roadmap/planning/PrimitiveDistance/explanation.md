# Primitive Distance Queries — Overview

## What it is
Exact formulas for the closest points and the distance between simple shapes: points, line segments,
spheres, capsules (a sphere swept along a segment) and oriented boxes. A negative distance means the
shapes overlap, and its size is how deep.

## Why it matters (embedded)
Robots are usually wrapped in capsules (arm links), spheres (grippers, heads) and boxes (tables, walls) for
collision checking. These checks run every control cycle in self-collision avoidance, human–robot speed
and separation monitoring, and reactive obstacle avoidance, so they must be exact, branch-light and
allocation-free.

## How it works (intuition)
Every rounded shape is a core (a point or a segment) inflated by a radius. The distance between two such
shapes is the distance between their cores minus the two radii, and the closest points move outwards along
the line joining the core points. The core distances are classic geometry: project a point onto a
segment and clamp it, or solve the two-parameter problem of two segments with careful handling of
parallel and zero-length cases. For a box, the point is moved into the box's own frame and clamped to its
half extents; points already inside are pushed out through the nearest face.

## Key parameters
- **shape sizes** — radii, segment endpoints, box pose and half extents.

## Reference
C. Ericson, *Real-Time Collision Detection* (2004), Ch. 5.

## See also
`ConvexCollision` (P5, GJK/EPA for pairs without a closed form), `SelfCollision` (P4), `PotentialField` (P2),
`math::SE3Transform` (NT53).
