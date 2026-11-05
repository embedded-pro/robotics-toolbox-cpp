# Motion Planning & Collision

Geometric queries and planners for collision-free robot motion.

## Algorithms

| Algorithm                                          | Description                                                                                                                 |
|----------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| [Primitive Distance Queries](PrimitiveDistance.md) | Closed-form closest points and signed distances between points, segments, spheres, capsules and oriented boxes — $O(1)$     |
| [Artificial Potential Fields](PotentialField.md)   | Quadratic-then-conic attraction plus Khatib repulsion as a clamped velocity, with local-minimum detection — $O(M)$          |
| [Self-Collision Checking](SelfCollision.md)        | Joint-to-joint capsules per link, pairwise signed distances with an allowed-collision matrix and a safety margin — $O(n^2)$ |
