# Motion Planning & Collision

Geometric queries and planners for collision-free robot motion.

## Algorithms

| Algorithm                                            | Description                                                                                                                                                                                               |
|------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Primitive Distance Queries](PrimitiveDistance.md)   | Closed-form closest points and signed distances between points, segments, spheres, capsules and oriented boxes — $O(1)$                                                                                   |
| [Artificial Potential Fields](PotentialField.md)     | Quadratic-then-conic attraction plus Khatib repulsion as a clamped velocity, with local-minimum detection — $O(M)$                                                                                        |
| [Self-Collision Checking](SelfCollision.md)          | Joint-to-joint capsules per link, pairwise signed distances with an allowed-collision matrix and a safety margin — $O(n^2)$                                                                               |
| [Convex Collision (GJK and EPA)](ConvexCollision.md) | Distance, witness points, penetration depth and contact normal between spheres, capsules, oriented boxes and convex hulls from their support functions                                                    |
| [Grid Search (A* and Dijkstra)](GridSearch.md)       | Optimal shortest paths on bounded 2-D/3-D grids with per-cell costs, an indexed binary heap and the admissible octile heuristic                                                                           |
| [RRT-Connect](RrtConnect.md)                         | Bidirectional rapidly-exploring random trees in joint space with fixed node pools, injected sampler and validity test, and random shortcutting                                                            |
| [Dubins Shortest Paths](DubinsPath.md)               | Closed-form shortest forward-only paths for a bounded-curvature car from the six Dubins words, with float-safe existence tests and a reusable car path type — $O(1)$                                      |
| [Dynamic Window Approach](DynamicWindowApproach.md)  | Reactive local obstacle avoidance: samples the reachable velocity window, keeps commands that can stop before the closed-form arc clearance and scores heading, clearance and speed — $O(N_v N_\omega M)$ |
