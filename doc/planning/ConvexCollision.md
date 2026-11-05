# Convex Collision (GJK and EPA)

## Overview & Motivation

Closed-form distances exist for spheres, capsules and a point against a box, but not for most pairs a planner meets: capsule against box, box against box, or an arbitrary convex hull against anything. Writing a dedicated routine for every pair does not scale.

The **Gilbert–Johnson–Keerthi (GJK)** algorithm computes the distance between any two convex shapes using only their **support functions**, the farthest point of the shape in a given direction. When the shapes overlap, the **expanding polytope algorithm (EPA)** continues from GJK's final simplex to find the penetration depth and contact normal.
Together they give one routine for distance, closest points, penetration depth and contact normal across spheres, capsules, oriented boxes and bounded convex hulls, with bounded memory and iteration counts.

## Mathematical Theory

### Minkowski Difference

Two shapes $A$ and $B$ intersect if and only if their Minkowski difference $A \ominus B = \{a - b\}$ contains the origin. Their distance equals the distance from the origin to $A \ominus B$. The support function of the difference is

$$s_{A \ominus B}(d) = s_A(d) - s_B(-d)$$

so the difference never has to be built explicitly.

### Core and Margin

Spheres and capsules are a **core** (a point or a segment) swept by a radius. GJK runs on the cores, and the radii are subtracted afterwards:

$$d(A, B) = d(\text{core}_A, \text{core}_B) - r_A - r_B$$

This is exact, and a negative value is an exact penetration depth as long as the cores themselves are separated. GJK on a full sphere would only approach the answer asymptotically.

### GJK

Starting from any point of $A \ominus B$, GJK keeps a simplex of up to four support points and the point $v$ of that simplex closest to the origin. Each iteration:

1. Takes a new support point $w = s_{A \ominus B}(-v)$.
2. Stops if $\|v\|^2 - v \cdot w \le \varepsilon\|v\|^2$ (no further progress) or if $w$ repeats.
3. Otherwise adds $w$, computes the closest point of the new simplex with Johnson's sub-algorithm, and keeps only the supporting face.

Johnson's sub-algorithm enumerates the up to 15 faces of the simplex, solves each face's small affine system, and keeps the closest point with non-negative barycentric weights. A full tetrahedron, or a closest point at the origin, means the shapes intersect. Otherwise the witness points follow from the barycentric weights.

### EPA

When the cores intersect, EPA expands a polytope inside $A \ominus B$ that contains the origin:

1. Build a tetrahedron from the GJK simplex, completed with supports along fixed directions when it is degenerate.
2. Repeatedly take the face closest to the origin with normal $n$, and the support point $w = s_{A \ominus B}(n)$.
3. If $w \cdot n$ barely exceeds the face distance, that face lies on the boundary: the depth is its distance and its normal is the contact normal.
4. Otherwise remove every face that sees $w$, including coplanar ones, and connect $w$ to the horizon edges.

Faces are oriented away from the initial tetrahedron's centroid rather than from the origin, because the origin can lie exactly on a face for touching or axis-aligned shapes.

## Complexity Analysis

| Phase           | Time                         | Space  | Notes                                             |
|-----------------|------------------------------|--------|---------------------------------------------------|
| Support mapping | $O(1)$, hulls $O(V)$         | $O(1)$ | $V$ hull vertices                                 |
| GJK             | $O(k \cdot 15)$ small solves | $O(1)$ | Typically 2–5 iterations for polytopes            |
| EPA             | $O(k \cdot F)$               | $O(F)$ | $F \le$ `MaxFaces` faces, about 6 KB for 64 faces |

## Step-by-Step Walkthrough

1. **Spheres**: radius 1 at the origin and radius $0.5$ at $(3, 1, 0)$ are $\sqrt{10} - 1.5 = 1.662$ apart, with the closest point on the first at $(0.949, 0.316, 0)$.
2. **Boxes**: a $1 \times 0.8 \times 0.6$ box at the origin and a smaller box rotated by $0.7$ rad about $(1, 1, 0)$ at $(1.4, 0.6, 0.5)$ are $0.511$ apart. This matches exhaustive vertex–face and edge–edge enumeration.
3. **Rounded overlap**: a unit sphere at $(1.8, 0, 0)$ overlaps a unit-half-extent cube by $0.2$. The cores are separated, so the depth is exact without EPA.
4. **Axis-aligned overlap**: two such cubes $1.5$ apart overlap by $0.5$ along $x$. The origin lies on the blown-up simplex boundary, which the centroid orientation handles.
5. **Random boxes**: across 300 random pairs, separated distances match exact enumeration to $10^{-5}$ and penetration depths match the separating-axis test to $10^{-4}$.

## Pitfalls & Edge Cases

- **Smooth shapes in EPA**: when the cores of a sphere or capsule intersect, EPA approximates a curved surface with facets and converges slowly. Prefer core-separated configurations, or accept a looser tolerance.
- **Origin on the boundary**: orienting faces from the origin fails for touching shapes. Orient them from an interior point.
- **Tied supports**: boxes return supports with tied coordinates. Treating coplanar faces as visible prevents degenerate, zero-area faces.
- **Budgets**: when the face budget is exhausted, the result is reported as not converged, with the best estimate so far.
- **Flat shapes**: a planar hull against a point has a flat Minkowski difference with no interior. EPA cannot build a polytope and reports non-convergence.
- **Non-convex shapes**: GJK requires convexity. Decompose concave geometry into convex parts.

## Variants & Generalizations

- **Signed-volume sub-algorithm**: a faster, more robust replacement for Johnson's enumeration (Montanari et al., 2017).
- **Incremental GJK**: warm-starting from the previous simplex for coherent motion.
- **Continuous collision detection**: conservative advancement and ray casting against $A \ominus B$.
- **Multi-point contact**: clipping-based manifold generation on top of the EPA normal.

## Applications

- Distance queries for collision-free motion planning and safety margins.
- Contact detection and penetration resolution in physics simulation.
- Clearance monitoring between robot links and obstacles.

## Connections to Other Algorithms

- [Primitive Distance Queries](PrimitiveDistance.md): closed forms for the simple pairs, and the references for these tests.
- [Self-Collision Checking](SelfCollision.md): capsule-based link checks that this routine generalizes to boxes and hulls.
- [RRT-Connect](RrtConnect.md): consumes these queries inside its validity test.

## References & Further Reading

- Gilbert, E.G., Johnson, D.W. and Keerthi, S.S. (1988). "A fast procedure for computing the distance between complex objects in three-dimensional space." *IEEE Journal on Robotics and Automation*, 4(2), 193–203.
- van den Bergen, G. (2001). "Proximity queries and penetration depth computation on 3D game objects." *Game Developers Conference*.
- Ericson, C. (2004). *Real-Time Collision Detection*. Morgan Kaufmann. Chapter 9.
- Montanari, M., Petrinic, N. and Barbieri, E. (2017). "Improving the GJK algorithm for faster and more reliable distance queries between convex objects." *ACM Transactions on Graphics*, 36(3), 30.
