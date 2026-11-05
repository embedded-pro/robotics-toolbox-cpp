# Primitive Distance Queries

## Overview & Motivation

Collision checking on a robot rarely uses the real link meshes. Arm links are wrapped in **capsules** (a sphere swept along a segment), grippers and heads in **spheres**, and tables, walls and fixtures in **oriented boxes**. These checks run every control cycle in self-collision avoidance, human–robot speed and separation monitoring, and reactive obstacle avoidance. They must be exact, constant-time, branch-light and allocation-free.

The queries return the **closest points** on both shapes and a **signed distance**: positive when the shapes are apart, negative when they overlap, where the magnitude is the penetration depth. The signed value lets one smooth function serve both clearance monitoring and contact response.

## Mathematical Theory

### Rounded Shapes

Every rounded shape is a **core** (a point or a segment) inflated by a radius $r$. For two rounded shapes with core closest points $c_1, c_2$ at core distance $d = \|c_2 - c_1\|$:

$$\delta = d - r_1 - r_2, \qquad n = \frac{c_2 - c_1}{d}, \qquad p_1 = c_1 + r_1 n, \qquad p_2 = c_2 - r_2 n$$

This is exact while the cores are apart. When the cores intersect, the true penetration direction is undefined from the cores alone. The value is then a lower bound on the depth, and a fixed normal is reported when $d \to 0$.

### Point to Segment

For the segment $a + t(b - a)$, $t \in [0, 1]$, the closest parameter is the clamped projection

$$t^\star = \operatorname{clamp}\left(\frac{(p - a)\cdot(b - a)}{\|b - a\|^2},\ 0,\ 1\right)$$

with $t^\star = 0$ for a zero-length segment.

### Segment to Segment

For segments $p_1 + s\,d_1$ and $p_2 + t\,d_2$ with $r = p_1 - p_2$, minimizing $\|p_1 + s d_1 - p_2 - t d_2\|^2$ gives the linear system

$$\begin{bmatrix} a & -b \\ b & -e \end{bmatrix}\begin{bmatrix} s \\ t \end{bmatrix} = \begin{bmatrix} -c \\ -f \end{bmatrix}, \quad a = d_1 \cdot d_1,\ b = d_1 \cdot d_2,\ c = d_1 \cdot r,\ e = d_2 \cdot d_2,\ f = d_2 \cdot r$$

The unconstrained solution is $s = (bf - ce)/(ae - b^2)$. Clamping it to $[0, 1]$, solving $t = (bs + f)/e$, and re-clamping $s$ when $t$ leaves $[0, 1]$ yields the constrained optimum (Ericson, §5.1.9). The function is convex in $(s, t)$, so one pass of this clamp–resolve sequence is enough. Two special cases need their own branch:

- **Parallel segments** ($ae - b^2 \approx 0$): a continuum of closest pairs. Fixing $s = 0$ and resolving $t$ returns one valid pair.
- **Degenerate segments** ($a \approx 0$ or $e \approx 0$): the problem reduces to point–segment or point–point.

### Point to Oriented Box

With box pose $(R, c)$ and half extents $h$, the point is expressed in the box frame, $l = R^T(p - c)$, which needs no matrix inverse.

- **Outside** (some $|l_i| > h_i$): the closest point is the component-wise clamp $q = \operatorname{clamp}(l, -h, h)$ and the distance is $\|l - q\|$.
- **Inside**: the point is pushed out through the nearest face $k = \arg\min_i (h_i - |l_i|)$. The signed distance is $-(h_k - |l_k|)$ and the outward normal is $\operatorname{sign}(l_k)\,R\,e_k$.

A sphere against a box is the point query on its centre, rounded by the radius. Inside the box, the sphere point is offset by $-r$ along the face normal.

## Complexity Analysis

| Query                   | Time   | Space  | Notes                                                    |
|-------------------------|--------|--------|----------------------------------------------------------|
| Point–segment           | $O(1)$ | $O(1)$ | One division, one square root                            |
| Segment–segment         | $O(1)$ | $O(1)$ | Up to three divisions, one square root, no iteration     |
| Point–box, sphere–box   | $O(1)$ | $O(1)$ | One $3 \times 3$ rotation, three clamps                  |
| Rounded pairs           | $O(1)$ | $O(1)$ | Core query plus one normalization                        |

There are no loops beyond the fixed three box axes, so timing is fully deterministic.

## Step-by-Step Walkthrough

**Segments** $[(0,0,0), (1,2,3)]$ and $[(2,0,1), (0,1,-1)]$:

1. $d_1 = (1,2,3)$, $d_2 = (-2,1,-2)$, $r = (-2,0,-1)$, giving $a = 14$, $b = -6$, $c = -5$, $e = 9$, $f = 6$.
2. $ae - b^2 = 90$, so $s = (bf - ce)/90 = (-36 + 45)/90 = 0.1$, inside $[0,1]$.
3. $t = (bs + f)/e = (-0.6 + 6)/9 = 0.6$, also inside.
4. Closest points $(0.1, 0.2, 0.3)$ and $(0.8, 0.6, -0.2)$, distance $\sqrt{0.9} = 0.9487$.

**Point** $(2, 1, 0)$ against a box rotated $30°$ about $z$, centred at $(1,0,0)$, half extents $(0.5, 0.25, 0.1)$:

1. $l = R^T (1, 1, 0) = (1.366, 0.366, 0)$, outside on $x$ and $y$.
2. Clamp to $q = (0.5, 0.25, 0)$, so $l - q = (0.866, 0.116, 0)$ and the distance is $0.8738$.
3. The closest surface point is $c + Rq = (1.308, 0.4665, 0)$.

## Pitfalls & Edge Cases

- **Tolerances on squared lengths**: degeneracy tests compare squared lengths with a tiny threshold. A threshold on the length itself would reject legitimately short links.
- **Parallel segments**: the solver must not divide by $ae - b^2$ when it vanishes. The pair it returns is one of many, so callers must not rely on which one.
- **Penetrating cores**: for capsules whose axes cross, the reported depth is a lower bound and the normal is arbitrary. Use a convex-penetration algorithm (EPA) when the exact depth matters.
- **Points on a box face**: classified as inside with zero depth, so the distance is continuous across the surface.
- **Pairs without a closed form**: capsule–box, box–box and convex hulls have no compact closed form. Use GJK/EPA rather than approximating them with these formulas.
- **Single precision**: distances near zero lose relative accuracy. Apply safety margins in the monitor, not by shrinking the shapes.

## Variants & Generalizations

- **Swept volumes**: continuous collision checking sweeps a capsule over a time step, so it becomes a segment–segment query in space-time.
- **Axis-aligned boxes**: the rotation step disappears, which is useful for broad-phase culling.
- **Distance derivatives**: the gradient of the signed distance with respect to a point is the unit normal $n$, which repulsive potential fields and collision-avoidance constraints use.
- **Spheres as degenerate capsules**: a zero-length capsule axis reproduces the sphere queries, so one capsule–capsule kernel can serve both.

## Applications

- Self-collision checking of an arm modelled as one capsule per link.
- Speed and separation monitoring between the robot and a human tracked as capsules and spheres.
- Repulsive terms in artificial potential fields and distance constraints in reactive controllers.
- Narrow-phase checks in sampling-based planners after a coarse broad phase.

## Connections to Other Algorithms

- [Forward Kinematics](../kinematics/ForwardKinematics.md): supplies the joint positions that become the capsule axes of an arm.
- The oriented-box pose uses the SE(3) rigid transform from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Ericson, C. (2004). *Real-Time Collision Detection*. Morgan Kaufmann. Chapter 5.
- Eberly, D. (2006). *3D Game Engine Design*. 2nd ed. Morgan Kaufmann. Chapter 14.
- Gilbert, E., Johnson, D. and Keerthi, S. (1988). "A fast procedure for computing the distance between complex objects in three-dimensional space." *IEEE J. Robotics and Automation*, 4(2), 193–203.
