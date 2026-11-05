# Self-Collision Checking

## Overview & Motivation

A manipulator can hit itself: a folded elbow drives the forearm into the base, or the wrist swings back into the upper arm. Planners must reject such configurations, controllers must slow down before reaching them, and safety monitors must catch them in real time.

The standard real-time approach wraps each link in a **capsule**, a cylinder with hemispherical caps, i.e. a sphere swept along a segment. Each capsule runs between consecutive joint origins, and every relevant pair of capsules is checked with a closed-form distance.
The result is a **signed minimum distance** plus the pair that attains it. It serves as a binary collision test, as a safety margin for sampling planners, and as the input of a repulsive term for reactive control.

## Mathematical Theory

### Link Model

From the frame chain (joint origins $o_i$ in the base frame and the tool point $t$), link $i$ is the capsule

$$\mathcal{C}_i = \left\{ x : \operatorname{dist}\left(x, [o_i, o_{i+1}]\right) \le r_i \right\}, \qquad o_n \equiv t$$

with a radius $r_i$ chosen to enclose the link's geometry.

### Pairwise Distance

For capsules $i$ and $j$, the signed distance is the segment–segment distance minus both radii:

$$d_{ij} = \min_{s,u \in [0,1]} \left\| o_i + s(o_{i+1} - o_i) - o_j - u(o_{j+1} - o_j) \right\| - r_i - r_j$$

A negative value is a penetration. It is exact while the core segments are apart and a lower bound on the depth once they cross. The arm's self-clearance is

$$d_{\min} = \min_{(i,j) \notin \mathcal{A}} d_{ij}$$

where the **allowed set** $\mathcal{A}$ excludes pairs that need no check.

### Allowed Pairs

Adjacent links share a joint, so their capsules always touch, and they are allowed by default. Further pairs belong in $\mathcal{A}$ when joint limits keep them permanently apart, or when the coarse model makes them permanently overlap (a housing over its neighbour).
A permanent false positive would otherwise hide every real collision behind it. The allowed set is a symmetric bit matrix, so membership is a single test.

### Margin

The configuration is flagged as colliding when $d_{\min} < m$. A positive margin $m$ absorbs model error, sensor latency and the motion between checks. At tool speed $v$ and control period $\Delta t$, use at least $m \ge v\,\Delta t$.

## Complexity Analysis

| Case       | Time     | Space  | Notes                                                               |
|------------|----------|--------|---------------------------------------------------------------------|
| $n$ links  | $O(n^2)$ | $O(n)$ | $n(n-1)/2$ pairs minus allowed ones; one segment–segment solve each |
| 7-link arm | 15 pairs | —      | All non-adjacent pairs; constant work per pair                      |

## Step-by-Step Walkthrough

A planar four-link arm has link lengths $(0.4, 0.8, 0.7, 0.5)$ m and radii $(0.10, 0.08, 0.06, 0.05)$ m. Adjacent pairs are allowed, which leaves three checked pairs: $(0,2)$, $(0,3)$ and $(1,3)$.

1. Straight arm: links 1 and 3 are collinear, with a $0.7$ m gap between their segments. Subtracting the radii gives $d = 0.7 - 0.08 - 0.05 = 0.57$ m. That is the closest pair and the arm is clear.
2. Folded at $q = (0, 2.4, 2.4, 0.5)$: link 2 doubles back over link 0. Their segments come within $0.141905$ m, below the summed radii of $0.16$ m, so $d = -0.018$ m and the arm collides.
3. Allowing $(0, 2)$, for instance because the real geometry clears there, reports the next-closest pair, $(0, 3)$, at $+0.053$ m.
4. At $q = (0, 2.6, 2.2, 1.0)$ the clearance is $0.088$ m: no collision at zero margin, but a $0.1$ m safety margin flags it.

## Pitfalls & Edge Cases

- **Coarse geometry**: joint-to-joint capsules miss offset parts such as elbow housings, cables and tools. Enlarge the radius, or add extra frames that model those parts as separate capsules.
- **Over-allowing**: every allowed pair is a blind spot. Justify each one with joint limits or permanent overlap, never just to silence a warning.
- **Crossing cores**: once two core segments intersect, the penetration depth is only a lower bound and the closest points are arbitrary. Treat any negative value as a collision rather than trusting its magnitude.
- **Tunnelling between checks**: a fast arm can pass through a thin link between two control cycles. Use a margin that covers the motion per period, or swept capsules for continuous checking.
- **Degenerate links**: coincident joint origins make a capsule a sphere. The segment distance handles it without special cases.

## Variants & Generalizations

- **Bounding-volume hierarchies**: sphere trees or OBB trees per link give tighter fits for complex geometry, with a broad phase on the capsules.
- **Convex hulls with GJK/EPA**: exact distances for convex link meshes at higher cost.
- **Swept volumes**: continuous collision checking along a motion segment instead of at sampled configurations.
- **Distance gradients**: the closest-point direction yields a repulsive velocity or a constraint Jacobian for optimization-based control.
- **Learned or precomputed self-collision maps**: offline sampling of the configuration space to build fast classifiers for high-DoF robots.

## Applications

- Validity checking inside sampling-based planners such as RRT-Connect.
- Real-time self-collision avoidance for teleoperated and hand-guided arms.
- Safety monitors that stop or slow the robot before contact.
- Repulsive terms in potential-field or null-space controllers that keep links apart.

## Connections to Other Algorithms

- [Primitive Distance Queries](PrimitiveDistance.md): the capsule–capsule distance evaluated for each pair.
- [Chain Pose Kinematics](../kinematics/ChainPoseKinematics.md): supplies the joint origins and tool point that define the capsules.
- [Artificial Potential Fields](PotentialField.md): consumes the minimum distance and direction as a repulsive input.

## References & Further Reading

- Ericson, C. (2004). *Real-Time Collision Detection*. Morgan Kaufmann. Chapters 4 and 5.
- Sucan, I.A., Moll, M. and Kavraki, L.E. (2012). "The Open Motion Planning Library." *IEEE Robotics & Automation Magazine*, 19(4), 72–82.
- Schulman, J. et al. (2014). "Motion planning with sequential convex optimization and convex collision checking." *International Journal of Robotics Research*, 33(9), 1251–1270.
