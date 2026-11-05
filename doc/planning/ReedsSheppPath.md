# Reeds–Shepp Shortest Paths

## Overview & Motivation

Dubins paths assume a car that only drives forward. Real cars, forklifts and tractors reverse: parallel parking, three-point turns and docking all rely on it. Reeds and Shepp (1990) extended Dubins' result to a car that may drive **both forwards and backwards** at a bounded curvature.
The shortest path is then one of **48 words** of at most five segments, arcs at the minimum turning radius or straight lines, with at most two **cusps** (changes of direction).

The extension matters for planning beyond parking. Because the car can back up, a small change of the goal only changes the shortest path a little. The Reeds–Shepp length is therefore **continuous** in the goal pose, and it is never longer than the Dubins length. That makes it the preferred distance metric and local connector for sampling planners of car-like robots.

## Mathematical Theory

### Normalised Problem

The goal is expressed in the start frame and scaled to unit turning radius:

$$T_{\text{rel}} = T_0^{-1} T_1, \qquad (x, y) = \frac{t_{\text{rel}}}{\rho}, \qquad \varphi = \theta_{\text{rel}}$$

Each segment has a signed length: positive drives forwards, negative reverses. The total cost is $\rho \sum_i |\ell_i|$.

### Base Formulas

Eight closed-form base formulas cover the path families, named after the paper's equations: two $CSC$ words (8.1 LSL and 8.2 LSR), one $C|C|C$ word (8.3), two $CC|CC$ words (8.7 and 8.8), two $C|C_{\pi/2}SC$ words (8.9 and 8.10) and one $C|C_{\pi/2}SC_{\pi/2}|C$ word (8.11). Here $|$ marks a cusp and $C_{\pi/2}$ a quarter-turn arc. For LSL, for example:

$$(u, t) = \operatorname{polar}(x - \sin\varphi,\; y - 1 + \cos\varphi), \qquad v = \operatorname{wrap}(\varphi - t)$$

The word exists if $t \ge 0$ and $v \ge 0$. Every other formula has a similar existence condition on its intermediate distances and angles. Formulas 8.3 and 8.11 contain typographical errors in the original paper. The corrected forms used here are those of OMPL's implementation.

### Symmetries

Each base formula, evaluated at a transformed goal, gives the remaining words:

| Symmetry                 | Goal evaluated      | Letters  | Lengths  |
|--------------------------|---------------------|----------|----------|
| Direct                   | $(x, y, \varphi)$   | as given | as given |
| Time flip                | $(-x, y, -\varphi)$ | as given | negated  |
| Reflection               | $(x, -y, -\varphi)$ | L ↔ R    | as given |
| Time flip and reflection | $(-x, -y, \varphi)$ | L ↔ R    | negated  |

A time flip drives the same word backwards. A reflection mirrors it across the $x$ axis. The $C|C|C$ and $C|C_{\pi/2}SC$ families also have **backwards** variants: the goal is replaced by $(x\cos\varphi + y\sin\varphi,\; x\sin\varphi - y\cos\varphi,\; \varphi)$, and the letters and lengths are reversed. With all symmetries, 44 candidates are evaluated, and the shortest wins.
On a tie, the first candidate found is kept.

### Tolerances

In single precision, existence conditions that hold with equality on boundary goals come out slightly violated. Each condition accepts a tolerance of $10^{-5}$ in units of $\rho$.

## Complexity Analysis

| Operation     | Time   | Space  | Notes                                                    |
|---------------|--------|--------|----------------------------------------------------------|
| Shortest path | $O(1)$ | $O(1)$ | 44 closed-form candidates, about 150 trigonometric calls |
| Sample        | $O(1)$ | $O(1)$ | At most five SE(2) exponentials                          |
| Car path      | —      | $O(1)$ | Up to five segments with signed lengths                  |

## Step-by-Step Walkthrough

From the origin facing along $x$, with $\rho = 1$ m:

1. **Straight reverse**: to $(-3, 0, 0)$, a single straight segment of $-3$ m.
2. **Forward when shortest**: to $(3, 0.5, 0.2)$, an all-forward LSL of $0.170$, $2.842$ and $0.030$, $3.042$ m in total, as Dubins would give.
3. **Sideways shift**: to $(0, 1, 0)$, a path with forward and reversing arcs and two cusps, $2.636$ m. Dubins needs $7.283$ m.
4. **Reversed quarter turn**: to $(-2, 1, \pi/2)$, an LSRL that reverses $0.730$, $0.236$ and a quarter turn, then drives forward $0.730$, $3.266$ m in total. The best $C|C|C$ alternative is only $0.0006$ m longer.
5. **Three arcs with a cusp**: to $(-1, -1, 0.5)$, LRL with $0.166$ forward, then $0.966$ and $0.632$ in reverse, $1.764$ m in total.
6. **Turning around**: reversing the heading on the spot takes three arcs of $\pi/3$ (forward, reverse, forward), $\pi$ m in total.
7. **Sanity relations**: over 80 goals around a rotated start with $\rho = 1.5$ m, every path is no longer than its Dubins counterpart, has the same length in both directions and reaches its goal.

## Pitfalls & Edge Cases

- **Typos in the paper**: implementing equations 8.3 and 8.11 exactly as printed gives paths that miss their goals. Use the corrected forms.
- **Cusps**: every cusp stops the vehicle. The shortest path is not always the fastest, so weigh cusps when time matters.
- **Curvature discontinuity**: arcs and straights join with a jump in curvature. Smooth the path or track it with a margin.
- **Cost**: 44 candidates per query is cheap once, but expensive inside an inner loop that runs millions of times.
- **Obstacles**: the paths ignore obstacles. Check them for collision before use.

## Variants & Generalizations

- **Dubins paths**: the forward-only special case with six words.
- **Continuous-curvature Reeds–Shepp**: clothoid transitions that respect a steering-rate limit.
- **Weighted reversing**: penalising backward segments or cusps in the cost.
- **Hybrid A-star**: grid search over car motions with Reeds–Shepp paths as the analytic expansion to the goal.

## Applications

- Parking and docking for autonomous cars, forklifts and delivery robots.
- Distance metric and steering function for sampling planners of car-like vehicles.
- Heuristic and analytic goal connection in Hybrid A-star.

## Connections to Other Algorithms

- [Dubins Shortest Paths](DubinsPath.md): the forward-only planner, and the car path type both share.
- [RRT-Connect](RrtConnect.md): a sampling planner where Reeds–Shepp paths can replace straight-line steering.
- [Car-Like Kinematics](../mobile/CarLikeKinematics.md): the vehicle model with the minimum turning radius.
- The SE(2) transforms and angle wrapping come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Reeds, J.A. and Shepp, L.A. (1990). "Optimal paths for a car that goes both forwards and backwards." *Pacific Journal of Mathematics*, 145(2), 367–393.
- Şucan, I.A., Moll, M. and Kavraki, L.E. (2012). "The Open Motion Planning Library." *IEEE Robotics & Automation Magazine*, 19(4), 72–82.
- LaValle, S.M. (2006). *Planning Algorithms*. Cambridge University Press. Chapter 15.3.
