# Nearest Inverse-Kinematics Solution

## Overview & Motivation

A closed-form inverse-kinematics solver returns every joint configuration that reaches a pose: up to eight for an ortho-parallel arm with a spherical wrist. A controller can use only one. Two facts decide which:

- **Joint limits.** A solution the joints cannot reach is no solution at all. Revolute solvers return angles wrapped to $(-\pi, \pi]$. A joint that turns further than half a revolution reaches the same pose at $\theta \pm 2\pi$. That equivalent angle may be the one inside the limits.
- **Continuity.** Streaming poses along a line or a jog calls the solver every control period. Picking the solution nearest the current joints keeps the arm on its present branch, so it never flips elbow or wrist between two samples.

The selection turns the solver's candidate set into one joint vector, or a reason why there is none.

## Mathematical Theory

### Equivalent Angles Within the Limits

For a revolute joint $j$ with limits $[q_{j,\min}, q_{j,\max}]$, a candidate angle $\theta_j$ has the equivalents

$$\theta_j + 2\pi k, \qquad k \in \{0, +1, -1\}$$

Among those inside the limits, the one closest to the current angle $q_j$ is kept:

$$\tilde{\theta}_j = \arg\min_{k \,:\, q_{j,\min} \le \theta_j + 2\pi k \le q_{j,\max}} \left| \theta_j + 2\pi k - q_j \right|$$

If no equivalent lies inside the limits, the candidate is rejected. One turn either way covers every joint whose range spans at most three revolutions.

### Nearest Candidate

Each candidate that survives the wrapping is compared with the current joints by squared Euclidean distance in joint space:

$$d = \lVert \tilde{\theta} - q \rVert^2$$

The candidate with the smallest $d$ is the result. The outcome is one of three:

| Outcome        | Meaning                                                               |
|----------------|-----------------------------------------------------------------------|
| Found          | At least one candidate fits the limits; the nearest is returned       |
| Outside limits | The pose is reachable, but every candidate leaves some joint's limits |
| Unreachable    | The solver returned no candidate at all                               |

The distinction matters to a caller. An unreachable pose is a planning error. A pose outside the limits may be reachable with another tool orientation or with wider limits.

## Complexity Analysis

| Case | Time        | Space  | Notes                                                         |
|------|-------------|--------|---------------------------------------------------------------|
| Any  | $O(c \, n)$ | $O(n)$ | $c$ candidates, $n$ joints, three equivalents tried per joint |

For an OPW arm, $c = 8$ and $n = 6$: at most 144 limit checks and eight distances.

## Step-by-Step Walkthrough

Two joints, limits $[-4, 4]$ rad on both, current joints $q = (3.0, 0)$, and one candidate $\theta = (-3.0, 0)$.

1. Joint 1: the equivalents are $-3.0$, $3.283$ and $-9.283$. The first two are inside the limits; $3.283$ is $0.283$ from the current $3.0$, against $6.0$ for $-3.0$. Keep $3.283$.
2. Joint 2: $0$ is inside the limits and equal to the current angle. Keep $0$.
3. The wrapped candidate is $(3.283, 0)$ at squared distance $0.080$. It is the only candidate, so it is the result.

Without the wrapping the arm would swing joint 1 through $6$ rad to reach the same pose.

## Pitfalls & Edge Cases

- **Revolute joints only**: the $2\pi$ shift assumes the joint is periodic. A prismatic joint must not be shifted; pass its candidate unchanged.
- **Distance in mixed units**: squared joint distance weighs every joint equally. Joints whose motion is costlier, such as a heavy shoulder, may deserve a weighted metric.
- **Singular wrist**: at a wrist singularity the solver fixes one wrist angle by convention. The nearest solution then keeps that convention; it does not search the free combination of the two wrist joints.
- **Limits at exactly $\pm\pi$**: a candidate on the boundary is inside; the comparison is inclusive.

## Variants & Generalizations

- **Weighted distance**: $d = (\tilde{\theta} - q)^T W (\tilde{\theta} - q)$ with a diagonal $W$ of joint costs.
- **More turns**: joints with a range beyond three revolutions need $k \in \{-2, \dots, 2\}$ or the closed-form $k = \operatorname{round}((q_j - \theta_j) / 2\pi)$ clamped into the limits.
- **Branch locking**: keep the solver's configuration flags (shoulder, elbow, wrist) and choose only among candidates with the same flags.

## Applications

- Streaming Cartesian lines and jogs: one solve and one selection per control period.
- Point-to-point moves to a Cartesian target from the present joints.
- Reachability checks that tell an unreachable pose apart from one blocked by the joint limits.

## Connections to Other Algorithms

- [Analytical Inverse Kinematics (OPW)](AnalyticalIkOpw.md): supplies the eight candidates; its nearest solve applies this selection.
- [Pose Inverse Kinematics](PoseInverseKinematics.md): an iterative solver that starts from the current joints and so stays near them by construction.
- [Generic Joint Link](../dynamics/JointLink.md): carries the joint limits used here.

## References & Further Reading

- M. Brandstötter, A. Angerer, M. Hofbaur, "An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist," Austrian Robotics Workshop, 2014.
- B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009, §2.12.
