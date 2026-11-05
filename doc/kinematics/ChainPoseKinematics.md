# Chain Pose Kinematics

## Overview & Motivation

Position-only forward kinematics answers "where is the tool point?". That is enough for reaching a point, but grasping, welding, screwing or keeping a camera level also needs the tool **orientation**.
Every Jacobian-based algorithm also needs each joint's **axis** and **origin** in one common frame:

- inverse kinematics;
- impedance and operational-space control;
- manipulability analysis.

Chain pose kinematics computes all of this in the same single pass over the chain. It returns the full tool pose, a rigid transform in SE(3), together with a **frame chain**: the origin, axis and type of every joint in the base frame.
The frame chain is model-independent, so a Jacobian routine consumes it without knowing whether the arm was described link by link, by Denavit–Hartenberg parameters or by screw axes.

## Mathematical Theory

### Rigid Transforms

A pose is a pair $(R, p)$ with $R \in SO(3)$ and $p \in \mathbb{R}^3$, composed as

$$(R_a, p_a)\,(R_b, p_b) = (R_a R_b,\; R_a p_b + p_a)$$

Each joint contributes a transform from its link frame to the parent frame, $X_i(q_i) = (R_i(q_i), r_i(q_i))$. A revolute joint rotates about its axis $\hat{z}_i$ at the fixed offset $r_i$. A prismatic joint keeps $R_i = \mathbf{I}_3$ and moves its offset to $r_i + \hat{z}_i q_i$.

### Frame Chain Recursion

Starting from the base frame $(R, o) = (\mathbf{I}_3, 0)$, for $i = 0, \dots, n-1$:

$$a_i = R\,\hat{z}_i, \qquad (R, o) \leftarrow (R, o)\,X_i(q_i), \qquad o_i = o$$

The axis is taken **before** folding in the joint's own rotation. This makes no difference for a revolute joint, since a rotation leaves its own axis unchanged. For a prismatic joint it is the slide direction in the parent's orientation. The origin $o_i$ is the joint position after its own offset.

After the last joint, the constant tool frame $X_t$, the tool pose in the last link's frame, is appended:

$$T_{\text{tool}} = X_0(q_0)\,X_1(q_1)\cdots X_{n-1}(q_{n-1})\,X_t$$

### Consistency with Position-Only Kinematics

The origins $o_i$ are exactly the joint positions of position-only forward kinematics. With $X_t = (\mathbf{I}_3, t)$, the tool translation equals its tool point. The tool frame's rotation is applied last, so it changes the reported orientation but never the position of a tool offset along the last link.

## Complexity Analysis

| Case | Time   | Space  | Notes                                                                             |
|------|--------|--------|-----------------------------------------------------------------------------------|
| All  | $O(n)$ | $O(n)$ | One axis–angle rotation, two $3 \times 3$ products per joint, $2n$ vectors stored |

The single pass costs the same order as position-only kinematics; the extra work is one matrix–vector product per joint for the axis.

## Step-by-Step Walkthrough

An arm with a $z$-axis base joint raised $0.2$ m, then two $y$-axis joints offset $0.5$ m up and $0.4$ m along $x$, and a tool $0.3$ m along the last link's $x$. Take $q = (\pi/2, \pi/2, 0)$:

1. Joint 0: axis $a_0 = \hat{z}$, origin $o_0 = (0, 0, 0.2)$. Fold in $R_z(\pi/2)$.
2. Joint 1: axis $a_1 = R_z(\pi/2)\,\hat{y} = (-1, 0, 0)$, origin $o_1 = o_0 + R_z(\pi/2)(0, 0, 0.5) = (0, 0, 0.7)$. Fold in $R_y(\pi/2)$.
3. Joint 2: running rotation $R_z(\pi/2) R_y(\pi/2)$ maps $\hat{x}$ to $(0, 0, -1)$, so $o_2 = (0, 0, 0.7) + (0, 0, -0.4) = (0, 0, 0.3)$.
4. Tool: $p_{\text{tool}} = o_2 + (0, 0, -0.3) = (0, 0, 0)$, with orientation $R_z(\pi/2) R_y(\pi/2)$: the arm folds straight down to the base plane.

## Pitfalls & Edge Cases

- **Axis frame**: joint axes are written in each link's own frame. Taking the axis after the joint's own rotation is harmless for revolute joints but wrong for prismatic joints mounted on rotated frames.
- **Tool frame order**: the tool transform is multiplied on the right. Pre-multiplying it would rotate the whole arm instead of the tool.
- **Base offset**: the first joint's offset is the mounting offset of the arm. It moves every origin and the tool, and must not be folded into the tool frame.
- **Drift of composed rotations**: repeated products of rotation matrices drift slowly from orthonormality in single precision. Each evaluation starts from the identity here, so the drift never accumulates across calls.
- **Zero-length links**: coincident origins are valid; they only make some Jacobian columns linearly dependent.

## Variants & Generalizations

- **Denavit–Hartenberg**: four parameters per joint produce the same frame chain with fewer numbers, at the price of convention-dependent frames.
- **Product of exponentials**: screw axes in the base frame give $T = e^{[S_0]q_0} \cdots e^{[S_{n-1}]q_{n-1}} M$, with no per-link frames at all.
- **Per-link poses**: storing every intermediate $(R, o)$ instead of only axes and origins supports collision checking against link geometry.
- **Branched chains**: trees repeat the recursion per branch, sharing prefixes.

## Applications

- Full-pose inverse kinematics and Cartesian trajectory tracking.
- The geometric Jacobian, manipulability measures and task-space control, all built from the frame chain.
- Tool-frame calibration and visualization of joint frames.
- Self-collision models whose capsules run between consecutive joint origins.

## Connections to Other Algorithms

- [Forward Kinematics](ForwardKinematics.md): the position-only special case; its joint positions equal the frame chain's origins.
- [Inverse Kinematics](InverseKinematics.md): iterates on forward kinematics; a pose-level variant needs the orientation computed here.
- [Generic Joint Link](../dynamics/JointLink.md): supplies the per-joint transform for revolute and prismatic joints.
- The SE(3) rigid transform comes from the shared primitives of [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 2.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapter 4.
- Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control*. 3rd ed. Pearson. Chapter 3.
