# Geometric Jacobian

## Overview & Motivation

The geometric Jacobian $J(q) \in \mathbb{R}^{6 \times n}$ is the linear map from joint rates to the tool's spatial velocity:

$$\begin{bmatrix} v \\ \omega \end{bmatrix} = J(q)\,\dot{q}$$

Here $v$ is the linear velocity of the **tool point** and $\omega$ the angular velocity of the tool frame, both in the base frame, linear part first. Nearly every task-space algorithm runs on it:

- damped-least-squares inverse kinematics inverts it;
- manipulability analysis decomposes it;
- redundancy resolution projects into its null space;
- impedance and operational-space control use it to map wrenches back to joint torques.

Its transpose gives the **statics** of the arm: the joint torques that balance a wrench applied at the tool are $\tau = J^T w$.

Built from a **frame chain**, the joint origins, axes and types in the base frame, the Jacobian is independent of how the arm was described. Link-by-link models, Denavit–Hartenberg tables and screw-axis descriptions all reduce to one implementation.

## Mathematical Theory

### Columns

Let $p$ be the tool point, and $o_i$, $\hat{z}_i$ the origin and unit axis of joint $i$ in the base frame. A unit rate of joint $i$ with all others locked moves the tool by

$$J_i = \begin{cases} \begin{bmatrix} \hat{z}_i \times (p - o_i) \\ \hat{z}_i \end{bmatrix} & \text{revolute} \\[2ex] \begin{bmatrix} \hat{z}_i \\ 0 \end{bmatrix} & \text{prismatic} \end{cases}$$

A revolute column depends on $o_i$ only through $p - o_i$ crossed with $\hat{z}_i$, so any point on the joint axis may serve as the origin.

### Statics

By the principle of virtual work, a wrench $w = (f; n)$ applied at the tool point (force first, moment about the tool point) does power $w^T J \dot{q}$, which must equal $\tau^T \dot{q}$ for every $\dot{q}$:

$$\tau = J^T w$$

A force whose line of action passes through every revolute axis produces no joint torque.

### Bias Acceleration

Differentiating $\dot{x} = J\dot{q}$ gives $\ddot{x} = J\ddot{q} + \dot{J}\dot{q}$. The **bias acceleration** $\dot{J}\dot{q}$ is the tool acceleration at zero joint acceleration, needed by operational-space control and Cartesian tracking.
Let $\omega_{i-1}$ be the angular velocity of the link that carries axis $i$; the axis then turns at $\dot{\hat{z}}_i = \omega_{i-1} \times \hat{z}_i$. Differentiating each column gives

$$\dot{J}_i\dot{q}_i = \begin{cases} \dot{q}_i \begin{bmatrix} \dot{\hat{z}}_i \times (p - o_i) + \hat{z}_i \times (\dot{p} - \dot{o}_i) \\ \dot{\hat{z}}_i \end{bmatrix} & \text{revolute} \\[2ex] \dot{q}_i \begin{bmatrix} \dot{\hat{z}}_i \\ 0 \end{bmatrix} & \text{prismatic} \end{cases}$$

Here $\dot{p} = J_v\dot{q}$ is the tool-point velocity and $\dot{o}_i$ the velocity of the joint origin. Both $\omega$ and $\dot{o}$ are accumulated base to tip in the same pass: $\dot{o}_i = \dot{o}_{i-1} + \omega_{i-1} \times (o_i - o_{i-1})$, plus the sliding rate of a prismatic joint. No second derivative of the kinematics is ever formed.

### Task Rows

Position-only tasks use the top three rows (the tool-point linear velocity). Full-pose tasks use all six. The same rows of $\dot{J}\dot{q}$ follow.

## Complexity Analysis

| Quantity          | Time    | Space  | Notes                                                |
|-------------------|---------|--------|------------------------------------------------------|
| Jacobian          | $O(n)$  | $O(n)$ | One cross product per revolute column                |
| Bias acceleration | $O(n)$  | $O(1)$ | One base-to-tip pass after the $O(6n)$ tool velocity |
| Joint torques     | $O(6n)$ | $O(n)$ | One transposed matrix–vector product                 |

## Step-by-Step Walkthrough

A planar two-link arm with unit links and joints about $\hat{z}$, stretched along $x$ ($q = (0, 0)$):

1. Frame chain: $o_0 = (0,0,0)$, $o_1 = (1,0,0)$, $\hat{z}_0 = \hat{z}_1 = \hat{z}$, tool $p = (2,0,0)$.
2. Column 0: $\hat{z} \times (2,0,0) = (0,2,0)$, angular $(0,0,1)$. Column 1: $\hat{z} \times (1,0,0) = (0,1,0)$, angular $(0,0,1)$.
3. $J = \begin{bmatrix} 0 & 0 \\ 2 & 1 \\ 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 1 & 1 \end{bmatrix}$. The linear columns are parallel, so the arm is singular: it cannot move the tool along $x$.
4. A tool force $(1,0,0)$ along the links gives $\tau = J^T w = (0, 0)$: the structure carries it without any joint torque.

## Pitfalls & Edge Cases

- **Row ordering**: this Jacobian is linear-first, $(v;\omega)$. Featherstone's spatial vectors and many robotics libraries order angular first; mixing the two silently swaps blocks.
- **Reference point**: the linear rows are the velocity of the tool point, not of the base origin. The "space Jacobian" of the product-of-exponentials literature differs by a frame shift and is a different matrix.
- **Wrench reference**: in $\tau = J^T w$, the moment must be taken about the tool point. A moment about another point must be shifted first.
- **Singularities**: near a singularity the columns become dependent and any inverse blows up. Use damping or a pseudo-inverse with a rank threshold, and monitor manipulability.
- **Frame-chain conventions**: for a standard Denavit–Hartenberg table, joint $i$ acts about the $z$ axis of the frame **before** its link; for the modified convention, about the frame **of** its link. A wrong frame shifts every column.

## Variants & Generalizations

- **Analytic Jacobian**: derivative of a minimal orientation parameterization (Euler angles, quaternion). It differs from the geometric Jacobian by an orientation-dependent map that is singular for Euler angles.
- **Body Jacobian**: velocities expressed in the tool frame, obtained by rotating both blocks by $R^T$.
- **Space Jacobian**: twist coordinates about the base origin, natural for product-of-exponentials kinematics.
- **Point Jacobians**: the same column formula with $p$ replaced by any point on any link, used for contact points and collision avoidance.

## Applications

- Damped-least-squares and Levenberg–Marquardt inverse kinematics.
- Resolved-rate and Cartesian velocity control; null-space redundancy resolution.
- Impedance, admittance and operational-space control, which map task-space forces to joint torques.
- Force estimation from joint torques and static payload compensation.

## Connections to Other Algorithms

- [Chain Pose Kinematics](ChainPoseKinematics.md): produces the frame chain from a link-by-link model.
- [Denavit-Hartenberg Parameters](DenavitHartenberg.md): produces the same frame chain from a DH table.
- [Inverse Kinematics](InverseKinematics.md): uses the position rows of this Jacobian in its damped-least-squares step.
- [Generic Joint Link](../dynamics/JointLink.md): defines the revolute and prismatic joint types that select each column's form.

## References & Further Reading

- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 3.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapter 5.
- Orin, D.E. and Schrader, W.W. (1984). "Efficient computation of the Jacobian for robot manipulators." *International Journal of Robotics Research*, 3(4), 66–75.
