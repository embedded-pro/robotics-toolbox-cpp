# Product of Exponentials

## Overview & Motivation

The **product-of-exponentials (PoE)** formula describes a serial arm by one **screw axis** per joint, all written once in the base frame at the home configuration, plus the tool pose at home. Denavit–Hartenberg needs per-link frames placed by convention. PoE needs none: every joint is just a line in space (revolute) or a direction (prismatic), read directly off a drawing or a CAD model.

Forward kinematics becomes a product of matrix exponentials, the screw axes give the **space Jacobian** with almost no extra work, and the formulation degrades gracefully where DH is ill-conditioned (nearly parallel axes). It is the language of modern robotics texts (Lynch and Park, Murray–Li–Sastry) and the basis of screw-theoretic calibration and dynamics.

## Mathematical Theory

### Screw Axes

A joint's screw axis $S = (v;\omega)$ uses the linear-first ordering:

- revolute about the unit axis $\omega$ through a point $a$: $S = (-\omega \times a;\ \omega)$, zero pitch;
- prismatic along the unit direction $v$: $S = (v;\ 0)$.

### Forward Kinematics

With $M$ the tool pose at $q = 0$:

$$T(q) = e^{[S_1]q_1}\, e^{[S_2]q_2} \cdots e^{[S_n]q_n}\, M$$

Each factor is the rigid motion produced by moving joint $i$ alone by $q_i$, expressed in the base frame. The order matters: the outer joints' motions are applied first in space, which is what the product encodes.

### Space Jacobian

The current screw of joint $i$ is its home screw carried by the motion of all joints before it:

$$J_{s,i}(q) = \mathrm{Ad}_{e^{[S_1]q_1} \cdots e^{[S_{i-1}]q_{i-1}}}\, S_i, \qquad \mathrm{Ad}_{(R,p)} \begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} R v + p \times R\omega \\ R\omega \end{bmatrix}$$

$J_s \dot{q}$ is the spatial twist $(v_s;\omega)$. Its linear part is the velocity of the body point that currently coincides with the **base origin**, not the velocity of the tool.

### Geometric Jacobian

The geometric Jacobian uses the tool point's velocity. Shifting the reference point from the base origin to the tool point $p$ gives, column by column:

$$J_{\text{geo}} = \begin{bmatrix} \mathbf{I} & -[p]_\times \\ 0 & \mathbf{I} \end{bmatrix} J_s, \qquad (v;\omega) \mapsto (v + \omega \times p;\ \omega)$$

### Frame Chain

From each current screw, a revolute joint's axis is $\omega/\|\omega\|$, and the point on it closest to the base origin is $(\omega \times v)/\|\omega\|^2$. A prismatic joint's direction is $v$. These form the same frame chain as link-by-link or DH descriptions, so the shared geometric Jacobian routine applies unchanged.

## Complexity Analysis

| Operation          | Time   | Space  | Notes                                                      |
|--------------------|--------|--------|------------------------------------------------------------|
| Forward kinematics | $O(n)$ | $O(1)$ | One SE(3) exponential and one product per joint            |
| Space Jacobian     | $O(n)$ | $O(n)$ | One adjoint–twist product per joint on the running product |
| Frame chain        | $O(n)$ | $O(n)$ | Same pass, plus one cross product per revolute joint       |

## Step-by-Step Walkthrough

A planar two-link arm with unit links has joints about $\hat{z}$ through $(0,0,0)$ and $(1,0,0)$. Its screws are $S_1 = (0;\hat{z})$ and $S_2 = (-\hat{z} \times (1,0,0);\hat{z}) = ((0,-1,0);\hat{z})$, with home $M = (\mathbf{I}, (2,0,0))$.

1. $q = (0, \pi/2)$: the first factor is the identity, and the second rotates the tool by $90°$ about the axis through $(1,0,0)$, carrying $(2,0,0)$ to $(1,1,0)$.
2. $q = (0.4, -0.7)$: the space Jacobian's first column is $S_1 = (0;\hat{z})$. The second is $S_2$ rotated by $0.4$ rad about $\hat{z}$, which is $((0.389, -0.921, 0);\hat{z})$. Its linear part is the velocity of the base origin's coincident body point.
3. Shifting to the tool point $p = (1.876, 0.094, 0)$ adds $\omega \times p$ to each linear part. That gives the geometric columns $(-0.094, 1.876, 0)$ and $(0.296, 0.955, 0)$, the familiar planar-arm Jacobian.
4. For a UR5 with the Lynch–Park screws, the home tool position is $(L_1 + L_2,\ W_1 + W_2,\ H_1 - H_2) = (0.817, 0.191, -0.006)$ m.

## Pitfalls & Edge Cases

- **Space versus geometric Jacobian**: using $J_s$ where the tool-point Jacobian is expected silently gives the wrong linear velocity everywhere except at the base origin. Convert first.
- **Ordering of the twist**: this formulation is linear-first $(v;\omega)$; Lynch–Park write $(\omega; v)$. Swapping the blocks is the most common porting error.
- **Unit axes**: a non-unit $\omega$ scales the joint angle. Author unit axes so $q$ is a true angle.
- **Pitched screws**: screws with $\omega \cdot v \ne 0$ (helical joints) have no point-and-axis frame-chain representation and are rejected at construction.
- **Home pose consistency**: $M$ must be the tool pose with every joint at zero, consistent with how the screws were measured. Mixing conventions (e.g. a joint offset in $q$) shifts every result.

## Variants & Generalizations

- **Body form**: $T = M e^{[B_1]q_1} \cdots e^{[B_n]q_n}$ with $B_i = \mathrm{Ad}_{M^{-1}} S_i$, giving the body Jacobian directly.
- **PoE calibration**: identifying screw-axis and home-pose errors avoids the singularities of DH-parameter calibration.
- **Screw-based dynamics**: Lie-group formulations of the recursive Newton–Euler algorithm use the same screws.
- **Closed chains**: loop-closure constraints written as products of exponentials equal to the identity.

## Applications

- Modelling arms directly from CAD joint axes without assigning DH frames.
- Kinematic calibration with well-conditioned parameter identification.
- Velocity kinematics for task-space control, through conversion to the geometric Jacobian.
- Teaching and analysis of screw theory and spatial velocities.

## Connections to Other Algorithms

- [Geometric Jacobian](GeometricJacobian.md): consumes the frame chain produced here, and equals the converted space Jacobian.
- [Denavit-Hartenberg Parameters](DenavitHartenberg.md): an alternative description producing the same poses and frame chain.
- [Chain Pose Kinematics](ChainPoseKinematics.md): the link-by-link alternative.
- The SE(3) exponential and adjoint come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Brockett, R.W. (1984). "Robotic manipulators and the product of exponentials formula." *Mathematical Theory of Networks and Systems*, Springer, 120–129.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapters 4 and 5.
- Murray, R.M., Li, Z. and Sastry, S.S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press. Chapter 3.
