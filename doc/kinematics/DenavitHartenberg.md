# Denavit-Hartenberg Parameters

## Overview & Motivation

A serial arm can be described link by link with arbitrary offsets and axes. The **Denavit–Hartenberg (DH)** convention instead attaches frames to the links by a fixed rule, so every joint–link pair is captured by just **four numbers** $(a, \alpha, d, \theta)$.
DH tables are the lingua franca of industrial robotics: manufacturers publish them, textbooks tabulate classic arms such as the PUMA-560, and calibration methods identify their errors. One of the four numbers is the joint variable: $\theta$ for a revolute joint, $d$ for a prismatic one.

From a DH table the model produces the tool pose and the frame chain (joint origins, axes and types in the base frame), so the same geometric Jacobian and every task-space algorithm built on it work on DH-described arms unchanged.

## Mathematical Theory

### Parameters

For link $i$:

- $a$ — link length, the distance between consecutive $z$ axes along their common normal $x$;
- $\alpha$ — link twist, the angle between consecutive $z$ axes about $x$;
- $d$ — link offset along $z$ (variable for a prismatic joint);
- $\theta$ — joint angle about $z$ (variable for a revolute joint).

Real tables carry constant offsets on the variable parameter as well. The joint variable is **added**, $\theta = \theta_0 + q$ or $d = d_0 + q$, never substituted.

### Standard (Distal) Convention

Frame $i$ sits at the distal end of link $i$, and joint $i$ acts about $z_{i-1}$:

$$A_i = \mathrm{Rot}_z(\theta)\,\mathrm{Trans}_z(d)\,\mathrm{Trans}_x(a)\,\mathrm{Rot}_x(\alpha) = \begin{bmatrix} c_\theta & -s_\theta c_\alpha & s_\theta s_\alpha & a c_\theta \\ s_\theta & c_\theta c_\alpha & -c_\theta s_\alpha & a s_\theta \\ 0 & s_\alpha & c_\alpha & d \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Modified (Proximal, Craig) Convention

Frame $i$ sits at the proximal end, and joint $i$ acts about $z_i$. Row $i$ holds the twist and length of the **previous** link:

$$A_i = \mathrm{Rot}_x(\alpha_{i-1})\,\mathrm{Trans}_x(a_{i-1})\,\mathrm{Rot}_z(\theta)\,\mathrm{Trans}_z(d) = \begin{bmatrix} c_\theta & -s_\theta & 0 & a_{i-1} \\ s_\theta c_\alpha & c_\theta c_\alpha & -s_\alpha & -s_\alpha d \\ s_\theta s_\alpha & c_\theta s_\alpha & c_\alpha & c_\alpha d \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

The conventions differ in row layout, not just factor order. The last standard $(a_n, \alpha_n)$ moves into the constant tool transform of the modified table. Described consistently, both yield the same tool pose and the same frame chain.

### Tool Pose and Frame Chain

$$T_{\text{tool}} = A_1(q_1)\,A_2(q_2) \cdots A_n(q_n)\,T_t$$

with $T_t$ the constant tool transform in the last DH frame. The frame chain records, for each joint, the origin and $z$ axis of the frame it acts about. In the standard convention that is the running product **before** $A_i$; in the modified convention, **after** it.
For revolute joints, any point on the axis is a valid origin. The modified origin, which includes $d_i$, therefore gives the same Jacobian column.

## Complexity Analysis

| Operation      | Time   | Space  | Notes                                                             |
|----------------|--------|--------|-------------------------------------------------------------------|
| Link transform | $O(1)$ | $O(1)$ | One sine–cosine pair; the twist's sine and cosine are precomputed |
| Tool pose      | $O(n)$ | $O(1)$ | $n$ rigid-transform products                                      |
| Frame chain    | $O(n)$ | $O(n)$ | Same products, plus $2n$ stored vectors                           |

## Step-by-Step Walkthrough

A planar two-link arm with unit links, standard table $(a, \alpha, d, \theta) = (1, 0, 0, q_1)$, $(1, 0, 0, q_2)$, at $q = (0, \pi/2)$:

1. $A_1(0)$: no rotation, translation $(1, 0, 0)$.
2. $A_2(\pi/2)$: rotation $\mathrm{Rot}_z(\pi/2)$, translation $(a c_\theta, a s_\theta, 0) = (0, 1, 0)$.
3. $T = A_1 A_2$: position $(1, 0, 0) + (0, 1, 0) = (1, 1, 0)$, the elbow bent upwards.
4. The modified table $(0, 0, 0, q_1)$, $(1, 0, 0, q_2)$ with tool translation $(1, 0, 0)$ gives $\mathrm{Rot}_z(q_1) \cdot (\mathrm{Trans}_x(1)\,\mathrm{Rot}_z(q_2)) \cdot \mathrm{Trans}_x(1)$, which is the same pose.
5. With a constant offset $\theta_{0,1} = \pi/2$ the stretched configuration points along $y$: $q = (0, 0)$ gives $(0, 2, 0)$.

## Pitfalls & Edge Cases

- **Mixing conventions**: a table written for one convention and evaluated with the other gives plausible but wrong poses. Check which one a datasheet uses, and remember the modified table's shifted $(a, \alpha)$.
- **Dropping offsets**: published tables often have constant $\theta$ offsets (e.g. $-\pi/2$ on a shoulder) so that the zero configuration matches the robot's home pose. Substituting $q$ instead of adding it moves the whole arm.
- **Parallel consecutive axes**: the common normal is not unique, and small manufacturing errors cause large parameter jumps. DH is ill-conditioned for calibration there; modified DH or extra parameters (Hayati) fix it.
- **Twist sign**: $\alpha = +\pi/2$ maps $z_{i}$ onto $-y_{i-1}$ in the standard form. A sign slip flips the following axes.
- **Prismatic joints**: the variable is $d$; $\theta$ stays constant and must still include any offset.

## Variants & Generalizations

- **Hayati–Mirmirani parameters**: an extra rotation about $y$ handles nearly parallel axes for calibration.
- **Product of exponentials**: screw axes in the base frame avoid per-link frame rules entirely and degrade gracefully near parallel axes.
- **Sheth–Uicker two-frame convention**: separates joint and link frames for general mechanisms.
- **Branched and closed chains**: DH applies per serial branch; closed loops add constraint equations.

## Applications

- Modelling industrial arms directly from manufacturer datasheets.
- Kinematic calibration, which identifies DH parameter errors from measured tool poses.
- Classical closed-form inverse kinematics for arms with spherical wrists, derived in DH frames.
- Teaching and verification against textbook tables (PUMA-560, Stanford arm, SCARA).

## Connections to Other Algorithms

- [Geometric Jacobian](GeometricJacobian.md): consumes the frame chain produced here; DH-described arms share its columns, statics and bias acceleration.
- [Chain Pose Kinematics](ChainPoseKinematics.md): the link-by-link alternative that produces the same frame chain.
- [Forward Kinematics](ForwardKinematics.md): the position-only special case for link-by-link models.
- Rigid transforms and their products come from the SE(3) primitives of [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Denavit, J. and Hartenberg, R.S. (1955). "A kinematic notation for lower-pair mechanisms based on matrices." *ASME Journal of Applied Mechanics*, 22, 215–221.
- Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control*. 3rd ed. Pearson. Chapter 3.
- Spong, M.W., Hutchinson, S. and Vidyasagar, M. (2020). *Robot Modeling and Control*. 2nd ed. Wiley. Chapter 3.
- Corke, P.I. (2007). "A simple and systematic approach to assigning Denavit–Hartenberg parameters." *IEEE Transactions on Robotics*, 23(3), 590–594.
