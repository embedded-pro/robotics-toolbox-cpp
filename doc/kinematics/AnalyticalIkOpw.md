# Analytical Inverse Kinematics (OPW)

## Overview & Motivation

Most industrial six-axis arms (KUKA, ABB, Fanuc, Stäubli, Yaskawa) share one layout: axes 2 and 3 are parallel to each other and orthogonal to axis 1, and the last three axes intersect in a **spherical wrist**. For this ortho-parallel layout the inverse kinematics has a closed form: every solution is found in constant time, with no iteration, no seed and no convergence failure.

The **OPW** formulation (Brandstötter, Angerer and Hofbaur, 2014) describes the arm with seven lengths plus a sign and an offset per joint, read straight from the datasheet. No Denavit–Hartenberg table or frame convention is involved.
It returns up to **eight** configurations: two shoulder poses, two elbow poses and two wrist poses. The caller picks one, typically the closest to the current joint values.

## Mathematical Theory

### Geometric Model

In the model zero pose, the upper arm and forearm point along $+z$. The parameters are:

| Parameter | Meaning                                     |
|-----------|---------------------------------------------|
| $a_1$     | Radial offset from axis 1 to axis 2         |
| $a_2$     | Forearm offset perpendicular to the forearm |
| $b$       | Lateral offset of the arm plane             |
| $c_1$     | Height of axis 2 above the base             |
| $c_2$     | Upper-arm length, axis 2 to axis 3          |
| $c_3$     | Forearm length, axis 3 to the wrist centre  |
| $c_4$     | Wrist centre to flange, along axis 6        |

Joint values map to model angles by $\theta_i = s_i q_i - o_i$, with sign $s_i = \pm 1$ and offset $o_i$. The forward kinematics is

$$T(q) = R_z(\theta_1)\,\mathrm{Tr}(a_1, b, c_1)\,R_y(\theta_2)\,\mathrm{Tr}(0, 0, c_2)\,R_y(\theta_3)\,\mathrm{Tr}(a_2, 0, c_3)\,R_z(\theta_4)\,R_y(\theta_5)\,R_z(\theta_6)\,\mathrm{Tr}(0, 0, c_4)$$

### Decoupling at the Wrist Centre

The spherical wrist decouples position from orientation. The wrist centre $C = p - c_4 R\hat{z}$ depends only on $\theta_1, \theta_2, \theta_3$. With the virtual forearm $k = \sqrt{a_2^2 + c_3^2}$ and its angle $\psi_3 = \operatorname{atan2}(a_2, c_3)$:

1. **Shoulder.** With $n = \sqrt{C_x^2 + C_y^2 - b^2} - a_1$, the front and back solutions are $\theta_1 = \operatorname{atan2}(C_y, C_x) \mp \operatorname{atan2}(b, n + a_1)$, the back one shifted by $\pi$.
2. **Elbow.** In the arm plane, the distances $s_1^2 = n^2 + z^2$ (front) and $s_2^2 = (n + 2a_1)^2 + z^2$ (back), with $z = C_z - c_1$, close triangles with sides $c_2$ and $k$. The law of cosines gives the angles $\beta$ at the shoulder and $\gamma$ at the elbow, and elbow up and down follow from $\pm\beta$, $\pm\gamma$.
3. **Wrist.** With the arm fixed, $W = (R_z(\theta_1)R_y(\theta_2 + \theta_3))^T R = R_z(\theta_4)R_y(\theta_5)R_z(\theta_6)$, a ZYZ Euler decomposition:

$$\theta_4 = \operatorname{atan2}(W_{12}, W_{02}), \qquad \theta_5 = \operatorname{atan2}\left(\sqrt{W_{02}^2 + W_{12}^2},\ W_{22}\right), \qquad \theta_6 = \operatorname{atan2}(W_{21}, -W_{20})$$

The flipped wrist $(\theta_4 + \pi, -\theta_5, \theta_6 - \pi)$ gives the same orientation.

### Solution Slots

Slots 0–3 hold the four arm branches with $\theta_5 \ge 0$: shoulder front with elbow a/b, then shoulder back with elbow a/b. Slots 4–7 hold the same arms with the wrist flipped. A branch whose cosine argument exceeds one is unreachable and left empty.

## Complexity Analysis

| Step                   | Time   | Space  | Notes                                              |
|------------------------|--------|--------|----------------------------------------------------|
| Wrist centre, shoulder | $O(1)$ | $O(1)$ | Two `atan2`, one square root                       |
| Elbow branches         | $O(1)$ | $O(1)$ | Four clamped `acos`                                |
| Wrist per arm branch   | $O(1)$ | $O(1)$ | One 3×3 product and three `atan2`                  |
| All eight solutions    | $O(1)$ | $O(1)$ | Fixed work; no iteration, Jacobian or linear solve |

## Step-by-Step Walkthrough

A KUKA KR6 R700 sixx has $a_1 = 0.025$, $a_2 = -0.035$, $b = 0$, $c_1 = 0.400$, $c_2 = 0.315$, $c_3 = 0.365$ and $c_4 = 0.080$ m. Its offsets are $(0, -\pi/2, 0, 0, 0, 0)$ and its signs $(-1, 1, 1, -1, 1, -1)$.

1. **Home pose**: $q = 0$ puts the flange at $(0.785, 0, 0.435)$ m with the tool axis along $+x$.
2. **Target**: $q^\ast = (0.3, -1.0, 1.2, 0.5, 0.6, 0.7)$ puts the flange at $(0.583, -0.203, 0.575)$ m.
3. **Eight solutions**: slot 0 recovers $q^\ast$, and slot 1 is the other elbow, $(0.3, 0.198, -1.009, 0.274, 1.553, 1.118)$. Slots 2 and 3 are the back-shoulder pair with $q_1 = -2.842$. Slots 4–7 flip the wrist. All eight reproduce the target pose.
4. **Wrist singularity**: with $q_5 = 0$, axes 4 and 6 align, so only $\theta_4 + \theta_6$ is determined. The solver returns $q_4 = 0$ and $q_6 = 1.2$, flags the solution as wrist-singular, and leaves the flipped slot empty.
5. **Unreachable target**: a flange at $(1.5, 0, 0.4)$ m is beyond reach, so every slot is empty.

## Pitfalls & Edge Cases

- **Layout assumption**: the formulas hold only for ortho-parallel arms with a spherical wrist. Collaborative arms with offset wrists (UR-type) need different closed forms.
- **Precision near singularities**: at full stretch, $\operatorname{acos}$ near $\pm 1$ amplifies rounding to about $\sqrt{2\varepsilon}$, roughly $5 \times 10^{-4}$ rad in single precision. Close to the wrist singularity, $\theta_4$ and $\theta_6$ are individually ill-conditioned while their sum stays accurate. The pose is reproduced in both cases.
- **Wrist singularity**: computing $\sin\theta_5$ as $\sqrt{1 - W_{22}^2}$ loses precision near zero. The norm of $(W_{02}, W_{12})$ keeps it accurate, so the singular branch triggers only at rounding level.
- **Joint limits**: they are not applied. Filter the solutions and choose, for example, the one nearest the current configuration.
- **Flange versus tool**: the target is the flange pose. For a tool, solve for $T_{\text{target}}\,T_{\text{tool}}^{-1}$.
- **Shoulder singularity**: when the wrist centre lies on axis 1 (with $b = 0$), any $\theta_1$ works and $\theta_1 = 0$ is returned.

## Variants & Generalizations

- **Offset-wrist arms**: UR-type closed forms with up to eight solutions from different geometry.
- **Seven-axis arms**: an arm-angle parameterization reduces redundant arms (e.g. KUKA iiwa) to closed form per elbow angle.
- **Numerical fallback**: damped least squares for arms without a closed form, seeded with an analytical solution when one exists.
- **Solution selection**: weighting joint distance, limits and singularity margin when choosing among the branches.

## Applications

- Real-time Cartesian jogging and teleoperation on industrial controllers.
- Offline programming and path planning that must enumerate all configurations.
- Seeding numerical IK and checking reachability.

## Connections to Other Algorithms

- [Pose Inverse Kinematics](PoseInverseKinematics.md): the iterative alternative for arbitrary geometries.
- [Denavit-Hartenberg Parameters](DenavitHartenberg.md): the frame-based description that OPW avoids.
- [Manipulability Index](ManipulabilityIndex.md): measures how close each returned configuration is to a singularity.
- The angle wrapping and SE(3) types come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Brandstötter, M., Angerer, A. and Hofbaur, M. (2014). "An analytical solution of the inverse kinematics problem of industrial serial manipulators with an ortho-parallel basis and a spherical wrist." *Proceedings of the Austrian Robotics Workshop*, 7–11.
- Pieper, D.L. (1968). *The Kinematics of Manipulators under Computer Control*. PhD thesis, Stanford University.
- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 2.
