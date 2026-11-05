# Mobile-Manipulator Kinematics

## Overview & Motivation

A **mobile manipulator** mounts a robot arm on a wheeled base. The base extends the arm's reach from a fixed workcell to a whole building. The combination is the standard platform for logistics picking, laboratory automation and service robots.

Controlling the tool needs one Jacobian that maps **all** the robot's inputs, base and arm together, to the tool twist in the world frame. A differential-drive base adds a complication: it has three planar coordinates but only two controls, forward speed and yaw rate, because its wheels cannot slide sideways.
The combined Jacobian therefore uses the base's two controls directly, so every column is a velocity the robot can actually command.

## Mathematical Theory

### Tool Pose

With the base pose $T_{wb}$ in SE(2), lifted to SE(3), the arm mounted at $T_{ba}$ on the base and the arm's tool pose $T_{at}(q)$:

$$T_{wt} = T_{wb}\,T_{ba}\,T_{at}(q)$$

### Combined Jacobian

The input is $u = (v, \omega, \dot{q})$: base forward speed, base yaw rate and arm joint rates. The tool twist $(\dot{p};\, \omega_t)$ in the world frame is $J u$, with $J \in \mathbb{R}^{6 \times (2 + n)}$ built column by column.

**Forward speed** moves the whole robot along the heading $\varphi$:

$$J_v = \begin{pmatrix} (\cos\varphi,\; \sin\varphi,\; 0) \\ 0 \end{pmatrix}$$

**Yaw rate** rotates the robot about the vertical axis through the axle midpoint. With the lever arm $r$ from the axle midpoint to the tool point:

$$J_\omega = \begin{pmatrix} \hat{z} \times r \\ \hat{z} \end{pmatrix}$$

**Arm joints** contribute the arm's geometric Jacobian, expressed in the arm-base frame and rotated into the world with $R_{wa} = R_z(\varphi)\,R_{ba}$:

$$J_q = \begin{pmatrix} R_{wa} & 0 \\ 0 & R_{wa} \end{pmatrix} J_{\text{arm}}(q)$$

No lever-arm correction is needed for the arm block, because its linear rows already describe the tool point.

### Nonholonomic Base

The base velocity in planar coordinates follows from the two controls:

$$\begin{pmatrix} \dot{x} \\ \dot{y} \\ \dot{\varphi} \end{pmatrix} = S(\varphi) \begin{pmatrix} v \\ \omega \end{pmatrix}, \qquad S(\varphi) = \begin{pmatrix} \cos\varphi & 0 \\ \sin\varphi & 0 \\ 0 & 1 \end{pmatrix}$$

Every base velocity of this form satisfies the no-slip constraint $-\sin\varphi\,\dot{x} + \cos\varphi\,\dot{y} = 0$. The wheel speeds for $(v, \omega)$ come from the differential-drive inverse kinematics.

## Complexity Analysis

| Operation         | Time   | Space  | Notes                                                      |
|-------------------|--------|--------|------------------------------------------------------------|
| Combined Jacobian | $O(n)$ | $O(n)$ | One arm Jacobian and tool pose, two $3 \times n$ rotations |
| Tool pose         | $O(n)$ | $O(1)$ | One arm forward kinematics and two compositions            |
| Base constraint   | $O(1)$ | $O(1)$ | One sine and one cosine                                    |

## Step-by-Step Walkthrough

A three-joint arm (a yaw joint, then two pitch joints with links of $0.2$ m and $0.3$ m and a $0.25$ m tool) is mounted at $(0.1, 0, 0.3)$ on the base, rotated $90°$. The base stands at $(1, 2)$ heading $\pi/6$, and the arm is at $q = (0.4, -0.6, 0.9)$:

1. **Tool pose**: the tool point is at $(0.699, 2.343, 0.696)$ in the world.
2. **Combined twist**: the input $(0.3, -0.5, 0.2, -0.1, 0.4)$ moves the tool at $(0.404, 0.200, -0.047)$ m/s and rotates it at $(-0.181, -0.239, -0.3)$ rad/s, matching a finite difference of the tool pose.
3. **Frame rotation**: stacking the arm Jacobian without rotating it into the world would give a twist error of $0.52$.
4. **Forward drive**: with the base heading along $x$, driving at $1$ m/s moves the tool at $(1, 0, 0)$ m/s without rotating it.
5. **Yaw**: turning the base at $1$ rad/s moves the tool sideways at $0.457$ m/s, the horizontal length of the lever arm.
6. **Base constraint**: for any heading, the base velocity has no sideways component.

## Pitfalls & Edge Cases

- **Frames**: the arm Jacobian lives in the arm-base frame. Forgetting the rotation into the world gives large errors as soon as the base turns.
- **Redundancy**: base plus arm has more inputs than the six task dimensions. Resolve it with a weighted pseudo-inverse or a null-space objective, for example the arm for fine motion and the base for reach.
- **Scaling**: the yaw column grows with the lever arm, and base speeds have different units from joint rates. Weight the columns before resolving redundancy.
- **Tool above the axle**: with no horizontal lever arm, yaw only rotates the tool and adds no translation.
- **Base dynamics**: the kinematic model ignores base acceleration limits and wheel slip. Saturate the base commands separately.

## Variants & Generalizations

- **Omnidirectional bases**: a third base column for lateral velocity, from the mecanum or Swedish-wheel kinematics.
- **Car-like bases**: steering angle and its rate limit replace the yaw rate.
- **Whole-body control**: task-priority and quadratic-programming controllers over the combined Jacobian.
- **Floating bases**: legged and aerial manipulators with a full 6-DOF base.

## Applications

- Fetch-and-carry and shelf picking in warehouses and hospitals.
- Laboratory automation moving samples between instruments.
- Mobile machining and inspection of large workpieces.

## Connections to Other Algorithms

- [Geometric Jacobian](GeometricJacobian.md): the arm block of the combined Jacobian.
- [Redundancy Resolution](RedundancyResolution.md): splits a task between base and arm.
- [Differential-Drive Kinematics](../mobile/DifferentialDrive.md): turns the base commands into wheel speeds.
- The SE(2) and SE(3) transforms come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Yamamoto, Y. and Yun, X. (1992). "Coordinating locomotion and manipulation of a mobile manipulator." *IEEE Conference on Decision and Control*, 2643–2648.
- Bayle, B., Fourquet, J.-Y. and Renaud, M. (2003). "Manipulability of wheeled mobile manipulators: application to motion generation." *International Journal of Robotics Research*, 22(7–8), 565–581.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapter 13.5.
