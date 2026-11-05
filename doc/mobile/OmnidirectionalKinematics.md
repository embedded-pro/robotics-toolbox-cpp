# Omnidirectional Kinematics (Mecanum and Swedish Wheels)

## Overview & Motivation

A **holonomic** base can move in any planar direction and rotate at the same time, with no need to turn before translating. It can slide sideways into a narrow aisle or dock against a workstation. Holonomic bases are built from **Swedish wheels**: wheels with free rollers around the rim, so each one drives in one direction and slides freely in another.

The common layouts are three omniwheels at $120°$ (the "kiwi" drive) and four mecanum wheels with $45°$ rollers. Both follow from one general model in which every wheel contributes one linear constraint between its speed and the body twist. With more than three wheels the constraints overdetermine the twist, which also allows slip to be detected.

## Mathematical Theory

### Wheel Constraint

Wheel $i$ sits at $(x_i, y_i)$ in the body frame, drives along angle $\beta_i$, has roller angle $\gamma_i$ (zero for omniwheels, $\pm 45°$ for mecanum) and radius $r_i$. With $a_i = \beta_i + \gamma_i$, its angular speed for a body twist $(v_x, v_y, \omega)$ is

$$u_i = \frac{1}{r_i\cos\gamma_i}\begin{pmatrix} \cos a_i & \sin a_i & x_i\sin a_i - y_i\cos a_i \end{pmatrix}\begin{pmatrix} v_x \\ v_y \\ \omega \end{pmatrix}$$

The roller absorbs the velocity component along its own axis. A roller aligned with the drive direction ($\cos\gamma_i = 0$) transmits nothing and is rejected.

### Inverse and Forward Kinematics

Stacking the rows gives the wheel Jacobian $H \in \mathbb{R}^{N \times 3}$:

$$u = H\,\xi, \qquad \xi = H^{+} u, \qquad H^{+} = (H^T H)^{-1} H^T$$

$H^{+}$ is computed once at construction from a $3 \times 3$ Cholesky factorization. It is exact for three wheels and a least-squares fit for more. The layout must span all three degrees of freedom, or $H^T H$ is singular.

### Standard Mecanum Cart

For wheels at $(\pm l_x, \pm l_y)$ with $\beta = 0$, rollers $-45°, +45°, +45°, -45°$ for front-left, front-right, rear-left, rear-right, and $k = l_x + l_y$:

$$r\,u = (v_x - v_y - k\omega,\;\; v_x + v_y + k\omega,\;\; v_x + v_y - k\omega,\;\; v_x - v_y + k\omega)$$

### Slip Detection

With $N > 3$, measured wheel speeds should lie in the range of $H$. The residual

$$\rho = \left\|u - H H^{+} u\right\|$$

is zero for consistent wheels and grows when a wheel slips, lifts or reports a faulty encoder reading.

### Saturation

When any wheel would exceed $\omega_{\max}$, all wheels are scaled by $\omega_{\max}/\max_i |u_i|$, so the body keeps moving in the commanded direction, more slowly.

## Complexity Analysis

| Operation          | Time   | Space  | Notes                                      |
|--------------------|--------|--------|--------------------------------------------|
| Inverse kinematics | $O(N)$ | $O(N)$ | $3N$ multiply-adds                         |
| Forward kinematics | $O(N)$ | $O(1)$ | $3N$ multiply-adds with the stored $H^{+}$ |
| Slip residual      | $O(N)$ | $O(N)$ | One forward and one inverse map            |
| Construction       | $O(N)$ | $O(N)$ | One $3 \times 3$ Cholesky factorization    |

## Step-by-Step Walkthrough

A mecanum cart with $r = 5$ cm, $l_x = 0.2$ m and $l_y = 0.15$ m, and a kiwi base with omniwheels at radius $0.2$ m:

1. **Mecanum Jacobian**: the rows are $(20, -20, -7)$, $(20, 20, 7)$, $(20, 20, -7)$ and $(20, -20, 7)$, the closed form scaled by $1/r$.
2. **Round trip**: the twist $(0.3, -0.2, 0.5)$ needs wheels at $(6.5, 5.5, -1.5, 13.5)$ rad/s, which map back to the same twist with zero slip residual.
3. **One wheel spinning**: wheels at $(10, 0, 0, 0)$ give the least-squares twist $(0.125, -0.125, -0.357)$ and a slip residual of $5$, flagging the inconsistency.
4. **Kiwi**: three wheels determine the twist exactly. Equal wheel speeds of $5$ rad/s spin the base in place at $1.25$ rad/s.
5. **Saturation**: $(1, 0.5, 1)$ needs $(3, 37, 23, 17)$ rad/s. Scaling by $20/37$ keeps the direction and gives $(0.541, 0.270, 0.541)$.

## Pitfalls & Edge Cases

- **Roller handedness**: a mecanum wheel mounted with the wrong roller direction inverts lateral motion. Confirm each roller sign by commanding pure sideways motion.
- **Degenerate layouts**: all wheels driving parallel cannot rotate or slide sideways. The construction rejects them.
- **Floor contact**: mecanum wheels need all four wheels on the ground. On uneven floors, slip and wheel lift-off are common, so watch the residual.
- **Per-wheel clipping**: saturating wheels independently changes the direction of motion. Scale them together.
- **Efficiency**: rollers waste energy and wear quickly, and mecanum traction is lower than a plain wheel's.

## Variants & Generalizations

- **Powered caster vehicles**: steerable offset wheels for holonomic motion without rollers.
- **Weighted least squares**: down-weighting wheels known to slip in the forward kinematics.
- **Dynamic models**: roller friction and wheel inertia for high-acceleration platforms.

## Applications

- Industrial carts and mobile manipulators in tight workcells.
- Robot soccer and education platforms.
- Omnidirectional camera dollies and patient-transfer devices.

## Connections to Other Algorithms

- [Differential-Drive Kinematics](DifferentialDrive.md): the nonholonomic two-wheel counterpart.
- [Planar Odometry](PlanarOdometry.md): integrates the forward-kinematics twist into a pose.
- The Cholesky factorization comes from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapter 13.2.
- Siegwart, R., Nourbakhsh, I.R. and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press. Chapter 3.
- Muir, P.F. and Neuman, C.P. (1987). "Kinematic modeling of wheeled mobile robots." *Journal of Robotic Systems*, 4(2), 281–340.
