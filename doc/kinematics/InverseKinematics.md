# Inverse Kinematics (Damped Least Squares)

## Overview & Motivation

Inverse Kinematics (IK) solves the problem dual to Forward Kinematics: given a desired 3D end-effector position $p_{\text{target}}$, find joint angles $q$ such that the end-effector reaches that position. This is the core computational problem in robot arm control, motion planning, and teleoperation.

The **Damped Least Squares** (DLS) method, also known as the Levenberg-Marquardt approach, iteratively refines a joint-angle estimate by solving a small linear system at each step. Unlike the plain pseudoinverse, DLS adds a damping term $\lambda^2 I$ that keeps the system numerically stable even when the manipulator is near a singular configuration (such as a fully extended arm).

## Mathematical Theory

### Jacobian of a Serial Chain

For a serial revolute-joint chain with $n$ joints, the **geometric Jacobian** $J \in \mathbb{R}^{3 \times n}$ maps joint velocities $\dot{q}$ to end-effector linear velocity:

$$\dot{p}_{\text{ee}} = J(q) \, \dot{q}$$

Column $i$ of $J$ is the linear velocity contribution of joint $i$:

$$J_i = a_i^w \times (p_{\text{ee}} - p_i)$$

where:
- $a_i^w = R_{0 \to i} \, a_i^{\text{link}}$ is the joint axis in world frame (accumulated rotation applied to the link-frame axis)
- $p_i$ is the world-frame position of joint $i$ (from Forward Kinematics)
- $p_{\text{ee}}$ is the current tool (end-effector) point, i.e. the last joint position plus the rotated tool offset

### DLS Update Rule

Define the position error:

$$e = p_{\text{target}} - p_{\text{ee}}(q) \in \mathbb{R}^3$$

The DLS joint update per iteration is:

$$\Delta q = J^\top \bigl(J J^\top + \lambda^2 I\bigr)^{-1} e$$

This is equivalent to solving the $3 \times 3$ linear system:

$$\underbrace{\bigl(J J^\top + \lambda^2 I\bigr)}_{A} \, y = e, \quad \Delta q = J^\top y$$

where $A \in \mathbb{R}^{3 \times 3}$ is always symmetric positive definite (the damping $\lambda^2 I$ guarantees this), so a small Gaussian elimination (or Cholesky factorization) solves it reliably.

The updated joint angles are:

$$q_{k+1} = q_k + \Delta q$$

### Convergence

The loop terminates when:
- $\|e\| < \epsilon_{\text{tol}}$ (success), or
- The maximum iteration count is reached (failure to converge — target unreachable or tolerance too tight)

### Why DLS over Plain Pseudoinverse

At a kinematic singularity (e.g., fully extended arm), $J J^\top$ becomes rank-deficient. The pseudoinverse $J^+ = J^\top (J J^\top)^{-1}$ would produce infinite joint velocities. The damping term $\lambda^2 I$ bounds the solution:

$$\|\Delta q\| \leq \frac{\|e\|}{2\lambda}$$

because each singular value $\sigma$ of $J$ is mapped to $\sigma / (\sigma^2 + \lambda^2) \le 1/(2\lambda)$.

Damping does **not** bias the converged solution of the iteration. A fixed point requires $J^\top (J J^\top + \lambda^2 I)^{-1} e = 0$, which for a full-rank $J$ implies $e = 0$: a reachable, non-singular target is reached exactly, and larger $\lambda$ only shortens the steps and slows convergence. The damping trade-off is therefore speed and step size versus robustness near singularities; only at a singular configuration, or for an unreachable target, does the iteration stop at a least-squares stationary point with $e \neq 0$. Choosing $\lambda \in [0.01, 0.2]$ (scaled to the robot's link lengths) is a common starting point.

## Complexity Analysis

| Case          | Time           | Space  | Notes                                  |
|---------------|----------------|--------|----------------------------------------|
| Per iteration | $O(n)$         | $O(n)$ | FK sweep + Jacobian + $3\times3$ solve |
| Total         | $O(k \cdot n)$ | $O(n)$ | $k$ = iterations (typically 10–50)     |

The $3 \times 3$ linear solve is $O(1)$ (constant-size), independent of the number of joints $n$.

## Step-by-Step Walkthrough

Consider a 2-link planar arm with $l_1 = 1.0$, $l_2 = 0.8$, both joints rotating about the Z-axis, target $p_{\text{target}} = (0.5, 1.0, 0)$, initial guess $q = (0, 0)$, $\lambda = 0.1$.

**Iteration 1:**

1. FK gives $p_{\text{ee}} = (1.8, 0, 0)$ (fully extended along X)
2. Error: $e = (-1.3, 1.0, 0)$, $\|e\| \approx 1.64$
3. Jacobian (Z-axis joints, cross-product form):
   - $J_1 = [0,0,1]^\top \times (p_{\text{ee}} - p_0) = [0,0,1]^\top \times [1.8, 0, 0]^\top = [0, 1.8, 0]^\top$
   - $J_2 = [0,0,1]^\top \times (p_{\text{ee}} - p_1) = [0,0,1]^\top \times [0.8, 0, 0]^\top = [0, 0.8, 0]^\top$
   - $J = \begin{bmatrix} 0 & 0 \\ 1.8 & 0.8 \\ 0 & 0 \end{bmatrix}$ (only Y-row is non-zero for planar X motion)
4. $A = J J^\top + 0.01 I$ — a $3 \times 3$ SPD matrix
5. Solve $Ay = e$, compute $\Delta q = J^\top y$
6. Update both joint angles toward a bent configuration

After 7 iterations the error drops below $10^{-4}$ and $p_{\text{ee}} \approx (0.5, 1.0, 0)$; with $\lambda = 0.5$ the same target still converges exactly, in 13 iterations.

## Pitfalls & Edge Cases

- **Unreachable targets**: If the target lies outside the workspace, the algorithm reaches the iteration limit and reports non-convergence; the iterate approaches a least-squares stationary point (typically the closest reachable point), but this is not guaranteed for non-convex workspaces.
- **Singularity handling**: Near singularities (e.g., fully extended arm), the damping $\lambda$ bounds the step size and prevents numerical blow-up at the cost of slower convergence. Adaptive or selectively damped schemes raise $\lambda$ only near singularities.
- **Local minima in redundant chains**: For $n > 3$ (redundant), DLS finds *a* solution (minimum-norm joint motion) but not necessarily the one with the best posture. Null-space techniques (projecting secondary objectives into $\ker(J)$) can improve this.
- **Initial guess sensitivity**: Starting closer to the expected solution reduces iterations and avoids winding through joint-limit regions. For repeated calls (e.g., tracking), use the previous solution as the initial guess.
- **Float-only**: The trigonometric and square-root evaluations make fixed-point types unsuitable.
- **Configuration preconditions**: The damping factor, the tolerance and the iteration budget must all be strictly positive. The damping term $\lambda^2 I$ makes $A$ symmetric positive definite only when $\lambda > 0$; these preconditions are checked when the solver is constructed, in every build type.
- **Tool and base geometry**: The target is compared against the tool point, so the tool offset and the base mounting offset must match the physical robot; a wrong tool offset makes the solver converge exactly to the wrong joint angles.

## Variants & Generalizations

- **Jacobian Transpose**: $\Delta q = \alpha J^\top e$ — no matrix solve, but slower convergence and step-size sensitive. Suitable for very constrained embedded targets.
- **Pseudoinverse (Moore-Penrose)**: DLS with $\lambda = 0$ — exact minimum-norm solution but numerically unstable at singularities.
- **FABRIK**: Forward-And-Backward Reaching IK — purely geometric, position-only, no Jacobian required. Very fast per iteration; preferred when orientation control is not needed.
- **Cyclic Coordinate Descent (CCD)**: Each joint independently minimizes error; simple but produces unnatural motion.
- **Full 6-DOF pose IK**: Extend $J$ to $6 \times n$ (stacking angular velocity rows) and $e$ to $\mathbb{R}^6$ to control both position and orientation.

## Applications

- **Robot arm target tracking**: Drive the end-effector to follow a moving 3D target in real-time.
- **Teleoperation**: Map operator hand position to robot joint angles continuously.
- **Motion planning**: Compute joint-space waypoints from Cartesian-space paths.
- **Animation & simulation**: Pose limbs in a physically plausible configuration to reach a point of interest.

## Connections to Other Algorithms

- [Forward Kinematics](ForwardKinematics.md): Called internally every iteration to compute current end-effector position and joint positions for the Jacobian.
- Gaussian elimination ([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)): Solves the $3 \times 3$ DLS system $Ay = e$ at the core of each iteration.
- 3D geometry primitives (cross product, axis–angle rotation, vector norm) from numerical-toolbox-cpp build the Jacobian columns, the accumulated rotations and the convergence test.

## References & Further Reading

- Buss, S.R. (2004). *Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods*. University of California, San Diego. [PDF](https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf)
- Nakamura, Y. and Hanafusa, H. (1986). "Inverse kinematic solutions with singularity robustness for robot manipulator control". *Journal of Dynamic Systems, Measurement, and Control*, 108(3).
- Siciliano, B. et al. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 3.
- Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control*. 3rd ed. Chapter 4.
