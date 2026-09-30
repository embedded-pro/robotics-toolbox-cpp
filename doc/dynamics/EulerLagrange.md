# Euler-Lagrange Dynamics

## Overview & Motivation

The Euler-Lagrange equations provide a systematic way to derive the equations of motion for mechanical systems from energy principles. Instead of tracking forces on each body (as in Newton's method), they express dynamics in terms of *generalized coordinates* $q$ and the system's kinetic and potential energy.

For a robotic manipulator or any articulated mechanism with $n$ degrees of freedom, the Euler-Lagrange formulation yields a compact matrix equation:

$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau$$

Where:
- $q \in \mathbb{R}^n$ — generalized coordinates (joint angles/positions)
- $\dot{q}, \ddot{q}$ — generalized velocities and accelerations
- $M(q) \in \mathbb{R}^{n \times n}$ — mass (inertia) matrix, symmetric positive definite
- $C(q, \dot{q})\dot{q} \in \mathbb{R}^n$ — Coriolis and centrifugal terms
- $g(q) \in \mathbb{R}^n$ — gravitational terms
- $\tau \in \mathbb{R}^n$ — generalized forces/torques (inputs)

This formulation supports two fundamental computations:
- **Forward dynamics**: given torques $\tau$, compute accelerations $\ddot{q} = M^{-1}(\tau - C\dot{q} - g)$
- **Inverse dynamics**: given desired accelerations $\ddot{q}$, compute required torques $\tau = M\ddot{q} + C\dot{q} + g$

## Mathematical Theory

### Prerequisites

The **Lagrangian** of a system is defined as:

$$\mathcal{L}(q, \dot{q}) = T(q, \dot{q}) - V(q)$$

where $T$ is kinetic energy and $V$ is potential energy.

### Core Definitions

The **Euler-Lagrange equation** for coordinate $q_i$ is:

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i \quad \text{for } i = 1, \ldots, n$$

For a rigid-body system, kinetic energy takes the form:

$$T = \frac{1}{2}\dot{q}^T M(q) \dot{q}$$

Applying the Euler-Lagrange equation to this quadratic form yields the standard manipulator equation:

$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau$$

### Mass Matrix $M(q)$

The mass matrix is always **symmetric** ($M = M^T$) and **positive definite** ($\dot{q}^T M \dot{q} > 0$ for $\dot{q} \neq 0$). These properties guarantee:
- The forward dynamics linear system $M\ddot{q} = f$ always has a unique solution
- Physical kinetic energy is always non-negative

### Coriolis Matrix $C(q, \dot{q})$

The Coriolis matrix captures velocity-dependent forces using **Christoffel symbols of the first kind**:

$$c_{ij} = \sum_{k=1}^{n} \frac{1}{2}\left(\frac{\partial m_{ij}}{\partial q_k} + \frac{\partial m_{ik}}{\partial q_j} - \frac{\partial m_{jk}}{\partial q_i}\right)\dot{q}_k$$

A key property: $\dot{M} - 2C$ is skew-symmetric, which is important for passivity-based control.

### Gravity Vector $g(q)$

$$g_i(q) = \frac{\partial V}{\partial q_i}$$

where $V(q)$ is the total potential energy of the system.

## Complexity Analysis

| Operation        | Time            | Space    | Notes                                                |
|------------------|-----------------|----------|------------------------------------------------------|
| Forward dynamics | $O(n^3)$        | $O(n^2)$ | Dominated by $M^{-1}$ solve via Gaussian elimination |
| Inverse dynamics | $O(n^2)$        | $O(n^2)$ | Matrix-vector multiplication $M \ddot{q}$            |
| Model evaluation | Model-dependent | $O(n^2)$ | Computing $M(q)$, $C(q,\dot{q})$, $g(q)$             |

For systems with $n \leq 6$ DOF (typical robotic manipulators), the $O(n^3)$ cost of Gaussian elimination is negligible. For larger systems, consider the Recursive Newton-Euler Algorithm (RNEA) which achieves $O(n)$ inverse dynamics.

## Step-by-Step Walkthrough

### Example: Simple Pendulum (1-DOF)

A point mass $m$ on a rigid rod of length $l$:

**System parameters:** $m = 1\,\text{kg}$, $l = 1\,\text{m}$, $g_0 = 9.81\,\text{m/s}^2$

**Model matrices:**
- $M(q) = ml^2 = 1.0$
- $C(q, \dot{q})\dot{q} = 0$ (single DOF → no Coriolis)
- $g(q) = mgl\sin(q) = 9.81\sin(q)$

**Forward dynamics** at $q = 0.5\,\text{rad}$, $\dot{q} = 0$, $\tau = 0$:

$$\ddot{q} = M^{-1}(\tau - C\dot{q} - g) = \frac{0 - 0 - 9.81\sin(0.5)}{1.0} = -4.703\,\text{rad/s}^2$$

**Inverse dynamics** for holding position ($\ddot{q} = 0$):

$$\tau = M\ddot{q} + C\dot{q} + g = 0 + 0 + 9.81\sin(0.5) = 4.703\,\text{N·m}$$

### Example: Two-Link Planar Arm (2-DOF)

For a 2-DOF arm with equal links ($m_1 = m_2 = 1\,\text{kg}$, $l_1 = l_2 = 1\,\text{m}$):

**At rest** ($q = [0, 0]^T$, $\dot{q} = [0, 0]^T$):
- $\sin(q_1) = 0$, $\sin(q_1 + q_2) = 0$
- $g = [0, 0]^T$ → gravity terms vanish when links hang vertically
- Forward dynamics with $\tau = 0$ produces $\ddot{q} = [0, 0]^T$ (equilibrium)

## Pitfalls & Edge Cases

- **Mass-matrix conditioning**: The joint-space mass matrix of a physically valid model is always symmetric positive definite, including at kinematic singularities (those make the *task-space* inertia singular, not $M$).
  Its conditioning is governed by the inertia ratio between proximal and distal links — a heavy shoulder with a light wrist gives a large condition number.
  A Cholesky factorization exploits the symmetry; Gaussian elimination with partial pivoting also works.
- **Fixed-point arithmetic**: Dynamics values (torques, accelerations) typically exceed the $[-1, 1)$ range of Q15/Q31 formats. Use `float` unless inputs are carefully scaled.
- **Coriolis computation**: Many matrices $C(q,\dot{q})$ produce the same vector $C(q,\dot{q})\dot{q}$, but only the Christoffel-symbol choice makes $\dot{M} - 2C$ skew-symmetric.
  Working with the vector $C(q,\dot{q})\dot{q}$ (as forward and inverse dynamics do) is independent of that choice; controllers that need the matrix itself (passivity-based and adaptive control, momentum observers) must use the Christoffel form.
- **Energy consistency**: The exact equations conserve energy for free motion whatever factorization of $C$ is used, because $\dot{q}^T(\dot{M} - 2C)\dot{q} = 0$ holds for every valid one. Energy drift observed in simulation comes from the integrator (step size and method), not from the model.
- **Units**: Be explicit about whether angles are in radians (standard) and torques in N·m.

## Variants & Generalizations

| Variant                              | Trade-off                                | Use Case                                             |
|--------------------------------------|------------------------------------------|------------------------------------------------------|
| **Euler-Lagrange** (this)            | $O(n^3)$ forward, closed-form, intuitive | Small systems ($n \leq 6$), control design, analysis |
| **Newton-Euler**                     | Body-by-body force balance               | Single rigid bodies, intuitive for simple systems    |
| **Recursive Newton-Euler (RNEA)**    | $O(n)$ inverse dynamics                  | Large kinematic chains, real-time control            |
| **Articulated Body Algorithm (ABA)** | $O(n)$ forward dynamics                  | Large chains, simulation                             |
| **Hamiltonian formulation**          | Phase-space, symplectic integration      | Long-horizon simulation, energy preservation         |

## Applications

- **Robot control**: Computed torque control, gravity compensation, impedance control
- **Simulation**: Forward dynamics integration for physics simulation
- **System identification**: Estimate dynamic parameters from measured motion
- **Trajectory optimization**: Inverse dynamics as constraints in optimization
- **Vibration analysis**: Linearized dynamics around equilibrium points

## Connections to Other Algorithms

- **Recursive Newton-Euler** ([RecursiveNewtonEuler.md](RecursiveNewtonEuler.md)) and **Articulated Body Algorithm** ([ArticulatedBodyAlgorithm.md](ArticulatedBodyAlgorithm.md)): compute the same $\tau$ and $\ddot{q}$ in $O(n)$ from a link description;
  RNEA also supplies $g(q)$ (zero velocity and acceleration) and $C(q,\dot{q})\dot{q}$ (zero acceleration, zero gravity), which is how a link-based model can implement this formulation.
- The following live in [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp):
  - **Linear solvers** (Gaussian elimination, Cholesky): solve $M\ddot{q} = f$ in forward dynamics.
  - **LQR / DARE**: linearizing the manipulator equation around an operating point gives the $(A, B)$ matrices for optimal state feedback.
  - **Kalman filters**: state estimation for partially observed systems modelled with these equations.

## References & Further Reading

- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapters 7–8.
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot Modeling and Control* (2nd ed.). Wiley. Chapter 6.
- Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. (For RNEA and ABA extensions.)
