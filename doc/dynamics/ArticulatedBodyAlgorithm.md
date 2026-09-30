# Articulated Body Algorithm (ABA)

## Overview & Motivation

The Articulated Body Algorithm (ABA) is the most efficient method for computing **forward dynamics** of serial kinematic chains. Given joint positions $q$, velocities $\dot{q}$, and applied torques $\tau$, it computes joint accelerations $\ddot{q}$ in $O(n)$ time — linear in the number of links.

This is the forward-dynamics counterpart of the [Recursive Newton-Euler Algorithm](RecursiveNewtonEuler.md) (which solves inverse dynamics). Together, they form the two fundamental $O(n)$ algorithms in rigid-body dynamics.

The mass-matrix alternative forms $M(q)$ (for example with the Composite Rigid Body Algorithm, $O(n^2)$) and factorizes it ($O(n^3)$). For short chains the two approaches cost about the same; in Featherstone's operation counts ABA becomes the cheaper option somewhere around $n \approx 6$–$9$ bodies, and its advantage grows linearly beyond that.

## Mathematical Theory

### Problem Statement

Given a serial chain of $n$ rigid bodies connected by revolute joints, find:

$$\ddot{q} = M(q)^{-1} [\tau - C(q, \dot{q})\dot{q} - g(q)]$$

ABA computes this without ever forming or inverting $M$.

### Algorithm Structure

ABA has three passes over the kinematic chain:

1. **Forward pass** (base → tip): Compute link kinematics — angular velocities, linear velocities, and velocity-product accelerations from joint positions and velocities.

2. **Backward pass** (tip → base): Compute articulated-body inertias $I^A_i$ and bias wrenches $p^A_i$ for each link. At each step, the joint projection removes one degree of freedom from the spatial inertia, and the remainder is propagated to the parent.

3. **Forward pass** (base → tip): Resolve joint accelerations using the articulated-body quantities. Each joint acceleration $\ddot{q}_i$ is computed from the projected articulated inertia and the transformed parent acceleration.

### Key Formulas

**Articulated-body inertia** starts as the rigid-body spatial inertia of each link, then accumulates child contributions:

$$I^A_i \leftarrow I^A_i + X^*_{i+1} \hat{I}^A_{i+1} X_{i+1}$$

where $\hat{I}^A$ is the inertia after removing the joint DOF (the "downdate"):

$$\hat{I}^A_i = I^A_i - \frac{U_i U_i^T}{D_i}$$

with $U_i = I^A_i s_i$, $D_i = s_i^T U_i$, and $s_i$ the joint motion subspace (joint axis for revolute joints).

**Joint acceleration**:

$$\ddot{q}_i = \frac{u_i - U_i^T \hat{a}_i}{D_i}$$

where $u_i = \tau_i - s_i^T p^A_i$ and $\hat{a}_i = {}^{i}X_{\lambda(i)}\, a_{\lambda(i)} + c_i$ is the parent's acceleration expressed in link $i$ plus the velocity-product acceleration $c_i = v_i \times s_i \dot{q}_i$. The link's own acceleration is then $a_i = \hat{a}_i + s_i \ddot{q}_i$.

**Bias-force propagation**: the articulated bias force passed to the parent is

$$p^a_i = p^A_i + \hat{I}^A_i c_i + U_i \frac{u_i}{D_i}, \qquad p^A_{\lambda(i)} \leftarrow p^A_{\lambda(i)} + {}^{\lambda(i)}X^*_i\, p^a_i$$

where $p^A_i$ starts as the rigid-body bias force $v_i \times^* I_i v_i$ (gyroscopic and centripetal terms).

## Complexity Analysis

| Case | Time   | Space  | Notes                              |
|------|--------|--------|------------------------------------|
| All  | $O(n)$ | $O(n)$ | Three linear passes over the chain |

Each pass visits every link exactly once, performing constant-time spatial algebra (3×3 matrix operations) per link.

## Step-by-Step Walkthrough

Consider a 2-link arm of uniform rods with unit masses and unit lengths, both lying along $\hat{x}$ at $q = 0$, rotating about $\hat{y}$, with zero velocities and zero applied torques under gravity $g = [0, 0, -9.81]^T$.

**Pass 1** — Forward kinematics: both links at $q = [0, 0]^T$ with $\dot{q} = [0, 0]^T$, so all velocities and velocity-product terms are zero.

**Pass 2** — Backward pass: starting from link 2, compute the rigid-body inertia, project out the joint DOF, transform to link 1's frame, and accumulate. The articulated inertia at the base now represents the effective inertia of the entire chain.

**Pass 3** — Forward pass: at the base, the gravitational acceleration is transformed into the base frame and used to compute $\ddot{q}_1$. The resulting acceleration is propagated to link 2 to compute $\ddot{q}_2$.

The result, for uniform rods ($I_{\text{CoM}} = ml^2/12$): $\ddot{q} = g\,[\,9/7,\ -12/7\,]^T \approx [12.61, -16.82]^T$.
The shoulder accelerates downward, but the elbow accelerates the *other* way: the outer link initially rotates upward relative to the inner one (its absolute angular acceleration is $\ddot{q}_1 + \ddot{q}_2 = -3g/7$), the classic whip-like start of a double pendulum released from rest.

## Pitfalls & Edge Cases

- **Vanishing articulated inertia**: $D_i$ is the effective inertia about joint $i$ of everything outboard of it. $D_i \to 0$ means a torque on that joint meets no resistance, so $\ddot{q}_i$ becomes unbounded (it does not "lock").
  This happens only for degenerate models, e.g. a massless outboard chain or a point mass lying on the joint axis; valid links with positive inertia about the joint axis keep $D_i > 0$.
- **Numerical precision**: The division by $D_i$ amplifies errors if $D_i$ is small. Use `float` (not fixed-point) for dynamics computations.
- **Floating-point only**: The algorithm involves trigonometric functions, divisions, and large dynamic ranges that are unsuitable for Q15/Q31 fixed-point.

## Variants & Generalizations

- **Composite Rigid Body Algorithm (CRBA)**: Computes the mass matrix $M(q)$ explicitly in $O(n^2)$. Useful when $M$ itself is needed (e.g., for operational-space control).
- **Extended ABA**: Supports branching chains (trees) by processing children before parents in the backward pass.
- **ABA with friction**: Joint friction torques can be added directly to $\tau$.

## Applications

- Real-time simulation of robot arms and multi-body systems
- Model-based control (computed torque, impedance control)
- Forward integration of manipulator dynamics

## Connections to Other Algorithms

- [Recursive Newton-Euler](RecursiveNewtonEuler.md): solves the inverse problem ($q, \dot{q}, \ddot{q} \to \tau$). ABA and RNEA are duals: `RNEA(q, qDot, ABA(q, qDot, tau)) ≈ tau`.
- [Euler-Lagrange](EulerLagrange.md): equivalent $O(n^3)$ formulation using the mass matrix.
- Gaussian elimination or Cholesky factorization ([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)): used if the mass-matrix approach is taken instead.

## References & Further Reading

- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapter 7.
- Featherstone, R. (1983). "The Calculation of Robot Dynamics Using Articulated-Body Inertias." *International Journal of Robotics Research*, 2(1), 13–30.
