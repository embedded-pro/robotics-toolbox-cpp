# PD Control with Gravity Compensation

## Overview & Motivation

Holding a manipulator at a commanded pose is the most common joint-control task: pick-and-place approach points, teach-in positions, parking and safe-stop poses. A plain PD loop on each joint behaves like a virtual spring and damper. Under gravity that spring sags, and each loaded joint settles short of its target by the deflection needed to carry the load.

**PD control with gravity compensation** adds the gravity torque $g(q)$ of the arm as a feed-forward term:

$$\tau = K_p\,(q_d - q) - K_d\,\dot{q} + g(q)$$

Takegaki and Arimoto (1981) proved that this law drives the arm to **any** constant set-point $q_d$ from **any** initial state, for **any** symmetric positive-definite gains. It needs only $g(q)$, not the mass matrix or Coriolis terms, which makes it the cheapest model-based joint controller with a global stability guarantee. On a microcontroller it is a handful of multiply–adds per joint plus one gravity evaluation.

## Mathematical Theory

### Closed-Loop Dynamics

The manipulator obeys $M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau$. With the error $e = q_d - q$ (constant $q_d$, so $\dot{e} = -\dot{q}$), substituting the control law cancels gravity exactly:

$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + K_d\,\dot{q} = K_p\,e$$

The closed loop is a nonlinear mass–spring–damper with the spring anchored at the set-point.

### Lyapunov Stability

Take the total energy of that virtual system as the candidate function:

$$V(e, \dot{q}) = \tfrac12\,\dot{q}^T M(q)\,\dot{q} + \tfrac12\,e^T K_p\,e$$

It is positive definite because $M$ and $K_p$ are. Differentiating along trajectories and using the skew-symmetry of $\dot{M} - 2C$ (a property of the Christoffel form of $C$):

$$\dot{V} = \dot{q}^T\left(K_p e - C\dot{q} - K_d\dot{q}\right) + \tfrac12\,\dot{q}^T \dot{M}\,\dot{q} - \dot{q}^T K_p\,e = -\dot{q}^T K_d\,\dot{q} \le 0$$

$\dot{V}$ vanishes only when $\dot{q} = 0$. On that set the dynamics reduce to $M\ddot{q} = K_p e$, so the motion stays at rest only if $e = 0$. By LaSalle's invariance principle the equilibrium $(e, \dot{q}) = (0, 0)$ is **globally asymptotically stable**.

### Steady State Without Compensation

Dropping $g(q)$ leaves the equilibrium condition $K_p e_{ss} = g(q_d - e_{ss})$. To first order, the arm settles with the droop $e_{ss} \approx K_p^{-1} g(q_d)$. Compensation removes this offset without raising the gains, so the loop can stay soft and well damped.

## Complexity Analysis

| Case           | Time                   | Space       | Notes                                                          |
|----------------|------------------------|-------------|----------------------------------------------------------------|
| Full gains     | $O(n^2) + T_g$         | $O(n^2)$    | Two gain matrix–vector products plus one gravity evaluation    |
| Diagonal gains | $O(n) + T_g$           | $O(n)$      | Joints decouple; $2n$ multiplies plus the gravity evaluation   |

$T_g$ is the cost of the gravity vector: $O(n)$ with a recursive Newton–Euler pass at zero velocity and acceleration, or a closed form for short chains.

## Step-by-Step Walkthrough

A two-joint arm with $K_p = \mathrm{diag}(100, 100)$, $K_d = \mathrm{diag}(20, 20)$, $q = (0.3, -0.6)$, $\dot{q} = (0.4, -1.0)$, $q_d = (0.5, -0.2)$ and gravity torque $g(q) = (1.5, -2.5)$ N·m:

1. Position error $e = q_d - q = (0.2, 0.4)$, so the spring torque is $K_p e = (20, 40)$.
2. Damping torque $-K_d \dot{q} = (-8, 20)$. Joint 2 moving negative is pushed back positive.
3. Gravity feed-forward $(1.5, -2.5)$.
4. Command $\tau = (13.5, 57.5)$ N·m.

At the set-point at rest, the spring and damping terms vanish and $\tau = g(q_d)$, exactly the holding torque.

## Pitfalls & Edge Cases

- **Set-point regulator only**: the stability proof needs constant $q_d$. Tracking a moving reference with this law lags, and the lag grows with speed. Use computed-torque or passivity-based tracking control instead.
- **Model mismatch**: an error $\Delta g$ in the gravity model leaves a steady offset $e_{ss} = K_p^{-1}\Delta g$. Payload changes are the usual source. Integral action or online payload estimation removes it, at the cost of the simple proof.
- **Actuator saturation**: large initial errors command $K_p e$ torques the motors cannot deliver. Saturated PD+g still converges for bounded gravity when the limits exceed $\max|g|$, but overshoot grows. Clamp or ramp the set-point.
- **Velocity noise**: $K_d$ multiplies a differentiated encoder signal. Filter it, or use a velocity observer, before raising $K_d$.
- **Friction**: Coulomb friction creates a dead band of radius $K_p^{-1}\tau_c$ around the set-point. Add friction feed-forward if that matters.
- **Gain choice**: per joint, $K_{d,i} \approx 2\sqrt{K_{p,i} M_{ii}}$ gives roughly critical damping around the operating pose.

## Variants & Generalizations

- **Desired gravity compensation**: use $g(q_d)$ instead of $g(q)$. It is cheaper, since it can be precomputed per set-point, and it is still globally stable when $K_p$ exceeds the Lipschitz constant of $g$ (Tomei, 1991).
- **PID + gravity**: integral action rejects constant model errors. Stability becomes semi-global and needs gain conditions.
- **Task-space PD + gravity**: $\tau = J^T(K_p \tilde{x} - K_d \dot{x}) + g(q)$ regulates the end-effector pose instead, the basis of impedance control without inertia shaping.
- **Saturated PD + gravity**: bounded spring functions (e.g. $\tanh$) keep commands within actuator limits while preserving global stability.
- **Energy shaping**: the PD term reshapes the potential energy into a bowl centred at $q_d$, and the damping injects dissipation. This is the starting point of passivity-based control.

## Applications

- Holding and positioning tasks: approach poses, parking, teach-in and inspection stops.
- A safe fallback when the full dynamic model is unavailable or untrusted.
- Gravity-compensated "float" or hand-guiding modes, with $K_p = 0$ and small $K_d$.
- Inner loop for set-point sequences generated by a higher-level planner.

## Connections to Other Algorithms

- [Euler-Lagrange](../../dynamics/EulerLagrange.md): the controller draws its gravity torque from the Euler–Lagrange model interface.
- [Recursive Newton-Euler](../../dynamics/RecursiveNewtonEuler.md): with zero velocity and acceleration, inverse dynamics returns exactly $g(q)$ in $O(n)$.
- [Composite Rigid Body Algorithm](../../dynamics/CompositeRigidBodyAlgorithm.md): the mass matrix in the Lyapunov function; it shapes the transient but is not needed by the law.
- Friction feed-forward and the generic PID building blocks come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Takegaki, M. and Arimoto, S. (1981). "A New Feedback Method for Dynamic Control of Manipulators." *ASME J. Dynamic Systems, Measurement, and Control*, 103(2), 119–125.
- Tomei, P. (1991). "Adaptive PD Controller for Robot Manipulators." *IEEE Trans. Robotics and Automation*, 7(4), 565–570.
- Spong, M.W., Hutchinson, S. and Vidyasagar, M. (2020). *Robot Modeling and Control*. 2nd ed. Wiley. Chapter 9.
- Kelly, R., Santibáñez, V. and Loría, A. (2005). *Control of Robot Manipulators in Joint Space*. Springer. Chapter 7.
