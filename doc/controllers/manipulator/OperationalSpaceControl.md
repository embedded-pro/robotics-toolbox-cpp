# Operational-Space Control

## Overview & Motivation

Many tasks are naturally specified at the tool: follow a weld seam, hold a camera on a target, press with a given stiffness. Joint-space controllers must first convert such a task into joint trajectories through inverse kinematics, and they still do not shape how the tool **responds** to errors and disturbances.

**Operational-space control** (Khatib, 1987) writes the manipulator dynamics directly in task coordinates. It commands a task acceleration through the **task-space inertia** $\Lambda(q)$, so the tool behaves like a decoupled, unit-mass second-order system along every task axis.
On redundant arms, a **dynamically consistent null-space projector** lets a secondary objective (posture, joint-limit avoidance) act through the remaining freedom without disturbing the task at all.

## Mathematical Theory

### Task-Space Dynamics

With the task Jacobian $J$ ($m \times n$), $\dot{x} = J\dot{q}$ and $\ddot{x} = J\ddot{q} + \dot{J}\dot{q}$. Substituting the joint dynamics $M\ddot{q} + C\dot{q} + g = \tau$ with $\tau = J^T F$ gives

$$\Lambda(\ddot{x} - \dot{J}\dot{q}) = F - J^{T\#}(C\dot{q} + g), \qquad \Lambda = (J M^{-1} J^T)^{-1}$$

### Control Law

The task command is a PD law with feed-forward:

$$a_x = \ddot{x}_d + K_d(\dot{x}_d - J\dot{q}) + K_p\,e, \qquad e = x_d \ominus x$$

and the joint torque is

$$\tau = J^T\Lambda\left(a_x - \dot{J}\dot{q}\right) + C\dot{q} + g + N\tau_0, \qquad N = I - J^T\bar{J}^T, \quad \bar{J} = M^{-1}J^T\Lambda$$

Here $\tau_0$ is a secondary joint torque. Substituting gives $\ddot{x} = a_x$ exactly: the error dynamics are $\ddot{e} + K_d\dot{e} + K_p e = 0$ per axis, independent of the arm's configuration.

### Why Compensate Coriolis and Gravity in Joint Space

The classic form $F = \Lambda(a_x - \dot{J}\dot{q}) + \bar{J}^T(C\dot{q} + g)$ compensates only the task-space part $J^T\bar{J}^T(C\dot{q} + g)$. On a redundant arm the null-space part $N(C\dot{q} + g)$ is left uncompensated, so the self-motion sags under gravity.
Adding $C\dot{q} + g$ in joint space equals the classic form plus that missing term. The task behaviour is unchanged, and at rest with zero error the torque is exactly $g$.

### Dynamic Consistency

$J M^{-1} N = 0$, so the secondary torque produces no task acceleration: $\tau_0$ only moves the arm within the null space of the task.

### Singularities

$\Lambda$ diverges when $J$ loses rank. A damping term $\sigma$ regularizes it as $\Lambda = (J M^{-1} J^T + \sigma^2 I)^{-1}$. The torque stays bounded, at the price of approximate task tracking and approximate dynamic consistency.

All inverses are applied through linear solves: first $X = M^{-1}J^T$, then the small $m \times m$ system for $\Lambda$. $M^{-1}$ is never formed.

## Complexity Analysis

| Step                   | Time          | Space    | Notes                                                   |
|------------------------|---------------|----------|---------------------------------------------------------|
| $M^{-1}J^T$            | $O(m n^3)$    | $O(n^2)$ | Column-wise elimination; $O(n^3 + m n^2)$ with Cholesky |
| Task inertia $\Lambda$ | $O(m^3)$      | $O(m^2)$ | One small solve                                         |
| Null-space projector   | $O(n^2 m)$    | $O(n^2)$ | $I - J^T\Lambda X^T$                                    |
| Torque                 | $O(nm + n^2)$ | $O(n)$   | Plus one mass-matrix, Coriolis and gravity evaluation   |

## Step-by-Step Walkthrough

1. **Unit inertia**: with $M = I$ and $J = I$, $\Lambda = I$ and the controller reduces to task-space PD, $\tau = \ddot{x}_d + K_d\dot{e} + K_p e$.
2. **Task inertia**: with $M = \operatorname{diag}(2, 3)$ and $J = I$, $\Lambda = \operatorname{diag}(2, 3)$ scales the command axis by axis.
3. **Non-square mapping**: with $M = I$ and $J = \begin{bmatrix} 1 & 0 \\ 1 & 2 \end{bmatrix}$, a commanded task acceleration $(1, 2)$ becomes the joint torque $J^{-1}(1, 2) = (1, 0.5)$.
4. **Redundant arm at rest**: a three-joint arm with a two-dimensional task, $M = \begin{bmatrix} 3 & 0.5 & 0.2 \\ 0.5 & 2 & 0.3 \\ 0.2 & 0.3 & 1 \end{bmatrix}$ and gravity torque $(4, -2, 1.5)$, returns exactly $(4, -2, 1.5)$. The classic $\bar{J}^T g$ form would return $(3.34, 0.27, -0.31)$ and let the arm sag.
5. **Secondary torque**: $\tau_0 = (1, -2, 0.5)$ changes the torque by $N\tau_0 = (0.389, -1.328, 1.062)$, and $J M^{-1} N\tau_0 = 0$: the task does not see it.

## Pitfalls & Edge Cases

- **Singularities**: with $\sigma = 0$ a rank-deficient Jacobian makes the task-inertia solve fail. Use $\sigma > 0$ near singular configurations.
- **Model errors**: $\Lambda$, $C\dot{q}$ and $g$ come from the model, so mismatch couples the task axes. The PD gains keep the loop stable, but decoupling degrades.
- **Orientation tasks**: the rotation-vector error rate equals $\omega_d - \omega$ only to first order, so large orientation errors are not exactly decoupled.
- **Cost**: the $M^{-1}J^T$ solve dominates. Factor $M$ once (Cholesky) and cache $\Lambda$ when the task dimension is small.
- **Secondary objectives**: $N\tau_0$ is a torque, not a velocity. Design $\tau_0$ as a joint-space PD on the posture, for example.

## Variants & Generalizations

- **Hierarchical (prioritized) control**: several tasks projected into each other's null spaces in priority order.
- **Whole-body control**: floating-base and contact-constrained operational-space formulations for humanoids.
- **Unified force/motion control**: force regulation along constrained directions inside the same framework.
- **Impedance shaping**: using $\Lambda$ to render a chosen task-space inertia instead of unit mass.

## Applications

- Task-space trajectory tracking on redundant arms with posture control.
- Humanoid and mobile-manipulator whole-body control.
- Haptic and teleoperation interfaces that need decoupled task dynamics.

## Connections to Other Algorithms

- [Impedance Control](ImpedanceControl.md): shares the task error and the task-space inertia solve; renders compliance instead of decoupled tracking.
- [Computed-Torque Control](ComputedTorqueControl.md): the joint-space counterpart of feedback linearization.
- [Redundancy Resolution](../../kinematics/RedundancyResolution.md): kinematic null-space projection, the velocity-level analogue.
- [Chain Dynamics Model](../../dynamics/ChainDynamicsModel.md): supplies $M$, $C\dot{q}$ and $g$.

## References & Further Reading

- Khatib, O. (1987). "A unified approach for motion and force control of robot manipulators: the operational space formulation." *IEEE Journal on Robotics and Automation*, 3(1), 43–53.
- Nakanishi, J., Cory, R., Mistry, M., Peters, J. and Schaal, S. (2008). "Operational space control: a theoretical and empirical comparison." *International Journal of Robotics Research*, 27(6), 737–757.
- Siciliano, B. and Khatib, O. (eds.) (2016). *Springer Handbook of Robotics*. Springer. Chapter 8.
