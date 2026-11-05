# Hybrid Position/Force Control

## Overview & Motivation

Contact tasks constrain some directions and leave others free. When wiping a table, the tool must press down with a chosen force while sliding freely across the surface. When inserting a peg, it pushes along the hole axis while keeping the lateral force near zero. Pure position control fights the constraint, and pure force control cannot follow a path.

**Hybrid position/force control** (Raibert and Craig, 1981) splits the task space into complementary subspaces in a **constraint frame** aligned with the contact. A selection matrix assigns each axis either to a position loop or to a force loop. The two loops act on disjoint axes, so they never fight.

## Mathematical Theory

### Constraint Frame and Selection

The rotation $R_c$ maps constraint-frame axes to the base frame. A diagonal selection matrix $S$ with entries 0 or 1 marks motion-controlled axes (1) and force-controlled axes (0). Since $S^2 = S$ and $S(I - S) = 0$, the two subspaces are complementary.

Base-frame quantities are rotated into the constraint frame:

$$e_x = R_c^T(x_d \ominus x), \qquad \dot{e}_x = R_c^T(\dot{x}_d - J\dot{q}), \qquad v_c = R_c^T J\dot{q}, \qquad e_f = R_c^T(f_d - f)$$

Here $f$ and $f_d$ are the measured and desired wrench the tool applies on the environment.

### Motion Loop

A PD law acts on the motion axes:

$$F_m = K_p e_x + K_d \dot{e}_x$$

### Force Loop

A PI law with feed-forward and velocity damping acts on the force axes:

$$F_f = R_c^T f_d + K_f e_f + K_i \int (I - S)\,e_f\,dt - K_{df} v_c$$

The integral removes steady force error against stiff surfaces. It accumulates only on force axes and is clamped element-wise to $\pm$`integralLimit` (anti-windup). The damping term $-K_{df} v_c$ suppresses oscillation on contact with stiff environments.

### Joint Torque

$$F = R_c\left(S F_m + (I - S) F_f\right), \qquad \tau = J^T F + C(q,\dot{q})\dot{q} + g(q)$$

Mapping with $J^T$ only (no inverse) keeps the torque bounded through singularities.

### Stability

Purely kinematic hybrid control, which maps the selected errors through $J^{-1}$, can be unstable (An and Hollerbach, 1987). This law compensates gravity and Coriolis torques and uses $J^T$. For full dynamic decoupling of the subspaces, combine the selection with the task-space inertia of operational-space control.

## Complexity Analysis

| Step                     | Time          | Space  | Notes                                        |
|--------------------------|---------------|--------|----------------------------------------------|
| Task velocity and errors | $O(mn + m^2)$ | $O(m)$ | $J\dot{q}$ and constraint-frame rotations    |
| Motion and force loops   | $O(m^2)$      | $O(m)$ | Gain products and one clamped integral       |
| Joint torque             | $O(mn)$       | $O(n)$ | Plus one Coriolis and one gravity evaluation |

## Step-by-Step Walkthrough

A planar task ($m = 2$) uses $S = \operatorname{diag}(1, 0)$: axis 0 is motion-controlled and axis 1 force-controlled. The gains are $K_p = 100$, $K_d = 20$, $K_f = 2$, $K_i = 5$ and $K_{df} = 3$, with $\Delta t = 1$ ms.

1. **Motion axis**: a position error of $0.05$ m and a velocity error of $0.2$ m/s on axis 0 give $100 \cdot 0.05 + 20 \cdot 0.2 = 9$ N there and nothing on axis 1.
2. **Force axis**: $f_d = 5$ N, a measured $3$ N and $0.4$ m/s of motion along axis 1 give $5 + 2 \cdot 2 + 5 \cdot 2 \cdot 0.001 - 3 \cdot 0.4 = 7.81$ N.
3. **Cross terms masked**: a position error on the force axis and a force error on the motion axis produce no torque at all.
4. **Rotated constraint**: with $R_c = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}$, the base-$y$ position error drives the motion loop, the base-$x$ error falls on the force axis and is ignored, and the force loop pushes along base $-x$. The resulting wrench is $(-9.01, 10)$ N.
5. **Anti-windup**: a persistent $10$ N force error saturates the integral at its limit of $1$, so the force term settles at $10 + 2 \cdot 10 + 5 \cdot 1 = 35$ N.

## Pitfalls & Edge Cases

- **Wrong constraint frame**: a misaligned $R_c$ lets the motion loop push into the surface. Align it with the measured contact normal.
- **Contact loss**: when the tool leaves the surface, the force error stays large and the integral winds up to its limit. Call `Reset()` on contact transitions.
- **Stiff environments**: high force gains on stiff contacts oscillate. Use velocity damping and keep $K_f$ low, with a low-pass-filtered force signal.
- **Non-binary selection**: the selection matrix must be diagonal with 0/1 entries; this is checked at construction.
- **Sensor frame**: express the measured wrench in the base frame, about the tool point, before passing it in.

## Variants & Generalizations

- **Dynamic hybrid control**: operational-space formulation with selection in task-inertia-weighted subspaces (Khatib, 1987).
- **Parallel force/position control**: both loops act on all axes, with the force loop dominating through an integral term.
- **Impedance control**: a single compliant relationship instead of separate subspaces.
- **Explicit contact models**: switching between free and constrained motion with contact detection.

## Applications

- Polishing, grinding and deburring with controlled normal force.
- Peg-in-hole and connector insertion.
- Surface following and contour tracing.
- Pushing, scraping and cleaning tasks.

## Connections to Other Algorithms

- [Impedance Control](ImpedanceControl.md): the compliant alternative; its contact wrench convention is the opposite sign.
- [Operational-Space Control](OperationalSpaceControl.md): adds task-space inertia for dynamically decoupled subspaces.
- [Geometric Jacobian](../../kinematics/GeometricJacobian.md): supplies $J$ and the tool pose.
- [Chain Dynamics Model](../../dynamics/ChainDynamicsModel.md): supplies the Coriolis and gravity torques.

## References & Further Reading

- Raibert, M.H. and Craig, J.J. (1981). "Hybrid position/force control of manipulators." *ASME Journal of Dynamic Systems, Measurement, and Control*, 103(2), 126–133.
- An, C.H. and Hollerbach, J.M. (1987). "Kinematic stability issues in force control of manipulators." *IEEE International Conference on Robotics and Automation*, 897–903.
- Khatib, O. (1987). "A unified approach for motion and force control of robot manipulators: the operational space formulation." *IEEE Journal on Robotics and Automation*, 3(1), 43–53.
- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 9.
