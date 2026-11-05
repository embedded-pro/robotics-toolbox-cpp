# Impedance Control

## Overview & Motivation

Position control makes a robot as stiff as possible: any deviation from the commanded trajectory is fought with full actuator effort. That is the wrong behaviour as soon as the tool touches something. A small position error against a rigid surface produces huge forces, and a person bumping into the arm meets a wall.

**Impedance control** (Hogan, 1985) regulates the **relationship** between motion and force instead of either alone. The tool behaves like a programmable mass–spring–damper attached to the desired pose: it follows the trajectory in free space and yields with a chosen stiffness on contact.
This is the basis of polishing, assembly, hand guiding and safe human–robot collaboration.

Two variants are provided:

- **Stiffness/damping impedance** needs only the Jacobian transpose and gravity compensation: no force sensor, no mass matrix, and robust at singularities.
- **Impedance with inertia shaping** also renders a chosen task-space inertia. It needs the mass matrix, the Jacobian derivative term and a measured contact wrench.

## Mathematical Theory

### Task Error

For a desired pose $T_d$ and the current tool pose $T(q)$, the task error $e = T_d \ominus T$ is

- the pose error $(p_d - p;\ \log(R_d R^T)^\vee)$ for six-dimensional tasks, never a difference of orientation parameters;
- the first $m$ components of $p_d - p$ for position tasks ($m \le 3$).

With the task Jacobian $J$, the error rate is $\dot{e} = \dot{x}_d - J\dot{q}$. The contact wrench $f_{\text{ext}}$ is the wrench the environment exerts on the tool, so the arm obeys

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau + J^T f_{\text{ext}}$$

### Stiffness/Damping Impedance

$$\tau = J^T\left(K e + D \dot{e}\right) + g(q)$$

At rest with $J$ of full rank, $K e = -f_{\text{ext}}$, so the tool deflects by $x - x_d = K^{-1} f_{\text{ext}}$: it yields **along** the external force with the programmed compliance. The inertia felt at the tool is the arm's own task-space inertia $\Lambda(q)$.

The Coriolis term is deliberately **not** cancelled. With the skew-symmetry of $\dot{M} - 2C$, the storage function $V = \tfrac12\dot{q}^T M\dot{q} + \tfrac12 e^T K e$ gives, for a constant position target,

$$\dot{V} = -\dot{x}^T D\,\dot{x} + \dot{x}^T f_{\text{ext}}$$

The controlled arm is **passive** at the port $(\dot{x}, f_{\text{ext}})$, so it remains stable in contact with any passive environment.

### Impedance with Inertia Shaping

To render the target impedance

$$M_d\,\ddot{\tilde{x}} + D\,\dot{\tilde{x}} + K\tilde{x} = f_{\text{ext}}, \qquad \tilde{x} = x - x_d = -e$$

the controller commands the task acceleration

$$a_x = \ddot{x}_d + M_d^{-1}\left(D\dot{e} + K e + f_{\text{ext}}\right)$$

and realizes it through the task-space inertia $\Lambda = (J M^{-1} J^T)^{-1}$:

$$\tau = J^T\left[\Lambda\left(a_x - \dot{J}\dot{q}\right) - f_{\text{ext}}\right] + C\dot{q} + g$$

$\Lambda$ is applied through linear solves: first $X = M^{-1}J^T$, then $(JX)\,F' = a_x - \dot{J}\dot{q}$. Neither $M^{-1}$ nor $\Lambda$ is formed explicitly. Substituting into the dynamics gives $\ddot{x} = a_x$ exactly, hence the target impedance for position tasks. For pose tasks the rotational part holds to first order about zero orientation error.

With $M_d = \Lambda(q)$ the contact-wrench terms cancel, and the law reduces to the stiffness/damping law plus feed-forward $\Lambda(\ddot{x}_d - \dot{J}\dot{q}) + C\dot{q}$.

## Complexity Analysis

| Variant           | Time               | Space         | Notes                                                                  |
|-------------------|--------------------|---------------|------------------------------------------------------------------------|
| Stiffness/damping | $O(mn)$ + gravity  | $O(mn)$       | $J\dot{q}$, $J^T F$ and one gravity pass                               |
| Inertia shaping   | $O(m n^3)$ + model | $O(mn + m^2)$ | Mass matrix, $m$ column solves for $M^{-1}J^T$, one $m \times m$ solve |
| Construction      | $O(m^3)$           | $O(m^2)$      | $M_d^{-1}$ computed once                                               |

## Step-by-Step Walkthrough

A planar task ($m = 2$) with $K = \operatorname{diag}(500, 300)$ N/m, $D = \operatorname{diag}(40, 30)$ N·s/m and $M_d = \operatorname{diag}(2, 1.5)$ kg drives a Cartesian two-axis plant with mass $\operatorname{diag}(1, 2)$ kg and gravity load $(0, 19.62)$ N.

1. **Free-space law.** With $J = I$, $e = (0.05, -0.05)$ m, $\dot{e} = (-0.1, 0.15)$ m/s and gravity torque $g$, the stiffness/damping torque is $K e + D\dot{e} + g = (25 - 4, -15 + 4.5) + g$.
2. **Jacobian transpose.** With $J = \begin{bmatrix} 1 & 0.5 \\ 0 & 2 \end{bmatrix}$, the task wrench $K e = (25, -15)$ maps to joint torques $J^T K e = (25, -17.5)$.
3. **Static contact.** A constant wrench $f_{\text{ext}} = (3, -4)$ N pushes the tool, which settles at $x - x_d = K^{-1} f_{\text{ext}} = (0.006, -0.0133)$ m with zero velocity.
4. **Rendered inertia.** With inertia shaping, the plant's deflection after a step contact follows the target system $M_d\ddot{\tilde{x}} + D\dot{\tilde{x}} + K\tilde{x} = f_{\text{ext}}$ to within $10^{-5}$ m over the whole transient, although the plant's own mass differs from $M_d$.

## Pitfalls & Edge Cases

- **Orientation error**: subtracting Euler angles or quaternions produces wrong and discontinuous restoring torques. Use the log-map pose error.
- **Cancelling Coriolis in the compliant law**: adding $C\dot{q}$ to the stiffness/damping law looks more exact but destroys passivity.
- **Force-sensor bias**: inertia shaping with $M_d \ne \Lambda$ feeds the measured wrench forward, so a sensor bias shifts the rendered equilibrium. Low-pass the measurement and match $D$ to the sensor bandwidth.
- **Singularities**: inertia shaping needs $\Lambda$, which blows up at singular configurations. Damp $J M^{-1} J^T + \sigma^2 I$ there; the stiffness/damping law uses only $J^T$ and stays bounded.
- **Very high stiffness**: approaches rigid position control and excites contact instability at a given sample rate.
- **Redundant arms**: both laws leave the null space uncontrolled. Add a null-space posture term.

## Variants & Generalizations

- **Admittance control**: the dual causality. Measure the wrench, integrate the target dynamics to a motion command and track it with a stiff position loop. Better on non-backdrivable robots.
- **Variable impedance**: time-varying $K$ and $D$, with passivity preserved through energy tanks.
- **Operational-space control**: full task-space dynamics decoupling with a dynamically consistent null space.
- **Hybrid position/force control**: force regulation along constrained directions and motion along free ones.

## Applications

- Peg-in-hole assembly with stiff normal and compliant tangential axes.
- Polishing, grinding and deburring with regulated contact.
- Hand guiding and kinesthetic teaching.
- Safe physical human–robot interaction.

## Connections to Other Algorithms

- [PD Control with Gravity Compensation](PdGravityCompensation.md): the joint-space special case of the stiffness/damping law.
- [Computed-Torque Control](ComputedTorqueControl.md): joint-space feedback linearization, the counterpart of inertia shaping.
- [Geometric Jacobian](../../kinematics/GeometricJacobian.md): supplies $J$, $\dot{J}\dot{q}$ and the tool pose.
- [Chain Dynamics Model](../../dynamics/ChainDynamicsModel.md): supplies $M$, $C\dot{q}$ and $g$.

## References & Further Reading

- Hogan, N. (1985). "Impedance control: an approach to manipulation, Parts I–III." *ASME Journal of Dynamic Systems, Measurement, and Control*, 107(1), 1–24.
- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 9.
- Ott, C. (2008). *Cartesian Impedance Control of Redundant and Flexible-Joint Robots*. Springer.
- Colgate, J.E. and Hogan, N. (1988). "Robust control of dynamically interacting systems." *International Journal of Control*, 48(1), 65–88.
