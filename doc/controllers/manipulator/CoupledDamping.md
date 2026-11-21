# Coupled Damping Design

## Overview & Motivation

An impedance or PD controller makes each joint behave as a spring of stiffness $K_i$ and a damper. On a single mass the damper that gives damping ratio $\zeta$ is $2\zeta\sqrt{K m}$. A robot arm is not a set of independent masses: its mass matrix couples the joints, and the shoulder and elbow move each other.

A diagonal damper sized from each joint's own inertia ignores that coupling. The fast coupled mode of two strongly coupled joints, whose inertia is much smaller than either joint's own, is then damped far beyond what a sampled controller can deliver, and it oscillates; a light wrist next to heavy proximal joints suffers the same way. Shaping the damper with the mass matrix itself gives every mode of the arm a sensible damping ratio, while each joint keeps the damping its own spring and inertia call for.

## Mathematical Theory

### Design

With $M$ the joint-space mass matrix, $K = \operatorname{diag}(K_i)$ the joint stiffnesses and $\zeta$ the damping ratio, each joint's natural frequency on its own diagonal inertia is

$$\omega_i = \sqrt{\frac{K_i}{M_{ii}}}$$

The damping matrix is the mass matrix scaled on both sides by the square roots of those frequencies:

$$D = 2\zeta\, W M W, \qquad W = \operatorname{diag}\left(\sqrt{\omega_i}\right)$$

### Properties

- **Diagonal**: $D_{ii} = 2\zeta\, \omega_i M_{ii} = 2\zeta \sqrt{K_i M_{ii}}$, the single-mass damper for joint $i$.
- **Coupling**: $D_{ij} = 2\zeta \sqrt{\omega_i \omega_j}\, M_{ij}$, so the damper couples the joints exactly where the arm does.
- **Equal frequencies**: if every $\omega_i = \omega$, then $D = 2\zeta\omega M$. The closed loop $M\ddot{e} + D\dot{e} + Ke = 0$ with $K = \omega^2 M$ then gives every mode the natural frequency $\omega$ and damping ratio $\zeta$.
- **Symmetry and definiteness**: $D$ is symmetric, and positive definite whenever $M$ is and every $K_i > 0$, because $W$ is a positive diagonal congruence.

A joint with no inertia on its diagonal has no defined frequency; it gets no damping, and neither do its coupling terms.

## Complexity Analysis

| Step        | Time     | Space    | Notes                                                    |
|-------------|----------|----------|----------------------------------------------------------|
| Frequencies | $O(n)$   | $O(n)$   | Two square roots per joint                               |
| Scaling     | $O(n^2)$ | $O(n^2)$ | Each entry scaled by two weights, no product of matrices |

## Step-by-Step Walkthrough

Two joints with $M = \begin{bmatrix} 2 & 0.6 \\ 0.6 & 0.5 \end{bmatrix}$, $\zeta = 0.7$ and the stiffnesses $K = (18, 4.5)$.

1. Frequencies: $\omega_1 = \sqrt{18 / 2} = 3$ and $\omega_2 = \sqrt{4.5 / 0.5} = 3$.
2. Weights: $W = \operatorname{diag}(\sqrt{3}, \sqrt{3})$.
3. Damping: $D = 2 \cdot 0.7 \cdot 3 \cdot M = 4.2\, M = \begin{bmatrix} 8.4 & 2.52 \\ 2.52 & 2.1 \end{bmatrix}$.

The frequencies are equal, so the damper is the scaled mass matrix and every mode is damped at $\zeta = 0.7$.

## Pitfalls & Edge Cases

- **Configuration dependence**: $M$ changes with the pose. Recompute $D$ every control period for an arm that moves far, or the coupling terms describe the wrong pose.
- **Rotor inertia**: the reflected rotor inertia of geared joints belongs on the diagonal of $M$ first, or the proximal joints are under-damped.
- **Unequal frequencies**: the per-mode damping ratio is exactly $\zeta$ only when all $\omega_i$ are equal. With very different frequencies the modes are damped near, not at, $\zeta$.
- **Sampling limits**: the damping per control period, $2\zeta\omega_i T$, must stay well below one. A stiff, light joint can ask for more damping than the loop can deliver at a given period.

## Variants & Generalizations

- **Task space**: the same design with the task inertia $\Lambda$ and a task stiffness damps a Cartesian impedance; near a singularity $\Lambda$ is unbounded, which is why a joint-space damper is often preferred there.
- **Per-joint damping ratios**: $D = W' M W'$ with $W' = \operatorname{diag}(\sqrt{2\zeta_i \omega_i})$ when the joints need different ratios.
- **Constant design**: evaluate $M$ at a nominal pose once when the arm moves little.

## Applications

- Joint-space impedance control with stiffnesses chosen per joint.
- Cartesian impedance control, with the stiffness the tool spring puts on each joint, $K_i = (J^T K_x J)_{ii}$.
- PD gains from a bandwidth: equal frequencies give $K_p = \omega^2 M$ and $K_d = 2\zeta\omega M$.

## Connections to Other Algorithms

- [Impedance Control](ImpedanceControl.md): the task-space spring and damper this damper complements in joint space.
- [PD Control with Gravity Compensation](PdGravityCompensation.md): the equal-frequency case gives its mass-scaled gains.
- [Composite Rigid Body Algorithm](../../dynamics/CompositeRigidBodyAlgorithm.md): computes the mass matrix.

## References & Further Reading

- A. Albu-Schäffer, C. Ott, U. Frese, G. Hirzinger, "Cartesian Impedance Control of Redundant Robots: Recent Results with the DLR-Light-Weight-Arms," *IEEE ICRA*, 2003.
- N. Hogan, "Impedance Control: An Approach to Manipulation," *ASME Journal of Dynamic Systems, Measurement, and Control*, 107(1), 1985.
