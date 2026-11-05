# Chain Dynamics Model

## Overview & Motivation

Model-based manipulator controllers are written against the **manipulator equation**

$$M(q)\,\ddot{q} + C(q, \dot{q})\,\dot{q} + g(q) = \tau$$

Different controllers need different pieces of it:

- PD control with gravity compensation needs only $g(q)$.
- Operational-space control needs $M(q)$.
- Computed-torque control needs the whole right-hand side for a commanded acceleration, ideally in a single pass.

The recursive algorithms compute these pieces efficiently from a physical link description, but they speak in links, frames and spatial vectors rather than in matrices of the joint space.

The chain dynamics model is the adapter between the two worlds. Given a link chain and the gravity vector, it answers every term of the manipulator equation through two small abstract interfaces. Controllers stay independent of how the dynamics are computed, can be tested against mock models, and can be bound to the concrete model on a hard real-time path.

## Mathematical Theory

### Terms from the Recursions

With inverse dynamics written as $\mathrm{ID}(q, \dot{q}, \ddot{q}, a_g)$ (the recursive Newton–Euler algorithm, with gravity entering as a base acceleration $a_g = -g$):

$$\begin{aligned}
g(q) &= \mathrm{ID}(q, 0, 0, -g) \\
C(q, \dot{q})\,\dot{q} &= \mathrm{ID}(q, \dot{q}, 0, 0) \\
M(q)\,\ddot{q} + C\dot{q} + g &= \mathrm{ID}(q, \dot{q}, \ddot{q}, -g) \\
M(q) &= \text{composite rigid body algorithm}
\end{aligned}$$

These identities hold because inverse dynamics is affine in $\ddot{q}$ and in the gravity vector, and quadratic in $\dot{q}$. Setting the other arguments to zero isolates each term. The mass matrix could also be assembled column by column, $M e_j = \mathrm{ID}(q, 0, e_j, 0)$, but the composite rigid body algorithm builds it directly at a lower cost.

### Vector, Not Matrix

The recursion returns the Coriolis and centrifugal **vector** $C(q, \dot{q})\dot{q}$, not the matrix $C(q, \dot{q})$. The vector is all that computed-torque control and simulation need. Passivity-based controllers and momentum observers need a particular factorization of the matrix itself: the Christoffel form, with $\dot{M} - 2C$ skew-symmetric. That form requires a modified recursion.

### Consistency

Because all four quantities come from the same link model, the identity $\mathrm{ID}(q, \dot{q}, \ddot{q}) = M\ddot{q} + C\dot{q} + g$ holds up to rounding. Forward dynamics by solving $M\ddot{q} = \tau - C\dot{q} - g$ reproduces the articulated-body result. Reflected actuator inertia (armature) appears consistently in $M$ and in the one-pass inverse dynamics.

### Model Updates

A changed payload or tool alters the last link's mass, centre of mass and inertia. Replacing the link description updates every term at once, since nothing is cached between calls.

## Complexity Analysis

| Term                        | Time     | Space    | Notes                                   |
|-----------------------------|----------|----------|-----------------------------------------|
| Gravity $g(q)$              | $O(n)$   | $O(n)$   | One inverse-dynamics pass               |
| Coriolis vector $C\dot{q}$  | $O(n)$   | $O(n)$   | One inverse-dynamics pass               |
| Inverse dynamics (one call) | $O(n)$   | $O(n)$   | One pass for $M\ddot{q} + C\dot{q} + g$ |
| Mass matrix $M(q)$          | $O(n^2)$ | $O(n^2)$ | Composite rigid body algorithm          |

Computing the three terms separately and combining them costs three passes plus an $O(n^2)$ product. The one-call inverse dynamics does it in a single $O(n)$ pass, which is why computed-torque control uses it.

## Step-by-Step Walkthrough

Two uniform rods rotate about horizontal parallel axes, with angles measured from the horizontal and gravity $9.81$ m/s² downwards. Rod 1 has $m_1 = 1.2$ kg and $l_1 = 0.6$ m; rod 2 has $m_2 = 0.7$ kg and $l_2 = 0.45$ m. The state is $q = (0.4, -0.9)$, $\dot{q} = (1.1, -0.6)$, $\ddot{q} = (0.8, -1.7)$.

1. Gravity: $g = -9.81\,[\,m_1 \tfrac{l_1}{2}\cos q_1 + m_2(l_1 \cos q_1 + \tfrac{l_2}{2}\cos(q_1{+}q_2)),\ m_2 \tfrac{l_2}{2}\cos(q_1{+}q_2)\,]$.
2. Coriolis: with $h = m_2 l_1 \tfrac{l_2}{2} \sin q_2$, $C\dot{q} = [-h(2\dot{q}_1\dot{q}_2 + \dot{q}_2^2),\ h\dot{q}_1^2]$.
3. Mass matrix: $M_{11} = \tfrac{m_1 l_1^2}{3} + m_2(l_1^2 + \tfrac{l_2^2}{3} + l_1 l_2 \cos q_2)$, $M_{12} = m_2(\tfrac{l_2^2}{3} + \tfrac{l_1 l_2}{2}\cos q_2)$, $M_{22} = \tfrac{m_2 l_2^2}{3}$.
4. Their sum at the given state is $\tau = (-8.2064, -1.4410)$ N·m, exactly the one-pass inverse dynamics.
5. Doubling $m_2$ (a payload) changes only the $m_2$ terms of $g$, and the model reports the new holding torques immediately.

## Pitfalls & Edge Cases

- **Gravity sign**: gravity is the physical vector, e.g. $(0, 0, -9.81)$. The recursion internally applies it as a base acceleration of $+9.81$ upwards; passing $+g$ flips every holding torque.
- **Coriolis matrix vs vector**: do not reconstruct $C(q, \dot{q})$ from the vector. Many matrices produce the same vector, and only the Christoffel one preserves the passivity property.
- **Friction is not dynamics**: joint friction belongs in a separate model added to the torque command. Folding it into $C\dot{q}$ breaks the energy structure that controllers rely on.
- **Virtual dispatch**: the abstract interfaces cost an indirect call per term. On hard real-time paths, bind the controller to the concrete model to devirtualize.
- **Stale payloads**: the model is only as good as its link parameters. Update the last link whenever the gripper picks up a known load.

## Variants & Generalizations

- **Christoffel Coriolis matrix and regressor**: a modified recursion gives $C(q, \dot{q})\dot{q}_r$ with the skew-symmetric property and the inertial-parameter regressor $Y(q, \dot{q}, \dot{q}_r, \ddot{q}_r)$ for adaptive control.
- **Friction and motor models**: Coulomb, viscous and Stribeck terms added beside the rigid-body model.
- **Floating base**: the same interface over a floating-base chain yields the dynamics of legged and aerial manipulators.
- **Identified models**: base-parameter identification replaces nominal link parameters with fitted ones while keeping the same interface.

## Applications

- PD control with gravity compensation (gravity term only).
- Computed-torque and inverse-dynamics control (one-pass inverse dynamics).
- Operational-space and impedance control with inertia shaping (mass matrix).
- Forward-dynamics simulation through $M\ddot{q} = \tau - C\dot{q} - g$, and model-in-the-loop controller tests.

## Connections to Other Algorithms

- [Recursive Newton-Euler](RecursiveNewtonEuler.md): computes the gravity, Coriolis and one-pass inverse-dynamics terms.
- [Composite Rigid Body Algorithm](CompositeRigidBodyAlgorithm.md): computes the mass matrix.
- [Euler-Lagrange](EulerLagrange.md): its forward and inverse dynamics run on any model that supplies $M$, $C\dot{q}$ and $g$, including this one.
- [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md): forward dynamics without forming $M$; it must agree with solving the manipulator equation.
- [PD Control with Gravity Compensation](../controllers/manipulator/PdGravityCompensation.md): consumes the gravity term.

## References & Further Reading

- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapters 5 and 6.
- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 7.
- Spong, M.W., Hutchinson, S. and Vidyasagar, M. (2020). *Robot Modeling and Control*. 2nd ed. Wiley. Chapters 7 and 9.
