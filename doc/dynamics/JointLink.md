# Generic Joint Link

## Overview & Motivation

A serial manipulator is a chain of rigid links joined by one-degree-of-freedom joints. Most textbook derivations assume every joint is **revolute**, but real machines are rarely all-revolute. SCARA arms, gantries, linear axes, telescoping booms and hydraulic rams contain **prismatic** (sliding) joints.
Two other per-joint quantities matter as much as the geometry:

- **Joint limits** bound position, velocity and effort. Every planner, trajectory generator and inverse-kinematics solver needs them.
- **Armature** is the actuator inertia reflected through the gearbox, $k_r^2 I_{\text{rotor}}$. On small geared arms it often dominates the effective inertia of the proximal joints, and ignoring it makes model-based control markedly worse.

The generic joint link carries the joint type, its limits and its armature next to the usual mass, inertia and geometry. Every kinematic and dynamic recursion then treats the joint through one abstraction, its **motion subspace**.

## Mathematical Theory

### Link Description

Link $i$ carries:

- mass $m_i$, rotational inertia $\bar{I}_i$ about its centre of mass, and centre of mass $c_i$, all in the link frame;
- a unit joint axis $\hat{z}_i$: the rotation axis of a revolute joint, or the slide direction of a prismatic one;
- the joint origin $r_i$ at $q_i = 0$, expressed in the parent frame;
- the joint type, limits $[q_{\min}, q_{\max}]$, $|\dot{q}| \le \dot{q}_{\max}$, $|\tau| \le \tau_{\max}$ (unbounded by default), and armature $J_{a,i}$.

### Joint Transform

The pose of the link frame relative to its parent depends on the joint variable:

$$\text{revolute: } \left(R(\hat{z}_i, q_i),\; r_i\right), \qquad \text{prismatic: } \left(\mathbf{I}_3,\; r_i + \hat{z}_i\, q_i\right)$$

A revolute joint rotates the frame about a fixed origin. A prismatic joint translates the origin along a fixed direction without rotating the frame, so its axis may be read in either the parent or the link frame.

### Motion Subspace

In spatial notation (angular part first), a one-degree-of-freedom joint moves along

$$S_i = \begin{bmatrix} s_{\omega,i} \\ s_{v,i} \end{bmatrix} = \begin{cases} (\hat{z}_i;\ 0) & \text{revolute} \\ (0;\ \hat{z}_i) & \text{prismatic} \end{cases}$$

The joint contributes the velocity $S_i \dot{q}_i$ to its link, and the actuator torque or force is the projection of the transmitted wrench onto the same subspace:

$$\tau_i = s_{\omega,i}^T n_i + s_{v,i}^T f_i + J_{a,i}\,\ddot{q}_i$$

For revolute joints this is the familiar axis–torque projection; for prismatic joints it becomes an axis–force projection. Writing the recursions in terms of $S_i$ makes one implementation serve both joint types without branching.

### What Changes in the Recursions

- **Velocity**: $\omega_i = R_i^T \omega_{i-1} + s_{\omega,i}\dot{q}_i$ and $v_i = R_i^T(v_{i-1} + \omega_{i-1} \times r_i(q)) + s_{v,i}\dot{q}_i$.
- **Acceleration of the joint origin**: a prismatic joint on a rotating link adds the Coriolis term $2\,\omega_i \times (\hat{z}_i \dot{q}_i)$ and the relative acceleration $\hat{z}_i \ddot{q}_i$.
- **Offsets**: every transform to the parent uses $r_i(q)$, which grows with a prismatic joint's displacement.
- **Inertia**: the armature adds only to the joint's own diagonal entry of the mass matrix and to the articulated inertia $D_i = S_i^T I^A_i S_i + J_{a,i}$. It changes neither kinematics nor gravity.

## Complexity Analysis

| Operation        | Time   | Space  | Notes                                                 |
|------------------|--------|--------|-------------------------------------------------------|
| Joint transform  | $O(1)$ | $O(1)$ | One axis–angle rotation (revolute) or one scaled add  |
| Motion subspace  | $O(1)$ | $O(1)$ | Selects the angular or linear channel                 |
| Per-link storage | —      | $O(1)$ | Type byte, four limit scalars and one armature scalar |

The recursions keep their order: $O(n)$ for forward kinematics, inverse dynamics and forward dynamics, and $O(n^2)$ for the mass matrix.

## Step-by-Step Walkthrough

**Vertical slider.** A $2$ kg carriage slides along $+z$ under gravity $9.81$ m/s² and accelerates upwards at $1$ m/s². Its joint origin accelerates at $a = g + \ddot{q}$ upwards relative to the free-fall frame, and the actuator force is the projection of $m a$ on the slide axis:

$$\tau = m(g + \ddot{q}) = 2 \cdot (9.81 + 1) = 21.62\ \text{N}$$

**Slider on a turntable.** A turntable spins about $z$ at $\omega = 1.5$ rad/s with $\dot{\omega} = 0.4$ rad/s². A $2$ kg point mass sits on a radial slider at $r = 0.5$ m, extending at $\dot{r} = 0.3$ m/s with $\ddot{r} = 0.2$ m/s². The base torque is

$$\tau_0 = m r^2 \dot{\omega} + 2 m r \dot{r} \omega = 0.2 + 0.9 = 1.1\ \text{N·m}$$

where the second term is the Coriolis contribution. The slider force is $\tau_1 = m(\ddot{r} - r\omega^2) = -1.85$ N, which is the centripetal pull.

**Armature.** A uniform $1$ kg, $1$ m rod with $0.2$ kg·m² of armature has effective inertia $\tfrac13 + 0.2 = 0.533$ kg·m² about its joint, in the inverse dynamics, the forward dynamics and the mass matrix alike.

## Pitfalls & Edge Cases

- **Forgetting the Coriolis term**: a slider riding on a rotating link feels $2\omega \times \hat{z}\dot{q}$. Revolute-only code that is "extended" by swapping the projection silently drops it.
- **Projecting the wrong channel**: prismatic joints must project the force, not the moment. A torque projection on a prismatic joint returns a meaningless value with the right units.
- **Non-unit axes**: a non-unit axis scales the rotation angle of a revolute joint and the displacement of a prismatic one. Normalize axes when the model is built.
- **Limits are not enforced by the dynamics**: limits are data for planners, inverse kinematics and safety layers. Inverse and forward dynamics never clamp.
- **Armature units**: kg·m² for revolute joints and kg for prismatic joints. Reflected inertia scales with the square of the gear ratio, so a small rotor behind a 100:1 gearbox easily outweighs a light link.
- **Singular models**: a prismatic joint whose outboard mass is zero and has no armature gives $D_i = 0$; forward dynamics through it is undefined.

## Variants & Generalizations

- **Helical (screw) joints**: $S = (\hat{z};\ p\,\hat{z})$ with pitch $p$ couples rotation and translation in one degree of freedom.
- **Multi-degree-of-freedom joints**: spherical, planar and floating joints have motion subspaces with several columns. They need the matrix form of the projections and the articulated-inertia update.
- **Denavit–Hartenberg descriptions**: the same joint types appear as variable $\theta_i$ (revolute) or variable $d_i$ (prismatic) in DH tables.
- **Motor models**: armature generalizes to a full rotor model with gyroscopic coupling for high-speed joints, and friction enters as an extra joint torque.

## Applications

- SCARA arms, Cartesian gantries and pick-and-place machines with linear axes.
- Telescopic manipulators, linear actuators and hydraulic cylinders modelled as prismatic joints.
- Accurate inverse dynamics and computed-torque control of geared arms, where armature dominates.
- Limit-aware planning, inverse kinematics and trajectory time-scaling.

## Connections to Other Algorithms

- [Forward Kinematics](../kinematics/ForwardKinematics.md): composes the joint transforms along the chain.
- [Recursive Newton-Euler](RecursiveNewtonEuler.md): propagates velocities and accelerations through each joint's motion subspace and projects wrenches back onto it.
- [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md): projects articulated inertias onto the motion subspace, with the armature added to $D_i$.
- [Composite Rigid Body Algorithm](CompositeRigidBodyAlgorithm.md): builds mass-matrix columns from the composite inertia acting on the motion subspace, plus the armature on the diagonal.

## References & Further Reading

- Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control*. 3rd ed. Pearson. Chapters 3 and 6.
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapter 4.
- Spong, M.W., Hutchinson, S. and Vidyasagar, M. (2020). *Robot Modeling and Control*. 2nd ed. Wiley. Chapter 6.
