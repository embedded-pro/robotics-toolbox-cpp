# Coriolis Matrix and Inertial Regressor

## Overview & Motivation

The recursive Newton–Euler algorithm returns the Coriolis and centrifugal **vector** $C(q,\dot{q})\dot{q}$, enough for inverse dynamics and simulation. Several advanced algorithms need more:

- **passivity-based adaptive control** needs $C(q,\dot{q})\dot{q}_r$ for a reference velocity $\dot{q}_r \ne \dot{q}$;
- **momentum-based collision observers** need $C^T(q,\dot{q})\dot{q}$;
- **identification and adaptive control** need the dynamics written **linearly in the inertial parameters**, $Y(q,\dot{q},\dot{q}_r,\ddot{q}_r)\,\pi$.

All three come from one modified recursive pass. It uses a particular factorization of the Coriolis terms, the **Christoffel** form, for which $\dot{M} - 2C$ is skew-symmetric. That is the passivity property every energy-based proof relies on.

## Mathematical Theory

### Christoffel Coriolis Matrix

Many matrices $C$ satisfy the same product $C\dot{q}$. The Christoffel choice is

$$C_{ij} = \sum_k \Gamma_{ijk}\,\dot{q}_k, \qquad \Gamma_{ijk} = \tfrac12\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)$$

and it is the unique choice that makes $N = \dot{M} - 2C$ skew-symmetric, so that $x^T N x = 0$ for every $x$.

### Modified Recursion

In spatial notation (motion vectors $(\omega; v)$, force vectors $(n; f)$, link frames), each link $i$ has the propagated quantities

$$\begin{aligned}
v_i &= X_i v_{i-1} + S_i \dot{q}_i, \qquad v_{r,i} = X_i v_{r,i-1} + S_i \dot{q}_{r,i} \\
a_i &= X_i a_{i-1} + S_i \ddot{q}_{r,i} + (v_i \times S_i)\,\dot{q}_{r,i}
\end{aligned}$$

with gravity entering as the base acceleration $a_{-1} = (0; -g)$. The body wrench uses a symmetric Coriolis operator:

$$f_i = I_i a_i + \tfrac12\left( v_i \times^{*} I_i v_{r,i} - I_i (v_i \times v_{r,i}) + v_{r,i} \times^{*} I_i v_i \right)$$

Wrenches are then accumulated tip to base, and each joint torque is the projection onto the joint's motion subspace: $\tau_i = S_i^T f_i$, plus the armature term. The result is

$$\tau = M(q)\,\ddot{q}_r + C(q,\dot{q})\,\dot{q}_r + g(q)$$

with $C$ in the Christoffel form (Echeandia and Wensing, 2021). For $\dot{q}_r = \dot{q}$ and $\ddot{q}_r = \ddot{q}$ the operator collapses to $v \times^{*} I v$ and the recursion is ordinary inverse dynamics.

### Coriolis Matrix and Its Transpose

Column $j$ of $C(q,\dot{q})$ is the recursion with $\dot{q}_r = e_j$, $\ddot{q}_r = 0$ and no gravity. Then $C^T\dot{q}$ follows by a matrix–vector product.

### Inertial Regressor

Each link contributes ten parameters about its joint origin: mass $m$, first moment $m c$, and the six independent entries of the rotational inertia $I_O = \bar{I} + m [c]_\times^T [c]_\times$. The spatial inertia acting on a motion $(\omega; u)$ is linear in them:

$$I\,(\omega; u) = A(\omega, u)\,\pi_i, \qquad A = \begin{bmatrix} 0 & -[u]_\times & L(\omega) \\ u & [\omega]_\times & 0 \end{bmatrix}$$

where $L(\omega)$ arranges $\omega$ so that $L(\omega)\,(I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz})^T = I_O\,\omega$. Substituting $A$ into the body wrench gives a $6 \times 10$ block per link.
Carrying it towards the base and projecting on each ancestor joint fills the regressor $Y$, with $Y\pi = M\ddot{q}_r + C\dot{q}_r + g$. $Y$ depends only on geometry and motion, never on the inertial parameters. The armature is not one of the ten link parameters.

## Complexity Analysis

| Quantity                       | Time     | Space    | Notes                                                     |
|--------------------------------|----------|----------|-----------------------------------------------------------|
| Modified inverse dynamics      | $O(n)$   | $O(n)$   | About twice an ordinary recursive Newton–Euler pass       |
| Coriolis matrix $C$            | $O(n^2)$ | $O(n^2)$ | One modified pass per column                              |
| $C^T\dot{q}$                   | $O(n^2)$ | $O(n^2)$ | Via the full matrix                                       |
| Regressor $Y$ ($n \times 10n$) | $O(n^2)$ | $O(n^2)$ | One $6 \times 10$ block per link carried to its ancestors |

## Step-by-Step Walkthrough

A planar two-link arm of uniform rods, with $h = m_2 l_1 \tfrac{l_2}{2} \sin q_2$:

1. The Christoffel matrix is $C = \begin{bmatrix} -h\dot{q}_2 & -h(\dot{q}_1 + \dot{q}_2) \\ h\dot{q}_1 & 0 \end{bmatrix}$.
2. Its product with $\dot{q}$ is $[-h(2\dot{q}_1\dot{q}_2 + \dot{q}_2^2),\ h\dot{q}_1^2]$, the familiar Coriolis and centrifugal vector.
3. The mass matrix varies only through $M_{11}$ and $M_{12}$, so $\dot{M}_{11} = -2h\dot{q}_2$ and $\dot{M}_{12} = -h\dot{q}_2$. Then $N = \dot{M} - 2C$ has $N_{12} = h(2\dot{q}_1 + \dot{q}_2) = -N_{21}$ and zero diagonal: skew-symmetric, as required.
4. For a spatial three-link arm the column-by-column construction matches finite-difference Christoffel symbols of the composite-rigid-body mass matrix to about $10^{-5}$ in single precision.

## Pitfalls & Edge Cases

- **Wrong factorization**: reconstructing $C$ from the vector $C\dot{q}$, or using a convenient factorization from symbolic tools, generally breaks the skew-symmetry, and with it the stability proofs of passivity-based controllers and observers.
- **Parameter frame**: the regressor's rotational inertia is about the **joint origin**, not the centre of mass. Mixing the two silently corrupts identification.
- **Identifiability**: the full $10n$-parameter vector is not identifiable from joint torques. Some parameters never appear and others only in combinations. Reduce to base parameters before estimation.
- **Armature and friction**: neither is among the ten link parameters. Extend the parameter vector with per-joint armature and friction terms when identifying real arms.
- **Cost of the full matrix**: forming $C$ is $O(n^2)$. When only $C\dot{q}_r$ is needed, call the modified recursion directly.

## Variants & Generalizations

- **Base parameters**: QR decomposition with column pivoting of a stacked regressor yields a minimal identifiable set.
- **Floating-base regressors**: the same construction applied to a free-floating base, for legged robots and humanoids.
- **Energy regressors**: the kinetic plus potential energy is linear in the same parameters, giving filtered identification without accelerations.
- **Symbolic Coriolis matrices**: closed forms for specific arms, which are exact but must be checked for the Christoffel property.

## Applications

- Slotine–Li adaptive control, which needs $C(q,\dot{q})\dot{q}_r$ and $Y$.
- Generalized-momentum collision detection, which needs $C^T\dot{q}$.
- Offline and recursive identification of inertial parameters from measured torques.
- Passivity analysis and energy-based controller design.

## Connections to Other Algorithms

- [Recursive Newton-Euler](RecursiveNewtonEuler.md): the special case $\dot{q}_r = \dot{q}$, and the cross-check for $C\dot{q}$.
- [Composite Rigid Body Algorithm](CompositeRigidBodyAlgorithm.md): its mass matrix defines the Christoffel symbols this construction reproduces, and its spatial inertia is shared.
- [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md): uses the same spatial algebra and frame transforms.
- [Chain Dynamics Model](ChainDynamicsModel.md): supplies $M$ and $g$ beside these Coriolis terms.

## References & Further Reading

- Echeandia, S. and Wensing, P.M. (2021). "Numerical methods to compute the Coriolis matrix and Christoffel symbols for rigid-body systems." *ASME Journal of Computational and Nonlinear Dynamics*, 16(9).
- Niemeyer, G. and Slotine, J.-J.E. (1991). "Performance in adaptive manipulator control." *International Journal of Robotics Research*, 10(2), 149–161.
- Atkeson, C.G., An, C.H. and Hollerbach, J.M. (1986). "Estimation of inertial parameters of manipulator loads and links." *International Journal of Robotics Research*, 5(3), 101–119.
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapters 2 and 5.
