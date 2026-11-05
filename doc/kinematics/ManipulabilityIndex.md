# Manipulability Index

## Overview & Motivation

Near a **singular configuration** a manipulator loses the ability to move its tool in some direction. A stretched arm cannot reach further outwards, and a wrist with aligned axes cannot rotate about a third one. Approaching such a pose is dangerous: Jacobian inverses blow up, joint rates spike, and small tool motions demand huge joint motions.

The **manipulability index** of Yoshikawa (1985) measures how far a configuration is from singularity. It is the volume of the **velocity ellipsoid**, the set of tool velocities reachable with joint rates of unit norm.
The singular values of the Jacobian give that ellipsoid's axes, and their ratio, the **condition number**, measures how isotropic the arm is.
Planners maximize manipulability, controllers monitor it before inverting the Jacobian, and arm designers use it to place the workspace in the dexterous region.

## Mathematical Theory

### Velocity Ellipsoid

For a task Jacobian block $B \in \mathbb{R}^{r \times n}$ with tool velocity $\dot{x} = B\dot{q}$, the image of the unit sphere $\|\dot{q}\| \le 1$ is the ellipsoid

$$\dot{x}^T (B B^T)^{-1} \dot{x} \le 1$$

whose semi-axes are the singular values $\sigma_1 \ge \sigma_2 \ge \dots$ of $B$, along the left singular vectors.

### Yoshikawa Index

$$w = \sqrt{\det(B B^T)} = \prod_i \sigma_i \quad (r \le n), \qquad w = \sqrt{\det(B^T B)} \quad (r > n)$$

With more task rows than joints, $B B^T$ has rank at most $n$ and its determinant is identically zero. $\sqrt{\det(B^T B)}$ is then the volume of the $n$-dimensional ellipsoid the arm actually spans. For a square block, $w = |\det B|$. In both cases $w = 0$ exactly at a singularity.

### Units

Linear rows have units of m/s per unit joint rate; angular rows, rad/s. A determinant over a full 6-row Jacobian mixes the two and changes with the length unit. The index is therefore computed separately for the **translational** block (rows 0–2) and the **rotational** block (rows 3–5).

### Condition Number

$$\kappa = \frac{\sigma_{\max}}{\sigma_{\min}} \in [1, \infty]$$

$\kappa = 1$ is an isotropic configuration that moves equally well in every direction; $\kappa \to \infty$ at a singularity. Unlike $w$, $\kappa$ is scale-invariant and depends only on the ellipsoid's shape. A smallest singular value below a relative threshold is reported as an infinite condition number.

## Complexity Analysis

| Quantity                 | Time                  | Space     | Notes                                             |
|--------------------------|-----------------------|-----------|---------------------------------------------------|
| Index $w$                | $O(r\,n\,m) + O(m^3)$ | $O(m^2)$  | Gram product and LU determinant, $m = \min(r, n)$ |
| Ellipsoid axes, $\kappa$ | $O(r\,n\,m)$          | $O(r\,n)$ | One Golub–Kahan SVD of the $r \times n$ block     |

The Gram/LU path is cheaper and suits a per-cycle monitor. The SVD is more accurate near singularity, since it avoids squaring the Jacobian, and also yields the axes and directions.

## Step-by-Step Walkthrough

A planar two-link arm with unit links, translational rows, elbow angle $q_2$:

1. The linear Jacobian has two independent columns spanning the plane, and $\sqrt{\det(B^T B)} = |\sin q_2|$.
2. At $q_2 = \pi/2$, $w = 1$, the maximum. The singular values are $(\varphi, 1/\varphi) = (1.618, 0.618)$, their product is $1$, and $\kappa = \varphi^2 = 2.618$.
3. At $q_2 = 0.1$, $w = 0.0998$ and $\kappa \approx 50$; at $q_2 = 0.01$, $\kappa \approx 500$. The ellipsoid flattens towards a line.
4. At $q_2 = 0$ (stretched) or $\pi$ (folded), $w = 0$ and $\kappa = \infty$: the tool cannot move along the arm.
5. A redundant three-link planar arm at $q = (0, \pi/2, \pi/2)$ has $B B^T = \begin{bmatrix} 2 & 1 \\ 1 & 2 \end{bmatrix}$, so $w = \sqrt{3}$ with axes $(\sqrt{3}, 1)$.

## Pitfalls & Edge Cases

- **Wrong Gram product**: with fewer joints than task rows, $\det(B B^T) \equiv 0$ regardless of configuration. Use $B^T B$.
- **Planar arms with 3D position rows**: the $z$ row is zero, so a redundant planar arm measured with three position rows reports $w \equiv 0$. Measure planar tasks with the planar rows only.
- **Mixed units**: never take one determinant over linear and angular rows together; the result changes with the choice of metres versus millimetres.
- **Single precision near singularity**: forming $B B^T$ squares the condition number. The determinant reaches the singular threshold once $\sigma_{\min}/\sigma_{\max} \lesssim 10^{-3}$, while the singular values stay accurate below that.
- **Not a distance**: $w$ is not linear in the distance to singularity, and its scale depends on link lengths. Compare values only within one arm, or normalize by the maximum.

## Variants & Generalizations

- **Force manipulability**: the force ellipsoid is the dual of the velocity ellipsoid, with axes $1/\sigma_i$. Directions that move fast resist force poorly.
- **Dynamic manipulability**: weights by the inverse mass matrix, $J M^{-1}$, to measure achievable accelerations under torque limits (Yoshikawa 1985b).
- **Inverse condition number and local conditioning index**: $1/\kappa \in [0, 1]$, often preferred as a bounded objective.
- **Global conditioning index**: the average of $1/\kappa$ over the workspace, a design criterion.
- **Manipulability gradients**: for null-space optimization of redundant arms, $\nabla_q w$ moves the arm away from singularities.

## Applications

- Singularity monitoring and switching to damped inverses before a Jacobian is inverted.
- Secondary objective for redundant arms: maximizing $w$ in the null space keeps them dexterous.
- Workspace and design analysis: placing tasks and choosing link lengths for high average dexterity.
- Teleoperation feedback that warns the operator before a singular pose.

## Connections to Other Algorithms

- [Geometric Jacobian](GeometricJacobian.md): the source of the task Jacobian blocks measured here.
- [Inverse Kinematics](InverseKinematics.md): damped least squares is the remedy once manipulability drops.
- [Chain Pose Kinematics](ChainPoseKinematics.md) and [Denavit-Hartenberg Parameters](DenavitHartenberg.md): produce the frame chains behind the Jacobian.
- The singular value and LU decompositions come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Yoshikawa, T. (1985). "Manipulability of robotic mechanisms." *International Journal of Robotics Research*, 4(2), 3–9.
- Salisbury, J.K. and Craig, J.J. (1982). "Articulated hands: force control and kinematic issues." *International Journal of Robotics Research*, 1(1), 4–17.
- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 3.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapter 5.
