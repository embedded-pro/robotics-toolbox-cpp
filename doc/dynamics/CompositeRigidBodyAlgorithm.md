# Composite Rigid Body Algorithm (CRBA)

## Overview & Motivation

The Composite Rigid Body Algorithm builds the **joint-space mass matrix** $M(q)$ of a serial chain, the symmetric positive-definite matrix in

$$M(q)\,\ddot{q} + C(q, \dot{q})\,\dot{q} + g(q) = \tau$$

that says how joint accelerations turn into joint torques. The [Recursive Newton-Euler Algorithm](RecursiveNewtonEuler.md) and the [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md) never form $M$, but several controllers and estimators need the matrix itself:

- operational-space control reflects it into task space;
- momentum-based collision observers integrate $M(q)\dot{q}$;
- forward-dynamics schemes factor it.

$M$ can also be built column by column with inverse dynamics, at $n$ calls of $O(n)$ each. That is also $O(n^2)$, but with a much larger constant. CRBA builds it directly, in $O(n^2)$ with a small constant (Walker and Orin, 1982).

## Mathematical Theory

### Spatial Inertia

Each link $i$ has a mass $m_i$, centre of mass $c_i$ and rotational inertia $\bar{I}_i$ about the centre of mass, all in its own frame. About the joint origin, acting on the spatial velocity $(\omega; v)$, its rigid-body inertia is the $6 \times 6$ block matrix

$$I_i = \begin{bmatrix} \bar{I}_i + m_i [c_i]_\times^T [c_i]_\times & m_i [c_i]_\times \\ m_i [c_i]_\times^T & m_i \mathbf{1}_3 \end{bmatrix}$$

so that $I_i (\omega; v) = (n; f)$ is the momentum (angular about the origin, then linear). The upper-left block is the parallel-axis theorem.

### Composite Inertias

The **composite body** $i$ is link $i$ together with every link outboard of it, frozen in its current configuration. Its inertia is accumulated from tip to base:

$$I^c_{n-1} = I_{n-1}, \qquad I^c_{i-1} = I_{i-1} + X^*_i\, I^c_i\, X_i$$

where $X^*_i I X_i$ re-expresses an inertia in the parent frame. With $R_i = R(\hat{z}_i, q_i)$ and the joint offset $r_i$, the blocks transform as

$$\begin{aligned}
A' &= R A R^T, \quad B' = R B R^T, \quad L' = R L R^T \\
I_{\text{parent}} &= \begin{bmatrix} A' - B'[r]_\times + [r]_\times B'^T - [r]_\times L' [r]_\times & B' + [r]_\times L' \\ \cdot & L' \end{bmatrix}
\end{aligned}$$

### Mass-Matrix Columns

A unit acceleration of joint $i$ with all other joints locked accelerates only composite body $i$, rigidly, about axis $\hat{z}_i$. From rest, the wrench it needs is

$$(n_i; f_i) = I^c_i\,(\hat{z}_i; 0)$$

The diagonal entry is the projection onto the joint's own axis, $M_{ii} = \hat{z}_i^T n_i$. The same wrench, carried towards the base through each joint,

$$f \leftarrow R_j f, \qquad n \leftarrow R_j n + r_j \times f$$

projected on each ancestor axis gives $M_{ji} = M_{ij} = \hat{z}_j^T n$ for $j < i$. Each column costs one projection per ancestor, which gives the triangular operation count.

### Properties

- $M$ is symmetric, and positive definite whenever every composite body has non-zero inertia about its joint axis.
- $M$ does not depend on the first joint angle when the base is fixed and gravity is ignored: rotating the whole chain about $\hat{z}_0$ is a symmetry of the kinetic energy.
- $\tfrac12 \dot{q}^T M \dot{q}$ equals the sum of the links' kinetic energies $\tfrac12(m_i \|v_{c,i}\|^2 + \omega_i^T \bar{I}_i \omega_i)$.

## Complexity Analysis

| Phase              | Time     | Space    | Notes                                                                   |
|--------------------|----------|----------|-------------------------------------------------------------------------|
| Rotations          | $O(n)$   | $O(n)$   | One axis–angle rotation per joint                                       |
| Composite inertias | $O(n)$   | $O(n)$   | One inertia transform (a few $3 \times 3$ products) per joint           |
| Columns            | $O(n^2)$ | $O(n^2)$ | $n(n-1)/2$ wrench transforms, $O(1)$ each, plus the $n \times n$ result |

For short chains ($n \le 6$) the constant matters more than the order: CRBA's per-entry work is one $3 \times 3$ rotation and a cross product, far below a full inverse-dynamics sweep per column.

## Step-by-Step Walkthrough

Two uniform rods rotating about parallel $\hat{z}$ axes: lengths $l_1 = l_2 = 1$ m, masses $m_1 = m_2 = 1$ kg, $q_2 = -0.5$ rad.

1. Each rod has $\bar{I} = \mathrm{diag}(0, \tfrac{1}{12}, \tfrac{1}{12})$ and $c = (\tfrac12, 0, 0)$. Its inertia about the joint has $z$-entry $\tfrac{1}{12} + \tfrac14 = \tfrac13$.
2. Composite body 2 is rod 2 alone: $M_{22} = \tfrac13$.
3. Composite body 1 adds rod 2, expressed in frame 1 and moved by $r_2 = (1, 0, 0)$. Its $z$-inertia about joint 1 is $\tfrac13 + \tfrac13 + 1 + \cos(0.5)$, so $M_{11} = \tfrac53 + \cos 0.5 = 2.544$.
4. Column 2 carried to joint 1: the wrench $n = \tfrac13\hat{z}$, $f = \tfrac12(-\sin q_2, \cos q_2, 0)$ in frame 1 gains the moment $r_2 \times f$, whose $z$-part is $\tfrac12\cos q_2$. So $M_{12} = \tfrac13 + \tfrac12\cos 0.5 = 0.772$.

These are the textbook closed-form entries of the planar two-link arm.

## Pitfalls & Edge Cases

- **Inertia frame**: the link tensor must be about the centre of mass, in the link frame. Supplying it about the joint origin double-counts the parallel-axis term.
- **Exact symmetry**: computing both triangles independently leaves round-off asymmetry that can upset a Cholesky factorization. Write each off-diagonal value once into both positions.
- **Singular mass matrix**: a composite body with zero inertia about its axis (a point mass on the axis, or a massless distal chain) makes $M$ singular. Forward dynamics through $M^{-1}$ then fails, so add reflected rotor inertia (armature) on the diagonal for real actuators.
- **Conditioning**: light distal links next to heavy proximal ones give condition numbers of $10^3$–$10^5$. Solve $M\ddot{q} = b$ with a Cholesky factorization, never an explicit inverse.
- **Non-unit axes**: the rotation formula assumes unit joint axes, and the projections scale with the axis norm.

## Variants & Generalizations

- **Armature and rotor inertia**: add $k_r^2 I_{\text{rotor}}$ to each diagonal entry.
- **Prismatic joints**: the unit motion is $(0; \hat{z}_i)$, so the column wrench becomes $I^c_i (0; \hat{z}_i)$ and the projection uses the force part.
- **Floating base**: the base becomes a 6-DOF joint and $M$ gains a $6 \times 6$ block and coupling rows (Featherstone, Ch. 9).
- **Branched trees**: the ancestor loop follows the parent array instead of $j - 1$. Sparsity can then be exploited in the factorization.
- **Column-by-column RNEA**: $M e_j = \mathrm{ID}(q, 0, e_j)$ with gravity off. Simpler to derive and useful as a test oracle, but $n$ times the cost of one inverse-dynamics pass.

## Applications

- Operational-space control and task-space inertia shaping, $\Lambda = (J M^{-1} J^T)^{-1}$.
- Generalized-momentum observers for collision detection, which need $M(q)\dot{q}$.
- Forward dynamics by factorization for simulation and model-in-the-loop tests of short chains.
- Inertia-aware trajectory scaling and kinetic-energy limits for collaborative robots.

## Connections to Other Algorithms

- [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md): shares the spatial inertia and its parent-frame transform. ABA "downdates" the inertia at each joint, while CRBA keeps the full composite.
- [Recursive Newton-Euler](RecursiveNewtonEuler.md): with zero velocity and gravity, inverse dynamics of a unit acceleration is one column of $M$, the standard cross-check.
- [Euler-Lagrange](EulerLagrange.md): $M(q)$ is the inertia term of the Lagrangian equations of motion.
- The Cholesky factorization used to solve with $M$ comes from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Walker, M.W. and Orin, D.E. (1982). "Efficient Dynamic Computer Simulation of Robotic Mechanisms." *ASME J. Dynamic Systems, Measurement, and Control*, 104(3), 205–211.
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapter 6.
- Siciliano, B. et al. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 7.
