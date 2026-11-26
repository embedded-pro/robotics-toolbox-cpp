# Effective Joint Inertia

## Overview & Motivation

How heavy does a joint feel? The answer depends on what the other joints do.

- With every other joint **locked**, joint $i$ moves its own subtree as one rigid body. The inertia it feels is the diagonal entry $M_{ii}$ of the mass matrix.
- With every other joint **free** (unactuated or back-drivable), part of a torque on joint $i$ is spent accelerating the neighbours through the coupling terms. The joint then feels a smaller inertia.

The second value sizes a joint's controller when the other joints are not holding it. A gain chosen from $M_{ii}$ is too stiff for a joint that, in practice, drags light neighbours along. It is also what a single-joint step test measures on a real arm, and what a simulated plant should report as each joint's load.

## Mathematical Theory

### Definition

The joint-space dynamics are $M(q)\,\ddot{q} = \tau - h(q, \dot{q})$. Apply a torque $\tau_i$ on joint $i$ alone, with every other joint torque zero. The joint accelerations are $\ddot{q} = M^{-1} e_i\, \tau_i$, so joint $i$ accelerates by

$$\ddot{q}_i = \left(M^{-1}\right)_{ii} \tau_i$$

The effective inertia of joint $i$ is the ratio of torque to that acceleration:

$$I_{\text{eff},i} = \frac{1}{\left(M^{-1}\right)_{ii}}$$

### Bounds

For a symmetric positive-definite $M$,

$$0 < I_{\text{eff},i} \le M_{ii}$$

with equality exactly when joint $i$ is decoupled from the others (row $i$ of $M$ is zero off the diagonal). The gap is the inertia the coupled joints absorb.

### Computation Through the Cholesky Factor

Inverting $M$ is unnecessary. With the Cholesky factor $M = L L^T$,

$$\left(M^{-1}\right)_{ii} = e_i^T L^{-T} L^{-1} e_i = \left\lVert L^{-1} e_i \right\rVert^2$$

so each joint needs one forward substitution $L\,y = e_i$ and the squared norm of $y$. A mass matrix that has no Cholesky factor is not positive definite, which no physical chain produces; the computation then reports failure.

## Complexity Analysis

| Step                 | Time     | Space    | Notes                                     |
|----------------------|----------|----------|-------------------------------------------|
| Cholesky factor      | $O(n^3)$ | $O(n^2)$ | Once per mass matrix                      |
| Forward substitution | $O(n^2)$ | $O(n)$   | One per joint                             |
| Total                | $O(n^3)$ | $O(n^2)$ | $n = 6$: one factor and six substitutions |

## Step-by-Step Walkthrough

Two coupled joints with

$$M = \begin{bmatrix} 2 & 1 \\ 1 & 1 \end{bmatrix}, \qquad L = \begin{bmatrix} \sqrt{2} & 0 \\ 1/\sqrt{2} & 1/\sqrt{2} \end{bmatrix}$$

1. Joint 1: $L\,y = e_1$ gives $y = (1/\sqrt{2},\ -1/\sqrt{2})$, $\lVert y \rVert^2 = 1$, so $I_{\text{eff},1} = 1$.
2. Joint 2: $L\,y = e_2$ gives $y = (0,\ \sqrt{2})$, $\lVert y \rVert^2 = 2$, so $I_{\text{eff},2} = 0.5$.

Joint 1 feels half its locked inertia $M_{11} = 2$, and joint 2 half of $M_{22} = 1$: each spends half of a torque accelerating the other.

## Pitfalls & Edge Cases

- **Configuration dependence**: $M$ depends on $q$, so the effective inertia does too. Gains sized at one pose may not suit another; recompute where the arm moves far.
- **Armature**: a geared joint's reflected rotor inertia belongs on the mass-matrix diagonal before the computation. Omitting it understates the effective inertia of the proximal joints badly.
- **Locked versus free is a modelling choice**: a joint held by a stiff controller behaves as locked for its neighbours. Use $M_{ii}$ when the other joints are servoed tightly, $I_{\text{eff},i}$ when they are free or soft.
- **Near-singular mass matrices**: a very light distal link makes $M$ ill-conditioned. The forward substitution stays stable, but the result inherits the model's uncertainty in that link.

## Variants & Generalizations

- **Partially locked chains**: lock a subset $\mathcal{L}$ and free the rest. The effective inertia of the free joints is the Schur complement of the locked block; of a single joint, $1 / (M_{\mathcal{F}\mathcal{F}}^{-1})_{ii}$ over the free set $\mathcal{F}$.
- **Operational-space inertia**: the same idea at the tool, $\Lambda = (J M^{-1} J^T)^{-1}$, gives the inertia felt along task directions.

## Applications

- Sizing per-joint controller gains and impedance stiffnesses from a bandwidth.
- Reporting each joint's load inertia in a simulated plant or a drive configuration tool.
- Predicting the result of a single-joint step or frequency-response test.

## Connections to Other Algorithms

- [Composite Rigid Body Algorithm](CompositeRigidBodyAlgorithm.md): computes the mass matrix this starts from.
- [Generic Joint Link](JointLink.md): carries the armature that belongs on the diagonal.
- The Cholesky factor and the triangular solve come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- O. Khatib, "Inertial Properties in Robotic Manipulation: An Object-Level Framework," *International Journal of Robotics Research*, 14(1), 1995.
- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008, chapter 6.
