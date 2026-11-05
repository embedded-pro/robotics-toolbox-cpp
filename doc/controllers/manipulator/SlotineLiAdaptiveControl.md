# Slotine–Li Adaptive Control

## Overview & Motivation

Model-based controllers such as computed torque cancel the arm's dynamics with a model. When the masses and inertias are wrong (an unknown payload, a tool change, worn cables), the cancellation is imperfect and tracking errors appear.

The **Slotine–Li adaptive controller** (1987) removes the need for accurate parameters. It uses the fact that the rigid-body dynamics are linear in the inertial parameters, and it adjusts a parameter estimate online, driven by the tracking error. The tracking error converges to zero **globally** for any initial estimate, while the parameters stay bounded.
It needs no joint-acceleration measurement and no inversion of the estimated mass matrix. It is the classic example of passivity-based adaptive control.

## Mathematical Theory

### Sliding Variable and Reference Motion

For a desired trajectory $q_d(t)$ with errors $\tilde{q} = q - q_d$ and $\dot{\tilde{q}} = \dot{q} - \dot{q}_d$, define

$$s = \dot{\tilde{q}} + \Lambda\tilde{q}, \qquad \dot{q}_r = \dot{q}_d - \Lambda\tilde{q}, \qquad \ddot{q}_r = \ddot{q}_d - \Lambda\dot{\tilde{q}}$$

so that $s = \dot{q} - \dot{q}_r$. With $\Lambda$ positive definite, $s \to 0$ implies $\tilde{q} \to 0$ exponentially. The reference acceleration $\ddot{q}_r$ is computed from velocities, so no acceleration is ever measured.

### Linear Parametrization

With $C$ in Christoffel form, the dynamics evaluated on the reference motion are linear in the parameter vector $a$:

$$M(q)\ddot{q}_r + C(q,\dot{q})\dot{q}_r + g(q) = Y(q, \dot{q}, \dot{q}_r, \ddot{q}_r)\,a$$

### Control and Adaptation Laws

$$\tau = Y\hat{a} - K_d\,s, \qquad \dot{\hat{a}} = -\Gamma\,Y^T s$$

The update is integrated with forward Euler at the control period $\Delta t$.

### Stability

With the estimation error $\tilde{a} = \hat{a} - a$, the closed loop is $M\dot{s} + Cs + K_d s = Y\tilde{a}$. The Lyapunov function

$$V = \tfrac12 s^T M s + \tfrac12 \tilde{a}^T\Gamma^{-1}\tilde{a}$$

has, using the skew-symmetry of $\dot{M} - 2C$,

$$\dot{V} = -s^T K_d s \le 0$$

So $s$ and $\tilde{a}$ stay bounded, and Barbalat's lemma gives $s \to 0$ and therefore $\tilde{q} \to 0$. The parameters converge to their true values only under persistent excitation. The proof depends on the Christoffel factorization: any other $C$ breaks the skew-symmetry.

## Complexity Analysis

| Step                 | Time                 | Space          | Notes                                                     |
|----------------------|----------------------|----------------|-----------------------------------------------------------|
| Errors and reference | $O(n^2)$             | $O(n)$         | Products with $\Lambda$                                   |
| Regressor            | $O(n \cdot p)$       | $O(n \cdot p)$ | Supplied by the injected regressor, $p = 10n$ for a chain |
| Torque and update    | $O(n \cdot p + p^2)$ | $O(p)$         | $Y\hat{a}$, $Y^T s$ and the $\Gamma$ product              |

## Step-by-Step Walkthrough

A horizontal planar two-link arm has three base parameters $a = (a_1, a_2, a_3)$, with $M = \begin{bmatrix} a_1 + 2a_2 c_2 & a_3 + a_2 c_2 \\ a_3 + a_2 c_2 & a_3 \end{bmatrix}$. The true values are $a = (3, 0.5, 0.6)$, and the controller starts from $\hat{a} = 0$ with $\Lambda = 5I$, $K_d = 20I$, $\Gamma = I$ and $\Delta t = 1$ ms.

1. **Errors**: at $\tilde{q} = (0.05, -0.1)$ and $\dot{\tilde{q}} = (0.1, -0.2)$, the sliding variable is $s = (0.35, -0.7)$. The reference velocity is $\dot{q}_d - \Lambda\tilde{q}$.
2. **First step**: with $\hat{a} = 0$ the torque is pure damping, $-K_d s$. The estimate moves by $-\Gamma Y^T s\,\Delta t$.
3. **Tracking**: following $q_d = (0.5\sin t, 0.3\cos 2t)$ from an initial offset of $(0.2, -0.1)$ rad, the tracking error falls below $10^{-2}$ rad within 10 s.
4. **Parameters**: the estimate of $a_1$ is still more than $0.1$ away from its true value. Tracking converges without parameter convergence.

## Pitfalls & Edge Cases

- **Wrong Coriolis factorization**: a regressor built from an arbitrary $C$ satisfying $C\dot{q}$ invalidates the proof. Use the Christoffel-consistent regressor.
- **Parameter drift**: measurement noise and unmodelled dynamics make $\hat{a}$ wander. Add projection onto a physically valid set, a dead zone on $s$, or $\sigma$-modification.
- **Large adaptation gain**: a high $\Gamma$ makes the estimate oscillate and can excite unmodelled flexible modes.
- **No persistent excitation**: the estimate settles on a manifold of parameters consistent with the data. It does not identify the true values.
- **Real-time cost**: the regressor is $n \times 10n$. Reduce to base parameters on embedded targets.

## Variants & Generalizations

- **Composite adaptation**: combine the tracking-error update with a prediction-error update for faster parameter convergence.
- **Robust adaptive control**: projection, dead zones and leakage terms for bounded disturbances.
- **Task-space Slotine–Li**: the same construction with a task-space sliding variable.
- **Physically consistent adaptation**: geodesic or Bregman-divergence updates that keep the estimate physical.

## Applications

- Tracking with unknown or changing payloads.
- Manipulators whose CAD inertial parameters are unreliable.
- Online compensation of slow dynamic changes such as cable drag.

## Connections to Other Algorithms

- [Computed-Torque Control](ComputedTorqueControl.md): the non-adaptive counterpart with a fixed model.
- [Coriolis Matrix and Inertial Regressor](../../dynamics/CoriolisMatrixAndRegressor.md): supplies the Christoffel-consistent regressor $Y$.
- [Dynamic Parameter Identification](../../dynamics/DynamicParameterIdentification.md): offline estimation of the same parameters, a good initial $\hat{a}$.
- [PD Control with Gravity Compensation](PdGravityCompensation.md): another passivity-based law, for set-points.

## References & Further Reading

- Slotine, J.-J.E. and Li, W. (1987). "On the adaptive control of robot manipulators." *International Journal of Robotics Research*, 6(3), 49–59.
- Slotine, J.-J.E. and Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall. Chapter 9.
- Spong, M.W., Hutchinson, S. and Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley. Chapter 8.
- Ortega, R. and Spong, M.W. (1989). "Adaptive motion control of rigid robots: a tutorial." *Automatica*, 25(6), 877–888.
