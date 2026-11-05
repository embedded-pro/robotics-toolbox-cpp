# Redundancy Resolution

## Overview & Motivation

A manipulator is **redundant** for a task when it has more joints than the task has dimensions, such as a 7-joint arm positioning and orienting its tool (6 dimensions), or any 4+ joint arm doing a pure positioning task (3). The extra freedom forms a continuous family of joint motions that leave the tool still: the elbow can swing while the hand holds a cup.

Redundancy resolution uses that freedom deliberately. The joint velocity is split into a **primary** part that achieves the commanded tool velocity and a **secondary** part confined to the Jacobian's null space. The secondary part pursues another objective without disturbing the tool:

- avoiding joint limits;
- staying away from singularities;
- keeping clear of obstacles;
- minimizing effort.

This is the classic gradient-projection method of Liégeois (1977), the foundation of whole-body and task-priority control.

## Mathematical Theory

### Primary and Secondary Motion

With the task Jacobian $J \in \mathbb{R}^{m \times n}$, $n > m$:

$$\dot{q} = J^{+}_{\lambda}\,\dot{x} + \left(\mathbf{I} - J^{+}J\right)\dot{q}_0$$

- $J^{+}_{\lambda} = J^T (J J^T + \lambda^2 \mathbf{I})^{-1}$ is the **damped** pseudo-inverse, which reproduces $\dot{x}$ with the minimum joint-velocity norm. Damping trades exactness for bounded joint rates near singularities.
- $P = \mathbf{I} - J^{+}J$ is the **null-space projector**. It maps any $\dot{q}_0$ to the closest joint velocity that produces no tool motion, so $J P = 0$.

### Singular Value Form

With the SVD $J = U \Sigma V^T$ and singular values $\sigma_i$:

$$J^{+}_{\lambda} = \sum_i \frac{\sigma_i}{\sigma_i^2 + \lambda^2}\, v_i u_i^T, \qquad P = \mathbf{I} - \sum_{\sigma_i > \epsilon\,\sigma_1} v_i v_i^T$$

The damped gain $\sigma/(\sigma^2 + \lambda^2)$ never exceeds $1/(2\lambda)$, so joint rates stay bounded as $\sigma_{\min} \to 0$.
The projector is built **undamped** from the rank-thresholded singular vectors, which keeps it an exact orthogonal projector: symmetric, idempotent and invisible to the task for any $\lambda$. The damped $\mathbf{I} - J^{+}_{\lambda}J$ is not, and leaks secondary motion into the tool.

### Task Accuracy Under Damping

With $\lambda > 0$ the primary task is met only approximately:

$$\|J\dot{q} - \dot{x}\| \le \frac{\lambda^2}{\sigma_{\min}^2 + \lambda^2}\,\|\dot{x}\|$$

Far from singularities ($\sigma_{\min} \gg \lambda$) the error is negligible; near them, accuracy is traded for bounded joint rates.

### Secondary Objectives

Choosing $\dot{q}_0 = -k\,\nabla H(q)$ descends a cost $H$ inside the self-motion manifold. Since $P$ is a projector, $\nabla H^T P \nabla H \ge 0$, and the projected motion never increases $H$ to first order. Common choices:

- the negative gradient of manipulability, to avoid singularities;
- the squared normalized distance to joint centres, to avoid joint limits;
- obstacle potentials evaluated on the links.

## Complexity Analysis

| Operation            | Time                         | Space    | Notes                                                  |
|----------------------|------------------------------|----------|--------------------------------------------------------|
| Resolve              | $O(m\,n\,\min(m,n)) + O(mn)$ | $O(mn)$  | One SVD; the projector is applied without being formed |
| Pseudo-inverse       | + $O(n\,m\,r)$               | $O(nm)$  | Diagnostic accessor                                    |
| Null-space projector | + $O(n^2 r)$                 | $O(n^2)$ | Diagnostic accessor; $r$ is the numerical rank         |

## Step-by-Step Walkthrough

A 7-joint anthropomorphic arm with a full-pose task ($m = 6$):

1. At a generic configuration the six singular values are all well above the threshold, so the null space has dimension $7 - 6 = 1$: the elbow's self-motion.
2. With $\lambda = 0$, the resolved velocity reproduces the commanded tool twist exactly, and $J J^{+} = \mathbf{I}_6$.
3. With $\lambda = 0.01$ and $\sigma_{\min} \approx 0.18$, the tool-velocity error is a few $10^{-4}$, while the null-space part still produces no tool motion at all.
4. Feeding $\dot{q}_0 = -\nabla H$ for $H = \tfrac12 \sum (q_i/q_{\max})^2$ with a zero task velocity moves the elbow so that $H$ decreases, while the hand stays put.
5. For a position-only task ($m = 3$) the same arm has a 4-dimensional null space. At a stretched configuration two singular values vanish, the rank drops to 4 and the full-pose null space grows from 1 to 3 dimensions.

## Pitfalls & Edge Cases

- **Damped projector leakage**: building $P$ from $J^{+}_{\lambda}$ lets secondary motion disturb the tool. Use the undamped, rank-thresholded projector.
- **Lost directions at singularities**: once a singular value drops below the threshold, its task direction joins the null space. Secondary motion may then move the tool along that direction, by design, because the arm cannot control it anyway.
- **Gain of the secondary term**: $\dot{q}_0$ is a velocity. Over-scaled gradients saturate joint rates without affecting the task, so scale them with the control period and joint limits.
- **Not a full priority hierarchy**: a single null-space term protects one task. Several prioritized tasks need recursive projectors (Siciliano–Slotine) or hierarchical quadratic programs.
- **Drift**: integrating velocities accumulates task error. Close the loop with a task-space error term ($\dot{x} = \dot{x}_d + K e$).

## Variants & Generalizations

- **Task-priority redundancy resolution**: secondary tasks with their own Jacobians, $\dot{q} = J_1^{+}\dot{x}_1 + (J_2 P_1)^{+}(\dot{x}_2 - J_2 J_1^{+}\dot{x}_1)$.
- **Weighted pseudo-inverse**: minimizes $\dot{q}^T W \dot{q}$ instead of the plain norm, for instance with $W = M(q)$ for the dynamically consistent inverse.
- **Acceleration-level resolution**: the same structure at the acceleration level, with the bias acceleration $\dot{J}\dot{q}$, for torque-controlled arms.
- **Inequality constraints**: joint limits as hard bounds lead to quadratic programs instead of projections.

## Applications

- Joint-limit and singularity avoidance on 7-joint collaborative arms.
- Obstacle avoidance with the elbow while the tool follows a path.
- Posture optimization for stiffness, manipulability or energy.
- Mobile manipulators, where the base adds redundancy to the arm.

## Connections to Other Algorithms

- [Geometric Jacobian](GeometricJacobian.md): provides the task Jacobian whose pseudo-inverse and null space are used.
- [Manipulability Index](ManipulabilityIndex.md): its gradient is a common secondary objective, and its value warns when damping matters.
- [Inverse Kinematics](InverseKinematics.md): damped least squares is the primary term without a null-space contribution.
- [Denavit-Hartenberg Parameters](DenavitHartenberg.md): describes the redundant arms these methods are typically applied to.
- The damped pseudo-inverse and exact null-space projector come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Liégeois, A. (1977). "Automatic supervisory control of the configuration and behavior of multibody mechanisms." *IEEE Transactions on Systems, Man, and Cybernetics*, 7(12), 868–871.
- Nakamura, Y. (1991). *Advanced Robotics: Redundancy and Optimization*. Addison-Wesley.
- Siciliano, B. and Slotine, J.-J. (1991). "A general framework for managing multiple tasks in highly redundant robotic systems." *Proc. ICAR*, 1211–1216.
- Chiaverini, S. (1997). "Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators." *IEEE Transactions on Robotics and Automation*, 13(3), 398–410.
