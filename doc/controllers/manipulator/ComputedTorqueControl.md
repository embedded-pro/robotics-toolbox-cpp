# Computed-Torque Control

## Overview & Motivation

A manipulator's joints are coupled. Accelerating one joint loads the others through the mass matrix; velocities create Coriolis and centrifugal torques; gravity pulls on every joint differently in every pose. Independent PD loops per joint fight these couplings and need high gains to track a trajectory.

**Computed-torque control** uses the dynamic model to cancel the couplings exactly. It commands the torque that makes the arm behave like a set of independent, unit-mass double integrators. A linear PD-plus-feedforward law then shapes each joint's tracking error.
It is the textbook example of **feedback linearization** in robotics, and the reference against which model-based joint controllers are compared.

## Mathematical Theory

### Control Law

For a desired trajectory $(q_d, \dot{q}_d, \ddot{q}_d)$ with errors $e = q_d - q$ and $\dot{e} = \dot{q}_d - \dot{q}$, the outer loop commands a joint acceleration

$$a_q = \ddot{q}_d + K_d\,\dot{e} + K_p\,e$$

and the inner loop computes the torque that produces it at the measured state:

$$\tau = M(q)\,a_q + C(q, \dot{q})\,\dot{q} + g(q)$$

### Closed-Loop Error Dynamics

Substituting into the manipulator equation $M\ddot{q} + C\dot{q} + g = \tau$ gives $M(q)(\ddot{q} - a_q) = 0$. Since $M$ is positive definite, $\ddot{q} = a_q$, and therefore

$$\ddot{e} + K_d\,\dot{e} + K_p\,e = 0$$

With diagonal gains every joint is an independent, linear, second-order system. Choosing $K_p = \omega_n^2$ and $K_d = 2\zeta\omega_n$ per joint sets its natural frequency and damping, and $\zeta = 1$ ($K_d = 2\sqrt{K_p}$) gives critical damping. The error converges exponentially for any symmetric positive-definite gains.

### One-Pass Evaluation

The torque is exactly inverse dynamics evaluated at $(q, \dot{q}, a_q)$. The recursive Newton–Euler algorithm returns $M a_q + C\dot{q} + g$ in a single $O(n)$ pass, without forming, multiplying or inverting the mass matrix. That keeps the controller linear-time and numerically benign.

### Model Error

With an imperfect model $\hat{M}, \hat{h}$, the closed loop becomes $\ddot{e} + K_d\dot{e} + K_p e = M^{-1}\left[(M - \hat{M})a_q + (h - \hat{h})\right]$. The error no longer vanishes but stays bounded for bounded model error, and higher gains shrink it at the price of bandwidth.

## Complexity Analysis

| Part                    | Time            | Space    | Notes                                                         |
|-------------------------|-----------------|----------|---------------------------------------------------------------|
| Outer loop              | $O(n^2)$        | $O(n^2)$ | Two gain products; $O(n)$ when the gains are diagonal         |
| Inverse dynamics        | $O(n)$          | $O(n)$   | One recursive Newton–Euler pass at the commanded acceleration |
| Total per control cycle | $O(n)$–$O(n^2)$ | $O(n^2)$ | No mass matrix is built or inverted                           |

## Step-by-Step Walkthrough

A two-joint arm with $K_p = \mathrm{diag}(100, 100)$ and $K_d = \mathrm{diag}(20, 20)$, so each joint is critically damped at $\omega_n = 10$ rad/s:

1. Errors $e = (0.1, -0.2)$ rad, $\dot{e} = 0$, $\ddot{q}_d = 0$ give the commanded acceleration $a_q = K_p e = (10, -20)$ rad/s².
2. Inverse dynamics at the measured $(q, \dot{q})$ with $\ddot{q} = a_q$ returns the torque that makes the arm accelerate exactly at $a_q$, including the coupling, velocity and gravity terms.
3. With an exact model, each joint's error then follows $e(t) = (e_0 + (\dot{e}_0 + 10 e_0)t)\,e^{-10t}$. Starting at rest with $e_0 = 0.3$ rad, the error is about $1.5 \times 10^{-4}$ rad after one second, with no overshoot.
4. Adding velocity and acceleration feedforward makes the same law track a moving trajectory with the same error dynamics.

## Pitfalls & Edge Cases

- **Model fidelity**: cancellation is only as good as the model. Payload, friction and unmodelled actuator dynamics appear as disturbances; add integral action or adaptation, or update the link parameters when the payload changes.
- **Actuator saturation**: aggressive gains or trajectories demand torques the motors cannot deliver. Once saturated, the cancellation and the linear error dynamics no longer hold. Plan trajectories within the torque limits.
- **Sample rate**: the continuous-time analysis assumes the loop runs much faster than the closed-loop bandwidth. A digital loop with delay tolerates lower gains than the ideal $K_p$, $K_d$.
- **Velocity noise**: $K_d$ and the Coriolis term both use the measured $\dot{q}$. Filter it or use an observer.
- **Measured versus desired state**: the inverse dynamics must use the **measured** $q, \dot{q}$. Evaluating them at the desired state gives the cheaper "feedforward" variant, which no longer linearizes the loop.

## Variants & Generalizations

- **PD plus feedforward**: $\tau = \mathrm{ID}(q_d, \dot{q}_d, \ddot{q}_d) + K_p e + K_d \dot{e}$, where the model is evaluated along the trajectory. It allows precomputation but only approximately decouples the joints.
- **Computed torque with integral action**: an integral term rejects constant model error and gravity mismatch.
- **Robust computed torque**: a sliding-mode or Lyapunov-redesign term bounds the effect of model uncertainty.
- **Adaptive control (Slotine–Li)**: estimates the inertial parameters online, keeping stability without an exact model.
- **Operational-space control**: the task-space analogue, which linearizes the end-effector dynamics instead of the joints.

## Applications

- High-performance trajectory tracking on industrial and research arms with accurate dynamic models.
- Fast pick-and-place where coupling torques dominate at high speed.
- Baseline controller for evaluating adaptive, robust and learning-based schemes.
- Model validation: tracking error under computed torque reveals model error directly.

## Connections to Other Algorithms

- [Chain Dynamics Model](../../dynamics/ChainDynamicsModel.md): supplies the one-call inverse dynamics this controller consumes.
- [Recursive Newton-Euler](../../dynamics/RecursiveNewtonEuler.md): evaluates $M a_q + C\dot{q} + g$ in one $O(n)$ pass.
- [PD Control with Gravity Compensation](PdGravityCompensation.md): the set-point special case that needs only $g(q)$ and no trajectory feedforward.
- [Cubic Spline Trajectory](../../trajectory/CubicSplineTrajectory.md): provides the $C^2$ reference $(q_d, \dot{q}_d, \ddot{q}_d)$ the feedforward needs.

## References & Further Reading

- Luh, J.Y.S., Walker, M.W. and Paul, R.P.C. (1980). "Resolved-acceleration control of mechanical manipulators." *IEEE Transactions on Automatic Control*, 25(3), 468–474.
- Spong, M.W., Hutchinson, S. and Vidyasagar, M. (2020). *Robot Modeling and Control*. 2nd ed. Wiley. Chapter 9.
- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 8.
