# Generalized-Momentum Observer

## Overview & Motivation

A collaborative robot must notice a collision within milliseconds, from the signals it already has: the commanded joint torques and the measured joint positions and velocities. Joint-torque sensors are expensive, and differentiating velocities to estimate accelerations amplifies noise until the estimate is useless.

The **generalized-momentum observer** sidesteps both. It tracks the momentum $p = M(q)\dot{q}$, which changes only because of known torques and the unknown external torque. The mismatch between the measured momentum and the integral of everything the model explains converges to the external torque with a first-order lag.
The result is a **residual** per joint, an estimate of the contact torque that serves for detection, isolation of the touched link, and reaction strategies.

## Mathematical Theory

### Momentum Dynamics

With $\tau_{\text{ext}}$ the joint torque produced by contact, the manipulator dynamics are

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau + \tau_{\text{ext}}$$

Differentiating $p = M\dot{q}$ and using the Christoffel property $\dot{M} = C + C^T$ gives

$$\dot{p} = \tau + C^T(q,\dot{q})\,\dot{q} - g(q) + \tau_{\text{ext}}$$

No acceleration appears. This is why the observer needs $C^T\dot{q}$, not the Coriolis vector $C\dot{q}$ that the recursive Newton–Euler algorithm returns.

### Residual

With $\beta(q,\dot{q}) = g(q) - C^T(q,\dot{q})\dot{q}$ and a diagonal gain $K_O$, the residual is

$$r(t) = K_O \left( p(t) - p(0) - \int_0^t \left(\tau - \beta + r\right) ds \right)$$

Differentiating shows $\dot{r} = K_O(\tau_{\text{ext}} - r)$: each component is a first-order low-pass of the external torque with bandwidth $K_{O,i}$. For a step $\tau_{\text{ext}}$ applied at $t_0$,

$$r(t) = \tau_{\text{ext}} \left(1 - e^{-K_O (t - t_0)}\right)$$

### Discrete Update

At each control period $\Delta t$ the integral is advanced by forward Euler:

$$\begin{aligned}
I_{k+1} &= I_k + (\tau_k - \beta_{k+1} + r_k)\,\Delta t \\
r_{k+1} &= K_O \left( M(q_{k+1})\dot{q}_{k+1} - p_0 - I_{k+1} \right)
\end{aligned}$$

It is stable while $K_O \Delta t \ll 1$. The discretization leaves an $O(\Delta t)$ residual during collision-free motion, which grows with joint speed.

### Detection

A collision is declared when any joint exceeds its threshold, $|r_i| > \bar{r}_i$. Thresholds sit above the largest residual seen in collision-free runs, which reflects model error, friction and discretization.

## Complexity Analysis

| Operation      | Time     | Space    | Notes                                   |
|----------------|----------|----------|-----------------------------------------|
| `Update`       | $O(n^2)$ | $O(n^2)$ | Mass matrix and $C^T\dot{q}$ dominate   |
| `Reset`        | $O(n^2)$ | $O(n)$   | One mass-matrix product                 |
| `Collision`    | $O(n)$   | $O(1)$   | One comparison per joint                |
| Observer state | —        | $O(n)$   | Initial momentum, integral and residual |

## Step-by-Step Walkthrough

A planar two-link arm of uniform rods (masses $2$ and $1.5$ kg, lengths $1$ and $0.8$ m) is simulated at $\Delta t = 1$ ms with gain $K_O = 50\ \text{s}^{-1}$, so $K_O\Delta t = 0.05$.

1. **Free motion.** Gravity-compensated sinusoidal torques drive the arm for one second. The residual stays below $10^{-2}$ N·m on both joints.
2. **Step contact.** At $t = 0.2$ s a torque of $0.8$ N·m is applied to joint 2. After one time constant ($20$ ms), $r_2 \approx 0.8(1 - e^{-1}) = 0.506$ N·m. After $0.3$ s it has settled at $0.8$ N·m, and $r_1$ stays near zero.
3. **Detection.** With thresholds $(0.5, 0.5)$ N·m, the flag is clear before the contact and set once the residual settles.
4. **Why $C^T\dot{q}$.** At $\dot{q} = (3, -4)$ rad/s, $C^T\dot{q}$ and $C\dot{q}$ differ by several N·m. An observer built on $C\dot{q}$ would report that difference as a phantom contact, while the correct observer's residual stays a few percent of it.

## Pitfalls & Edge Cases

- **Wrong Coriolis term**: substituting the recursive Newton–Euler vector $C\dot{q}$ for $C^T\dot{q}$ produces a residual proportional to the velocity squared during free motion.
- **Friction**: if friction is not part of $M$, $C$ and $g$, it appears as a false external torque. Subtract a friction model from the commanded torque first.
- **Commanded versus applied torque**: motor saturation, gear losses and current-loop lag make the applied torque differ from the command. Use the measured motor current where available.
- **Gain and noise**: higher gains detect faster but pass more model error and velocity noise. Keep $K_O \Delta t \ll 1$ for the forward-Euler update.
- **Initialization**: call `Reset` with the current state before the first update. Starting from a stale momentum produces a transient that decays with time constant $1/K_O$.
- **Zero gain**: the residual is identically zero, which disables detection.

## Variants & Generalizations

- **Higher-order observers**: second-order filters on the momentum residual trade a little delay for better noise rejection.
- **Energy observer**: a scalar residual on the total energy, cheaper but unable to tell which joint was hit.
- **Collision isolation and identification**: the residual pattern locates the touched link and, with the contact Jacobian, the contact force.
- **Sliding-mode and Kalman variants**: robustness to model error, or explicit noise models, at higher cost.

## Applications

- Collision detection and safe reaction (stop, retract, comply) on collaborative arms.
- Contact-force estimation for hand guiding and kinesthetic teaching without force sensors.
- Monitoring of payload changes and model drift during operation.

## Connections to Other Algorithms

- [Coriolis Matrix and Inertial Regressor](CoriolisMatrixAndRegressor.md): supplies $C^T\dot{q}$.
- [Chain Dynamics Model](ChainDynamicsModel.md): supplies $M(q)$ and $g(q)$.
- [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md): simulates the plant in the tests.
- [Recursive Newton-Euler](RecursiveNewtonEuler.md): its external-wrench overload maps an estimated contact wrench to joint torques.

## References & Further Reading

- De Luca, A., Albu-Schäffer, A., Haddadin, S. and Hirzinger, G. (2006). "Collision detection and safe reaction with the DLR-III lightweight manipulator arm." *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 1623–1630.
- Haddadin, S., De Luca, A. and Albu-Schäffer, A. (2017). "Robot collisions: a survey on detection, isolation, and identification." *IEEE Transactions on Robotics*, 33(6), 1292–1312.
- De Luca, A. and Mattone, R. (2005). "Sensorless robot collision detection and hybrid force/motion control." *IEEE International Conference on Robotics and Automation*, 999–1004.
