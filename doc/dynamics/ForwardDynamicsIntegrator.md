# Forward-Dynamics Integration

## Overview & Motivation

A controller should meet a simulated plant before it meets hardware. Simulating a manipulator means advancing its state $(q, \dot{q})$ in time under applied joint torques $\tau$:

$$\ddot{q} = \mathrm{FD}(q, \dot{q}, \tau), \qquad \frac{d}{dt}\begin{bmatrix} q \\ \dot{q} \end{bmatrix} = \begin{bmatrix} \dot{q} \\ \mathrm{FD}(q, \dot{q}, \tau) \end{bmatrix}$$

The [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md) evaluates $\mathrm{FD}$ in $O(n)$. What remains is a **time-stepping rule**. The choice matters more than it looks. A naive explicit Euler step pumps energy into an undamped pendulum until it spins over the top. A good rule keeps the simulated energy honest over long runs, or reaches high accuracy with few steps.

This component pairs the articulated-body dynamics with two fixed-step integrators. **Semi-implicit (symplectic) Euler** is cheap and keeps the energy error bounded. **Classical fourth-order Runge–Kutta (RK4)** is accurate per step. Both are deterministic and allocation-free, so the same step function serves unit tests, host simulators and hardware-in-the-loop rigs.

## Mathematical Theory

### Zero-Order Hold

Digital controllers update $\tau$ once per sample period $h$ and hold it in between. The integrator therefore treats $\tau$ as constant over each step. The system is time-invariant, so the step only depends on the state and the held input.

### Semi-Implicit Euler

$$\dot{q}_{k+1} = \dot{q}_k + h\,\mathrm{FD}(q_k, \dot{q}_k, \tau_k), \qquad q_{k+1} = q_k + h\,\dot{q}_{k+1}$$

The velocity is updated first and the **new** velocity moves the position. For separable mechanical systems (velocity-independent forces) the map is **symplectic**: it preserves phase-space area. By backward error analysis, it exactly conserves a modified energy $\tilde{H} = H + O(h)$. The true energy error therefore oscillates within an $O(h)$ band for exponentially long times instead of drifting (Hairer, Lubich, Wanner). The method is first-order accurate and costs one dynamics evaluation per step.

Velocity-dependent terms of a manipulator (Coriolis, centrifugal) break exact separability. In practice the energy error stays bounded for the step sizes used in control simulation.

### Runge–Kutta 4

On the first-order state $x = (q; \dot{q})$ with $f(x) = (\dot{q}; \mathrm{FD}(q, \dot{q}, \tau))$:

$$\begin{aligned}
k_1 &= f(x_k), & k_2 &= f\!\left(x_k + \tfrac{h}{2}k_1\right), & k_3 &= f\!\left(x_k + \tfrac{h}{2}k_2\right), & k_4 &= f(x_k + h\,k_3) \\
x_{k+1} &= x_k + \tfrac{h}{6}\left(k_1 + 2k_2 + 2k_3 + k_4\right)
\end{aligned}$$

The local error is $O(h^5)$ and the global error $O(h^4)$, at four dynamics evaluations per step. RK4 is not symplectic, so energy slowly drifts (usually dissipates) over long horizons, but at fine steps the drift is far below the float resolution of a typical run.

### Logged Acceleration

Each step also reports $\ddot{q}_k = \mathrm{FD}(q_k, \dot{q}_k, \tau_k)$ at the start of the step. This is useful for logging, for cross-checking with inverse dynamics, and for acceleration feed-forward in model-in-the-loop tests.

## Complexity Analysis

| Method              | Dynamics calls / step | Time / step | Space    | Accuracy (global) |
|---------------------|-----------------------|-------------|----------|-------------------|
| Semi-implicit Euler | 1 (+1 logged)         | $O(n)$      | $O(n)$   | $O(h)$, bounded energy error |
| Runge–Kutta 4       | 4 (+1 logged)         | $O(n)$      | $O(n)$   | $O(h^4)$, slow energy drift  |

The logged acceleration costs one extra evaluation. Where cycles matter it can be dropped, or taken from the integrator's first stage.

## Step-by-Step Walkthrough

A uniform rod ($m = 1$ kg, $L = 1$ m) pivots about a horizontal axis and hangs straight down. Its moment of inertia about the pivot is $I = mL^2/3$ and its centre of mass is $L/2$ below the pivot, so small oscillations have

$$\omega_0 = \sqrt{\frac{m g L/2}{m L^2/3}} = \sqrt{\frac{3g}{2L}} = 3.836\ \text{rad/s}, \qquad T = \frac{2\pi}{\omega_0} = 1.638\ \text{s}$$

1. Release it from rest at $0.05$ rad off vertical with $h = 1$ ms.
2. RK4 evaluates the articulated-body dynamics four times per step. The measured period matches $1.638$ s to within $0.02\,\%$, which is the finite-amplitude correction $\theta_0^2/16$. After 10 s (about six periods) the relative energy error is near $10^{-7}$.
3. The same experiment with semi-implicit Euler has an energy error that oscillates around zero at the oscillation frequency and never grows.
4. For a rod spinning about a vertical axis under constant torque $\tau$ (gravity is parallel to the axis and does no work), the angle follows $q(t) = \tfrac12(\tau/I)t^2$ exactly, since RK4 integrates polynomials up to degree four without error.

## Pitfalls & Edge Cases

- **Step size versus stiffness**: explicit methods are stable only while $h\,\omega_{\max}$ stays small, where $\omega_{\max}$ is the highest natural frequency, including stiff contact springs and high-gain controllers in the loop. Semi-implicit Euler needs $h\,\omega_{\max} < 2$; RK4 tolerates about $2.8$.
- **Damping belongs in the torque**: joint friction and viscous damping should enter as torques ($\tau - b\dot{q}$), not as ad-hoc velocity scaling after the step, which breaks the integrator's structure.
- **Controller sample rate**: when the plant step is finer than the controller period, hold $\tau$ across several plant steps. Mixing the two rates silently changes the closed-loop dynamics.
- **Long horizons in float**: angles grow without bound under continuous rotation, and float spacing grows with magnitude. Wrap revolute angles for long simulations.
- **Singular configurations**: the articulated-body recursion needs a positive joint-space inertia at every joint. Massless distal links make it ill-conditioned; add armature.

## Variants & Generalizations

- **Velocity Verlet / Störmer–Verlet**: a second-order symplectic method at one extra evaluation per step.
- **Adaptive steps**: embedded Runge–Kutta pairs (Dormand–Prince) control the local error, at the price of non-deterministic step counts.
- **Implicit and semi-implicit (linearized) methods**: needed for stiff contact and compliant joints. They solve with the mass matrix or its articulated-body equivalent each step.
- **Lie-group integrators**: for floating bases, integrate the base pose on SE(3) with the exponential map instead of adding velocities to orientation parameters.
- **Constraint stabilization**: closed chains and contacts need Baumgarte or projection terms to keep constraint drift bounded.

## Applications

- Model-in-the-loop unit tests of joint controllers (PD with gravity compensation, computed torque, impedance) on a deterministic plant.
- Host simulators and visualizers of manipulator motion.
- Hardware-in-the-loop rigs where the plant model runs on the target alongside the controller.
- Generating synthetic data for identification and observer development.

## Connections to Other Algorithms

- [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md): supplies $\mathrm{FD}(q, \dot{q}, \tau)$ in $O(n)$ at every stage.
- [Composite Rigid Body Algorithm](CompositeRigidBodyAlgorithm.md): gives the kinetic energy $\tfrac12\dot{q}^T M \dot{q}$ used to monitor integration quality.
- [Recursive Newton-Euler](RecursiveNewtonEuler.md): inverse dynamics of the logged acceleration must return the applied torque, a consistency check for the whole loop.
- The semi-implicit Euler and Runge–Kutta integrators are generic solvers from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Hairer, E., Lubich, C. and Wanner, G. (2006). *Geometric Numerical Integration*. 2nd ed. Springer. Chapters I and IX.
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapter 7.
- Butcher, J.C. (2016). *Numerical Methods for Ordinary Differential Equations*. 3rd ed. Wiley.
