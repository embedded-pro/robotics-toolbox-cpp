# Forward-Dynamics Integrator — Overview

## What it is
A fixed-step time-stepping routine that advances a robot arm's joint positions and velocities under
given joint torques, using the articulated body algorithm for the accelerations.

## Why it matters (embedded)
Controllers are best tested against a simulated plant before touching hardware. A deterministic,
heap-free step function lets the same code run in unit tests, on a host simulator and in
hardware-in-the-loop rigs.

## How it works (intuition)
Each step asks the dynamics for the joint accelerations at the current state and torque, then moves
the state forward in time. Semi-implicit Euler updates the velocity first and uses the new velocity
for the position, which keeps the energy error bounded; the classical fourth-order Runge-Kutta
method samples the dynamics four times per step for much higher accuracy.

## Key parameters
- **Step size** — accuracy and stability versus cost.
- **Method** — semi-implicit Euler (cheap, bounded energy error) or RK4 (accurate).

## Reference
E. Hairer, C. Lubich, G. Wanner, *Geometric Numerical Integration* (2006), Ch. I;
R. Featherstone, *Rigid Body Dynamics Algorithms* (2008), Ch. 7.

## See also
`ArticulatedBodyAlgorithm`, upstream Runge-Kutta integrators (numerical-toolbox-cpp), the Robot Arm
simulator.
