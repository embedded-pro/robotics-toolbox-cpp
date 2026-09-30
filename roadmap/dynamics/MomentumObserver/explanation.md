# Generalized-Momentum Observer — Overview

## What it is
A disturbance observer that estimates the torque an unexpected contact applies to each joint, using
only the motor command and the measured joint positions and velocities.

## Why it matters (embedded)
Collaborative robots must stop or react within milliseconds of hitting something or someone. The
observer needs no joint-torque sensors and no differentiated velocities, runs at the control rate
and turns a model the controller already has into a safety signal.

## How it works (intuition)
The robot's generalized momentum `M(q)q̇` changes only because of applied torques, gravity, a
velocity-dependent term, and any external torque. The observer integrates everything it knows and
compares the result with the momentum it actually measures; the mismatch, fed back through a gain,
converges to the external torque with a first-order lag. Using momentum instead of acceleration is
what avoids differentiating noisy velocities.

## Key parameters
- **Observer gain** — bandwidth of the estimate (detection speed versus noise).
- **Thresholds** — per-joint residual levels that declare a collision.

## Reference
A. De Luca, A. Albu-Schäffer, S. Haddadin, G. Hirzinger, "Collision Detection and Safe Reaction with
the DLR-III Lightweight Manipulator Arm," *IEEE/RSJ IROS*, 2006; S. Haddadin, A. De Luca,
A. Albu-Schäffer, "Robot Collisions: A Survey on Detection, Isolation, and Identification," *IEEE
Trans. Robotics*, 33(6), 2017.

## See also
`ChainDynamicsModel` (M29), `CoriolisMatrixAndRegressor` (M31), `FrictionCompensation` (M4),
`RneaExternalWrench` (M32).
