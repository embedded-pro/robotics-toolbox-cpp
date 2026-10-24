# Planar Odometry on SE(2) — Overview

## What it is
Dead reckoning for ground robots: it accumulates the robot's pose from its measured velocity (forward
speed, sideways speed, turn rate), sample by sample, without external references.

## Why it matters (embedded)
Every wheeled robot runs odometry at its control rate: it is the pose estimate between GPS fixes or scan
matches, and the motion model inside the localization filters. It has to be cheap, deterministic and
free of systematic integration bias, because its errors only grow.

## How it works (intuition)
Over one sample period the robot moves with a nearly constant velocity, which traces a circular arc (a
line if it is not turning). Simple integration (move along the current heading, then turn) cuts every
arc short and slowly bends the estimated path. Here each step jumps to the exact end of the arc with the
planar exponential map, so only sensor errors remain. The integrator also reports the motion between
any two poses in the robot's own frame, which is what the localization filters consume.

## Key parameters
- **twist `(vx, vy, ω)`** — body velocity from a wheel model (or a displacement from encoder increments).
- **sample period `dt`**.

## Reference
R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed. (2011),
Ch. 5; J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in robotics" (2018).

## See also
`math::SE2Transform` (NT66, the exponential, from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)); `DifferentialDrive` (W1),
`OmnidirectionalKinematics` (W3) and `SkidSteerKinematics` (W4) as twist sources.
