# Omnidirectional Kinematics — Overview

## What it is
The kinematic model of holonomic wheeled bases — robots on mecanum or omni wheels that can drive
sideways and diagonally and rotate while translating. It converts a desired body motion into
individual wheel speeds and measured wheel speeds back into body motion.

## Why it matters (embedded)
Omnidirectional bases are common in warehouse logistics, research platforms and robot competitions
because they manoeuvre in tight spaces without parking manoeuvres. The conversion runs every control
cycle; with four or more wheels the redundancy also lets the controller notice a slipping wheel or bad
encoder, at almost no extra cost.

## How it works (intuition)
Each wheel can push the robot only along one direction — its drive direction tilted by the angle of the
rollers on its rim — and slides freely in the perpendicular direction. A wheel's speed is therefore the
component of the robot's motion at that wheel along its push direction, including the part caused by
rotation about the centre. Stacking these relations for all wheels gives a small matrix. Going from body
motion to wheel speeds is a matrix multiply; going back uses the best-fitting body motion when there are
more wheels than degrees of freedom, and the mismatch measures how inconsistent the wheels are.

## Key parameters
- **wheel positions**, **drive directions** and **roller angles** (0° for omni wheels, ±45° for mecanum).
- **wheel radius** and **max wheel speed**.

## Reference
P. F. Muir, C. P. Neuman, "Kinematic modeling of wheeled mobile robots," *J. Robotic Systems*, 4(2), 1987;
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 13.2.

## See also
`DifferentialDrive` (W1), `PlanarOdometry` (F1), `math::SE2Transform` (NT66).
