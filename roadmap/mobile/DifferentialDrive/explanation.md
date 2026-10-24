# Differential-Drive Kinematics — Overview

## What it is
The kinematic model of a robot with two independently driven wheels on a common axle, such as robot
vacuums, warehouse carts or educational robots. It converts the two wheel speeds into the robot's
forward speed and turn rate, and back.

## Why it matters (embedded)
The motor controllers speak wheel speeds, while planners and trackers speak forward speed and turn rate.
This conversion runs every control cycle in both directions: commands go down to the wheels, and
measured wheel speeds come up to odometry. Saturating wheel commands in the right way keeps the robot on
its planned curve when it asks for more than the motors can give.

## How it works (intuition)
Both wheels rolling at the same speed drive straight; a speed difference turns the robot about a point on
the axle line, and opposite speeds spin it in place. The average of the two rim speeds is the forward
speed, and their difference divided by the track width is the turn rate. When a command would push a
wheel past its limit, both wheel speeds are scaled down by the same factor. The ratio of turn rate to
speed, and so the curve, stays the same, and the robot simply drives it more slowly.

## Key parameters
- **wheel radius `r`** and **track width `B`** — set the conversion; calibrate them for good odometry.
- **max wheel speed** — the motor limit used for saturation.

## Reference
R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed. (2011),
Ch. 3; J. Borenstein, L. Feng, "Measurement and correction of systematic odometry errors in mobile
robots," *IEEE T-RA*, 12(6), 1996.

## See also
`PlanarOdometry` (F1), `SkidSteerKinematics` (W4, the slipping generalisation), `PurePursuit` (W5) and
`KanayamaController` (W7), `MobileManipulatorKinematics` (M24).
