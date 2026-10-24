# Wheel + Gyro Odometry Fusion — Overview

## What it is
A small Kalman filter that combines a wheeled robot's wheel encoders with a yaw-rate gyro. It gives a
better pose estimate than either alone, learns the gyro's bias as it goes, and notices when the wheels
are slipping.

## Why it matters (embedded)
Wheel odometry is good at distance but bad at heading: slip and uneven floors twist the estimated path.
A MEMS gyro is good at short-term heading but drifts with its bias. Fusing them on the robot's
microcontroller, at the control rate and with a 4-element state, gives a dead-reckoning estimate that
holds its heading far longer. It also feeds localization (E3, E4) a better motion estimate.

## How it works (intuition)
Every cycle the robot moves forward by the wheel-measured speed and turns by the gyro rate minus the
current bias estimate, along an exact arc. The uncertainty grows with distance and with the gyro noise.
The wheels also report a turn rate; when it agrees with the bias-corrected gyro, the difference refines
the bias, which is easiest at standstill. When the two disagree beyond what noise can explain, the wheels
are slipping, and the comparison is skipped and reported. Absolute heading or position fixes, when
available, correct the pose through the same update.

## Key parameters
- **speed noise** (floor plus a fraction of speed), **gyro noise**, **bias random walk**.
- **wheel yaw-rate noise** and the **slip gate** (χ² threshold).

## Reference
S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* (2005), Ch. 3 and 5; J. Borenstein, L. Feng,
"Gyrodometry: a new method for combining data from gyros and odometry in mobile robots," *ICRA* 1996.

## See also
`PlanarOdometry` (F1), `DifferentialDrive` (W1), `SkidSteerKinematics` (W4); upstream `ResidualKalmanUpdate`
(NT74) and `SE2Transform` (NT66).
