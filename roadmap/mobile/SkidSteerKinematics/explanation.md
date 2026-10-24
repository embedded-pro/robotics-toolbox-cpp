# Skid-Steer Kinematics — Overview

## What it is
A kinematic model for vehicles that steer by driving their left and right sides at different speeds
while the wheels or tracks skid sideways — tracked robots, excavators, and four-wheel skid-steer
platforms. It also includes a small estimator that learns the model's slip parameters from driving data.

## Why it matters (embedded)
Treating a skid-steer vehicle like an ideal two-wheeled robot overestimates how fast it turns, often by
a third or more, which ruins odometry and makes path trackers oscillate. The corrected model costs the
same few multiplications per cycle, and its three parameters can be identified on the vehicle itself
from a short calibration drive.

## How it works (intuition)
When a vehicle turns, every point of it rotates about one point on the ground: the instantaneous centre
of rotation. Each track has its own such point where the track does not slide on the ground. For an
ideal vehicle these lie exactly under the tracks. For a skidding vehicle they sit farther out, and the
vehicle's own centre of rotation shifts forward or back, adding a small sideways slip. Measuring where
these points are, from turns with known motion, turns the ideal formulas into accurate ones. Each
logged sample adds to three running averages, which give the three positions directly.

## Key parameters
- **ICR positions** — left and right tread ICR (lateral) and vehicle ICR (longitudinal), identified per
  terrain.
- **sprocket radius** and **max sprocket speed**.
- **minimum excitation** — how much turning the estimator needs before it trusts its result.

## Reference
J. L. Martínez, A. Mandow, J. Morales, S. Pedraza, A. García-Cerezo, "Approximating kinematics for
tracked mobile robots," *IJRR*, 24(10), 2005; A. Mandow, J. L. Martínez, J. Morales, J. L. Blanco,
A. García-Cerezo, J. González, "Experimental kinematics for wheeled skid-steer mobile robots," *IROS* 2007.

## See also
`DifferentialDrive` (W1, the no-slip special case), `PlanarOdometry` (F1).
