# EKF Landmark Localization — Overview

## What it is
A Kalman-filter localizer for a robot that knows where certain landmarks are — reflective posts,
fiducial markers, poles — and measures their distance and direction. It keeps a pose estimate and its
uncertainty, moves it with odometry, and corrects it with every landmark sighting.

## Why it matters (embedded)
Many industrial AGVs, lawn robots and warehouse vehicles navigate this way: a laser picks out
reflectors, or a camera picks out markers, at surveyed positions. With a 3-element state the filter is
tiny, so it runs at sensor rate on a microcontroller and gives a consistent uncertainty for safety checks
and planners.

## How it works (intuition)
Between sightings the pose moves with the odometry increment, and the uncertainty grows in proportion to
how far and how much the robot turned. When a landmark is seen, the filter predicts where it should
appear given the current estimate. The difference between seen and predicted range and bearing corrects
the pose, weighted by how uncertain each is, and the bearing difference is taken the short way round the
circle. If a sighting carries no identity, the landmark it most plausibly belongs to is chosen; sightings
too far from every landmark are rejected as clutter.

## Key parameters
- **motion noise** — growth of position and heading uncertainty with distance and rotation.
- **range and bearing noise** of the sensor.
- **gate** — how implausible a sighting may be before it is rejected.

## Reference
S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* (2005), Ch. 7; J. J. Leonard, H. F. Durrant-Whyte,
"Mobile robot localization by tracking geometric beacons," *IEEE T-RA*, 7(3), 1991.

## See also
`PlanarOdometry` (F1, the increment), `OdometryFusion` (E1); upstream `ResidualKalmanUpdate` (NT74),
`SE2Transform` (NT66).
