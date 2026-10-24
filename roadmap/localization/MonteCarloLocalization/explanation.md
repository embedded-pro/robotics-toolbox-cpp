# Monte Carlo Localization — Overview

## What it is
A way for a robot to find and track its position on a known map using a cloud of guesses. Each guess
("particle") is a possible pose. The cloud moves with the odometry and is re-weighted by how well the
laser scan fits the map from that pose, so wrong guesses fade and right ones multiply.

## Why it matters (embedded)
Monte Carlo localization is the standard indoor localizer (ROS AMCL is built on it). It copes with
ambiguous places, starting with no idea of the position, and occasional clutter. With a fixed particle
count and a precomputed distance map, each scan costs a predictable amount of work. Because the random
numbers come from the caller, test and field runs are reproducible.

## How it works (intuition)
When the robot moves, every particle moves by the same odometry step plus a little random noise, more
for longer or more twisting steps. When a scan arrives, each particle projects the beam end points into
the map from its own pose. Points that land near walls score well, using a precomputed map of the
distance from every cell to the nearest wall. Scores become weights, and when only a few particles carry
most of the weight the cloud is resampled around them. The reported pose is the weighted average, with
headings averaged as directions so that +179° and −179° average to 180°, not 0°.

## Key parameters
- **particle count** (compile-time) and **resampling threshold**.
- **motion noise** — how much the cloud spreads per metre and per radian.
- **hit sigma**, **hit / random weights**, **beam stride** — sensor model and its cost.

## Reference
F. Dellaert, D. Fox, W. Burgard, S. Thrun, "Monte Carlo localization for mobile robots," *ICRA* 1999;
S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* (2005), Ch. 6.4 (likelihood field) and Ch. 8.3.

## See also
`OccupancyGridMap` (E2), `LandmarkLocalization` (E3, the Gaussian alternative), `PlanarOdometry` (F1);
upstream `ParticleFilter` (NT70).
