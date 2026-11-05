# Trajectory Generation

Time-parameterized joint and Cartesian references for manipulators.

## Algorithms

| Algorithm                                                               | Description                                                                                                           |
|-------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------|
| [Cubic Spline Trajectory](CubicSplineTrajectory.md)                     | $C^2$ joint trajectory through timed via points, rest-to-rest by default — $O(nK)$ plan, $O(\log K + n)$ sample       |
| [Cartesian Path with SLERP Orientation](CartesianSlerpInterpolation.md) | Straight-line position and shortest-arc quaternion SLERP on one time law, with twist feed-forward — $O(1)$ per sample |
