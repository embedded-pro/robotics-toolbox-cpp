# Changelog

All notable changes to this project will be documented in this file.

## 1.0.0

Initial release. Robot-manipulator kinematics and dynamics extracted from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), which is now
consumed as a dependency via `FetchContent`.

### Features

- **Kinematics**: Forward Kinematics, Inverse Kinematics (Damped Least Squares).
- **Dynamics**: Euler-Lagrange, Newton-Euler, Recursive Newton-Euler (RNEA), Articulated Body
  Algorithm (ABA).
- **Simulator**: Qt-based Robot Arm visualization.
