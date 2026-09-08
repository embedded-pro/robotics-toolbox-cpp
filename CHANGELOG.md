# Changelog

All notable changes to this project will be documented in this file.

## [1.0.1](https://github.com/embedded-pro/robotics-toolbox-cpp/compare/v1.0.0...v1.0.1) (2026-09-08)


### Miscellaneous

* Add controllers roadmap ([#7](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/7)) ([8037258](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/80372581af94815e507213e205427f3a83818d1f))


### Build System

* Bump github/codeql-action/upload-sarif ([#11](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/11)) ([967a1bb](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/967a1bb395aadd09639b170fa53ae7d80008193d))
* Bump github/codeql-action/upload-sarif ([#12](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/12)) ([0522532](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/0522532880cb0ba769c9a61827c81403c66c2a0c))
* Bump github/codeql-action/upload-sarif ([#9](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/9)) ([1f001b1](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/1f001b1fd38e86c62286064bb7b9254947decfac))
* Bump oxsecurity/megalinter/flavors/c_cpp from 9.6.0 to 10.0.0 ([#10](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/10)) ([e7e180c](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/e7e180c136d88d2c14eb82ff0d171c453eedf306))
* Bump the patch-minor-action-updates group with 3 updates ([#13](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/13)) ([450ff16](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/450ff1673caacb94f2d6c48fb501d3837e3bf6c9))

## 1.0.0

Initial release. Robot-manipulator kinematics and dynamics extracted from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), which is now
consumed as a dependency via `FetchContent`.

### Features

- **Kinematics**: Forward Kinematics, Inverse Kinematics (Damped Least Squares).
- **Dynamics**: Euler-Lagrange, Newton-Euler, Recursive Newton-Euler (RNEA), Articulated Body
  Algorithm (ABA).
- **Simulator**: Qt-based Robot Arm visualization.
