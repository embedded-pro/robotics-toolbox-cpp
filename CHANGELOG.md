# Changelog

All notable changes to this project will be documented in this file.

## [2.0.0](https://github.com/embedded-pro/robotics-toolbox-cpp/compare/v1.0.0...v2.0.0) (2026-10-24)


### ⚠ BREAKING CHANGES

* robotics headers no longer force O3 and fast-math. Consumers that want them pass them for whole translation units, e.g. -O2/-O3 -ffast-math -fno-finite-math-only.

### Features

* **simulator:** Render the robot arm panel through ui-cpp ([#17](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/17)) ([4689156](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/46891568a6151ac4d5652c9d7b80555f919fb320))
* **simulator:** Render the robot arm through ui-cpp ([#16](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/16)) ([9bd403b](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/9bd403bc962b3708eb184fba8f1cccc88eb8abfd))


### Bug Fixes

* **build:** Inherit NumericalToolbox_ENABLE_OPTIMIZATIONS from numerical.math ([#21](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/21)) ([b6a7848](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/b6a7848199c888dc147f4ae6bbc2bd3b96df71ad))
* FK/IK geometry, build/consume path and simulator physics; correct docs and roadmap specs ([#19](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/19)) ([c6712b4](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/c6712b458fb10677fcf402a92250906524931561))


### Miscellaneous

* Add controllers roadmap ([#7](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/7)) ([8037258](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/80372581af94815e507213e205427f3a83818d1f))


### Build System

* Bump actions/upload-artifact ([#23](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/23)) ([21376e5](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/21376e5f63f8b7a282a54cdadb62f0306df4dab7))
* Bump github/codeql-action/upload-sarif ([#11](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/11)) ([967a1bb](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/967a1bb395aadd09639b170fa53ae7d80008193d))
* Bump github/codeql-action/upload-sarif ([#12](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/12)) ([0522532](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/0522532880cb0ba769c9a61827c81403c66c2a0c))
* Bump github/codeql-action/upload-sarif ([#22](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/22)) ([9f3127c](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/9f3127c09bc05b0f5b31af0687a413568cb2bde0))
* Bump github/codeql-action/upload-sarif ([#9](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/9)) ([1f001b1](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/1f001b1fd38e86c62286064bb7b9254947decfac))
* Bump numerical-toolbox to adfc10d ([#26](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/26)) ([03afbe1](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/03afbe1ca0408b5c69a05408bffc1b6b5b53a0ba))
* Bump oxsecurity/megalinter/flavors/c_cpp from 9.6.0 to 10.0.0 ([#10](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/10)) ([e7e180c](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/e7e180c136d88d2c14eb82ff0d171c453eedf306))
* Bump the patch-minor-action-updates group across 1 directory with 3 updates ([#15](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/15)) ([cbc9ec5](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/cbc9ec553f0148944c36407b8f1b1fad9d7117e1))
* Bump the patch-minor-action-updates group with 2 updates ([#18](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/18)) ([bb218b2](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/bb218b24edc07daa1544288ddc881865b779844f))
* Bump the patch-minor-action-updates group with 3 updates ([#13](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/13)) ([450ff16](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/450ff1673caacb94f2d6c48fb501d3837e3bf6c9))
* Bump the patch-minor-action-updates group with 3 updates ([#24](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/24)) ([377f3b4](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/377f3b425c6d24b2ccf919559c99e21c105f437f))
* Bump the patch-minor-action-updates group with 3 updates ([#29](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/29)) ([8676750](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/8676750a48f7bc37dcc5225e6ceac84511b6ed11))


### Performance Improvements

* Drop per-header optimize pragmas; move generic roadmap items upstream ([#25](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/25)) ([b054159](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/b05415985737cc92d0e7693cb9d1b2fe0231e27b))
* Protected interface destructors; bump numerical-toolbox to 63c9b91 ([#30](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/30)) ([ee4594e](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/ee4594e590a21073248f14105a11afd85096714d))

## 1.0.0

Initial release. Robot-manipulator kinematics and dynamics extracted from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), which is now
consumed as a dependency via `FetchContent`.

### Features

- **Kinematics**: Forward Kinematics, Inverse Kinematics (Damped Least Squares).
- **Dynamics**: Euler-Lagrange, Newton-Euler, Recursive Newton-Euler (RNEA), Articulated Body
  Algorithm (ABA).
- **Simulator**: Qt-based Robot Arm visualization.
