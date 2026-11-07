# Changelog

All notable changes to this project will be documented in this file.

## [2.0.0](https://github.com/embedded-pro/robotics-toolbox-cpp/compare/v1.0.0...v2.0.0) (2026-11-07)


### ⚠ BREAKING CHANGES

* robotics headers no longer force O3 and fast-math. Consumers that want them pass them for whole translation units, e.g. -O2/-O3 -ffast-math -fno-finite-math-only.

### Features

* **localization:** Add MonteCarloLocalization (E4) ([#41](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/41)) ([5d50077](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/5d500772b85815928d3e730a99b7ed5e143aeb18))
* **localization:** Add OdometryFusion (E1) and LandmarkLocalization (E3) ([#39](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/39)) ([33d39fd](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/33d39fd2f47722ed7a5a9f006a9ac0df09056986))
* **localization:** Add ScanMatcher (E5) ([#40](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/40)) ([de4c164](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/de4c16495d095e46b4a469112907b65cd3a97a8f))
* Mobile wave roadmap items (F1, W1–W10, E2, M24) ([#38](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/38)) ([52832fa](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/52832fa714a11b19ee22f0ff235d3ea34019be39))
* **simulator:** Render the robot arm panel through ui-cpp ([#17](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/17)) ([4689156](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/46891568a6151ac4d5652c9d7b80555f919fb320))
* **simulator:** Render the robot arm through ui-cpp ([#16](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/16)) ([9bd403b](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/9bd403bc962b3708eb184fba8f1cccc88eb8abfd))
* **trajectory:** Add TimeOptimalPathParameterization (M27) ([#42](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/42)) ([6161423](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/6161423e5d8d8cec7ab0c932c49afdccc9f5dfa3))
* Wave 1 roadmap items (M33, M28, P1, M5, M34) ([#33](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/33)) ([d095f8c](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/d095f8cceac34adc13a20846a9573c5654b4be88))
* Wave 3 roadmap items (M1, M30, M8, M7, M29) ([#34](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/34)) ([35aac71](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/35aac714a9e23dca0f007f4a3b2869f13dd89fcb))
* Wave 4 roadmap items (M32, M12, M11, M14, M10, P2) ([#35](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/35)) ([20f230f](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/20f230fbe3b36759d66d1ce5dbc54bef9a45261a))
* Wave 5 roadmap items (P4, M15, M31, M16, M22, M13, M17, P3) ([#36](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/36)) ([0685e5c](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/0685e5c208e6f3b822d25bc8ed8f66a504c605a6))
* Wave 6 roadmap items (M26, M21, M20, M18, M19, P6, M23, M25, P5) ([#37](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/37)) ([a3ebb6f](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/a3ebb6fe703bbb7f971e273600e1bfab2f591c6d))


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
* Bump the patch-minor-action-updates group with 2 updates ([#43](https://github.com/embedded-pro/robotics-toolbox-cpp/issues/43)) ([114a30a](https://github.com/embedded-pro/robotics-toolbox-cpp/commit/114a30a09bf8c56a91a3004ecb3bfba25db93e4b))
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
