# Robotics Toolbox — Roadmap

Prioritized backlog by robot family. Manipulators (**M**), wheeled robots (**W**, with **F1**), localization
(**E**) and planning (**P**) have specs (`roadmap/<domain>/<Name>/`); the other families are entries only so
far, and their specs come family by family:

| Prefix | Family                                                        | Section                                                                   |
|--------|---------------------------------------------------------------|---------------------------------------------------------------------------|
| M      | Serial, parallel, cable and continuum arms                    | [Robot manipulators](#robot-manipulators--other-manipulator-types)        |
| F      | Shared foundation (floating base, SE(2), online trajectories) | [Shared foundation](#shared-foundation)                                   |
| W      | Wheeled mobile robots                                         | [Wheeled mobile robots](#wheeled-mobile-robots)                           |
| A      | Multirotors and fixed-wing aircraft                           | [Aerial robots](#aerial-robots)                                           |
| L      | Legged robots and humanoids                                   | [Legged robots and humanoids](#legged-robots-and-humanoids)               |
| S      | Surface vessels and underwater vehicles                       | [Marine and underwater vehicles](#marine-and-underwater-vehicles)         |
| G      | Grasping                                                      | [Grasping](#grasping-and-manipulation-of-objects)                         |
| P      | Motion planning and collision                                 | [Motion planning and collision](#motion-planning-and-collision)           |
| E      | Localization and mapping                                      | [Localization and mapping](#localization-and-mapping)                     |
| C      | Calibration                                                   | [Calibration](#calibration)                                               |
| H      | Actuation, safety and human–robot interaction                 | [Actuation, safety and HRI](#actuation-safety-and-humanrobot-interaction) |

IDs are stable: an item keeps its ID when it moves or ships, and a retired ID is never reused. Shared
numerical primitives (`math`, `solvers`, …) are consumed from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp); upstream already
provides `Matrix`, `Quaternion`, `MatrixExponential`, Gaussian elimination, Cholesky, LU, QR, SVD
(tall and wide), Jacobi eigen-solver, Runge-Kutta integrators, Kalman filters and LQR/MPC. Generic pieces
this roadmap needs that upstream does not ship yet are specified and built upstream; see
[Upstream prerequisites](#upstream-prerequisites-numerical-toolbox).

**Conventions (normative for every family):**

- World frame z-up (ENU); body frame x-forward, y-left, z-up (FLU), as in ROS REP 103. Aerial and marine
  literature written in NED/FRD is converted at the model boundary, never mixed inside an algorithm.
- 6-vectors are linear part first — twists `(v; ω)`, wrenches `(f; n)` (NT53); planar poses are SE(2)
  `(x, y, θ)` with twists `(vx, vy, ω)` (NT66); headings are wrapped to `(−π, π]` (NT54).
- Quaternions are Hamilton, scalar first `(w, x, y, z)`, as upstream `math::Quaternion`.
- SI units, *float-only*, no heap: planners, maps and filters use compile-time capacities (bounded node
  pools, grids and particle sets) and injected deterministic samplers.

## Upstream prerequisites (numerical-toolbox)

Generic algorithms live in numerical-toolbox-cpp and are referenced here as **NT** plus its
master-list item number (NT53 is upstream item 53). A robotics item that depends on one needs a
numerical-toolbox pin that ships it. NT53–NT66 have shipped upstream, and their links point to the upstream
docs; the current pin (`dc147fb`) includes them.

| #    | Component                                             | Upstream module      | Spec / doc                          | Needed by                                                                  |
|------|-------------------------------------------------------|----------------------|-------------------------------------|----------------------------------------------------------------------------|
| NT53 | SE(3) transform, twists, wrenches, exp/log, PoseError | `math`               | [SE3Transform][nt53]                | M7, M8, M10, M13, M15, M21, M23, M25, M26, M30, F3, A1, A6, L3, S2, G1, C1 |
| NT54 | Angle utilities (wrap, shortest difference, unwrap)   | `math`               | [AngleUtilities][nt54]              | M21, W6, W9, W10, E1, E3, S1                                               |
| NT55 | Polynomial point-to-point profile (+ profile types)   | `motion_profiles`    | [PolynomialProfile][nt55]           | M10, F4, L2                                                                |
| NT56 | Trapezoidal (LSPB) profile + duration sync            | `motion_profiles`    | [TrapezoidalProfile][nt56]          | M10 (time law, optional), F4                                               |
| NT57 | Friction compensation (Coulomb + viscous + Stribeck)  | `controllers`        | [FrictionCompensation][nt57]        | M5, M12, M29 (optional feedforward)                                        |
| NT58 | Symplectic integrators (semi-implicit Euler, Verlet)  | `solvers`            | [SymplecticIntegrators][nt58]       | M34                                                                        |
| NT59 | Tridiagonal (Thomas) solver                           | `math`               | [TridiagonalSolver][nt59]           | NT61                                                                       |
| NT60 | S-curve (jerk-limited) profile                        | `motion_profiles`    | [SCurveProfile][nt60]               | M10 (time law, optional)                                                   |
| NT61 | Cubic spline interpolation (vector-valued)            | `math`               | [CubicSpline][nt61]                 | M33                                                                        |
| NT62 | Damped pseudo-inverse + null-space projector          | `solvers`            | [DampedPseudoInverse][nt62]         | M14, H5                                                                    |
| NT63 | Incremental QR least squares (rank-revealing)         | `estimators/offline` | [IncrementalQrLeastSquares][nt63]   | M22                                                                        |
| NT64 | Levenberg–Marquardt (retraction, pluggable damping)   | `optimization`       | [LevenbergMarquardt][nt64]          | M13, M23, E5, C2                                                           |
| NT65 | Dense convex QP (Goldfarb–Idnani)                     | `solvers`            | [ActiveSetQp][nt65]                 | M25 (exact path), L6, G2, G3                                               |
| NT66 | SE(2) transform, twists, exp/log, pose error          | `math`               | [SE2Transform][nt66]                | F1, W1–W10, M24, E1, E2, E3, E5                                            |
| NT67 | Kabsch / Umeyama point-set alignment                  | `solvers`            | [PointSetAlignment][nt67] (spec)    | E5, C1                                                                     |
| NT68 | Lie-group Jacobians and ⊕/⊖ (SO(3), SE(3), SE(2))     | `math`               | [master list][ntrm], no spec yet    | C1 (and NT71)                                                              |
| NT69 | Control allocation (redistributed pseudo-inverse, QP) | `controllers`        | [master list][ntrm], no spec yet    | A2, S3                                                                     |
| NT70 | Particle filter (bounded, systematic resampling)      | `filters/active`     | [ParticleFilter][nt70] (spec)       | E4                                                                         |
| NT71 | Error-state EKF on Lie groups                         | `filters/active`     | [master list][ntrm], no spec yet    | L5                                                                         |
| NT72 | Minimum-derivative (min-snap) piecewise polynomials   | `motion_profiles`    | [master list][ntrm], no spec yet    | A7                                                                         |
| NT73 | Online jerk-limited trajectory from arbitrary state   | `motion_profiles`    | [master list][ntrm], no spec yet    | F2                                                                         |
| NT74 | Residual-form Kalman update with χ² gating            | `filters/active`     | [ResidualKalmanUpdate][nt74] (spec) | E1, E3                                                                     |

[ntrm]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/ROADMAP.md#master-list-by-priority
[nt53]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/math/SE3Transform.md
[nt54]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/math/AngleUtilities.md
[nt55]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/motion_profiles/PolynomialProfile.md
[nt56]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/motion_profiles/TrapezoidalProfile.md
[nt57]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/controllers/FrictionCompensation.md
[nt58]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/solvers/SymplecticIntegrators.md
[nt59]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/math/TridiagonalSolver.md
[nt60]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/motion_profiles/SCurveProfile.md
[nt61]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/math/CubicSpline.md
[nt62]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/solvers/DampedPseudoInverse.md
[nt63]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/estimators/IncrementalQrLeastSquares.md
[nt64]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/optimization/LevenbergMarquardt.md
[nt65]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/solvers/ActiveSetQp.md
[nt66]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/doc/math/SE2Transform.md
[nt67]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/solvers/PointSetAlignment/implementation.md
[nt70]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/filters/active/ParticleFilter/implementation.md
[nt74]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/filters/active/ResidualKalmanUpdate/implementation.md

## Robot manipulators & other manipulator types

The library already has the core of manipulator **modelling**: forward/inverse dynamics
([RNEA](robotics/dynamics/RecursiveNewtonEuler.hpp), [ABA](robotics/dynamics/ArticulatedBodyAlgorithm.hpp),
[Euler-Lagrange](robotics/dynamics/EulerLagrangeSolver.hpp)), the joint-space mass matrix
([CRBA](robotics/dynamics/CompositeRigidBodyAlgorithm.hpp)), a fixed-step simulation step
([ForwardDynamicsIntegrator](robotics/dynamics/ForwardDynamicsIntegrator.hpp)), the manipulator-equation adapter
([ChainDynamicsModel](robotics/dynamics/ChainDynamicsModel.hpp)), position forward kinematics with base and
tool offsets, and damped-least-squares [IK](robotics/kinematics/InverseKinematics.hpp). Full-pose
kinematics and the geometric Jacobian have shipped (see the note below); what is missing is the
**planning and control layer** that turns those models into a usable manipulator stack.

> **Full-pose kinematics shipped.** Links are generic ([JointLink](robotics/dynamics/JointLink.hpp):
> revolute or prismatic, limits, armature); [ChainPoseKinematics](robotics/kinematics/ChainPoseKinematics.hpp)
> and [DenavitHartenberg](robotics/kinematics/DenavitHartenberg.hpp) produce a frame chain, and
> [GeometricJacobian](robotics/kinematics/GeometricJacobian.hpp) builds the 6×N Jacobian from it behind
> the [JacobianProvider](robotics/kinematics/JacobianProvider.hpp) seam.

**Conventions (normative for every item):** 6-vectors are ordered linear part first — twists `(v; ω)`,
wrenches `(f; n)` — as defined by upstream `math::SE3Transform` (**NT53**); the geometric Jacobian
has linear rows first; orientation errors use the SE(3) logarithm (`PoseError`), never angle
subtraction. Controllers receive dynamics and Jacobians through small interfaces
(`EulerLagrangeDynamics`, `InverseDynamicsModel`, `JacobianProvider`) implemented by M29 / M8, so
they can be tested with StrictMock and bound to concrete types on hard real-time paths.

All items are *float-only* — torques, lengths, and inertias exceed the `Q15`/`Q31` range, matching the
existing `dynamics/` convention.

### Manipulator list (by priority)

| #   | Component                                         | Target module      | Depends on  | Difficulty |
|-----|---------------------------------------------------|--------------------|-------------|------------|
| M24 | Mobile-manipulator / nonholonomic-base kinematics | `kinematics`       | M8, M14, W1 | ★★★★☆      |
| M27 | Time-optimal path parameterization (TOPP-RA)      | `trajectory` (new) | M29         | ★★★★★      |

M2 (polynomial profile), M3 (trapezoidal profile), M4 (friction compensation), M6 (SE(3)) and M9
(S-curve profile) moved to numerical-toolbox as NT55, NT56, NT57, NT53 and NT60; their numbers are
retired here.

### Tier 2 — Easy ★★☆☆☆

### Tier 3 — Moderate ★★★☆☆

### Tier 4 — Advanced ★★★★☆

**M24. Mobile-manipulator / nonholonomic-base kinematics.** Combined base + arm Jacobian in the world frame with differential-drive constraints.
- *Algorithm / paper:* Y. Yamamoto, X. Yun, *IEEE Trans. Automatic Control*, 39(6), 1994.
- *Reuses / builds on:* the W1 differential-drive model for the base.

### Tier 5 — Hard / research-grade ★★★★★

**M27. Time-optimal path parameterization (TOPP-RA).** Minimum-time traversal of a fixed path under joint velocity and torque limits via reachability analysis (two-variable LPs per grid stage, solved locally by Fourier–Motzkin elimination).
- *Algorithm / paper:* J. Bobrow, S. Dubowsky, J. Gibson, *IJRR*, 4(3), 1985; H. Pham, Q.-C. Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis," *IEEE T-RO*, 34(3), 2018.

---

## Shared foundation

Cross-family building blocks that several of the families below depend on.

| #  | Component                                                    | Target module | Depends on      | Difficulty |
|----|--------------------------------------------------------------|---------------|-----------------|------------|
| F2 | Online multi-axis trajectory generation (time-synchronized)  | `trajectory`  | NT73, M1        | ★★☆☆☆      |
| F3 | Floating-base RNEA / ABA / CRBA                              | `dynamics`    | M28, NT53       | ★★★☆☆      |
| F4 | Via-point blending (joint and Cartesian)                     | `trajectory`  | NT55, NT56, M10 | ★★★☆☆      |
| F5 | Contact Jacobians and constrained (contact) forward dynamics | `dynamics`    | F3, M8          | ★★★★☆      |

**F2. Online multi-axis trajectory generation.** Joint-space wrapper over the upstream online generator: per-joint limits from M1, recomputation every cycle, all joints time-synchronized to the slowest.
- *Algorithm / paper:* T. Kröger, F. Wahl, *IEEE T-RO*, 26(1), 2010.
- *Reuses / builds on:* NT73, M1.

**F3. Floating-base dynamics.** RNEA, ABA and CRBA for a chain on a free 6-DOF base (the base as a floating joint), the model for legged robots, humanoids, aerial and space manipulators.
- *Algorithm / paper:* R. Featherstone, *Rigid Body Dynamics Algorithms* (2008), Ch. 9.
- *Reuses / builds on:* M28 spatial algebra, NT53.

**F4. Via-point blending.** Continuous motion through intermediate points without stopping: parabolic blends in joint space and polynomial blends between Cartesian segments, under the same limits as the upstream profiles.
- *Algorithm / paper:* J. J. Craig, *Introduction to Robotics*, Ch. 7; J. Lloyd, V. Hayward, *IEEE T-RA*, 9(3), 1993.
- *Reuses / builds on:* NT55, NT56, M10.

**F5. Contact Jacobians and constrained dynamics.** Contact-point Jacobians from the frame chain and the KKT system `[M Jcᵀ; Jc 0]` for accelerations and contact forces, with Baumgarte stabilization.
- *Algorithm / paper:* Featherstone (2008), Ch. 8 and 11; P.-B. Wieber, R. Tedrake, S. Kuindersma, "Modeling and control of legged robots," *Springer Handbook of Robotics* (2016), Ch. 48.
- *Reuses / builds on:* F3, M8.

## Wheeled mobile robots

| #   | Component                                          | Target module | Depends on | Difficulty |
|-----|----------------------------------------------------|---------------|------------|------------|
| W6  | Stanley path tracking                              | `mobile`      | W2, W5     | ★★☆☆☆      |
| W7  | Kanayama Lyapunov trajectory tracking              | `mobile`      | W1, NT66   | ★★☆☆☆      |
| W8  | Dubins shortest paths (+ car path type)            | `planning`    | NT66       | ★★☆☆☆      |
| W9  | Dynamic window approach (local obstacle avoidance) | `planning`    | W1, NT66   | ★★★☆☆      |
| W10 | Reeds–Shepp shortest paths (forward and reverse)   | `planning`    | W8, NT54   | ★★★☆☆      |

**W6. Stanley controller.** Front-axle cross-track and heading-error steering law for car-like vehicles.
- *Algorithm / paper:* S. Thrun et al., "Stanley: The robot that won the DARPA Grand Challenge," *J. Field Robotics*, 23(9), 2006.
- *Reuses / builds on:* W2, W5 (path).

**W7. Kanayama tracking controller.** Lyapunov-stable tracking of a reference pose and twist for unicycle-type bases, error expressed in the robot frame.
- *Algorithm / paper:* Y. Kanayama, Y. Kimura, F. Miyazaki, T. Noguchi, *ICRA* 1990.
- *Reuses / builds on:* W1, NT66.

**W8. Dubins paths.** Closed-form shortest forward-only paths for a bounded-curvature car (6 words), with float-safe existence tests; defines the `CarPath` type W10 reuses.
- *Algorithm / paper:* L. E. Dubins, *Am. J. Math.*, 79(3), 1957; A. Shkel, V. Lumelsky, *Robotics and Autonomous Systems*, 34(4), 2001.
- *Reuses / builds on:* NT66.

**W9. Dynamic window approach.** Samples the reachable `(v, ω)` window per cycle and scores heading, clearance and speed; clearance is the closed-form arc distance to the points of a laser scan, so no collision library (P1) is needed.
- *Algorithm / paper:* D. Fox, W. Burgard, S. Thrun, *IEEE Robotics & Automation Magazine*, 4(1), 1997.
- *Reuses / builds on:* W1, NT66.

**W10. Reeds–Shepp paths.** Closed-form shortest paths for a bounded-curvature car that may reverse (48 words from 8 base formulas and their symmetries); continuous in the goal, the metric for parking and car-like planners.
- *Algorithm / paper:* J. A. Reeds, L. A. Shepp, *Pacific J. Math.*, 145(2), 1990; corrected formulas as in OMPL.
- *Reuses / builds on:* W8 (`CarPath`), NT54.

## Aerial robots

| #  | Component                                                    | Target module | Depends on | Difficulty |
|----|--------------------------------------------------------------|---------------|------------|------------|
| A1 | Multirotor rigid-body model and mixer geometry               | `aerial`      | NT53       | ★★☆☆☆      |
| A2 | Motor mixing with saturation handling                        | `aerial`      | A1, NT69   | ★★☆☆☆      |
| A3 | Cascaded attitude / rate control (quaternion attitude error) | `aerial`      | A1         | ★★★☆☆      |
| A4 | Fixed-wing L₁ lateral guidance                               | `aerial`      | —          | ★★★☆☆      |
| A5 | Fixed-wing total energy control system (TECS)                | `aerial`      | —          | ★★★☆☆      |
| A6 | Geometric tracking control on SE(3)                          | `aerial`      | A1, NT53   | ★★★★☆      |
| A7 | Minimum-snap trajectory with differential flatness           | `aerial`      | NT72, A6   | ★★★★☆      |

**A1. Multirotor model.** Rigid-body dynamics with thrust and drag-torque coefficients per rotor and the mixer matrix from arm geometry (quad, hexa, octo, X/+).
- *Algorithm / paper:* R. Mahony, V. Kumar, P. Corke, *IEEE Robotics & Automation Magazine*, 19(3), 2012.
- *Reuses / builds on:* NT53.

**A2. Motor mixing with saturation.** Collective thrust and body torques to rotor speeds with yaw-first desaturation, so roll/pitch authority survives motor limits.
- *Algorithm / paper:* M. Faessler, D. Falanga, D. Scaramuzza, *IEEE RA-L*, 2(2), 2017.
- *Reuses / builds on:* A1, NT69.

**A3. Cascaded attitude control.** Reduced/full quaternion attitude error to body-rate command, then a rate loop on the upstream PID — the standard flight-controller inner loops.
- *Algorithm / paper:* D. Brescianini, M. Hehn, R. D'Andrea, "Nonlinear quadrocopter attitude control," ETH Zürich, 2013.
- *Reuses / builds on:* A1; upstream `PidIncremental`.

**A4. L₁ guidance.** Lateral-acceleration command towards a reference point a fixed distance ahead on the path; handles lines and loiter circles.
- *Algorithm / paper:* S. Park, J. Deyst, J. P. How, *AIAA GNC Conference*, 2004.

**A5. TECS.** Couples throttle and pitch through total and distributed energy errors to track altitude and airspeed together.
- *Algorithm / paper:* A. A. Lambregts, *AIAA Paper* 83-2239, 1983.

**A6. Geometric SE(3) control.** Position and attitude tracking with errors on SO(3), almost-global stability and no Euler-angle singularities.
- *Algorithm / paper:* T. Lee, M. Leok, N. H. McClamroch, *IEEE CDC*, 2010.
- *Reuses / builds on:* A1, NT53.

**A7. Minimum-snap trajectories.** Waypoint trajectories minimizing snap per axis, then differential flatness to feed-forward attitude, body rates and thrust into A6.
- *Algorithm / paper:* D. Mellinger, V. Kumar, *ICRA* 2011.
- *Reuses / builds on:* NT72, A6.

## Legged robots and humanoids

| #  | Component                                               | Target module | Depends on | Difficulty |
|----|---------------------------------------------------------|---------------|------------|------------|
| L1 | Capture point / divergent component of motion           | `legged`      | —          | ★★☆☆☆      |
| L2 | Footstep placement (Raibert) and swing-leg trajectories | `legged`      | NT55       | ★★☆☆☆      |
| L3 | Centroidal dynamics and centroidal momentum matrix      | `legged`      | F3, NT53   | ★★★☆☆      |
| L4 | LIPM / ZMP preview control                              | `legged`      | —          | ★★★☆☆      |
| L5 | Contact-aided state estimation (leg kinematics + IMU)   | `legged`      | NT71, M30  | ★★★★☆      |
| L6 | Task-priority whole-body control (hierarchical QP)      | `legged`      | F5, NT65   | ★★★★★      |

**L1. Capture point / DCM.** Instantaneous capture point of the linear inverted pendulum and DCM tracking for push recovery and step timing.
- *Algorithm / paper:* J. Pratt, J. Carff, S. Drakunov, A. Goswami, *Humanoids* 2006; J. Englsberger, C. Ott, A. Albu-Schäffer, *IEEE T-RO*, 31(2), 2015.

**L2. Footstep placement and swing trajectories.** Raibert's velocity-feedback foot placement and lift-off-to-touch-down swing curves with height clearance.
- *Algorithm / paper:* M. Raibert, *Legged Robots That Balance* (MIT Press, 1986).
- *Reuses / builds on:* NT55.

**L3. Centroidal dynamics.** Centroidal momentum matrix `A_G(q)` and its rate term, linking joint velocities to whole-body linear and angular momentum.
- *Algorithm / paper:* D. Orin, A. Goswami, S.-H. Lee, *Autonomous Robots*, 35(2–3), 2013.
- *Reuses / builds on:* F3, NT53.

**L4. ZMP preview control.** Cart-table (LIPM) model with an LQ preview controller tracking a ZMP reference to generate the CoM trajectory.
- *Algorithm / paper:* S. Kajita et al., *ICRA* 2003.
- *Reuses / builds on:* upstream `DiscreteAlgebraicRiccatiEquation` / `Lqr`.

**L5. Contact-aided state estimation.** Error-state EKF fusing IMU propagation with leg-kinematics measurements of stance feet to estimate base pose and velocity.
- *Algorithm / paper:* M. Bloesch et al., *RSS* 2012.
- *Reuses / builds on:* NT71, M30.

**L6. Whole-body control.** Strict task hierarchy (contacts, balance, posture) solved as a sequence of QPs with friction-cone and torque limits.
- *Algorithm / paper:* A. Escande, N. Mansard, P.-B. Wieber, *IJRR*, 33(7), 2014; L. Sentis, O. Khatib, *IJHR*, 2(4), 2005.
- *Reuses / builds on:* F5, NT65.

## Marine and underwater vehicles

| #  | Component                                                             | Target module | Depends on | Difficulty |
|----|-----------------------------------------------------------------------|---------------|------------|------------|
| S1 | Line-of-sight (LOS) path-following guidance                           | `marine`      | NT54       | ★★☆☆☆      |
| S2 | 6-DOF vessel / AUV model (rigid body, added mass, damping, restoring) | `marine`      | NT53       | ★★★☆☆      |
| S3 | Thruster allocation (fixed and azimuth thrusters)                     | `marine`      | S2, NT69   | ★★★☆☆      |

**S1. LOS guidance.** Heading command from cross-track error and a look-ahead distance, with integral action against currents.
- *Algorithm / paper:* T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed. (2021), Ch. 12.
- *Reuses / builds on:* NT54.

**S2. 6-DOF marine craft model.** `M ν̇ + C(ν)ν + D(ν)ν + g(η) = τ` with rigid-body and added-mass inertia, Coriolis, linear + quadratic damping and restoring forces.
- *Algorithm / paper:* Fossen (2021), Ch. 6–8.
- *Reuses / builds on:* NT53.

**S3. Thruster allocation.** Generalized force to thruster commands with rate and magnitude limits, including azimuth thrusters through an extended thrust configuration.
- *Algorithm / paper:* Fossen (2021), Ch. 11; T. A. Johansen, T. I. Fossen, *Automatica*, 49(5), 2013.
- *Reuses / builds on:* S2, NT69.

## Grasping and manipulation of objects

| #  | Component                                       | Target module | Depends on | Difficulty |
|----|-------------------------------------------------|---------------|------------|------------|
| G1 | Grasp matrix and hand Jacobian                  | `grasping`    | NT53, M8   | ★★☆☆☆      |
| G2 | Contact-force distribution under friction cones | `grasping`    | G1, NT65   | ★★★☆☆      |
| G3 | Force-closure test and grasp-quality margin     | `grasping`    | G1, NT65   | ★★★★☆      |

**G1. Grasp matrix.** Object-wrench map from contact frames and contact models (point with/without friction, soft finger), plus the hand Jacobian.
- *Algorithm / paper:* D. Prattichizzo, J. Trinkle, "Grasping," *Springer Handbook of Robotics* (2016), Ch. 38.
- *Reuses / builds on:* NT53, M8.

**G2. Contact-force distribution.** Minimum-effort contact forces that balance an object wrench inside linearized friction cones.
- *Algorithm / paper:* Prattichizzo & Trinkle (2016); M. Buss, H. Hashimoto, J. Moore, *IEEE T-RA*, 12(3), 1996.
- *Reuses / builds on:* G1, NT65.

**G3. Force closure.** Force-closure test (grasp matrix rank plus a strictly interior force solution) and a QP-based margin as grasp quality.
- *Algorithm / paper:* R. Murray, Z. Li, S. Sastry (1994), Ch. 5; C. Ferrari, J. Canny, *ICRA* 1992.
- *Reuses / builds on:* G1, NT65.

## Motion planning and collision

| # | Component | Target module | Depends on | Difficulty |
|---|-----------|---------------|------------|------------|

## Localization and mapping

| #  | Component                                              | Target module  | Depends on       | Difficulty |
|----|--------------------------------------------------------|----------------|------------------|------------|
| E1 | Wheel + gyro odometry fusion (EKF on SE(2))            | `localization` | F1, NT66, NT74   | ★★★☆☆      |
| E2 | Occupancy-grid mapping (log-odds, ray casting)         | `localization` | NT66             | ★★★☆☆      |
| E3 | EKF landmark localization (known map, range–bearing)   | `localization` | F1, NT66, NT74   | ★★★☆☆      |
| E4 | Monte Carlo localization                               | `localization` | NT70, E2, F1     | ★★★★☆      |
| E5 | 2-D scan matching (point-to-point / point-to-line ICP) | `localization` | NT67, NT64, NT66 | ★★★★☆      |

**E1. Odometry fusion.** EKF on `(x, y, θ, gyro bias)`: wheel speed and gyro rate drive the prediction, the wheel yaw rate is a χ²-gated bias measurement (a rejected update flags slip), and optional heading and position fixes share the same update.
- *Algorithm / paper:* S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* (2005), Ch. 5 and 7; J. Borenstein, L. Feng, "Gyrodometry," *ICRA* 1996.
- *Reuses / builds on:* F1, NT66, NT74 (upstream `ExtendedKalmanFilter` cannot wrap residuals or gate).

**E2. Occupancy-grid mapping.** Log-odds cell updates from range scans along Bresenham rays on a bounded grid.
- *Algorithm / paper:* A. Elfes, *Computer*, 22(6), 1989; Thrun et al. (2005), Ch. 9.
- *Reuses / builds on:* NT66.

**E3. EKF landmark localization.** Pose tracking against a known landmark map with range–bearing measurements and gated data association.
- *Algorithm / paper:* Thrun et al. (2005), Ch. 7.
- *Reuses / builds on:* F1 (odometry noise), NT66, NT74.

**E4. Monte Carlo localization.** Particle-filter localization in an occupancy grid with the odometry motion model and a likelihood-field sensor model (two-pass chamfer distance map); samplers injected by the caller.
- *Algorithm / paper:* F. Dellaert, D. Fox, W. Burgard, S. Thrun, *ICRA* 1999.
- *Reuses / builds on:* NT70, E2, F1.

**E5. Scan matching.** Iterative closest point between consecutive laser scans or against a map: point-to-point (Kabsch) or point-to-line (Levenberg–Marquardt), outlier rejection and degeneracy detection.
- *Algorithm / paper:* P. Besl, N. McKay, *IEEE T-PAMI*, 14(2), 1992; A. Censi, *ICRA* 2008.
- *Reuses / builds on:* NT67, NT64, NT66.

## Calibration

| #  | Component                            | Target module | Depends on       | Difficulty |
|----|--------------------------------------|---------------|------------------|------------|
| C1 | Hand-eye calibration (AX = XB)       | `calibration` | NT53, NT67, NT68 | ★★★☆☆      |
| C2 | Kinematic (DH-parameter) calibration | `calibration` | M7, NT64         | ★★★★☆      |

**C1. Hand-eye calibration.** Camera-to-flange (or base-to-world) transform from pairs of robot and sensor motions, rotation via log maps and alignment, then translation by least squares.
- *Algorithm / paper:* F. Park, B. Martin, *IEEE T-RA*, 10(5), 1994.
- *Reuses / builds on:* NT53, NT67, NT68.

**C2. Kinematic calibration.** Identifiable DH-parameter errors from measured tool poses, by Levenberg–Marquardt on the pose residual.
- *Algorithm / paper:* W. Khalil, E. Dombre, *Modeling, Identification and Control of Robots* (2002), Ch. 11; J. Hollerbach, C. Wampler, *IJRR*, 15(6), 1996.
- *Reuses / builds on:* M7, NT64.

## Actuation, safety and human–robot interaction

| #  | Component                                                      | Target module                 | Depends on | Difficulty |
|----|----------------------------------------------------------------|-------------------------------|------------|------------|
| H1 | Online trajectory time-scaling under joint limits              | `safety`                      | M1         | ★★☆☆☆      |
| H2 | ISO/TS 15066 speed-and-separation and power-and-force limiting | `safety`                      | P1, M30    | ★★★☆☆      |
| H3 | Flexible-joint / series-elastic actuator model and control     | `controllers/manipulator`     | M29        | ★★★★☆      |
| H4 | Passivity-based bilateral teleoperation                        | `controllers/teleoperation`   | —          | ★★★★☆      |
| H5 | Image- and pose-based visual servoing                          | `controllers/visual_servoing` | NT62, M8   | ★★★★☆      |

**H1. Trajectory time-scaling.** Slows the time law of any trajectory online so joint velocity, acceleration and torque limits hold, then recovers nominal speed.
- *Algorithm / paper:* O. Dahl, L. Nielsen, *IEEE T-RA*, 6(5), 1990.
- *Reuses / builds on:* M1.

**H2. Collaborative-robot safety.** Protective separation distance from robot and human speeds, stopping time and sensor latency, plus speed limits from allowed transient contact energy.
- *Algorithm / paper:* ISO/TS 15066:2016; J. Marvel, R. Norcross, *Robotics and Computer-Integrated Manufacturing*, 44, 2017.
- *Reuses / builds on:* P1, M30.

**H3. Flexible joints and SEAs.** Two-mass joint model with spring stiffness, and singular-perturbation / cascaded torque control for elastic and series-elastic joints.
- *Algorithm / paper:* M. W. Spong, *ASME J. Dyn. Sys. Meas. Control*, 109(4), 1987; G. Pratt, M. Williamson, *IROS* 1995.
- *Reuses / builds on:* M29.

**H4. Bilateral teleoperation.** Time-domain passivity observer and controller keeping a master–slave haptic loop passive under delay and sampling.
- *Algorithm / paper:* B. Hannaford, J.-H. Ryu, *IEEE T-RA*, 18(1), 2002.

**H5. Visual servoing.** IBVS with the point-feature interaction matrix and PBVS on the SE(3) error, mapped to joint rates through the arm Jacobian.
- *Algorithm / paper:* F. Chaumette, S. Hutchinson, *IEEE Robotics & Automation Magazine*, 13(4), 2006.
- *Reuses / builds on:* NT62, M8.

---
