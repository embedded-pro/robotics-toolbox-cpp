# Robotics Toolbox — Roadmap

Prioritized backlog by robot family. Manipulators (**M**) are the most developed; the other families are
entries only so far — their specs (`roadmap/<domain>/<Name>/`) come family by family:

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
numerical-toolbox pin that ships it.

| #    | Component                                             | Upstream module      | Spec                              | Needed by                                                                  |
|------|-------------------------------------------------------|----------------------|-----------------------------------|----------------------------------------------------------------------------|
| NT53 | SE(3) transform, twists, wrenches, exp/log, PoseError | `math`               | [SE3Transform][nt53]              | M7, M8, M10, M13, M15, M21, M23, M25, M26, M30, F3, A1, A6, L3, S2, G1, C1 |
| NT54 | Angle utilities (wrap, shortest difference, unwrap)   | `math`               | [AngleUtilities][nt54]            | M21, S1                                                                    |
| NT55 | Polynomial point-to-point profile (+ profile types)   | `motion_profiles`    | [PolynomialProfile][nt55]         | M10, F4, L2                                                                |
| NT56 | Trapezoidal (LSPB) profile + duration sync            | `motion_profiles`    | [TrapezoidalProfile][nt56]        | M10 (time law, optional), F4                                               |
| NT57 | Friction compensation (Coulomb + viscous + Stribeck)  | `controllers`        | [FrictionCompensation][nt57]      | M5, M12, M29 (optional feedforward)                                        |
| NT58 | Symplectic integrators (semi-implicit Euler, Verlet)  | `solvers`            | [SymplecticIntegrators][nt58]     | M34                                                                        |
| NT59 | Tridiagonal (Thomas) solver                           | `math`               | [TridiagonalSolver][nt59]         | NT61                                                                       |
| NT60 | S-curve (jerk-limited) profile                        | `motion_profiles`    | [SCurveProfile][nt60]             | M10 (time law, optional)                                                   |
| NT61 | Cubic spline interpolation (vector-valued)            | `math`               | [CubicSpline][nt61]               | M33                                                                        |
| NT62 | Damped pseudo-inverse + null-space projector          | `solvers`            | [DampedPseudoInverse][nt62]       | M14, H5                                                                    |
| NT63 | Incremental QR least squares (rank-revealing)         | `estimators/offline` | [IncrementalQrLeastSquares][nt63] | M22                                                                        |
| NT64 | Levenberg–Marquardt (retraction, pluggable damping)   | `optimization`       | [LevenbergMarquardt][nt64]        | M13, M23, E5, C2                                                           |
| NT65 | Dense convex QP (Goldfarb–Idnani)                     | `solvers`            | [ActiveSetQp][nt65]               | M25 (exact path), L6, G2, G3                                               |
| NT66 | SE(2) transform, twists, exp/log, pose error          | `math`               | [master list][ntrm], no spec yet  | F1, W1, W2, W3, W5, W7, W8, E1, E2, E3, E5                                 |
| NT67 | Kabsch / Umeyama point-set alignment                  | `solvers`            | [master list][ntrm], no spec yet  | E5, C1                                                                     |
| NT68 | Lie-group Jacobians and ⊕/⊖ (SO(3), SE(3), SE(2))     | `math`               | [master list][ntrm], no spec yet  | C1 (and NT71)                                                              |
| NT69 | Control allocation (redistributed pseudo-inverse, QP) | `controllers`        | [master list][ntrm], no spec yet  | A2, S3                                                                     |
| NT70 | Particle filter (bounded, systematic resampling)      | `filters/active`     | [master list][ntrm], no spec yet  | E4                                                                         |
| NT71 | Error-state EKF on Lie groups                         | `filters/active`     | [master list][ntrm], no spec yet  | L5                                                                         |
| NT72 | Minimum-derivative (min-snap) piecewise polynomials   | `motion_profiles`    | [master list][ntrm], no spec yet  | A7                                                                         |
| NT73 | Online jerk-limited trajectory from arbitrary state   | `motion_profiles`    | [master list][ntrm], no spec yet  | F2                                                                         |

[ntrm]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/ROADMAP.md#master-list-by-priority
[nt53]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/math/SE3Transform/implementation.md
[nt54]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/math/AngleUtilities/implementation.md
[nt55]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/motion_profiles/PolynomialProfile/implementation.md
[nt56]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/motion_profiles/TrapezoidalProfile/implementation.md
[nt57]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/controllers/FrictionCompensation/implementation.md
[nt58]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/solvers/SymplecticIntegrators/implementation.md
[nt59]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/math/TridiagonalSolver/implementation.md
[nt60]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/motion_profiles/SCurveProfile/implementation.md
[nt61]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/math/CubicSpline/implementation.md
[nt62]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/solvers/DampedPseudoInverse/implementation.md
[nt63]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/estimators/offline/IncrementalQrLeastSquares/implementation.md
[nt64]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/optimization/LevenbergMarquardt/implementation.md
[nt65]: https://github.com/embedded-pro/numerical-toolbox-cpp/blob/main/roadmap/solvers/ActiveSetQp/implementation.md

## Robot manipulators & other manipulator types

The library already has the core of manipulator **modelling**: forward/inverse dynamics
([RNEA](robotics/dynamics/RecursiveNewtonEuler.hpp), [ABA](robotics/dynamics/ArticulatedBodyAlgorithm.hpp),
[Euler-Lagrange](robotics/dynamics/EulerLagrangeSolver.hpp)), position forward kinematics with base and
tool offsets, and damped-least-squares [IK](robotics/kinematics/InverseKinematics.hpp). What is missing is
the **full-pose kinematics, model plumbing, planning and control layer** that turns those models into a
usable manipulator stack. Two current limitations gate many items below:

> **Position-only, revolute-only.** [ForwardKinematics.hpp](robotics/kinematics/ForwardKinematics.hpp)
> returns joint and tool *positions* (a 3×N Jacobian lives privately inside IK), and only
> [RevoluteJointLink](robotics/dynamics/RevoluteJointLink.hpp) exists. Items **M1** (prismatic joints,
> limits, armature), **M30** (full-pose kinematics) and **M8** (6×N Jacobian) lift these limits.

**Conventions (normative for every item):** 6-vectors are ordered linear part first — twists `(v; ω)`,
wrenches `(f; n)` — as defined by upstream `math::SE3Transform` (**NT53**); the geometric Jacobian
has linear rows first; orientation errors use the SE(3) logarithm (`PoseError`), never angle
subtraction. Controllers receive dynamics and Jacobians through small interfaces
(`EulerLagrangeDynamics`, `InverseDynamicsModel`, `JacobianProvider`) implemented by M29 / M8, so
they can be tested with StrictMock and bound to concrete types on hard real-time paths.

All items are *float-only* — torques, lengths, and inertias exceed the `Q15`/`Q31` range, matching the
existing `dynamics/` convention.

### Manipulator list (by priority)

| #   | Component                                                       | Target module             | Depends on     | Difficulty |
|-----|-----------------------------------------------------------------|---------------------------|----------------|------------|
| M1  | Generic joint link (prismatic, limits, armature)                | `dynamics` + `kinematics` | —              | ★☆☆☆☆      |
| M5  | PD + gravity compensation control                               | `controllers/manipulator` | M29            | ★☆☆☆☆      |
| M32 | Inverse dynamics with an external tool wrench                   | `dynamics`                | —              | ★☆☆☆☆      |
| M33 | Joint-space cubic spline through via points                     | `trajectory` (new)        | NT61           | ★☆☆☆☆      |
| M28 | Composite Rigid Body Algorithm (mass matrix)                    | `dynamics`                | —              | ★★☆☆☆      |
| M29 | Chain dynamics model (link chain → M, C, g, inverse dynamics)   | `dynamics`                | M28            | ★★☆☆☆      |
| M30 | Chain pose kinematics (full-pose FK, frame chain)               | `kinematics`              | NT53           | ★★☆☆☆      |
| M7  | Denavit-Hartenberg parameters (standard + modified)             | `kinematics`              | NT53, M30      | ★★☆☆☆      |
| M8  | Geometric Jacobian (6×N), bias term, Jacobian provider          | `kinematics`              | NT53, M30      | ★★☆☆☆      |
| M10 | Cartesian path + orientation (SLERP) interpolation              | `trajectory` (new)        | NT53, NT55     | ★★☆☆☆      |
| M11 | Manipulability ellipsoid / Yoshikawa index                      | `kinematics`              | M8             | ★★☆☆☆      |
| M14 | Redundancy resolution / null-space projection                   | `kinematics`              | M8, NT62       | ★★☆☆☆      |
| M34 | Forward-dynamics integrator (simulation step)                   | `dynamics`                | NT58           | ★★☆☆☆      |
| M12 | Computed-torque (inverse-dynamics) control                      | `controllers/manipulator` | M29            | ★★★☆☆      |
| M13 | Full 6-DOF pose IK (position + orientation)                     | `kinematics`              | NT53, NT64, M8 | ★★★☆☆      |
| M15 | Product-of-Exponentials forward kinematics                      | `kinematics`              | NT53           | ★★★☆☆      |
| M16 | Momentum-based collision-detection observer                     | `dynamics`                | M29, M31       | ★★★☆☆      |
| M17 | Impedance control                                               | `controllers/manipulator` | M8, M29        | ★★★☆☆      |
| M31 | Coriolis matrix and inertial-parameter regressor                | `dynamics`                | M28            | ★★★☆☆      |
| M22 | Dynamic (base-parameter) identification                         | `dynamics`                | M31, NT63      | ★★★☆☆      |
| M18 | Operational-space (task-space) control                          | `controllers/manipulator` | M8, M29        | ★★★★☆      |
| M19 | Hybrid position/force control                                   | `controllers/manipulator` | M8, M29        | ★★★★☆      |
| M20 | Passivity-based adaptive control (Slotine-Li)                   | `controllers/manipulator` | M31            | ★★★★☆      |
| M21 | Analytical IK for ortho-parallel 6R arms with a spherical wrist | `kinematics`              | NT53, NT54     | ★★★★☆      |
| M23 | Parallel-manipulator kinematics (Stewart-Gough, Delta)          | `kinematics`              | NT53, NT64     | ★★★★☆      |
| M24 | Mobile-manipulator / nonholonomic-base kinematics               | `kinematics`              | M8, M14, W1    | ★★★★☆      |
| M25 | Cable-driven tension distribution                               | `controllers/manipulator` | NT53, NT65     | ★★★★☆      |
| M26 | Continuum / soft constant-curvature kinematics                  | `kinematics`              | NT53           | ★★★★☆      |
| M27 | Time-optimal path parameterization (TOPP-RA)                    | `trajectory` (new)        | M29            | ★★★★★      |

M2 (polynomial profile), M3 (trapezoidal profile), M4 (friction compensation), M6 (SE(3)) and M9
(S-curve profile) moved to numerical-toolbox as NT55, NT56, NT57, NT53 and NT60; their numbers are
retired here.

### Tier 1 — Trivial ★☆☆☆☆

**M1. Generic joint link.** Evolve `RevoluteJointLink` (compatibly, via trailing defaulted fields) into a link with a joint type (revolute/prismatic), joint limits and armature, and add the prismatic branches to FK, RNEA, ABA, CRBA and the Jacobian.
- *Algorithm / paper:* J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed., Ch. 3 and 6; Featherstone (2008), Ch. 4.

**M5. PD + gravity compensation control.** The simplest globally-stable set-point regulator: `τ = Kp·e − Kd·q̇ + g(q)`.
- *Algorithm / paper:* M. Takegaki, S. Arimoto, *ASME J. Dyn. Sys. Meas. Control*, 1981.
- *Reuses / builds on:* `g(q)` from M29; optional friction feedforward from NT57.

**M32. Inverse dynamics with an external tool wrench.** RNEA overload taking the environment's wrench on the tool; satisfies `τ(w) − τ(0) = −Jᵀw`.
- *Algorithm / paper:* Luh, Walker, Paul (1980).

**M33. Joint-space cubic spline through via points.** Timed via points to a C² joint trajectory, rest-to-rest by default; defines the shared `JointTrajectoryState`.
- *Algorithm / paper:* Biagiotti & Melchiorri (2008), Ch. 4.
- *Reuses / builds on:* upstream `math::CubicSpline` (NT61), which carries the tridiagonal solve.

### Tier 2 — Easy ★★☆☆☆

**M28. Composite Rigid Body Algorithm.** `O(n²)` joint-space mass matrix, sharing the spatial algebra of ABA.
- *Algorithm / paper:* Walker & Orin (1982); Featherstone (2008), Ch. 6.

**M29. Chain dynamics model.** Adapter implementing `EulerLagrangeDynamics` (M via CRBA, `Cq̇` and `g` via RNEA) and a one-call `InverseDynamicsModel` for the controllers.
- *Reuses / builds on:* RNEA, M28; optional friction model from NT57.

**M30. Chain pose kinematics.** Full tool pose and a model-independent frame chain (joint origins/axes in the base frame) for the link model.
- *Algorithm / paper:* Siciliano et al. (2009), Ch. 2.

**M7. Denavit-Hartenberg parameters.** Standard and modified `(a, α, d, θ)` descriptions with joint offsets, producing the M30 frame chain.
- *Algorithm / paper:* Craig, *Introduction to Robotics*, Ch. 3.

**M8. Geometric Jacobian (6×N).** Jacobian and `J̇q̇` from any frame chain, statics `τ = Jᵀw`, and the `JacobianProvider` seam used by IK and the task-space controllers; replaces IK's private 3×N Jacobian.
- *Algorithm / paper:* Siciliano et al. (2009), Ch. 3; Lynch & Park (2017), Ch. 5.

**M10. Cartesian path + orientation interpolation.** Straight-line position with SLERP orientation, one shared time law, twist feed-forward.
- *Algorithm / paper:* Lynch & Park, Ch. 9; K. Shoemake, *SIGGRAPH* 1985.
- *Reuses / builds on:* NT53 poses; any upstream motion profile (NT55, NT56, NT60) as the time law.

**M11. Manipulability ellipsoid / Yoshikawa index.** `√det(JJᵀ)` (or `√det(JᵀJ)` when the task has more rows than joints), condition number and ellipsoid axes via SVD, translational and rotational parts separately.
- *Algorithm / paper:* T. Yoshikawa, *Int. J. Robotics Research*, 4(2), 1985.

**M14. Redundancy resolution / null-space projection.** `q̇ = J⁺_λ ẋ + (I − J⁺J)·q̇₀` with an exact (undamped, rank-thresholded) projector, bound to the arm's task Jacobian.
- *Algorithm / paper:* A. Liégeois, *IEEE Trans. SMC*, 7(12), 1977.
- *Reuses / builds on:* upstream `solvers::DampedPseudoInverse` (NT62).

**M34. Forward-dynamics integrator.** Fixed-step semi-implicit Euler / RK4 simulation step on ABA for model-in-the-loop tests.
- *Algorithm / paper:* Hairer, Lubich, Wanner (2006); Featherstone (2008), Ch. 7.
- *Reuses / builds on:* upstream `solvers::SemiImplicitEuler` (NT58) and `solvers::RungeKutta4`.

### Tier 3 — Moderate ★★★☆☆

**M12. Computed-torque (inverse-dynamics) control.** `τ = ID(q, q̇, q̈_d + Kd·ė + Kp·e)` in one `O(n)` call, yielding decoupled error dynamics.
- *Algorithm / paper:* Spong et al., *Robot Modeling and Control*, Ch. 8; Luh, Walker, Paul (1980).

**M13. Full 6-DOF pose IK.** Damped least squares on the SE(3) log-map error with position/rotation weighting, adaptive damping, step and joint-limit clamping.
- *Algorithm / paper:* S. R. Buss (2004); Nakamura & Hanafusa (1986); Chiaverini (1997).
- *Reuses / builds on:* upstream `optimization::LevenbergMarquardt` (NT64) with a manipulability damping policy.

**M15. Product-of-Exponentials forward kinematics.** Screw-theory FK and the space Jacobian, with conversion to the geometric Jacobian.
- *Algorithm / paper:* Lynch & Park, *Modern Robotics*, Ch. 4.

**M16. Momentum-based collision-detection observer.** External joint-torque estimate from the generalized-momentum residual, using `Cᵀq̇`.
- *Algorithm / paper:* A. De Luca, A. Albu-Schäffer, S. Haddadin, G. Hirzinger, *IROS*, 2006.

**M17. Impedance control.** Stiffness/damping impedance via `Jᵀ` (no force sensor), and inertia-shaping impedance with measured contact wrench via the task-space inertia.
- *Algorithm / paper:* N. Hogan, *ASME J. Dyn. Sys. Meas. Control*, 1985.

**M31. Coriolis matrix and inertial regressor.** Modified RNEA giving the Christoffel `C(q,q̇)q̇_r` (skew-symmetric `Ṁ − 2C`), `Cᵀq̇`, and the 10-parameter-per-link regressor.
- *Algorithm / paper:* Echeandia & Wensing (2021); Niemeyer & Slotine (1991); Atkeson, An, Hollerbach (1986).

**M22. Dynamic (base-parameter) identification.** Regressor rows streamed into a rank-revealing least-squares solve; the kept directions are the base parameters.
- *Algorithm / paper:* Atkeson, An, Hollerbach (1986); Gautier & Khalil (1990).
- *Reuses / builds on:* M31; upstream `estimators::IncrementalQrLeastSquares` (NT63).

### Tier 4 — Advanced ★★★★☆

**M18. Operational-space control.** Task-space inertia `Λ = (J M⁻¹ Jᵀ)⁻¹`, `J̇q̇` compensation, full joint-space gravity/Coriolis compensation and a dynamically-consistent null space.
- *Algorithm / paper:* O. Khatib, *IEEE J. Robotics and Automation*, 3(1), 1987.

**M19. Hybrid position/force control.** Selection matrix in a constraint frame, PI force loop with anti-windup and damping, motion PD.
- *Algorithm / paper:* M. Raibert, J. Craig, 1981; An & Hollerbach, 1987.

**M20. Passivity-based adaptive control (Slotine-Li).** Tracks trajectories while estimating inertial parameters on-line through the regressor `Y(q,q̇,q̇r,q̈r)`.
- *Algorithm / paper:* J.-J. Slotine, W. Li, *Int. J. Robotics Research*, 6(3), 1987.

**M21. Analytical IK for ortho-parallel 6R arms with a spherical wrist (OPW).** Closed-form, all eight solutions, covering shoulder, lateral and elbow offsets of common industrial arms.
- *Algorithm / paper:* M. Brandstötter, A. Angerer, M. Hofbaur, Austrian Robotics Workshop, 2014; D. Pieper, PhD thesis, Stanford, 1968.
- *Reuses / builds on:* NT53; joint wrapping from NT54.

**M23. Parallel-manipulator kinematics.** Stewart-Gough (leg-length IK, Newton FK) and Delta (per-arm closed-form IK, three-sphere FK).
- *Algorithm / paper:* J.-P. Merlet, *Parallel Robots*, 2nd ed. (2006); R. Clavel (1990).
- *Reuses / builds on:* NT53 retraction; the Stewart Newton FK runs on upstream `LevenbergMarquardt` (NT64).

**M24. Mobile-manipulator / nonholonomic-base kinematics.** Combined base + arm Jacobian in the world frame with differential-drive constraints.
- *Algorithm / paper:* Y. Yamamoto, X. Yun, *IEEE Trans. Automatic Control*, 39(6), 1994.
- *Reuses / builds on:* the W1 differential-drive model for the base.

**M25. Cable-driven tension distribution.** Structure matrix from cable geometry, Pott's closed-form (improved) tension distribution within `[tMin, tMax]`, and an exact QP path for the workspace boundary.
- *Algorithm / paper:* Pott, Bruckmann, Mikelsons (2009); A. Pott (2014).
- *Reuses / builds on:* upstream `solvers::ActiveSetQp` (NT65) for the exact path.

**M26. Continuum / soft constant-curvature kinematics.** Piecewise-constant-curvature FK and single-section IK.
- *Algorithm / paper:* R. Webster, B. Jones, *Int. J. Robotics Research*, 29(13), 2010.

### Tier 5 — Hard / research-grade ★★★★★

**M27. Time-optimal path parameterization (TOPP-RA).** Minimum-time traversal of a fixed path under joint velocity and torque limits via reachability analysis (two-variable LPs per grid stage, solved locally by Fourier–Motzkin elimination).
- *Algorithm / paper:* J. Bobrow, S. Dubowsky, J. Gibson, *IJRR*, 4(3), 1985; H. Pham, Q.-C. Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis," *IEEE T-RO*, 34(3), 2018.

---

## Shared foundation

Cross-family building blocks that several of the families below depend on.

| #  | Component                                                    | Target module | Depends on      | Difficulty |
|----|--------------------------------------------------------------|---------------|-----------------|------------|
| F1 | Planar odometry on SE(2) (exact arc integration)             | `mobile`      | NT66            | ★★☆☆☆      |
| F2 | Online multi-axis trajectory generation (time-synchronized)  | `trajectory`  | NT73, M1        | ★★☆☆☆      |
| F3 | Floating-base RNEA / ABA / CRBA                              | `dynamics`    | M28, NT53       | ★★★☆☆      |
| F4 | Via-point blending (joint and Cartesian)                     | `trajectory`  | NT55, NT56, M10 | ★★★☆☆      |
| F5 | Contact Jacobians and constrained (contact) forward dynamics | `dynamics`    | F3, M8          | ★★★★☆      |

**F1. Planar odometry on SE(2).** Integrates a body twist `(vx, vy, ω)` over a sample period with the SE(2) exponential (exact constant-twist arc), shared by every wheeled model and the localization filters.
- *Algorithm / paper:* R. Siegwart, I. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed. (2011), Ch. 5.
- *Reuses / builds on:* NT66.

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

| #  | Component                                            | Target module | Depends on | Difficulty |
|----|------------------------------------------------------|---------------|------------|------------|
| W1 | Differential-drive kinematics                        | `mobile`      | NT66       | ★☆☆☆☆      |
| W2 | Car-like (Ackermann / kinematic bicycle) kinematics  | `mobile`      | NT66       | ★☆☆☆☆      |
| W3 | Omnidirectional (mecanum / Swedish-wheel) kinematics | `mobile`      | NT66       | ★☆☆☆☆      |
| W4 | Skid-steer kinematics with slip (ICR model)          | `mobile`      | W1         | ★★☆☆☆      |
| W5 | Pure-pursuit path tracking                           | `mobile`      | W1, NT66   | ★★☆☆☆      |
| W6 | Stanley path tracking                                | `mobile`      | W2         | ★★☆☆☆      |
| W7 | Kanayama Lyapunov trajectory tracking                | `mobile`      | W1, NT66   | ★★☆☆☆      |
| W8 | Dubins and Reeds–Shepp shortest paths                | `planning`    | NT66       | ★★★☆☆      |
| W9 | Dynamic window approach (local obstacle avoidance)   | `planning`    | W1, P1     | ★★★☆☆      |

**W1. Differential-drive kinematics.** Body twist from wheel speeds and back, wheel-speed limits mapped to `(v, ω)` limits; the base model of M24.
- *Algorithm / paper:* Siegwart et al. (2011), Ch. 3.
- *Reuses / builds on:* NT66.

**W2. Car-like kinematics.** Kinematic bicycle model with steering-angle and steering-rate limits, Ackermann wheel angles for the two front wheels.
- *Algorithm / paper:* R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed. (2012), Ch. 2.
- *Reuses / builds on:* NT66.

**W3. Omnidirectional kinematics.** Wheel-speed ↔ body-twist maps for mecanum and Swedish-wheel bases, including the roller-angle geometry.
- *Algorithm / paper:* P. Muir, C. Neuman, "Kinematic modeling of wheeled mobile robots," *J. Robotic Systems*, 4(2), 1987.
- *Reuses / builds on:* NT66.

**W4. Skid-steer kinematics.** Instantaneous-centre-of-rotation model with identified ICR offsets for tracked and skid-steered vehicles.
- *Algorithm / paper:* J. Martínez, J. Morales, A. Mandow, A. García-Cerezo, *IJRR*, 24(10), 2005.
- *Reuses / builds on:* W1.

**W5. Pure pursuit.** Curvature command towards a look-ahead point on the path, with speed-scheduled look-ahead distance.
- *Algorithm / paper:* R. C. Coulter, "Implementation of the pure pursuit path tracking algorithm," CMU-RI-TR-92-01, 1992.
- *Reuses / builds on:* W1 (or W2), NT66.

**W6. Stanley controller.** Front-axle cross-track and heading-error steering law for car-like vehicles.
- *Algorithm / paper:* S. Thrun et al., "Stanley: The robot that won the DARPA Grand Challenge," *J. Field Robotics*, 23(9), 2006.
- *Reuses / builds on:* W2.

**W7. Kanayama tracking controller.** Lyapunov-stable tracking of a reference pose and twist for unicycle-type bases, error expressed in the robot frame.
- *Algorithm / paper:* Y. Kanayama, Y. Kimura, F. Miyazaki, T. Noguchi, *ICRA* 1990.
- *Reuses / builds on:* W1, NT66.

**W8. Dubins and Reeds–Shepp paths.** Closed-form shortest paths for a bounded-curvature car, forward only (Dubins, 6 words) or with reversing (Reeds–Shepp, 48 words).
- *Algorithm / paper:* L. E. Dubins, *Am. J. Math.*, 79(3), 1957; J. A. Reeds, L. A. Shepp, *Pacific J. Math.*, 145(2), 1990.
- *Reuses / builds on:* NT66.

**W9. Dynamic window approach.** Samples the reachable `(v, ω)` window per cycle and scores heading, clearance and speed against obstacle distances.
- *Algorithm / paper:* D. Fox, W. Burgard, S. Thrun, *IEEE Robotics & Automation Magazine*, 4(1), 1997.
- *Reuses / builds on:* W1, P1.

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

| #  | Component                                                         | Target module | Depends on | Difficulty |
|----|-------------------------------------------------------------------|---------------|------------|------------|
| P1 | Primitive distance queries (point, segment, sphere, capsule, box) | `planning`    | —          | ★★☆☆☆      |
| P2 | Artificial potential fields                                       | `planning`    | P1         | ★★☆☆☆      |
| P3 | A* / Dijkstra on bounded occupancy grids                          | `planning`    | —          | ★★★☆☆      |
| P4 | Self-collision checking for a link chain (capsule model)          | `planning`    | P1, M30    | ★★★☆☆      |
| P5 | GJK + EPA convex distance and penetration                         | `planning`    | —          | ★★★★☆      |
| P6 | RRT-Connect in joint space with a fixed node pool                 | `planning`    | P4         | ★★★★☆      |

**P1. Primitive distances.** Closest points and distances between the convex primitives robots are usually wrapped in.
- *Algorithm / paper:* C. Ericson, *Real-Time Collision Detection* (2004), Ch. 5.

**P2. Potential fields.** Attractive goal and repulsive obstacle potentials as a reactive velocity/torque term, with the known local-minimum caveat.
- *Algorithm / paper:* O. Khatib, *IJRR*, 5(1), 1986.
- *Reuses / builds on:* P1.

**P3. Grid search.** A* and Dijkstra on 2-D/3-D occupancy grids with a bounded binary heap and closed set (no heap allocation).
- *Algorithm / paper:* P. Hart, N. Nilsson, B. Raphael, *IEEE Trans. Systems Science and Cybernetics*, 4(2), 1968.

**P4. Self-collision checking.** Capsule model per link, pairwise distances with an allowed-collision matrix, from the M30 frame chain.
- *Algorithm / paper:* Ericson (2004), Ch. 4–5.
- *Reuses / builds on:* P1, M30.

**P5. GJK + EPA.** Distance between convex shapes from support functions, and penetration depth/normal when they overlap.
- *Algorithm / paper:* E. Gilbert, D. Johnson, S. Keerthi, *IEEE J. Robotics and Automation*, 4(2), 1988; G. van den Bergen, *Collision Detection in Interactive 3D Environments* (2003).

**P6. RRT-Connect.** Bidirectional rapidly-exploring random trees in joint space with a compile-time node pool and a deterministic, injected sampler.
- *Algorithm / paper:* J. Kuffner, S. LaValle, *ICRA* 2000.
- *Reuses / builds on:* P4.

## Localization and mapping

| #  | Component                                              | Target module  | Depends on       | Difficulty |
|----|--------------------------------------------------------|----------------|------------------|------------|
| E1 | Wheel + IMU odometry fusion (EKF on SE(2))             | `localization` | F1, NT66         | ★★★☆☆      |
| E2 | Occupancy-grid mapping (log-odds, ray casting)         | `localization` | NT66             | ★★★☆☆      |
| E3 | EKF landmark localization (known map, range–bearing)   | `localization` | NT66             | ★★★☆☆      |
| E4 | Monte Carlo localization                               | `localization` | NT70, E2, F1     | ★★★★☆      |
| E5 | 2-D scan matching (point-to-point / point-to-line ICP) | `localization` | NT67, NT64, NT66 | ★★★★☆      |

**E1. Odometry fusion.** EKF combining wheel odometry with gyro (and optionally accelerometer) on SE(2), with slip-aware process noise.
- *Algorithm / paper:* S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* (2005), Ch. 5 and 7.
- *Reuses / builds on:* F1, NT66; upstream `ExtendedKalmanFilter`.

**E2. Occupancy-grid mapping.** Log-odds cell updates from range scans along Bresenham rays on a bounded grid.
- *Algorithm / paper:* A. Elfes, *Computer*, 22(6), 1989; Thrun et al. (2005), Ch. 9.
- *Reuses / builds on:* NT66.

**E3. EKF landmark localization.** Pose tracking against a known landmark map with range–bearing measurements and gated data association.
- *Algorithm / paper:* Thrun et al. (2005), Ch. 7.
- *Reuses / builds on:* NT66; upstream `ExtendedKalmanFilter`.

**E4. Monte Carlo localization.** Particle-filter localization in an occupancy grid with the odometry motion model and a likelihood-field sensor model.
- *Algorithm / paper:* F. Dellaert, D. Fox, W. Burgard, S. Thrun, *ICRA* 1999.
- *Reuses / builds on:* NT70, E2, F1.

**E5. Scan matching.** Iterative closest point between consecutive laser scans or against a map, with point-to-line error and outlier rejection.
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
