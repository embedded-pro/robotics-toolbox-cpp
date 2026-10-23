# Robotics Toolbox — Roadmap

Prioritized backlog for the robot-manipulator stack (kinematics, dynamics, trajectories,
and manipulator control). Shared numerical primitives (`math`, `solvers`, …) are consumed from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp); upstream already
provides `Matrix`, `Quaternion`, `MatrixExponential`, Gaussian elimination, Cholesky, LU, QR, SVD
(tall and wide), Jacobi eigen-solver and Runge-Kutta integrators. Generic pieces this roadmap needs that
upstream does not ship yet — SE(3), motion profiles, a dense QP solver and others — are specified and
built upstream; see [Upstream prerequisites](#upstream-prerequisites-numerical-toolbox).

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

### Upstream prerequisites (numerical-toolbox)

Generic algorithms live in numerical-toolbox-cpp and are referenced here as **NT** plus its
master-list item number (NT53 is upstream item 53). A robotics item that depends on one needs a
numerical-toolbox pin that ships it.

| #    | Component                                             | Upstream module      | Spec                              | Needed by                                      |
|------|-------------------------------------------------------|----------------------|-----------------------------------|------------------------------------------------|
| NT53 | SE(3) transform, twists, wrenches, exp/log, PoseError | `math`               | [SE3Transform][nt53]              | M7, M8, M10, M13, M15, M21, M23, M25, M26, M30 |
| NT54 | Angle utilities (wrap, shortest difference, unwrap)   | `math`               | [AngleUtilities][nt54]            | M21                                            |
| NT55 | Polynomial point-to-point profile (+ profile types)   | `motion_profiles`    | [PolynomialProfile][nt55]         | M10                                            |
| NT56 | Trapezoidal (LSPB) profile + duration sync            | `motion_profiles`    | [TrapezoidalProfile][nt56]        | M10 (time law, optional)                       |
| NT57 | Friction compensation (Coulomb + viscous + Stribeck)  | `controllers`        | [FrictionCompensation][nt57]      | M5, M12, M29 (optional feedforward)            |
| NT58 | Symplectic integrators (semi-implicit Euler, Verlet)  | `solvers`            | [SymplecticIntegrators][nt58]     | M34                                            |
| NT59 | Tridiagonal (Thomas) solver                           | `math`               | [TridiagonalSolver][nt59]         | NT61                                           |
| NT60 | S-curve (jerk-limited) profile                        | `motion_profiles`    | [SCurveProfile][nt60]             | M10 (time law, optional)                       |
| NT61 | Cubic spline interpolation (vector-valued)            | `math`               | [CubicSpline][nt61]               | M33                                            |
| NT62 | Damped pseudo-inverse + null-space projector          | `solvers`            | [DampedPseudoInverse][nt62]       | M14                                            |
| NT63 | Incremental QR least squares (rank-revealing)         | `estimators/offline` | [IncrementalQrLeastSquares][nt63] | M22                                            |
| NT64 | Levenberg–Marquardt (retraction, pluggable damping)   | `optimization`       | [LevenbergMarquardt][nt64]        | M13, M23                                       |
| NT65 | Dense convex QP (Goldfarb–Idnani)                     | `solvers`            | [ActiveSetQp][nt65]               | M25 (exact path)                               |

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
| M24 | Mobile-manipulator / nonholonomic-base kinematics               | `kinematics`              | M8, M14        | ★★★★☆      |
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

**M25. Cable-driven tension distribution.** Structure matrix from cable geometry, Pott's closed-form (improved) tension distribution within `[tMin, tMax]`, and an exact QP path for the workspace boundary.
- *Algorithm / paper:* Pott, Bruckmann, Mikelsons (2009); A. Pott (2014).
- *Reuses / builds on:* upstream `solvers::ActiveSetQp` (NT65) for the exact path.

**M26. Continuum / soft constant-curvature kinematics.** Piecewise-constant-curvature FK and single-section IK.
- *Algorithm / paper:* R. Webster, B. Jones, *Int. J. Robotics Research*, 29(13), 2010.

### Tier 5 — Hard / research-grade ★★★★★

**M27. Time-optimal path parameterization (TOPP-RA).** Minimum-time traversal of a fixed path under joint velocity and torque limits via reachability analysis (two-variable LPs per grid stage, solved locally by Fourier–Motzkin elimination).
- *Algorithm / paper:* J. Bobrow, S. Dubowsky, J. Gibson, *IJRR*, 4(3), 1985; H. Pham, Q.-C. Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis," *IEEE T-RO*, 34(3), 2018.

---
