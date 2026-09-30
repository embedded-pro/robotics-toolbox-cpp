# Robotics Toolbox — Roadmap

Prioritized backlog for the robot-manipulator stack (kinematics, dynamics, trajectories,
and manipulator control). Shared numerical primitives (`math`, `solvers`, …) are consumed from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp); upstream already
provides `Matrix`, `Quaternion`, `MatrixExponential`, Gaussian elimination, Cholesky, LU, QR, SVD,
Jacobi eigen-solver and Runge-Kutta integrators, but no SE(3) type and no general QP solver.

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
wrenches `(f; n)` — as defined by **M6**; the geometric Jacobian has linear rows first; orientation
errors use the SE(3) logarithm (`PoseError`), never angle subtraction. Controllers receive dynamics and
Jacobians through small interfaces (`EulerLagrangeDynamics`, `InverseDynamicsModel`, `JacobianProvider`)
implemented by M29 / M8, so they can be tested with StrictMock and bound to concrete types on
hard real-time paths.

All items are *float-only* — torques, lengths, and inertias exceed the `Q15`/`Q31` range, matching the
existing `dynamics/` convention.

### Manipulator list (by priority)

| #   | Component                                                       | Target module             | Depends on | Difficulty |
|-----|-----------------------------------------------------------------|---------------------------|------------|------------|
| M1  | Generic joint link (prismatic, limits, armature)                | `dynamics` + `kinematics` | —          | ★☆☆☆☆      |
| M2  | Cubic / quintic polynomial joint trajectory                     | `trajectory` (new)        | —          | ★☆☆☆☆      |
| M3  | Trapezoidal (LSPB) velocity profile + synchronization           | `trajectory` (new)        | —          | ★☆☆☆☆      |
| M4  | Friction compensation (Coulomb + viscous + Stribeck)            | `dynamics`                | —          | ★☆☆☆☆      |
| M5  | PD + gravity compensation control                               | `controllers/manipulator` | M29        | ★☆☆☆☆      |
| M32 | Inverse dynamics with an external tool wrench                   | `dynamics`                | —          | ★☆☆☆☆      |
| M6  | SE(3) transform, twists, wrenches, adjoint, exp/log             | `kinematics`              | —          | ★★☆☆☆      |
| M28 | Composite Rigid Body Algorithm (mass matrix)                    | `dynamics`                | —          | ★★☆☆☆      |
| M29 | Chain dynamics model (link chain → M, C, g, inverse dynamics)   | `dynamics`                | M28        | ★★☆☆☆      |
| M30 | Chain pose kinematics (full-pose FK, frame chain)               | `kinematics`              | M6         | ★★☆☆☆      |
| M7  | Denavit-Hartenberg parameters (standard + modified)             | `kinematics`              | M6, M30    | ★★☆☆☆      |
| M8  | Geometric Jacobian (6×N), bias term, Jacobian provider          | `kinematics`              | M6, M30    | ★★☆☆☆      |
| M9  | S-curve (jerk-limited) trajectory                               | `trajectory` (new)        | M3         | ★★☆☆☆      |
| M10 | Cartesian path + orientation (SLERP) interpolation              | `trajectory` (new)        | M6         | ★★☆☆☆      |
| M11 | Manipulability ellipsoid / Yoshikawa index                      | `kinematics`              | M8         | ★★☆☆☆      |
| M33 | Cubic spline through via points                                 | `trajectory` (new)        | —          | ★★☆☆☆      |
| M34 | Forward-dynamics integrator (simulation step)                   | `dynamics`                | —          | ★★☆☆☆      |
| M12 | Computed-torque (inverse-dynamics) control                      | `controllers/manipulator` | M29        | ★★★☆☆      |
| M13 | Full 6-DOF pose IK (position + orientation)                     | `kinematics`              | M6, M8     | ★★★☆☆      |
| M14 | Redundancy resolution / null-space projection                   | `kinematics`              | M8         | ★★★☆☆      |
| M15 | Product-of-Exponentials forward kinematics                      | `kinematics`              | M6         | ★★★☆☆      |
| M16 | Momentum-based collision-detection observer                     | `dynamics`                | M29, M31   | ★★★☆☆      |
| M17 | Impedance control                                               | `controllers/manipulator` | M8, M29    | ★★★☆☆      |
| M31 | Coriolis matrix and inertial-parameter regressor                | `dynamics`                | M28        | ★★★☆☆      |
| M18 | Operational-space (task-space) control                          | `controllers/manipulator` | M8, M29    | ★★★★☆      |
| M19 | Hybrid position/force control                                   | `controllers/manipulator` | M8, M29    | ★★★★☆      |
| M20 | Passivity-based adaptive control (Slotine-Li)                   | `controllers/manipulator` | M31        | ★★★★☆      |
| M21 | Analytical IK for ortho-parallel 6R arms with a spherical wrist | `kinematics`              | M6         | ★★★★☆      |
| M22 | Dynamic (base-parameter) identification                         | `dynamics`                | M31        | ★★★★☆      |
| M23 | Parallel-manipulator kinematics (Stewart-Gough, Delta)          | `kinematics`              | M6         | ★★★★☆      |
| M24 | Mobile-manipulator / nonholonomic-base kinematics               | `kinematics`              | M8, M14    | ★★★★☆      |
| M25 | Cable-driven tension distribution                               | `controllers/manipulator` | M6         | ★★★★☆      |
| M26 | Continuum / soft constant-curvature kinematics                  | `kinematics`              | M6         | ★★★★☆      |
| M27 | Time-optimal path parameterization (TOPP-RA)                    | `trajectory` (new)        | M29        | ★★★★★      |

### Tier 1 — Trivial ★☆☆☆☆

**M1. Generic joint link.** Evolve `RevoluteJointLink` (compatibly, via trailing defaulted fields) into a link with a joint type (revolute/prismatic), joint limits and armature, and add the prismatic branches to FK, RNEA, ABA, CRBA and the Jacobian.
- *Algorithm / paper:* J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed., Ch. 3 and 6; Featherstone (2008), Ch. 4.

**M2. Cubic / quintic polynomial joint trajectory.** Point-to-point motion with matched position/velocity(/acceleration) boundary conditions via closed-form polynomial coefficients.
- *Algorithm / paper:* Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 5.

**M3. Trapezoidal (LSPB) velocity profile.** Linear-segment-with-parabolic-blends profile respecting velocity/acceleration limits, with duration-constrained planning for multi-axis synchronization.
- *Algorithm / paper:* L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (2008), Ch. 3.

**M4. Friction compensation model.** Feedforward Coulomb + viscous + Stribeck joint-friction term, clamped, added to any torque controller or folded into the chain dynamics model.
- *Algorithm / paper:* B. Armstrong-Hélouvry, P. Dupont, C. Canudas de Wit, *Automatica*, 30(7), 1994.

**M5. PD + gravity compensation control.** The simplest globally-stable set-point regulator: `τ = Kp·e − Kd·q̇ + g(q)`.
- *Algorithm / paper:* M. Takegaki, S. Arimoto, *ASME J. Dyn. Sys. Meas. Control*, 1981.
- *Reuses / builds on:* `g(q)` from M29.

**M32. Inverse dynamics with an external tool wrench.** RNEA overload taking the environment's wrench on the tool; satisfies `τ(w) − τ(0) = −Jᵀw`.
- *Algorithm / paper:* Luh, Walker, Paul (1980).

### Tier 2 — Easy ★★☆☆☆

**M6. SE(3) transform, twists and wrenches.** Rigid transforms, adjoint, twist/wrench transforms, exponential/logarithm and `PoseError`; fixes the library's `(v; ω)` convention.
- *Algorithm / paper:* K. Lynch, F. Park, *Modern Robotics* (2017), Ch. 3; Murray, Li, Sastry (1994).
- *Reuses / builds on:* upstream `Geometry3D`, `Quaternion`.

**M28. Composite Rigid Body Algorithm.** `O(n²)` joint-space mass matrix, sharing the spatial algebra of ABA.
- *Algorithm / paper:* Walker & Orin (1982); Featherstone (2008), Ch. 6.

**M29. Chain dynamics model.** Adapter implementing `EulerLagrangeDynamics` (M via CRBA, `Cq̇` and `g` via RNEA) and a one-call `InverseDynamicsModel` for the controllers.
- *Reuses / builds on:* RNEA, M28.

**M30. Chain pose kinematics.** Full tool pose and a model-independent frame chain (joint origins/axes in the base frame) for the link model.
- *Algorithm / paper:* Siciliano et al. (2009), Ch. 2.

**M7. Denavit-Hartenberg parameters.** Standard and modified `(a, α, d, θ)` descriptions with joint offsets, producing the M30 frame chain.
- *Algorithm / paper:* Craig, *Introduction to Robotics*, Ch. 3.

**M8. Geometric Jacobian (6×N).** Jacobian and `J̇q̇` from any frame chain, statics `τ = Jᵀw`, and the `JacobianProvider` seam used by IK and the task-space controllers; replaces IK's private 3×N Jacobian.
- *Algorithm / paper:* Siciliano et al. (2009), Ch. 3; Lynch & Park (2017), Ch. 5.

**M9. S-curve (jerk-limited) trajectory.** Seven-segment jerk-bounded rest-to-rest profile with all short-move cases in closed form.
- *Algorithm / paper:* Biagiotti & Melchiorri (2008), Ch. 3.

**M10. Cartesian path + orientation interpolation.** Straight-line position with SLERP orientation, one shared time law, twist feed-forward.
- *Algorithm / paper:* Lynch & Park, Ch. 9; K. Shoemake, *SIGGRAPH* 1985.

**M11. Manipulability ellipsoid / Yoshikawa index.** `√det(JJᵀ)` (or `√det(JᵀJ)` when the task has more rows than joints), condition number and ellipsoid axes via SVD, translational and rotational parts separately.
- *Algorithm / paper:* T. Yoshikawa, *Int. J. Robotics Research*, 4(2), 1985.

**M33. Cubic spline through via points.** C² multi-joint spline with clamped or natural ends, Thomas-algorithm solve.
- *Algorithm / paper:* Biagiotti & Melchiorri (2008), Ch. 4.

**M34. Forward-dynamics integrator.** Fixed-step semi-implicit Euler / RK4 simulation step on ABA for model-in-the-loop tests.
- *Algorithm / paper:* Hairer, Lubich, Wanner (2006); Featherstone (2008), Ch. 7.

### Tier 3 — Moderate ★★★☆☆

**M12. Computed-torque (inverse-dynamics) control.** `τ = ID(q, q̇, q̈_d + Kd·ė + Kp·e)` in one `O(n)` call, yielding decoupled error dynamics.
- *Algorithm / paper:* Spong et al., *Robot Modeling and Control*, Ch. 8; Luh, Walker, Paul (1980).

**M13. Full 6-DOF pose IK.** Damped least squares on the SE(3) log-map error with position/rotation weighting, adaptive damping, step and joint-limit clamping.
- *Algorithm / paper:* S. R. Buss (2004); Nakamura & Hanafusa (1986); Chiaverini (1997).

**M14. Redundancy resolution / null-space projection.** `q̇ = J⁺_λ ẋ + (I − J⁺J)·q̇₀` with an exact (undamped, rank-thresholded) projector.
- *Algorithm / paper:* A. Liégeois, *IEEE Trans. SMC*, 7(12), 1977.

**M15. Product-of-Exponentials forward kinematics.** Screw-theory FK and the space Jacobian, with conversion to the geometric Jacobian.
- *Algorithm / paper:* Lynch & Park, *Modern Robotics*, Ch. 4.

**M16. Momentum-based collision-detection observer.** External joint-torque estimate from the generalized-momentum residual, using `Cᵀq̇`.
- *Algorithm / paper:* A. De Luca, A. Albu-Schäffer, S. Haddadin, G. Hirzinger, *IROS*, 2006.

**M17. Impedance control.** Stiffness/damping impedance via `Jᵀ` (no force sensor), and inertia-shaping impedance with measured contact wrench via the task-space inertia.
- *Algorithm / paper:* N. Hogan, *ASME J. Dyn. Sys. Meas. Control*, 1985.

**M31. Coriolis matrix and inertial regressor.** Modified RNEA giving the Christoffel `C(q,q̇)q̇_r` (skew-symmetric `Ṁ − 2C`), `Cᵀq̇`, and the 10-parameter-per-link regressor.
- *Algorithm / paper:* Echeandia & Wensing (2021); Niemeyer & Slotine (1991); Atkeson, An, Hollerbach (1986).

### Tier 4 — Advanced ★★★★☆

**M18. Operational-space control.** Task-space inertia `Λ = (J M⁻¹ Jᵀ)⁻¹`, `J̇q̇` compensation, full joint-space gravity/Coriolis compensation and a dynamically-consistent null space.
- *Algorithm / paper:* O. Khatib, *IEEE J. Robotics and Automation*, 3(1), 1987.

**M19. Hybrid position/force control.** Selection matrix in a constraint frame, PI force loop with anti-windup and damping, motion PD.
- *Algorithm / paper:* M. Raibert, J. Craig, 1981; An & Hollerbach, 1987.

**M20. Passivity-based adaptive control (Slotine-Li).** Tracks trajectories while estimating inertial parameters on-line through the regressor `Y(q,q̇,q̇r,q̈r)`.
- *Algorithm / paper:* J.-J. Slotine, W. Li, *Int. J. Robotics Research*, 6(3), 1987.

**M21. Analytical IK for ortho-parallel 6R arms with a spherical wrist (OPW).** Closed-form, all eight solutions, covering shoulder, lateral and elbow offsets of common industrial arms.
- *Algorithm / paper:* M. Brandstötter, A. Angerer, M. Hofbaur, Austrian Robotics Workshop, 2014; D. Pieper, PhD thesis, Stanford, 1968.

**M22. Dynamic (base-parameter) identification.** Streaming QR least squares on the regressor with SVD rank reduction to the base parameters.
- *Algorithm / paper:* Atkeson, An, Hollerbach (1986); Gautier & Khalil (1990).

**M23. Parallel-manipulator kinematics.** Stewart-Gough (leg-length IK, Newton FK) and Delta (per-arm closed-form IK, three-sphere FK).
- *Algorithm / paper:* J.-P. Merlet, *Parallel Robots*, 2nd ed. (2006); R. Clavel (1990).

**M24. Mobile-manipulator / nonholonomic-base kinematics.** Combined base + arm Jacobian in the world frame with differential-drive constraints.
- *Algorithm / paper:* Y. Yamamoto, X. Yun, *IEEE Trans. Automatic Control*, 39(6), 1994.

**M25. Cable-driven tension distribution.** Structure matrix from cable geometry and Pott's closed-form (improved) tension distribution within `[tMin, tMax]`.
- *Algorithm / paper:* Pott, Bruckmann, Mikelsons (2009); A. Pott (2014).

**M26. Continuum / soft constant-curvature kinematics.** Piecewise-constant-curvature FK and single-section IK.
- *Algorithm / paper:* R. Webster, B. Jones, *Int. J. Robotics Research*, 29(13), 2010.

### Tier 5 — Hard / research-grade ★★★★★

**M27. Time-optimal path parameterization (TOPP-RA).** Minimum-time traversal of a fixed path under joint velocity and torque limits via reachability analysis (two-variable LPs per grid stage).
- *Algorithm / paper:* J. Bobrow, S. Dubowsky, J. Gibson, *IJRR*, 4(3), 1985; H. Pham, Q.-C. Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis," *IEEE T-RO*, 34(3), 2018.

---
