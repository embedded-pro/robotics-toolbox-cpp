# Robotics Toolbox — Roadmap

Prioritized backlog for the robot-manipulator stack (kinematics, dynamics, trajectories,
and manipulator control). Shared numerical primitives (`math`, `solvers`, `controllers`)
are consumed from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## Robot manipulators & other manipulator types

The library already has the hard parts of manipulator **modelling**: forward/inverse dynamics
([RNEA](robotics/dynamics/RecursiveNewtonEuler.hpp), [ABA](robotics/dynamics/ArticulatedBodyAlgorithm.hpp),
[Euler-Lagrange](robotics/dynamics/EulerLagrangeSolver.hpp) giving $M$, $C$, $g$) plus
damped-least-squares [IK](robotics/kinematics/InverseKinematics.hpp). What is missing is the
**control, planning, and full-pose kinematics layer** that turns those models into a usable
manipulator stack. Two current limitations gate most of the items below:

> **Position-only, revolute-only.** [ForwardKinematics.hpp](robotics/kinematics/ForwardKinematics.hpp)
> returns joint *positions* (a 3×N Jacobian lives privately inside IK), and only
> [RevoluteJointLink](robotics/dynamics/RevoluteJointLink.hpp) exists. Items **M1** (prismatic/generic
> joints) and **M8** (full 6×N spatial Jacobian) lift these limits and unblock the rest.

All manipulator items are *(float-first)* — torques, lengths, and inertias exceed the `Q15`/`Q31`
range, matching the existing `dynamics/` convention.

### Manipulator list (by priority)

| #   | Component                                               | Target module                   | Difficulty |
|-----|---------------------------------------------------------|---------------------------------|------------|
| M1  | Prismatic / generic joint link                          | `dynamics` + `kinematics`       | ★☆☆☆☆      |
| M2  | Cubic / quintic polynomial joint trajectory             | `trajectory` (new)              | ★☆☆☆☆      |
| M3  | Trapezoidal (LSPB) velocity profile                     | `trajectory` (new)              | ★☆☆☆☆      |
| M4  | Friction compensation (Coulomb + viscous + Stribeck)    | `dynamics`                      | ★☆☆☆☆      |
| M5  | PD + gravity compensation control                       | `controllers/manipulator` (new) | ★☆☆☆☆      |
| M6  | Homogeneous transform / SE(3) + adjoint (twists)        | `math`                          | ★★☆☆☆      |
| M7  | Denavit-Hartenberg parameters                           | `kinematics`                    | ★★☆☆☆      |
| M8  | Geometric / analytic Jacobian (6×N)                     | `kinematics`                    | ★★☆☆☆      |
| M9  | S-curve (jerk-limited) trajectory                       | `trajectory` (new)              | ★★☆☆☆      |
| M10 | Cartesian path + orientation (SLERP) interpolation      | `trajectory` (new)              | ★★☆☆☆      |
| M11 | Manipulability ellipsoid / Yoshikawa index              | `kinematics`                    | ★★☆☆☆      |
| M12 | Computed-torque (inverse-dynamics) control              | `controllers/manipulator` (new) | ★★★☆☆      |
| M13 | Full 6-DOF pose IK (position + orientation)             | `kinematics`                    | ★★★☆☆      |
| M14 | Redundancy resolution / null-space projection           | `kinematics`                    | ★★★☆☆      |
| M15 | Product-of-Exponentials forward kinematics              | `kinematics`                    | ★★★☆☆      |
| M16 | Momentum-based collision-detection observer             | `estimators/online`             | ★★★☆☆      |
| M17 | Impedance / admittance control                          | `controllers/manipulator` (new) | ★★★☆☆      |
| M18 | Operational-space (task-space) control                  | `controllers/manipulator` (new) | ★★★★☆      |
| M19 | Hybrid position/force control                           | `controllers/manipulator` (new) | ★★★★☆      |
| M20 | Passivity-based adaptive control (Slotine-Li)           | `controllers/manipulator` (new) | ★★★★☆      |
| M21 | Analytical IK (Pieper, wrist-partitioned 6R)            | `kinematics`                    | ★★★★☆      |
| M22 | Dynamic (base-parameter) identification                 | `estimators/offline`            | ★★★★☆      |
| M23 | Parallel-manipulator kinematics (Delta / Stewart-Gough) | `kinematics`                    | ★★★★☆      |
| M24 | Mobile-manipulator / nonholonomic-base kinematics       | `kinematics`                    | ★★★★☆      |
| M25 | Cable-driven tension distribution                       | `controllers/manipulator` (new) | ★★★★☆      |
| M26 | Continuum / soft constant-curvature kinematics          | `kinematics`                    | ★★★★☆      |
| M27 | Time-optimal path parameterization (TOPP)               | `trajectory` (new)              | ★★★★★      |

### Tier 1 — Trivial ★☆☆☆☆

**M1. Prismatic / generic joint link.** Generalize `RevoluteJointLink` to a joint type carrying an axis + type (revolute/prismatic), so FK/IK/dynamics handle sliding joints (SCARA, gantries, hydraulic actuators).
- *Algorithm / paper:* J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed., Ch. 3.
- *Reuses / builds on:* [RevoluteJointLink.hpp](robotics/dynamics/RevoluteJointLink.hpp); unblocks FK, IK, RNEA for mixed chains.

**M2. Cubic / quintic polynomial joint trajectory.** Point-to-point motion with matched position/velocity(/acceleration) boundary conditions via closed-form polynomial coefficients.
- *Algorithm / paper:* Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 5 (polynomial trajectories).
- *Reuses / builds on:* scalar `math`; new `trajectory/` module.

**M3. Trapezoidal (LSPB) velocity profile.** Linear-segment-with-parabolic-blends profile respecting velocity/acceleration limits.
- *Algorithm / paper:* L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (2008), Ch. 3.
- *Reuses / builds on:* new `trajectory/` module.

**M4. Friction compensation model.** Feedforward Coulomb + viscous + Stribeck joint-friction term added to any torque controller.
- *Algorithm / paper:* B. Armstrong-Hélouvry, P. Dupont, C. Canudas de Wit, "A survey of models, analysis tools and compensation methods for the control of machines with friction," *Automatica*, 30(7), 1994.
- *Reuses / builds on:* `dynamics`; composes with M5/M12.

**M5. PD + gravity compensation control.** The simplest globally-stable set-point regulator: `τ = Kp·e − Kd·q̇ + g(q)`.
- *Algorithm / paper:* M. Takegaki, S. Arimoto, "A New Feedback Method for Dynamic Control of Manipulators," *ASME J. Dyn. Sys. Meas. Control*, 1981.
- *Reuses / builds on:* $g(q)$ from [RNEA](robotics/dynamics/RecursiveNewtonEuler.hpp) / Euler-Lagrange; new `controllers/manipulator/` module.

### Tier 2 — Easy ★★☆☆☆

**M6. Homogeneous transform / SE(3) + adjoint.** Rigid-body transforms (4×4), twists/wrenches (6-vectors), and the adjoint map — the algebra all modern manipulator code is built on.
- *Algorithm / paper:* K. Lynch, F. Park, *Modern Robotics* (2017), Ch. 3; Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation* (1994).
- *Reuses / builds on:* [Geometry3D.hpp](numerical/math/Geometry3D.hpp), item 18 (Quaternion).

**M7. Denavit-Hartenberg parameters.** Standard `(a, α, d, θ)` link description and per-joint transform generation.
- *Algorithm / paper:* Craig, *Introduction to Robotics*, Ch. 3 (DH convention).
- *Reuses / builds on:* M6, `math::Matrix`.

**M8. Geometric / analytic Jacobian (6×N).** Full spatial Jacobian mapping joint rates → end-effector linear + angular velocity (and its transpose for force mapping).
- *Algorithm / paper:* Lynch & Park, *Modern Robotics*, Ch. 5 (velocity kinematics).
- *Reuses / builds on:* promotes the private 3×N Jacobian in [InverseKinematics.hpp](robotics/kinematics/InverseKinematics.hpp); unblocks M11, M13, M14, M17, M18.

**M9. S-curve (jerk-limited) trajectory.** Seven-segment jerk-bounded profile for smooth, low-vibration motion.
- *Algorithm / paper:* Biagiotti & Melchiorri, *Trajectory Planning*, Ch. 3 (double-S profiles).
- *Reuses / builds on:* M3; new `trajectory/` module.

**M10. Cartesian path + orientation interpolation.** Straight-line/screw position paths with SLERP orientation blending for task-space moves.
- *Algorithm / paper:* Lynch & Park, Ch. 9; K. Shoemake, SLERP, *SIGGRAPH* 1985.
- *Reuses / builds on:* item 18 (Quaternion), M6.

**M11. Manipulability ellipsoid / Yoshikawa index.** Scalar dexterity/singularity measure `√det(J Jᵀ)` for posture optimization and singularity avoidance.
- *Algorithm / paper:* T. Yoshikawa, "Manipulability of Robotic Mechanisms," *Int. J. Robotics Research*, 4(2), 1985.
- *Reuses / builds on:* M8, item 43 (SVD) or determinant of `math::Matrix`.

### Tier 3 — Moderate ★★★☆☆

**M12. Computed-torque (inverse-dynamics) control.** Feedback-linearizing manipulator law `τ = M(q)(q̈_d + Kd·ė + Kp·e) + C(q,q̇)q̇ + g(q)` yielding decoupled error dynamics.
- *Algorithm / paper:* Spong et al., *Robot Modeling and Control*, Ch. 8; Luh, Walker, Paul (1980).
- *Reuses / builds on:* **directly leverages existing [RNEA](robotics/dynamics/RecursiveNewtonEuler.hpp) / [Euler-Lagrange](robotics/dynamics/EulerLagrangeSolver.hpp)** for $M$, $C$, $g$; item 40 (feedback linearization).

**M13. Full 6-DOF pose IK.** Extend damped-least-squares IK to a position **and** orientation target using the 6×N Jacobian and a quaternion/log orientation error.
- *Algorithm / paper:* S. R. Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods," 2004; Nakamura & Hanafusa (1986).
- *Reuses / builds on:* [InverseKinematics.hpp](robotics/kinematics/InverseKinematics.hpp), M8, item 18.

**M14. Redundancy resolution / null-space projection.** Exploit extra DOF (7-DOF arms) via `q̇ = J⁺ẋ + (I − J⁺J)·q̇₀` for secondary objectives (joint-limit / obstacle avoidance).
- *Algorithm / paper:* A. Liégeois, "Automatic supervisory control of the configuration and behavior of multibody mechanisms," *IEEE Trans. SMC*, 7(12), 1977.
- *Reuses / builds on:* M8, item 27 (QR) / 43 (SVD) for the pseudo-inverse.

**M15. Product-of-Exponentials forward kinematics.** Screw-theory FK (`T = e^{[S₁]θ₁}···e^{[Sₙ]θₙ}·M`), avoiding DH frame bookkeeping.
- *Algorithm / paper:* Lynch & Park, *Modern Robotics*, Ch. 4.
- *Reuses / builds on:* M6 (SE(3)/twists), item 29 (matrix exponential).

**M16. Momentum-based collision-detection observer.** Estimate external joint torques from generalized-momentum residual — no joint-torque sensors or acceleration needed.
- *Algorithm / paper:* A. De Luca, A. Albu-Schäffer, S. Haddadin, G. Hirzinger, "Collision Detection and Safe Reaction with the DLR-III Lightweight Manipulator Arm," *IROS*, 2006.
- *Reuses / builds on:* RNEA, `estimators/online`.

**M17. Impedance / admittance control.** Render a programmable mass-spring-damper at the end-effector for safe contact and compliant assembly.
- *Algorithm / paper:* N. Hogan, "Impedance Control: An Approach to Manipulation, Parts I–III," *ASME J. Dyn. Sys. Meas. Control*, 1985.
- *Reuses / builds on:* M8, M12, `dynamics`; new `controllers/manipulator/` module.

### Tier 4 — Advanced ★★★★☆

**M18. Operational-space (task-space) control.** Control directly in Cartesian space using the task-space inertia `Λ = (J M⁻¹ Jᵀ)⁻¹` and dynamically-consistent null-space projection.
- *Algorithm / paper:* O. Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation," *IEEE J. Robotics and Automation*, 3(1), 1987.
- *Reuses / builds on:* M8, M12, item 28 (LU) for the $M^{-1}$ solve.

**M19. Hybrid position/force control.** Partition task directions into force-controlled and motion-controlled subspaces via a selection matrix.
- *Algorithm / paper:* M. Raibert, J. Craig, "Hybrid Position/Force Control of Manipulators," *ASME J. Dyn. Sys. Meas. Control*, 1981.
- *Reuses / builds on:* M8, M17, `controllers/manipulator/`.

**M20. Passivity-based adaptive control (Slotine-Li).** Track trajectories while online-estimating inertial parameters, exploiting linearity-in-parameters `Y(q,q̇,q̈)·a = τ`.
- *Algorithm / paper:* J.-J. Slotine, W. Li, "On the Adaptive Control of Robot Manipulators," *Int. J. Robotics Research*, 6(3), 1987.
- *Reuses / builds on:* RNEA regressor form, `estimators/online`; item 47 (MRAC) kinship.

**M21. Analytical IK (Pieper, wrist-partitioned 6R).** Closed-form inverse kinematics for the common 6R arm with a spherical wrist (all real solutions, no iteration).
- *Algorithm / paper:* D. Pieper, "The Kinematics of Manipulators Under Computer Control," PhD thesis, Stanford, 1968.
- *Reuses / builds on:* M6/M7, item 23 (CORDIC) or trig for the closed-form angles.

**M22. Dynamic (base-parameter) identification.** Least-squares estimation of link inertial parameters from excitation trajectories via the linear regressor.
- *Algorithm / paper:* C. Atkeson, C. An, J. Hollerbach, "Estimation of Inertial Parameters of Manipulator Loads and Links," *Int. J. Robotics Research*, 5(3), 1986.
- *Reuses / builds on:* RNEA regressor, item 27 (QR) / 12 (poly LS), `estimators/offline`.

**M23. Parallel-manipulator kinematics (Delta / Stewart-Gough).** Closed-form inverse kinematics and iterative forward kinematics for parallel platforms (pick-and-place Delta, 6-DOF hexapods).
- *Algorithm / paper:* J.-P. Merlet, *Parallel Robots*, 2nd ed. (2006); R. Clavel, delta robot (1990).
- *Reuses / builds on:* M6, item 28 (LU) / Newton iteration for the forward solve.

**M24. Mobile-manipulator / nonholonomic-base kinematics.** Combined base + arm Jacobian with nonholonomic (differential-drive) constraints.
- *Algorithm / paper:* Y. Yamamoto, X. Yun, "Coordinating Locomotion and Manipulation of a Mobile Manipulator," *IEEE Trans. Automatic Control*, 39(6), 1994.
- *Reuses / builds on:* M8, M14 (redundancy).

**M25. Cable-driven tension distribution.** Compute non-negative cable tensions realizing a desired wrench (cable robots, tendon-driven hands) via a bounded QP/LP.
- *Algorithm / paper:* T. Bruckmann, A. Pott (eds.), *Cable-Driven Parallel Robots* (2013); Pott tension-distribution methods.
- *Reuses / builds on:* [MPC](numerical/controllers/implementations/Mpc.hpp) QP machinery, M8.

**M26. Continuum / soft constant-curvature kinematics.** Piecewise-constant-curvature FK/IK for tendon/pneumatic continuum arms.
- *Algorithm / paper:* R. Webster, B. Jones, "Design and Kinematic Modeling of Constant Curvature Continuum Robots: A Review," *Int. J. Robotics Research*, 29(13), 2010.
- *Reuses / builds on:* M6 (SE(3)), item 18 (Quaternion).

### Tier 5 — Hard / research-grade ★★★★★

**M27. Time-optimal path parameterization (TOPP).** Minimum-time traversal of a fixed geometric path subject to joint torque/velocity limits.
- *Algorithm / paper:* J. Bobrow, S. Dubowsky, J. Gibson, "Time-Optimal Control of Robotic Manipulators Along Specified Paths," *IJRR*, 4(3), 1985; Q.-C. Pham, TOPP-RA, *IEEE T-RO*, 2014.
- *Reuses / builds on:* RNEA (torque limits along path), M10 (path), new `trajectory/` module.

---
