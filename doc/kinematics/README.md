# Kinematics

Geometry and motion algorithms for serial kinematic chains.

## Algorithms

| Algorithm                                             | Description                                                                                                                                                        |
|-------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Forward Kinematics](ForwardKinematics.md)            | Computes 3D joint positions and the tool point from joint angles for serial chains — $O(n)$ complexity                                                             |
| [Chain Pose Kinematics](ChainPoseKinematics.md)       | Full tool pose in SE(3) plus every joint origin and axis in the base frame (the frame chain) — $O(n)$                                                              |
| [Denavit-Hartenberg Parameters](DenavitHartenberg.md) | Standard and modified DH tables to tool pose and frame chain, with constant joint offsets — $O(n)$                                                                 |
| [Geometric Jacobian](GeometricJacobian.md)            | $6 \times n$ Jacobian, bias acceleration $\dot{J}\dot{q}$ and statics $\tau = J^T w$ from any frame chain — $O(n)$                                                 |
| [Manipulability Index](ManipulabilityIndex.md)        | Yoshikawa index, velocity-ellipsoid axes and condition number per unit-consistent Jacobian block — singularity monitor                                             |
| [Product of Exponentials](ProductOfExponentials.md)   | Screw-axis forward kinematics, space Jacobian and its conversion to the geometric Jacobian — $O(n)$                                                                |
| [Continuum Kinematics](ContinuumKinematics.md)        | Piecewise constant-curvature arcs $(\kappa, \phi, s)$: singularity-free section transform, multi-section forward kinematics and closed-form single-section inverse |
| [Redundancy Resolution](RedundancyResolution.md)      | Damped pseudo-inverse primary task plus exact null-space projection of a secondary objective for redundant arms                                                    |
| [Inverse Kinematics](InverseKinematics.md)            | Solves joint angles that place the tool point on a target via Damped Least Squares — $O(k \cdot n)$ iterative                                                      |
| [Pose Inverse Kinematics](PoseInverseKinematics.md)   | Full 6-DoF pose IK: damped least squares on the weighted log-map pose error with manipulability-adaptive damping, step clamping and joint limits                   |
