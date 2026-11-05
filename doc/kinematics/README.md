# Kinematics

Geometry and motion algorithms for serial kinematic chains.

## Algorithms

| Algorithm                                             | Description                                                                                                            |
|-------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------|
| [Forward Kinematics](ForwardKinematics.md)            | Computes 3D joint positions and the tool point from joint angles for serial chains — $O(n)$ complexity                 |
| [Chain Pose Kinematics](ChainPoseKinematics.md)       | Full tool pose in SE(3) plus every joint origin and axis in the base frame (the frame chain) — $O(n)$                  |
| [Denavit-Hartenberg Parameters](DenavitHartenberg.md) | Standard and modified DH tables to tool pose and frame chain, with constant joint offsets — $O(n)$                     |
| [Geometric Jacobian](GeometricJacobian.md)            | $6 \times n$ Jacobian, bias acceleration $\dot{J}\dot{q}$ and statics $\tau = J^T w$ from any frame chain — $O(n)$     |
| [Manipulability Index](ManipulabilityIndex.md)        | Yoshikawa index, velocity-ellipsoid axes and condition number per unit-consistent Jacobian block — singularity monitor |
| [Inverse Kinematics](InverseKinematics.md)            | Solves joint angles that place the tool point on a target via Damped Least Squares — $O(k \cdot n)$ iterative          |
