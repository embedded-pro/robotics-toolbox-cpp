# Kinematics

Geometry and motion algorithms for serial kinematic chains.

## Algorithms

| Algorithm                                       | Description                                                                                                   |
|-------------------------------------------------|---------------------------------------------------------------------------------------------------------------|
| [Forward Kinematics](ForwardKinematics.md)      | Computes 3D joint positions and the tool point from joint angles for serial chains — $O(n)$ complexity        |
| [Chain Pose Kinematics](ChainPoseKinematics.md) | Full tool pose in SE(3) plus every joint origin and axis in the base frame (the frame chain) — $O(n)$         |
| [Inverse Kinematics](InverseKinematics.md)      | Solves joint angles that place the tool point on a target via Damped Least Squares — $O(k \cdot n)$ iterative |
