# Kinematics

Geometry and motion algorithms for serial kinematic chains.

## Algorithms

| Algorithm                                  | Description                                                                                                   |
|--------------------------------------------|---------------------------------------------------------------------------------------------------------------|
| [Forward Kinematics](ForwardKinematics.md) | Computes 3D joint positions and the tool point from joint angles for serial chains — $O(n)$ complexity        |
| [Inverse Kinematics](InverseKinematics.md) | Solves joint angles that place the tool point on a target via Damped Least Squares — $O(k \cdot n)$ iterative |
