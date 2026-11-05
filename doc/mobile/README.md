# Wheeled Mobile Robots

Kinematics, odometry and path tracking for ground vehicles moving in the plane.

## Algorithms

| Algorithm                                                  | Description                                                                                                                                               |
|------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Planar Odometry](PlanarOdometry.md)                       | Dead reckoning with the SE(2) exponential: exact arc integration of body twists, body-frame increments and increment noise — $O(1)$                       |
| [Differential-Drive Kinematics](DifferentialDrive.md)      | Two-wheel unicycle kinematics: forward and inverse maps, curvature-preserving wheel saturation and the speed envelope — $O(1)$                            |
| [Car-Like Kinematics](CarLikeKinematics.md)                | Kinematic bicycle twist at any reference point, steering for a curvature, angle and rate limits, and Ackermann wheel angles — $O(1)$                      |
| [Omnidirectional Kinematics](OmnidirectionalKinematics.md) | Swedish-wheel and mecanum Jacobian for any layout, least-squares forward kinematics, slip residual and uniform wheel saturation — $O(N)$                  |
| [Skid-Steer Kinematics](SkidSteerKinematics.md)            | ICR model for skid-steered and tracked vehicles with lateral slip, plus closed-form least-squares identification of the ICRs — $O(1)$                     |
| [Pure-Pursuit Path Tracking](PurePursuit.md)               | Look-ahead goal on a bounded polyline path with monotone progress and a curvature command for unicycles and cars; includes the planar path type — $O(w)$  |
| [Kanayama Trajectory Tracking](KanayamaController.md)      | Lyapunov-stable tracking of a reference pose and velocity for unicycle-type bases, with the error in the robot frame and critically damped gains — $O(1)$ |
