# Dynamics & Modeling

Rigid-body dynamics algorithms for robotics and mechanical systems.

## Algorithms

| Algorithm                                                               | Description                                                                                                                                |
|-------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md)               | $O(n)$ forward dynamics for serial chains — computes joint accelerations from applied torques                                              |
| [Chain Dynamics Model](ChainDynamicsModel.md)                           | Manipulator equation terms $M$, $C\dot{q}$, $g$ and one-pass inverse dynamics from a link chain, behind abstract interfaces                |
| [Composite Rigid Body Algorithm](CompositeRigidBodyAlgorithm.md)        | $O(n^2)$ joint-space mass matrix $M(q)$ from composite inertias, exactly symmetric                                                         |
| [Coriolis Matrix and Inertial Regressor](CoriolisMatrixAndRegressor.md) | Modified RNEA for $C(q,\dot{q})\dot{q}_r$ in Christoffel form, $C^T\dot{q}$, and the 10-parameter-per-link regressor $Y\pi$                |
| [Generalized-Momentum Observer](MomentumObserver.md)                    | Sensorless collision detection: a first-order estimate of the external joint torque from commanded torque, $M(q)$, $g(q)$ and $C^T\dot{q}$ |
| [Euler-Lagrange](EulerLagrange.md)                                      | Forward and inverse dynamics via the Lagrangian formulation $M\ddot{q} + C\dot{q} + g = \tau$                                              |
| [Forward-Dynamics Integration](ForwardDynamicsIntegrator.md)            | Fixed-step simulation of a chain on the articulated-body dynamics — semi-implicit Euler or RK4                                             |
| [Generic Joint Link](JointLink.md)                                      | Revolute or prismatic joints with limits and armature through one motion-subspace abstraction                                              |
| [Newton-Euler](NewtonEuler.md)                                          | Single rigid-body forward and inverse dynamics using Newton's and Euler's equations                                                        |
| [Recursive Newton-Euler](RecursiveNewtonEuler.md)                       | $O(n)$ inverse dynamics for serial kinematic chains via recursive velocity/force propagation, optionally with an external tool wrench      |
