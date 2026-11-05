# Manipulator Control

Joint- and task-space controllers for robot manipulators.

## Algorithms

| Algorithm                                                        | Description                                                                                                                                                                   |
|------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [PD Control with Gravity Compensation](PdGravityCompensation.md) | Globally stable set-point regulator $\tau = K_p e - K_d \dot{q} + g(q)$ — needs only the gravity torque                                                                       |
| [Computed-Torque Control](ComputedTorqueControl.md)              | Feedback linearization: one inverse-dynamics pass at the commanded acceleration gives decoupled error dynamics $\ddot{e} + K_d\dot{e} + K_p e = 0$                            |
| [Impedance Control](ImpedanceControl.md)                         | Task-space mass–spring–damper behaviour: passive $J^T(Ke + D\dot{e}) + g$ without force sensing, or full inertia shaping through $\Lambda(q)$ and a measured wrench           |
| [Operational-Space Control](OperationalSpaceControl.md)          | Decoupled task-space tracking through $\Lambda(q)$, joint-space Coriolis/gravity compensation and a dynamically consistent null space for secondary torques                   |
| [Hybrid Position/Force Control](HybridPositionForceControl.md)   | Constraint-frame selection between a motion PD and a damped force PI with anti-windup, mapped through $J^T$ with gravity and Coriolis compensation                            |
| [Slotine–Li Adaptive Control](SlotineLiAdaptiveControl.md)       | Globally convergent tracking with unknown inertial parameters: regressor feedforward, sliding-variable damping and the passivity-based update $\dot{\hat{a}} = -\Gamma Y^T s$ |
