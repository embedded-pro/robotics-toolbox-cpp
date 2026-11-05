# Manipulator Control

Joint- and task-space controllers for robot manipulators.

## Algorithms

| Algorithm                                                        | Description                                                                                                                                        |
|------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| [PD Control with Gravity Compensation](PdGravityCompensation.md) | Globally stable set-point regulator $\tau = K_p e - K_d \dot{q} + g(q)$ — needs only the gravity torque                                            |
| [Computed-Torque Control](ComputedTorqueControl.md)              | Feedback linearization: one inverse-dynamics pass at the commanded acceleration gives decoupled error dynamics $\ddot{e} + K_d\dot{e} + K_p e = 0$ |
