# Manipulator Control

Joint- and task-space controllers for robot manipulators.

## Algorithms

| Algorithm                                                        | Description                                                                                                                                                                   |
|------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [PD Control with Gravity Compensation](PdGravityCompensation.md) | Globally stable set-point regulator $\tau = K_p e - K_d \dot{q} + g(q)$ — needs only the gravity torque                                                                       |
| [Computed-Torque Control](ComputedTorqueControl.md)              | Feedback linearization: one inverse-dynamics pass at the commanded acceleration gives decoupled error dynamics $\ddot{e} + K_d\dot{e} + K_p e = 0$                            |
| [Impedance Control](ImpedanceControl.md)                         | Task-space mass–spring–damper behaviour: passive $J^T(Ke + D\dot{e}) + g$ without force sensing, or full inertia shaping through $\Lambda(q)$ and a measured wrench           |
| [Slotine–Li Adaptive Control](SlotineLiAdaptiveControl.md)       | Globally convergent tracking with unknown inertial parameters: regressor feedforward, sliding-variable damping and the passivity-based update $\dot{\hat{a}} = -\Gamma Y^T s$ |
