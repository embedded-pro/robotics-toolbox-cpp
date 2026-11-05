# Manipulator Control

Joint- and task-space controllers for robot manipulators.

## Algorithms

| Algorithm                                                        | Description                                                                                             |
|------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------|
| [PD Control with Gravity Compensation](PdGravityCompensation.md) | Globally stable set-point regulator $\tau = K_p e - K_d \dot{q} + g(q)$ — needs only the gravity torque |
