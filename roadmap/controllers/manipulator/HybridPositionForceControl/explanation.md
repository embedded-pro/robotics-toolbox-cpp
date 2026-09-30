# Hybrid Position/Force Control — Overview

## What it is
A task-space controller that splits the end-effector's directions into two disjoint groups: some
axes are **position-controlled**, the rest are **force-controlled**. The axes are those of a
*constraint frame* attached to the contact surface; a diagonal selection matrix `S` decides which is
which, and its complement `I−S` handles the others.

## Why it matters (embedded)
Many contact tasks are naturally hybrid: sliding a tool on a surface, you want to *track a path*
tangentially while *regulating the normal force* — you cannot command both position and force on the
same axis, because the environment already fixes one of them. Real-time force regulation with clean
axis partitioning is essential for deburring, polishing, assembly, and grinding on resource-limited
controllers.

## How it works (intuition)
Rotate the pose error, velocity and forces into the constraint frame. Along motion axes a PD law pulls
the tool toward the reference path. Along force axes a PI law drives the measured contact force to the
desired force, with a little velocity damping to calm impacts and a clamped integral so it cannot wind
up when contact is lost. The two commands live in orthogonal subspaces, are summed, rotated back to the
base frame, and mapped to joint torques by `Jᵀ`, with gravity and Coriolis compensated. Because `S` and
`I−S` never overlap, the loops do not fight. Purely kinematic hybrid schemes can go unstable (An &
Hollerbach); the dynamic compensation here — or the fully decoupled operational-space form — avoids it.

## Key parameters
- **Rc (constraint rotation)** — orientation of the contact frame relative to the base.
- **S (selection matrix)** — which constraint axes are motion (1) vs force (0).
- **Kp, Kd** — motion-subspace position/velocity gains.
- **Kf, Ki, Kdf** — force-subspace proportional, integral and velocity-damping gains.
- **integralLimit, dt** — anti-windup clamp and sample period of the force integral.

## Reference
M. Raibert, J. Craig, "Hybrid Position/Force Control of Manipulators," *ASME J. Dyn. Sys. Meas.
Control*, 1981. C. An, J. Hollerbach, "Kinematic Stability Issues in Force Control of Manipulators,"
*Proc. IEEE ICRA*, 1987.

## See also
`ImpedanceControl` (M17, compliant unified motion/force), `OperationalSpaceControl` (M18,
dynamically decoupled task space), `GeometricJacobian` (M8, the `Jᵀ` wrench map).
