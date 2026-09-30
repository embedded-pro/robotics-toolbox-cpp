# Redundancy Resolution — Overview

## What it is
A way to use a robot that has *more* joints than the task needs. A 7-DOF arm holding a 6-DOF pose — or
any arm tracking only a 3-D position — has spare freedom; redundancy resolution splits the joint motion
into a **primary** part that achieves the task and a **secondary** part, living in the Jacobian's null
space, that pursues an extra goal without disturbing the tool.

## Why it matters (embedded)
Extra joints let a robot dodge its own joint limits, avoid obstacles, or stay away from singularities
*while still doing the job*. The formula `q̇ = J⁺ẋ + (I − J⁺J)q̇₀` is cheap enough to run every control
tick — the difference between an arm that jams at a joint stop and one that reconfigures around it.

## How it works (intuition)
Factor the Jacobian into singular directions. The primary term inverts it along each direction it can
move the tool in — with a little damping so that a nearly-lost direction does not demand huge joint
rates — which gives the smallest joint motion producing (almost exactly) the desired tool velocity.
The projector removes from any secondary joint velocity every component the tool can feel; it must be
built from the *undamped*, rank-thresholded factorization, because a damped projector leaks a small
amount of motion into the task. Pick a secondary velocity — typically the downhill direction of a cost
such as distance to joint limits — project it, and add it on: the tool does not notice, and the spare
freedom does useful work.

## Key parameters
- **task rate `ẋ`** and **task dimension** (position only or full pose).
- **secondary rate `q̇₀`** — gradient of the objective optimized in the null space.
- **damping λ** — singularity robustness of the primary term (task error `≈ λ²/σ_min²`).
- **rank tolerance** — which singular values count as lost directions.

## Reference
A. Liégeois, "Automatic Supervisory Control of the Configuration and Behavior of Multibody
Mechanisms," *IEEE Trans. Systems, Man, and Cybernetics*, 7(12), 1977; S. Chiaverini, "Singularity-Robust
Task-Priority Redundancy Resolution," *IEEE Trans. Robotics and Automation*, 13(3), 1997.

## See also
`JacobianProvider` (M8, supplies `J`), `SingularValueDecomposition`
([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)), `ManipulabilityIndex`
(M11, a common secondary objective), `PoseInverseKinematics` (M13).
