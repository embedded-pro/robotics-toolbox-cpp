# Pose Inverse Kinematics — Overview

## What it is
Given a desired **pose** — position *and* orientation — of the tool, find joint values that reach it.
It extends position-only IK to all six task dimensions by driving a 6-vector error (three for
translation, three for rotation) to zero with weighted, damped least-squares Newton steps.

## Why it matters (embedded)
Real tasks are pose tasks: point a welding torch, align a gripper, hold a camera level. A numerically
robust solver that survives singularities without commanding huge joint jumps, respects joint limits
and runs a bounded number of iterations is what makes an arm safe to drive from a real-time loop,
warm-started from the previous cycle.

## How it works (intuition)
Each step linearizes the arm with the Jacobian and asks "what joint change best cancels the current
pose error?" The plain least-squares answer explodes near singularities; adding `λ²I` inside the
inverse keeps the step bounded. Because the method *iterates*, damping does not leave an error
behind: whenever the target is reachable and the arm is not singular, the only fixed point is zero
error — damping just makes each step shorter. Adaptive damping switches it on only when the
manipulability drops, so steps are full Newton steps elsewhere. Two details keep the error honest:
rotation is measured with the true logarithm of the relative rotation (always the short way, exact
angle), and radians are converted to metres with a characteristic length so that position and
orientation are weighed fairly in both the step and the stopping test.

## Key parameters
- **damping λ₀ and threshold w₀** — maximum damping and the manipulability below which it engages.
- **characteristic length ρ** — how many metres one radian of orientation error is worth.
- **tolerance, max iterations, max step, joint limits** — stopping rule and safety bounds.
- **seed `q0`** — warm-starting from the last cycle keeps iteration counts low.

## Reference
Y. Nakamura, H. Hanafusa, "Inverse Kinematic Solutions with Singularity Robustness for Robot
Manipulator Control," *ASME J. Dyn. Sys. Meas. Control*, 108, 1986; S. Chiaverini, "Singularity-Robust
Task-Priority Redundancy Resolution," *IEEE Trans. Robotics and Automation*, 13(3), 1997;
S. R. Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped
Least Squares methods," 2004.

## See also
`JacobianProvider` / `GeometricJacobian` (M8, the linearization), `SE3Transform::PoseError` (M6, the
orientation error), `ManipulabilityIndex` (M11), `AnalyticalIkOpw` (M21, the closed-form alternative
for ortho-parallel 6R arms), `LuDecomposition`
([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), the 6×6 solve).
