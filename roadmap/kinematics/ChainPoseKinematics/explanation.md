# Chain Pose Kinematics — Overview

## What it is
Forward kinematics that returns the full tool pose (position and orientation) of a serial arm
described link by link, together with every joint's axis and origin in the base frame.

## Why it matters (embedded)
Position-only kinematics is enough for reaching a point, but grasping, welding or keeping a camera
level needs orientation too, and every Jacobian-based algorithm needs the joint axes in one common
frame. Producing them in the same single pass that computes the positions keeps the cost linear in
the number of joints.

## How it works (intuition)
Walk the chain from the base: each joint's origin is the previous origin plus the previous
orientation applied to the link offset; each joint's axis is the previous orientation applied to the
axis as written in the link; then the joint's own rotation is folded into the running orientation.
After the last joint the constant tool frame is appended. The collected origins and axes form a
"frame chain" that any Jacobian routine can consume without knowing how the arm was described.

## Key parameters
- **Link offsets and joint axes** — the chain description.
- **Tool frame** — constant pose of the tool in the last link's frame.

## Reference
B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control* (2009),
Ch. 2 (direct kinematics).

## See also
`ForwardKinematics` (shipped, position-only), `SE3Transform` (M6), `GeometricJacobian` (M8).
