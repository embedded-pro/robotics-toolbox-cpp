# Mobile-Manipulator Kinematics — Overview

## What it is
Kinematics for an arm mounted on a differential-drive base — a robot that can both reach and roam. It
combines the arm's Jacobian with the base's motion into one map from "drive speed, turn rate and joint
rates" to the tool's linear and angular velocity in the world, respecting that wheels cannot slide
sideways.

## Why it matters (embedded)
Warehouse, service and field robots must coordinate base and arm as one system. A unified Jacobian
lets a single controller command the whole platform, and the extra freedom of "drive there *and*
reach" gives these robots their large workspace. Computing it on board each cycle needs a compact,
frame-consistent model.

## How it works (intuition)
The tool velocity is the sum of what the arm does on a stationary base and what the base does while
carrying a frozen arm. The arm's Jacobian is known in the arm's own base frame, so it is first rotated
into the world by the base heading and the mounting orientation. The base contributes two columns:
driving forward translates the tool along the heading, and turning swings it about the vertical axis
through the axle midpoint — the farther the tool from that axis, the faster it moves. Because a
differential drive cannot slip sideways, only these two base motions exist; there is no lateral
column to command. Base plus arm usually provides more freedom than the task needs, and null-space
resolution decides how much each contributes.

## Key parameters
- **base pose `(x, y, φ)`** and **controls `(v, ω)`** — axle-midpoint pose, drive speed and turn rate.
- **arm joints `q`** and the arm's Jacobian provider.
- **mount transform** — arm base in the mobile-base frame.
- **wheel base** — converts `(v, ω)` to wheel speeds.

## Reference
Y. Yamamoto, X. Yun, "Coordinating Locomotion and Manipulation of a Mobile Manipulator," *IEEE Trans.
Automatic Control*, 39(6), 1994.

## See also
`JacobianProvider` (M8, the arm block), `SE3Transform` (M6, pose composition), `RedundancyResolution`
(M14, splitting base vs arm).
