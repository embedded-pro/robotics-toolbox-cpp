# Artificial Potential Fields — Overview

## What it is
A reactive way to move a robot towards a goal while staying clear of obstacles. The goal attracts like a
spring, each nearby obstacle pushes back harder the closer it is, and the robot follows the sum of these
pushes and pulls.

## Why it matters (embedded)
Potential fields need no map search and no planning horizon: one evaluation per control cycle, a few
operations per nearby obstacle. That makes them a common last-line avoidance layer under a global planner,
on drones, mobile bases and manipulators alike, and easy to run on a microcontroller.

## How it works (intuition)
The attraction grows with distance to the goal up to a cap, so far goals do not overpower everything
else. Repulsion only starts inside an influence radius and rises steeply as the gap closes. The
resulting velocity is clamped to a safe speed. When an obstacle sits exactly between robot and goal the two
effects can cancel, and the robot stops in a local minimum; the method detects and reports this, so a
higher-level planner can supply a detour.

## Key parameters
- **attractive gain** and **conic distance** — pull strength and where it stops growing.
- **repulsive gain** and **influence distance** — how hard and from how far obstacles push.
- **maximum speed**, **stall speed** and **goal tolerance**.

## Reference
O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots," *IJRR*, 5(1), 1986.

## See also
`PrimitiveDistance` (P1, obstacle proximity), `GridSearch` (P3) and `RrtConnect` (P6, global plans that escape
local minima).
