# Dynamic Window Approach — Overview

## What it is
A local obstacle-avoidance method for wheeled robots. Every control cycle it decides which forward speed
and turn rate to use next by testing many candidates against the robot's acceleration limits and the
obstacles its sensors currently see. It keeps the candidate that heads towards the goal, stays clear of
obstacles and moves briskly.

## Why it matters (embedded)
Service robots, AGVs and robot vacuums must react to people and clutter within a fraction of a second,
on modest processors. The dynamic window reasons directly in the velocity space the motors accept, so
every command it returns is reachable within the next cycle and leaves room to brake. It runs in a fixed
time set by the number of samples and sensor points.

## How it works (intuition)
Holding a speed and turn rate constant moves a wheeled robot along a circular arc. The method samples a
small grid of speed/turn-rate pairs reachable from the current motion in one control period. For each
it finds, exactly, how far the robot could travel along that arc before touching any obstacle point, and
discards the pairs that could not brake to a stop within that distance. The survivors are scored on how
well they point at the goal after a short look-ahead, how much free space they leave, and how fast they
go. The best score wins. If nothing survives, the robot is moving too fast for the situation and must
brake.

## Key parameters
- **speed, turn-rate and acceleration limits**, and the **control period** (window size).
- **robot radius** (with safety margin) and **clearance cap**.
- **weights** of the heading, clearance and speed terms; **prediction time** for the heading term.
- **grid size** `NumV × NumW` (compile-time).

## Reference
D. Fox, W. Burgard, S. Thrun, "The dynamic window approach to collision avoidance," *IEEE Robotics &
Automation Magazine*, 4(1), 1997.

## See also
`DifferentialDrive` (W1), `PurePursuit` (W5, path for intermediate goals), `math::SE2Transform` (NT66).
