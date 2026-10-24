# Kanayama Trajectory Tracking — Overview

## What it is
A controller that makes a two-wheeled (or tracked) robot follow a moving reference, a "ghost" robot
driving the planned trajectory with known speed and turn rate. It outputs the speed and turn rate that
bring the real robot onto the ghost and keep it there.

## Why it matters (embedded)
Time-parameterised tracking is what coordinated fleets, docking and trajectory-optimised motion need:
the robot must be at the right place at the right time, not just on the right path. The law is a few
trigonometric operations per cycle and comes with a stability proof, so its behaviour is predictable
on any trajectory that keeps moving.

## How it works (intuition)
The error is measured from the robot's own point of view: how far the ghost is ahead, how far to the
side, and how much its heading differs. Being behind raises the speed; the ghost being off to one side
or pointing differently adds turning, in proportion to how fast the ghost moves. The robot cannot slide
sideways, so it closes a sideways gap by steering towards it while moving. A carefully chosen energy-like
quantity can be shown never to increase, which guarantees convergence as long as the reference keeps
moving.

## Key parameters
- **along-track gain** `kx`, **cross-track gain** `ky`, **heading gain** `kθ` (critically damped when
  `kθ = 2√ky`).
- **reference speed and turn rate** — the feedforward from the trajectory.

## Reference
Y. Kanayama, Y. Kimura, F. Miyazaki, T. Noguchi, "A stable tracking control method for an autonomous
mobile robot," *ICRA* 1990.

## See also
`DifferentialDrive` (W1, saturation), `PurePursuit` (W5, path rather than trajectory tracking),
`math::SE2Transform` (NT66).
