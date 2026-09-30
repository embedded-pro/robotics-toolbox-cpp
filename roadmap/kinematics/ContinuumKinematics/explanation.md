# Continuum Kinematics — Overview

## What it is
Kinematics for arms that bend continuously instead of pivoting at discrete joints — tendon-driven,
pneumatic or "soft" robots shaped like an elephant's trunk. The standard model treats each section as
a **circular arc of constant curvature**, described by three numbers: how sharply it bends, in which
plane, and over what length.

## Why it matters (embedded)
Continuum robots reach into cluttered, delicate spaces — inside the body for surgery, around obstacles
for inspection — where a rigid arm cannot go. The constant-curvature model reduces a flexible body to
three parameters per section, small and cheap enough to evaluate on the embedded controller that
drives the tendons or air chambers.

## How it works (intuition)
Each section bows into a circular arc. The curvature `κ` sets how tight the arc is, the plane angle
`φ` which way it bends, and the arc length `s` how far along it runs; the total bend is `θ = κ·s`. The
tip lies on that circle and the tip frame is the base frame rotated by `θ` within the bending plane.
Writing the tip position as `s` times a function of `θ` avoids dividing by the curvature, so a nearly
straight section needs only a short series instead of a special case that blows up. Chaining section
transforms — like multiplying joint transforms on a rigid arm — gives the whole backbone. For one
section the inverse is closed form: the tip's direction gives the plane, the ratio of its sideways to
forward distance gives half the bend angle, and the straight-line distance to the tip — the chord
`2·sin(θ/2)/κ` — gives the curvature. The "robot-independent" arc map stays separate from the
"robot-specific" step that turns tendon pulls or chamber pressures into arc parameters.

## Key parameters
- **curvature `κ`** — inverse bend radius; its sign can be folded into the plane angle.
- **bending-plane angle `φ`** — bend direction (undefined when straight).
- **arc length `s`** — section length.

## Reference
R. J. Webster III, B. A. Jones, "Design and Kinematic Modeling of Constant Curvature Continuum Robots:
A Review," *Int. J. Robotics Research*, 29(13), 2010.

## See also
`SE3Transform` (M6, per-section transform and composition), `PoseInverseKinematics` (M13, the
iterative multi-section inverse), `Quaternion`
([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), orientation blending).
