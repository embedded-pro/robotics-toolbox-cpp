# SE(3) Transform, Twists and Wrenches — Overview

## What it is
The algebra of rigid-body motion: a transform is a rotation plus a translation; a *twist* is a rigid
body's instantaneous velocity (linear and angular); a *wrench* is a force together with a moment. The
adjoint map moves twists and wrenches between frames, and the exponential/logarithm maps convert
between a constant twist held for some time and the finite motion it produces.

## Why it matters (embedded)
Every pose-level algorithm in the roadmap — full-pose forward kinematics, the 6×N Jacobian, pose
inverse kinematics, product-of-exponentials kinematics, Cartesian interpolation and task-space
control — needs the same small set of operations. Defining them once, with one ordering convention,
prevents the silent sign and ordering mismatches that otherwise appear when independently written
modules exchange 6-vectors.

## How it works (intuition)
A transform stores a 3×3 rotation and a 3-vector translation; composing two transforms rotates and
shifts the second by the first. A twist written in one frame is re-expressed in another with the
adjoint: rotate both parts, then add the "lever-arm" effect of the frame offset on the linear part.
Wrenches transform with the dual rule so that power (force times velocity) is the same in every
frame. The exponential turns "rotate about this line while sliding along it" into a finite motion —
Rodrigues' formula for the rotation plus a matching integral for the translation — and the logarithm
recovers that motion, which is exactly what pose-error feedback needs.

## Key parameters
- **Ordering convention** — linear part first for twists `(v; ω)` and wrenches `(f; n)`.
- **Singular regions** — small angles and angles near 180° in the logarithm.

## Reference
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 3; R. M. Murray, Z. Li, S. S. Sastry,
*A Mathematical Introduction to Robotic Manipulation* (1994), Ch. 2. (Both use `(ω; v)` ordering; this
library uses `(v; ω)`, which only permutes the blocks.)

## See also
`GeometricJacobian` (M8), `PoseInverseKinematics` (M13), `ProductOfExponentials` (M15),
`CartesianSlerpInterpolation` (M10), `ChainPoseKinematics` (M30).
