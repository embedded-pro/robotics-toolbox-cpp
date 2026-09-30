# Dynamic Parameter Identification — Overview

## What it is
A procedure that estimates a robot's inertial parameters (masses, centers of mass, inertia tensors)
from torque and motion measurements, by exploiting the fact that rigid-body dynamics are linear in
those parameters.

## Why it matters (embedded)
Model-based control (gravity compensation, computed torque, impedance, collision detection) is only
as good as the model. CAD data rarely matches the real arm with cables, motors and grippers.
Identifying the parameters on the real robot — during commissioning or after a tool change — makes
every model-based controller more accurate.

## How it works (intuition)
For each recorded sample the dynamics give one linear equation per joint in the unknown parameters.
Stacking many samples gives an over-determined least-squares problem. Some parameters never affect
the torques (or only in fixed combinations), so the problem is rank-deficient; a rank-revealing
solution keeps only the identifiable combinations, the "base parameters". Processing samples one at a
time with orthogonal rotations keeps memory fixed and numerics well-conditioned.

## Key parameters
- **Excitation trajectory** — decides how well-conditioned the problem is.
- **Rank tolerance** — which singular values count as identifiable.

## Reference
C. Atkeson, C. An, J. Hollerbach, "Estimation of Inertial Parameters of Manipulator Loads and Links,"
*Int. J. Robotics Research*, 5(3), 1986; M. Gautier, W. Khalil, "Direct Calculation of Minimum Set of
Inertial Parameters of Serial Robots," *IEEE Trans. Robotics and Automation*, 6(3), 1990.

## See also
`CoriolisMatrixAndRegressor` (M31), `SlotineLiAdaptiveControl` (M20, on-line counterpart),
`FrictionCompensation` (M4).
