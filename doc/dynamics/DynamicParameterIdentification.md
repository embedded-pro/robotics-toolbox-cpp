# Dynamic Parameter Identification

## Overview & Motivation

Model-based control, collision observers and simulators all need the arm's inertial parameters: masses, centre-of-mass positions and inertia tensors. CAD values ignore cables, motors, gearboxes and paint, and they are often wrong by tens of percent.
**Dynamic identification** estimates them from data recorded on the real robot: joint positions, velocities, accelerations and torques along an exciting trajectory.

The rigid-body model is **linear in the inertial parameters**, so identification reduces to linear least squares. The difficulty is structural. Some parameters never affect the joint torques, and others act only in fixed combinations.
Only the **base parameters**, a minimal set of identifiable linear combinations, can be recovered. The solver must reveal that rank and report which combinations it identified.

## Mathematical Theory

### Linear Parametrization

With the inertial regressor $Y$ (ten parameters per link: mass, first moment, rotational inertia about the joint origin),

$$\tau = Y(q,\dot{q},\ddot{q})\,\pi, \qquad Y \in \mathbb{R}^{n \times 10n}$$

Stacking $N$ samples gives the overdetermined system

$$\underbrace{\begin{bmatrix} Y_1 \\ \vdots \\ Y_N \end{bmatrix}}_{W} \pi = \underbrace{\begin{bmatrix} \tau_1 \\ \vdots \\ \tau_N \end{bmatrix}}_{b}$$

### Rank Deficiency and Base Parameters

$W$ never has full column rank. Some columns are identically zero, for example inertia components about axes perpendicular to a planar arm's joints. Others are linear combinations of the rest: the mass of link 2 enters link 1's dynamics only through $m_2 l_1$ and $m_2 l_1^2$, which merge with link 1's first moment and inertia.
The identifiable subspace is the row space of $W$. Its dimension, the number of base parameters, is far below $10n$.

### Streaming Rank-Revealing Least Squares

Samples are folded into an upper-triangular factor $R$ by Givens rotations, so memory stays at $P^2 + P$ scalars ($P = 10n$) regardless of $N$:

$$W = Q R, \qquad Q^T b = (d;\ e)$$

At the end, a singular value decomposition $R = U \Sigma V^T$ reveals the rank: singular values below $\epsilon\,\sigma_{\max}$ are treated as zero. The minimum-norm estimate is

$$\hat{\pi} = \sum_{\sigma_i > \epsilon\sigma_{\max}} v_i \frac{u_i^T d}{\sigma_i}$$

The first $r$ rows of $V^T$ span the identifiable combinations. For each of them, $v_i^T \hat{\pi} = v_i^T \pi_{\text{true}}$ on noiseless data, while components outside that span are set to zero.

### Validation

The estimate is validated by predicting torques $\hat{\tau} = Y\hat{\pi}$ on held-out data. Predictions are exact even though $\hat{\pi}$ differs from $\pi_{\text{true}}$, because the difference lies in the null space of every regressor.

## Complexity Analysis

| Operation   | Time       | Space     | Notes                                                    |
|-------------|------------|-----------|----------------------------------------------------------|
| `AddSample` | $O(n P^2)$ | $O(P^2)$  | One regressor evaluation and $n$ Givens-row updates      |
| `Solve`     | $O(P^3)$   | $O(P^2)$  | One $P \times P$ singular value decomposition            |
| `Predict`   | $O(n P)$   | $O(nP)$   | One regressor evaluation and product                     |
| Storage     | —          | $P^2 + P$ | Independent of the number of samples; 6 DoF: about 14 KB |

## Step-by-Step Walkthrough

A planar two-link arm of uniform rods (masses $2$ and $1.5$ kg, lengths $1$ and $0.8$ m) follows a three-harmonic Fourier trajectory, sampled 200 times over one period. Torques come from the recursive Newton–Euler algorithm on the true parameters.

1. **Joint axes horizontal.** Gravity acts in the plane of motion. The stacked regressor has rank 6, matching the classical base set $ZZ_1^R = ZZ_1 + m_2 l_1^2$, $ZZ_2$, $MX_1^R = MX_1 + m_2 l_1$, $MY_1$, $MX_2$ and $MY_2$, with $Z$ along the joint axes.
2. **Joint axes vertical.** Gravity is parallel to the axes, so link 1's first moments no longer act: rank 4 ($ZZ_1^R$, $ZZ_2$, $MX_2$, $MY_2$).
3. **True values.** For the horizontal arm, $ZZ_1^R = 2/3 + 1.5 = 2.167$ kg·m², $MX_1^R = 1.0 + 1.5 = 2.5$ kg·m, $ZZ_2 = 0.32$ kg·m² and $MX_2 = 0.6$ kg·m. The projections of $\hat{\pi}$ onto the identified directions match those of $\pi_{\text{true}}$ to about $10^{-3}$ of $\|\pi\|$.
4. **Prediction.** On held-out samples at other times, $Y\hat{\pi}$ reproduces the torques to $10^{-3}$ relative.

## Pitfalls & Edge Cases

- **Poor excitation**: a slow or low-amplitude trajectory leaves directions nearly unobservable, so the rank collapses or noise dominates. Optimize the trajectory (e.g. a finite Fourier series) to minimize the condition number of the identified part.
- **Mixed units**: regressor columns mix masses, first moments and inertias. Scale them before accumulating so that a relative rank tolerance means the same thing for every column.
- **Differentiated signals**: velocities and accelerations from numerical differentiation are noisy. Filter $q$, $\dot{q}$, $\ddot{q}$ and $\tau$ with the same zero-phase low-pass filter offline.
- **Friction and armature**: unmodelled friction biases the estimate. Append friction columns ($\operatorname{sign}\dot{q}_i$, $\dot{q}_i$) and armature columns to the regressor.
- **Physical consistency**: the least-squares estimate may imply a negative mass or an invalid inertia tensor. Enforcing consistency needs a constrained (semidefinite) fit.
- **No data**: `Solve` returns no result before the first sample or after `Reset`.

## Variants & Generalizations

- **Weighted least squares**: weight each joint by its torque noise level.
- **Physically consistent identification**: semidefinite programming with linear-matrix-inequality constraints on each link's pseudo-inertia.
- **Energy and power models**: integrated forms that avoid accelerations.
- **Closed-loop identification**: instrumental-variable methods that remove the bias caused by feedback.
- **Payload identification**: the same formulation restricted to the last link's ten parameters.

## Applications

- Commissioning of model-based controllers (computed torque, passivity-based, impedance).
- Calibration of collision observers, whose thresholds depend on model accuracy.
- Accurate simulation and digital twins.
- Payload estimation after a tool change.

## Connections to Other Algorithms

- [Coriolis Matrix and Inertial Regressor](CoriolisMatrixAndRegressor.md): supplies the regressor $Y$.
- [Recursive Newton-Euler](RecursiveNewtonEuler.md): generates reference torques for validation.
- [Generalized-Momentum Observer](MomentumObserver.md): benefits directly from identified parameters.
- The streaming rank-revealing least squares comes from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Atkeson, C.G., An, C.H. and Hollerbach, J.M. (1986). "Estimation of inertial parameters of manipulator loads and links." *International Journal of Robotics Research*, 5(3), 101–119.
- Gautier, M. and Khalil, W. (1990). "Direct calculation of minimum set of inertial parameters of serial robots." *IEEE Transactions on Robotics and Automation*, 6(3), 368–373.
- Swevers, J., Ganseman, C., Tükel, D.B., De Schutter, J. and Van Brussel, H. (1997). "Optimal robot excitation and identification." *IEEE Transactions on Robotics and Automation*, 13(5), 730–740.
- Wensing, P.M., Kim, S. and Slotine, J.-J.E. (2018). "Linear matrix inequalities for physically consistent inertial parameter identification." *IEEE Robotics and Automation Letters*, 3(1), 60–67.
