# EKF Landmark Localization (Known Map, Range–Bearing)

## Overview & Motivation

Odometry drifts without bound. To keep a robot localised over hours, its pose must be corrected against fixed features of the environment: reflectors on warehouse walls, fiducial markers, poles or trees.
When the positions of these **landmarks** are known and a sensor (laser scanner, camera, radar) measures their **range and bearing**, an extended Kalman filter tracks the pose accurately at very low cost.

Each motion step composes the odometry increment with noise that grows with distance and rotation. Each observation is matched to a landmark, either by a known identity or by the smallest Mahalanobis distance, then checked against a χ² gate and applied with the bearing residual wrapped. Matching is also called data association, and the gate is what rejects spurious detections.

## Mathematical Theory

### Motion Step

The state is the pose $x = (p_x, p_y, \theta)$. The odometry increment $\Delta = (d_x, d_y, d_\theta)$ in the body frame is composed onto the pose, $T \leftarrow T\,\Delta$. Its Jacobians with respect to the pose and to the increment are

$$F = \begin{pmatrix} 1 & 0 & -d_x\sin\theta - d_y\cos\theta \\ 0 & 1 & d_x\cos\theta - d_y\sin\theta \\ 0 & 0 & 1 \end{pmatrix}, \qquad J = \begin{pmatrix} R(\theta) & 0 \\ 0 & 1 \end{pmatrix}$$

The increment noise $\Sigma_\Delta = \mathrm{diag}(\sigma_{xy}^2, \sigma_{xy}^2, \sigma_\theta^2)$ comes from the odometry noise model, with deviations that grow with the distance and rotation travelled. Then

$$P \leftarrow F P F^T + J\,\Sigma_\Delta\,J^T$$

The noise is added in the body frame and rotated into the world, so forward and reverse motion are treated alike.

### Range–Bearing Measurement

For a landmark at $m$, with $d = m - (p_x, p_y)$ and $q = d^T d$:

$$\hat{z} = \begin{pmatrix} \sqrt{q} \\ \operatorname{atan2}(d_y, d_x) - \theta \end{pmatrix}, \qquad H = \begin{pmatrix} -d_x/\sqrt{q} & -d_y/\sqrt{q} & 0 \\ d_y/q & -d_x/q & -1 \end{pmatrix}$$

The residual is $r = (z_r - \hat{z}_r,\; \operatorname{wrap}(z_b - \hat{z}_b))$, with noise $R = \mathrm{diag}(\sigma_r^2, \sigma_b^2)$.

### Data Association and Gating

A labelled observation (a fiducial with an identity) uses its landmark directly. An unlabelled one is matched to the landmark with the smallest normalised innovation squared,

$$\text{NIS}_k = r_k^T \left(H_k P H_k^T + R\right)^{-1} r_k$$

and then gated against the χ²(2) quantile, for example $9.21$ at 99 %. The update uses the Joseph form, and observations taken at the same time are applied one after another.

## Complexity Analysis

| Operation         | Time   | Space  | Notes                                                       |
|-------------------|--------|--------|-------------------------------------------------------------|
| Predict           | $O(1)$ | $O(1)$ | Two $3 \times 3$ congruences                                |
| Labelled update   | $O(L)$ | $O(1)$ | Identity search over $L$ landmarks, one $2 \times 2$ update |
| Unlabelled update | $O(L)$ | $O(1)$ | One NIS per landmark, each a $2 \times 2$ Cholesky solve    |
| Map and state     | —      | $O(L)$ | Three scalars per landmark and 12 for the state             |

## Step-by-Step Walkthrough

Four landmarks at $(5, 0)$, $(0, 5)$, $(5, 5)$ and $(-3, 2)$. The estimate starts at $(1, 1, 0.3)$ with variances $(0.05, 0.05, 0.01)$, while the true pose is $(1.1, 0.9, 0.32)$. The noise is $\sigma_r = 0.1$ m and $\sigma_b = 0.02$ rad, with a 99 % gate of $9.21$:

1. **Predict**: an increment of $0.5$ m forward and $0.1$ rad moves the estimate to $(1.478, 1.148, 0.4)$ and grows the variances to $(0.0628, 0.0649, 0.013)$.
2. **Reverse motion**: from zero covariance, $0.5$ m backwards gives the same covariance as forwards, $(0.0125, 0.0125, 0.0025)$.
3. **Association**: an unlabelled observation of landmark 2 has NIS $0.002$ against it, against $119.6$, $118.1$ and $371.8$ for the others. It is matched to landmark 2 and moves the estimate to $(1.002, 0.996, 0.296)$.
4. **Convergence**: four labelled observations bring the estimate to $(1.093, 0.904, 0.319)$, close to the truth, and shrink the variances to $(0.0027, 0.0021, 0.00012)$.
5. **Outlier**: a detection at range $0.5$ and bearing $2.5$ is closest to landmark 3 with NIS $219$. It is gated, and the estimate is unchanged.
6. **Bearing seam**: a robot at $(6, 4.9)$ sees landmark 1 almost straight behind, at bearing $3.125$, and measures $-\pi + 0.01$. The wrapped residual is $0.027$ rad instead of $-6.26$, so the correction stays small (NIS $0.06$).

## Pitfalls & Edge Cases

- **Ambiguous association**: two landmarks inside the gate at once can be swapped. Prefer labelled observations, or space landmarks wider than the 3σ position uncertainty.
- **Bearing wrap**: an unwrapped bearing residual near $\pm\pi$ throws the filter off by a full turn. Always wrap.
- **Large initial uncertainty**: the linearisation fails when the heading is unknown. Initialise from a global method, such as Monte Carlo localization, first.
- **Unknown landmarks**: an identity missing from the map, or an empty map, returns a singular result and leaves the state unchanged.
- **Overconfidence**: too little motion noise makes the gate reject valid observations after a disturbance. Monitor the gating rate.

## Variants & Generalizations

- **EKF SLAM**: adding the landmark positions to the state to build the map while localising.
- **Unscented and particle filters**: for strongly nonlinear measurements or multimodal pose beliefs.
- **Joint compatibility association**: testing sets of matches together instead of one at a time.
- **Bearing-only and range-only variants**: for cameras without depth or radio beacons.

## Applications

- Automated guided vehicles navigating with wall-mounted reflectors.
- Robots localising with fiducial markers such as AprilTags.
- Agricultural and forestry robots using trees or poles as landmarks.

## Connections to Other Algorithms

- [Odometry Fusion](OdometryFusion.md): the wheel and gyroscope filter that can supply the odometry increments.
- [Planar Odometry](../mobile/PlanarOdometry.md): the increment convention and the odometry noise model.
- [Occupancy-Grid Mapping](OccupancyGridMap.md): a dense map representation, as opposed to sparse landmarks.
- The residual-form Kalman update, SE(2) transform and angle wrapping come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Leonard, J.J. and Durrant-Whyte, H.F. (1991). "Mobile robot localization by tracking geometric beacons." *IEEE Transactions on Robotics and Automation*, 7(3), 376–382.
- Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapter 7.
- Bar-Shalom, Y., Li, X.-R. and Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*. Wiley. Chapter 5.
