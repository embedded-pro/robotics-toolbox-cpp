# Skid-Steer Kinematics (Instantaneous Centres of Rotation)

## Overview & Motivation

Tracked vehicles and four-wheel skid-steered robots have no steering mechanism. They turn by driving the left and right sides at different speeds, which forces the treads or wheels to **skid** sideways. The ideal differential-drive model assumes rolling without slip, so it over-predicts the turn rate of these vehicles, often by a third or more.

The **instantaneous centre of rotation (ICR)** model keeps the linear structure of the differential drive but replaces the geometric half-track with identified ICR positions for each tread. It also adds a longitudinal offset of the vehicle's ICR, which produces the lateral slip velocity seen in turns. The three parameters can be identified from logged driving data with a closed-form least-squares fit.

## Mathematical Theory

### Tread ICRs

A body point $(x, y)$ moves at $(v_x - \omega y,\; v_y + \omega x)$. A tread driven at ground speed $v_t$ slides at $v_x - \omega y - v_t$, which vanishes at the tread's ICR line $y_t = (v_x - v_t)/\omega$. With identified left and right ICR positions $y_L > y_R$, sprocket radius $r$ and tread speeds $v_L = r s_L$, $v_R = r s_R$:

$$\omega = \frac{v_R - v_L}{y_L - y_R}, \qquad v_x = \frac{v_R\,y_L - v_L\,y_R}{y_L - y_R}, \qquad v_y = -x_{\text{ICR}}\,\omega$$

### Inverse Kinematics

$$s_L = \frac{v_x - \omega\,y_L}{r}, \qquad s_R = \frac{v_x - \omega\,y_R}{r}$$

The lateral velocity follows from the ICR offset and cannot be commanded.

### Ideal Case

With $y_L = B/2$, $y_R = -B/2$ and $x_{\text{ICR}} = 0$, the model reduces exactly to differential drive.

### Identification

Each logged sample, with sprocket speeds and an externally measured twist, gives three linear equations in the three parameters:

$$y_L\,\omega = v_x - v_L, \qquad y_R\,\omega = v_x - v_R, \qquad x_{\text{ICR}}\,\omega = -v_y$$

Their least-squares solution over all samples has a closed form, accumulated in four running sums:

$$y_L = \frac{\sum \omega(v_x - v_L)}{\sum \omega^2}, \qquad y_R = \frac{\sum \omega(v_x - v_R)}{\sum \omega^2}, \qquad x_{\text{ICR}} = -\frac{\sum \omega\,v_y}{\sum \omega^2}$$

Straight driving ($\omega = 0$) carries no information, so the estimate is only produced once $\sum\omega^2$ exceeds a threshold.

## Complexity Analysis

| Operation         | Time   | Space  | Notes                                  |
|-------------------|--------|--------|----------------------------------------|
| Forward / inverse | $O(1)$ | $O(1)$ | A few multiplications and one division |
| Add a sample      | $O(1)$ | $O(1)$ | Four running sums                      |
| Estimate          | $O(1)$ | $O(1)$ | Three divisions                        |
| State             | —      | $O(1)$ | Three ICR parameters, radius and limit |

## Step-by-Step Walkthrough

A tracked vehicle with sprocket radius $0.1$ m and identified ICRs $y_L = 0.30$, $y_R = -0.28$, $x_{\text{ICR}} = 0.05$ m:

1. **Ideal model**: with a geometric track width of $0.4$ m, sprockets at $(5, 8)$ rad/s would give $(0.65, 0, 0.75)$, matching differential drive.
2. **Identified model**: the same sprockets give $v_x = 0.655$ m/s, $v_y = -0.026$ m/s and $\omega = 0.517$ rad/s. The vehicle turns 31 % slower than the ideal prediction and slips sideways.
3. **Inverse**: $(0.655, 0.517)$ maps back to sprockets at $(5, 8)$ rad/s.
4. **Identification**: five consistent samples covering both turn directions recover $(0.30, -0.28, 0.05)$ exactly.
5. **Straight driving**: samples with equal sprocket speeds leave $\sum\omega^2 = 0$, and no estimate is produced.

## Pitfalls & Edge Cases

- **Terrain dependence**: the ICR positions change with surface, load and speed. Identify per terrain and reset the estimator when the surface changes.
- **Measurement source**: the identification needs a twist measured independently of the wheels, from GNSS, motion capture, a total station or scan matching.
- **Excitation**: identification needs turns in both directions. Straight-line logs make the fit ill-posed.
- **Lateral slip in trackers**: path trackers that assume zero lateral velocity see the slip as a disturbance. Odometry should integrate the full twist.
- **Long accumulations**: single-precision sums lose accuracy over millions of samples. Accumulate in blocks or reset periodically.

## Variants & Generalizations

- **Speed-dependent ICRs**: parameters scheduled on speed and turn rate.
- **Dynamic skid-steer models**: track–soil interaction forces for heavy vehicles.
- **Online identification**: recursive least squares or a Kalman filter tracking the ICRs while driving.

## Applications

- Tracked robots, construction and agricultural machinery.
- Four-wheel skid-steer research platforms and planetary rovers.
- Odometry correction for slippery terrain.

## Connections to Other Algorithms

- [Differential-Drive Kinematics](DifferentialDrive.md): the no-slip special case, and the shared curvature-preserving saturation.
- [Planar Odometry](PlanarOdometry.md): integrates the full twist, including lateral slip.

## References & Further Reading

- Martínez, J.L., Mandow, A., Morales, J., Pedraza, S. and García-Cerezo, A. (2005). "Approximating kinematics for tracked mobile robots." *International Journal of Robotics Research*, 24(10), 867–878.
- Mandow, A., Martínez, J.L., Morales, J., Blanco, J.L., García-Cerezo, A. and González, J. (2007). "Experimental kinematics for wheeled skid-steer mobile robots." *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 1222–1227.
- Siegwart, R., Nourbakhsh, I.R. and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press. Chapter 3.
