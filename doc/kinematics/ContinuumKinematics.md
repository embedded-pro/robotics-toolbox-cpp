# Continuum Kinematics (Constant Curvature)

## Overview & Motivation

Continuum robots have no discrete joints: tendon-driven endoscopes, concentric-tube needles, pneumatic soft arms and elephant-trunk manipulators bend continuously along their length. Rigid-link kinematics does not apply, because there is no finite set of joint axes.

The **piecewise constant-curvature** model describes each section as a circular arc, set by three numbers: curvature $\kappa$, bending-plane angle $\phi$ and arc length $s$. It captures the shape of tendon- and pressure-driven sections well, and gives closed-form forward and single-section inverse kinematics.
It also separates the **robot-independent** map $(\kappa, \phi, s) \to SE(3)$ from the **robot-specific** map from actuator values (tendon lengths, chamber pressures) to arc parameters. Only the latter changes between designs.

## Mathematical Theory

### One Section

The section starts at its base frame with tangent $\hat{z}$ and bends by a total angle $\theta = \kappa s$ in the plane at angle $\phi$ about $\hat{z}$. The tip pose is

$$R = R_z(\phi)\,R_y(\theta)\,R_z(-\phi), \qquad p = s\left(f_1(\theta)\begin{bmatrix}\cos\phi \\ \sin\phi \\ 0\end{bmatrix} + f_2(\theta)\,\hat{z}\right)$$

with

$$f_1(\theta) = \frac{1 - \cos\theta}{\theta} = \frac{2\sin^2(\theta/2)}{\theta}, \qquad f_2(\theta) = \frac{\sin\theta}{\theta}$$

$R$ rotates by $\theta$ about the bending axis $(-\sin\phi, \cos\phi, 0)$. Its third column is the tip tangent, equal to $dp/ds$ at the tip.

### Straight Limit

The textbook form $p = \frac{1}{\kappa}(1 - \cos\theta)\ldots$ divides by zero for a straight section. Written as $s\,f(\theta)$, the formula stays finite. Below $|\theta| = 10^{-2}$ the series

$$f_1 \approx \frac{\theta}{2} - \frac{\theta^3}{24}, \qquad f_2 \approx 1 - \frac{\theta^2}{6}$$

is accurate to better than $10^{-9}$, and the straight pose $(I, (0, 0, s))$ follows exactly at $\theta = 0$, independent of $\phi$.

### Multiple Sections

Sections are stacked by composing their transforms, each expressed in the previous section's tip frame:

$$T_{\text{tip}} = T_1\,T_2 \cdots T_n$$

### Single-Section Inverse

From a tip position $p$ in the section base frame, with $\rho = \sqrt{p_x^2 + p_y^2}$ and $L = \|p\|$, the chord identity $L = 2\sin(\theta/2)/\kappa$ gives

$$\phi = \operatorname{atan2}(p_y, p_x), \qquad \theta = 2\operatorname{atan2}(\rho, p_z), \qquad \kappa = \frac{2\rho}{L^2}, \qquad s = L\,\frac{\theta/2}{\sin(\theta/2)}$$

The result has $\kappa \ge 0$ and $\phi \in (-\pi, \pi]$. A negative curvature folds into the plane angle: $(-\kappa, \phi) \equiv (\kappa, \phi + \pi)$. The inverse holds for $\kappa s < 2\pi$.

## Complexity Analysis

| Operation          | Time   | Space  | Notes                                   |
|--------------------|--------|--------|-----------------------------------------|
| Section transform  | $O(1)$ | $O(1)$ | Two sine/cosine pairs and one 3×3 build |
| Forward kinematics | $O(n)$ | $O(1)$ | One SE(3) product per section           |
| Single-section IK  | $O(1)$ | $O(1)$ | Two `atan2`, two square roots, one sine |

## Step-by-Step Walkthrough

1. **Straight section** $(\kappa, \phi, s) = (0, 0, 1)$: pure translation to $(0, 0, 1)$.
2. **Quarter circle** $(\pi/2, 0, 1)$: radius $2/\pi$, so the tip is at $(0.6366, 0, 0.6366)$ with its tangent along $\hat{x}$. The chord is $2\sqrt{2}/\pi = 0.9003$.
3. **Bending plane** $\phi = \pi/2$: the same arc swings into the $yz$ plane, with the tip at $(0, 0.6366, 0.6366)$.
4. **General section** $(2, 0.7, 1.2)$, $\theta = 2.4$: tip $(0.6644, 0.5596, 0.3377)$, chord $\sin(1.2) = 0.9320$, tangent $(0.5166, 0.4351, -0.7374)$. Two such sections stacked reach $(0.3490, 0.2939, -0.4981)$.
5. **Inverse**: the tip of $(-1.5, 0.3, 0.8)$ returns $(1.5,\ 0.3 - \pi,\ 0.8)$, the same arc with the sign folded into $\phi$.

## Pitfalls & Edge Cases

- **Division by curvature**: the $1/\kappa$ form fails for straight sections. Use the $s\,f(\theta)$ form with its series.
- **Undefined plane angle**: for a straight tip, $\phi$ is arbitrary. The inverse returns $\phi = 0$ and $s = p_z$.
- **Full loops**: at $\theta = 2\pi$ the tip returns to the base, and the inverse is ambiguous. Keep $\kappa s < 2\pi$.
- **Model error**: gravity, tip loads and friction bend real sections off the arc. Use Cosserat-rod models when accuracy matters under load.
- **Multi-section inverse**: no closed form exists. Iterate on $(\kappa_i, \phi_i, s_i)$ with damped least squares.

## Variants & Generalizations

- **Variable curvature**: Cosserat-rod and Kirchhoff-rod models with distributed loads.
- **Concentric-tube robots**: precurved tubes rotated and translated relative to each other, with torsional coupling.
- **Actuator maps**: tendon lengths $\ell_i = s - \kappa\,r\cos(\phi - \alpha_i)$ for tendons at radius $r$ and angle $\alpha_i$.
- **Velocity kinematics**: the Jacobian of the arc map, for resolved-rate control of continuum sections.

## Applications

- Minimally invasive surgical instruments and steerable catheters.
- Soft robotic arms and grippers driven by pressure.
- Inspection snakes for turbines, pipes and confined spaces.

## Connections to Other Algorithms

- [Pose Inverse Kinematics](PoseInverseKinematics.md): the iterative solver pattern for multi-section inverse kinematics.
- [Product of Exponentials](ProductOfExponentials.md): the rigid-link counterpart built on the same SE(3) composition.
- The rotation and SE(3) composition come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Webster, R.J. III and Jones, B.A. (2010). "Design and kinematic modeling of constant curvature continuum robots: a review." *International Journal of Robotics Research*, 29(13), 1661–1683.
- Burgner-Kahrs, J., Rucker, D.C. and Choset, H. (2015). "Continuum robots for medical applications: a survey." *IEEE Transactions on Robotics*, 31(6), 1261–1280.
- Rucker, D.C. and Webster, R.J. III (2011). "Statics and dynamics of continuum robots with general tendon routing and external loading." *IEEE Transactions on Robotics*, 27(6), 1033–1044.
