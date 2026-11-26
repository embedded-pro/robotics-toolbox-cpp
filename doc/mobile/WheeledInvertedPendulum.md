# Wheeled Inverted Pendulum

## Overview & Motivation

A two-wheeled balancing robot is a body that pivots on the axle of two driven wheels. Left alone, it falls. The wheels must keep the axle under the centre of mass, and they also move the robot. One common wheel torque therefore controls both the tilt and the travel.

Controller design for this robot starts from a linear model about the upright pose. This page derives that model as a continuous-time state-space system. A consumer discretises it at the loop rate and computes gains, for example with a linear-quadratic regulator.

## Mathematical Theory

### Model and Parameters

The model has two rigid bodies. The body pivots freely about the axle. The wheels roll without slip on flat, level ground. Pitch and travel decouple from yaw, so the model splits into a planar pitch model and a yaw model.

| Symbol   | Meaning                                                 | Unit  |
|----------|---------------------------------------------------------|-------|
| $m$      | Body mass: everything that pivots about the axle        | kg    |
| $M$      | Mass of both wheels                                     | kg    |
| $\ell$   | Distance from the axle to the body centre of mass       | m     |
| $J$      | Body pitch inertia about its centre of mass             | kg·m² |
| $J_w$    | Inertia of one wheel about its axle, rotor included     | kg·m² |
| $J_\psi$ | Yaw inertia of body and wheels about the vertical axis  | kg·m² |
| $r$      | Wheel radius                                            | m     |
| $b$      | Track width between the contact points                  | m     |
| $g$      | Gravitational acceleration                              | m/s²  |
| $\theta$ | Body pitch from vertical, positive with the top forward | rad   |
| $x$      | Axle travel, positive forward                           | m     |
| $\psi$   | Yaw, positive counter-clockwise seen from above         | rad   |
| $\tau_c$ | Common torque on the wheels, $\tau_L + \tau_R$          | N·m   |
| $\tau_d$ | Differential torque on the wheels, $\tau_R - \tau_L$    | N·m   |

The motors act between body and wheels. A wheel torque therefore puts an equal and opposite torque on the body.

### Nonlinear Pitch Dynamics

With the body centre of mass at $(x + \ell\sin\theta,\ \ell\cos\theta)$, Lagrange's equations for $q = (x, \theta)$ give

$$\left(M + m + \frac{2J_w}{r^2}\right)\ddot{x} + m\ell\cos\theta\,\ddot{\theta} - m\ell\sin\theta\,\dot{\theta}^2 = \frac{\tau_c}{r}$$

$$m\ell\cos\theta\,\ddot{x} + \left(J + m\ell^2\right)\ddot{\theta} - m g\ell\sin\theta = -\tau_c$$

### Linear Pitch Model

About $\theta = 0$, take $\sin\theta \approx \theta$, $\cos\theta \approx 1$, and drop $\dot{\theta}^2\theta$. With

$$M_t = M + m + \frac{2J_w}{r^2}, \qquad J_t = J + m\ell^2, \qquad \Delta = M_t J_t - m^2\ell^2$$

the state $z = (\theta, \dot{\theta}, x, \dot{x})$ and the input $u = \tau_c$ give $\dot{z} = A z + B u$:

$$A = \begin{pmatrix} 0 & 1 & 0 & 0 \\ \dfrac{M_t m g \ell}{\Delta} & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 \\ -\dfrac{m^2\ell^2 g}{\Delta} & 0 & 0 & 0 \end{pmatrix}, \qquad B = \begin{pmatrix} 0 \\ -\dfrac{M_t + m\ell/r}{\Delta} \\ 0 \\ \dfrac{J_t/r + m\ell}{\Delta} \end{pmatrix}$$

$\Delta = (M + 2J_w/r^2) J_t + mJ$, which is positive for every physical robot. Only a point body on massless wheels gives $\Delta = 0$, and the model rejects it.

### Yaw Model

Each wheel turns at $\pm\dot{\psi}\,b/(2r)$ relative to the axle centre, so the wheel spin adds $J_w b^2/(2r^2)$ to the yaw inertia. The wheel torques give the forces $\tau_L/r$ and $\tau_R/r$ at $\pm b/2$. With the state $(\psi, \dot{\psi})$ and the input $\tau_d$:

$$\left(J_\psi + \frac{J_w b^2}{2r^2}\right)\ddot{\psi} = \frac{b}{2r}\,\tau_d$$

### Fall Time Constant

With the axle held still, the pitch equation is $J_t\ddot{\theta} = m g\ell\,\theta$. Its unstable root is $1/\tau$ with

$$\tau = \sqrt{\frac{J + m\ell^2}{m g \ell}}$$

The free pitch model has a faster unstable root, $\sqrt{M_t m g\ell/\Delta}$, because the wheels roll back as the body falls. Both are useful: $\tau$ bounds the time to catch a fall with the wheels held, and the free root is the open-loop pole of $A$.

## Complexity Analysis

| Operation      | Time   | Space  | Notes                                    |
|----------------|--------|--------|------------------------------------------|
| Pitch model    | $O(1)$ | $O(1)$ | Six non-zero entries, four over $\Delta$ |
| Yaw model      | $O(1)$ | $O(1)$ | Two entries                              |
| Time constants | $O(1)$ | $O(1)$ | One square root each                     |

## Step-by-Step Walkthrough

A small robot: $m = 0.8$ kg, $M = 0.1$ kg, $\ell = 0.1$ m, $J = 0.004$ kg·m², $J_w = 5\times10^{-5}$ kg·m², $r = 0.034$ m, $g = 9.81$ m/s².

1. $M_t = 0.9 + 1\times10^{-4}/0.001156 = 0.98651$ kg and $J_t = 0.004 + 0.008 = 0.012$ kg·m².
2. $\Delta = 0.98651 \times 0.012 - 0.0064 = 0.0054381$.
3. $A_{21} = 0.98651 \times 0.7848 / 0.0054381 = 142.37$ s⁻² and $A_{41} = -0.0064 \times 9.81 / 0.0054381 = -11.545$ m/s².
4. $B_2 = -(0.98651 + 2.3529)/0.0054381 = -614.09$ and $B_4 = (0.35294 + 0.08)/0.0054381 = 79.613$.
5. $\tau = \sqrt{0.012/0.7848} = 0.12366$ s. The free root is $\sqrt{142.37} = 11.932$ s⁻¹.
6. Yaw, with $J_\psi = 0.01$ kg·m² and $b = 0.145$ m: the effective inertia is $0.01 + 0.00045469 = 0.010455$ kg·m², and $B_2 = 0.145/(2 \times 0.034 \times 0.010455) = 203.96$.

## Pitfalls & Edge Cases

- **Inertia reference**: $J$ is about the body centre of mass. An inertia measured about the axle already holds $m\ell^2$. Subtract it first.
- **Wheel inertia**: include the motor rotor and the gearbox, reflected through the square of the gear ratio. For a small robot it is a large part of $M_t$.
- **Sign conventions**: positive $\tau_c$ drives the robot forward and pitches the body back. A wrong sign in the motor or the IMU mapping makes every gain destabilise the robot.
- **Range**: the linear model holds to about 15°. At 35° the small-angle error in $\sin\theta$ is above 6 %.
- **Degenerate input**: the model rejects a body mass, height, wheel radius, track width or gravity that is not positive. It also rejects a negative inertia or wheel mass, and $\Delta = 0$.
- **Not modelled**: slip, slopes, motor saturation, backlash, friction, flexible structure and yaw-pitch coupling at high turn rates.

## Variants & Generalizations

- **Voltage input**: add the motor model to map voltage to torque, with back-EMF as extra damping on $\dot{x}$ and $\dot{\theta}$.
- **Coupled model**: keep the centripetal coupling between yaw rate and pitch for fast turns.
- **Slope**: gravity along the slope moves the equilibrium pitch away from zero.

## Applications

- LQR and pole-placement design for self-balancing robots and personal transporters.
- Plant models for simulation and for state estimators.

## Connections to Other Algorithms

- [Differential-Drive Kinematics](DifferentialDrive.md): maps the yaw rate and the travel speed to wheel speeds. It uses the same $r$ and $b$.

## References & Further Reading

- F. Grasser, A. D'Arrigo, S. Colombi, A. Rufer, "JOE: A Mobile, Inverted Pendulum", IEEE Transactions on Industrial Electronics 49(1), 2002.
- S. Kim, S. Kwon, "Dynamic Modeling of a Two-wheeled Inverted Pendulum Balancing Mobile Robot", International Journal of Control, Automation and Systems 13, 2015.
- K. Ogata, *Modern Control Engineering*, 5th ed., cart-pole linearisation.
