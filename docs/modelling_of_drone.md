# Modelling of drone

For any kind of control system, it is essential to have a good representation of the plant (the system to control) in order to determine the best strategy for control, as well as ensure system stability. For more complex systems, it can be beneficial to determine the transfer functions for smaller components in the system, and then combining them later on. This technique will be utilized here.

---

## Motors

The first critical element of quadcopter dynamics is the motor, propeller and motor controller. It is possible to measure each of these components characteristics individually, but as far as thrust is concerned they are essentially a SISO (single input, single output) system, thus may only be interested in the input/output characteristic. The measurement of step response and results of the motor, propeller and motor controller combo used for this project can be found in the [following document finding the time constant](/docs/determining_thrust_time_constant.md), in combination with [this document finding the gain](/docs/motor_model_gain.md) a first order transfer function can be determined of the form:

$$F(s) = \frac{G}{\tau s+1} \left[\frac{\text{N}}{\text{dshot}}\right]$$

Where $G$ is the steady-state relationship between input dshot command and thrust in newton around the operating point corresponding to a hover. We also have $\tau$ being the time constant of the first order system. The measurements resulted in the following transfer function:

$$F(s) = \frac{4.436\cdot10^{-3}}{50\cdot10^{-3}s+1}$$

---

## Equation of motion for pitch and roll

In [this document](/docs/equations_of_motion_pitch.md) the equation of motion for pitch and roll were determined

$$G(s)= \frac{2D}{Js+B} \left[\frac{\omega}{\text{N}}\right]$$

Where $D$ is the distance from the between two propellers, $J$ is the moment of inertial of the drone around the given axis, and $B$ is the air resistance. Here, $D$ is the only easily measureable constant. 

The moment of inertia is determined by applying a constant torque around the axis, and measuring the change in velocity over time. A constant torque can be accomplished using a known mass $M$ hanging by gravity $g$ from a string wound around a cylinder with radius $r$ giving : $\tau_t = M \cdot g \cdot r$. The moment of inertia $J$ can then be found by fitting the measurement to the following model:

$$\omega(s) = \frac{Mgr}{Js}$$

The value of $J$ for the pitch axis was found during the test described in [this document](/docs/determining_moment_of_inertia.md) to have a value of $1.2\cdot 10^{-3} \text{kg}/\text{m}^2$. A dampening constant $B$ was also found during the test, though this also included friction from the test setup, so the air resistance is approximated as half of this, resulting in the following transfer function for pitch:

$$G(s)= \frac{170\cdot 10^{-3}}{1.2\cdot 10^{-3}s+200\cdot 10^{-6}}$$

The moment of inertia for roll will not be explicitly measured, but it assumed to be $\approx 10$% lower due to the orientation of the battery.