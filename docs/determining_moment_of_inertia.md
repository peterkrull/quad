# Determining roll moment of inertia

The moment of inertia of some object is not always easy to measure. This is especially true if the object is of an odd size, shape and density. But if we are able to treat our object as a 'free body' where all quantities can be accounted for, then we can determine the moment of inertia by doing a test, where the change in angle over time is recorded when we apply a constant torque.

The moment of inertia is determined by applying a constant torque around the axis, and measuring the change in velocity over time. A constant torque can be accomplished using a known mass $M$ hanging by gravity $g$ from a string wound around a cylinder with radius $r$ giving : $\tau_t = M \cdot g \cdot r$. The moment of inertia $J$ can then be found by fitting the measurement to the following model:

A [3D-printed part](/3dparts/droneframe_v1/pitch_bearing_pivot_wide.obj) was designed specifically for this test, that is meant to replace the usual bearing holder for the pitch/roll test stand. This part has a diameter of $35 \text{ mm}$, so $r = (35/2)\cdot 10^{-3}$. Next, i used a roll of electrical tape, weighing in at $26 \text{ g}$, so $M = 26\cdot 10^{-3}$ and finally $g = 9.82$.

![](/images/inertia_torquer.png)

For the test, a camera was mounted on a tripod recording in 4K with a fast shutter speed to prevent motion blur. Then, the drone was held at an angle to raise the roll of tape up high, and then let go. This is functionally a step in gravity and thus torque, that allows us to observe the change in angle over time. The video is processed in [Physlets tracker](https://physlets.org/tracker/) where two points are tracked on each opposing propeller. This gives us two sets of coordinates ([1](/tests/inertia_meas/pitch_inertia_left_point.csv) and [2](/tests/inertia_meas/pitch_inertia_right_point.csv)), where it is possible to determine the angle between them.

![](/images/inertia_test_setup.png)

Finally, data processing!

A [matlab script](/software/misc-tools/roll_moment_inertia.m) has been developed to calculate the angle using $\texttt{atan2}$, and angular velocity based on the timstamps from the video, as well as determine the moment of inertia that best fits the following transfer function, where some damping $B$ is expected:

$$\omega(s) = \frac{Mgr}{Js} \approx \frac{Mgr}{Js + B}$$

Both the moment of inertia, as well as the damping observed in the test were determined, though the dampening is of limited use at this point.
$$J = 1.2\cdot 10^{-3}$$$$B = 0.4\cdot 10^{-3}$$

![](/images/roll_moment_of_inertia.jpg)