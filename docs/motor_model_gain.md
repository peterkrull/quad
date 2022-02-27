# Determining the motor model gain

Since the motor model is meant to relate the output thrust in newton to the input dhsot command, the gain of the transfer function, or the constant in the numerator, is what defines this relation.

--- 

## Test setup and procedure

The thrust is measured in grams at 50 Hz using a strain guage of the type [Eilersen SPSX](https://www.eilersen.com/single-point-load-cell/product/single-point-load-cell-spsx/) in combination with the [Eilersen 4X79A](https://www.eilersen.com/analog-weighing-module/product/analog-output-module-4x79a/) digital readout module, connected to a computer via ethernet, and read using a [program](/software/misc_tools/TCP_strain_guage.py) that establishes and logs a TCP data stream. A single motor with propeller is rigidy mounted to the strain guage using [3D-printed parts](/3dparts/teststand_thrust/) and driven by a 30 A DShot ESC and a 4S Li-Po battery charged to 15.8 V.

A [firmware](/software/ESP32_thrust_step_firmware/src/main.cpp) has been written for the ESP32-based flight controller that increments the amount of thrust to the motor over a period of time, from the minimum value to the maximum. 

---

## Results

Only one test was performed due to time constraints. The data collected using the TCP script mentioned above was imported and processed using a [Matlab script](/software/misc_tools/plotting_thrust_increaser.m), where the data is converted to the correct units by scaling along both axes, then fitted to a first order polynomial, or a linear function. The resulting plot can be seen below.

![](/images/thrust_linearization.jpg)

Here we are only interested in the slope of the orange function, which is $4.436e-3$. This means that for the motor model, we have:

$$F(s)=\frac{4.436\cdot 10^{-3}}{\tau s + 1}$$