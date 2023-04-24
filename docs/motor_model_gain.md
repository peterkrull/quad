# Determining the motor model gain

Since the motor model is meant to relate the output thrust in newton to the input Dhsot command, the gain of the transfer function, or the constant in the numerator, is what defines this relation.

---

## Test setup and procedure

The thrust is measured in grams at 50 Hz using a strain gauge of the type [Eilersen SPSX](https://www.eilersen.com/single-point-load-cell/product/single-point-load-cell-spsx/) in combination with the [Eilersen 4X79A](https://www.eilersen.com/analog-weighing-module/product/analog-output-module-4x79a/) digital readout module, connected to a computer via ethernet, and read using a [program](/software/misc-tools/TCP_strain_guage.py) that establishes and logs a TCP data stream. A single motor with propeller is rigidly mounted to the strain gauge using [3D-printed parts](/3dparts/teststand_thrust/) and driven by a 30 A DShot ESC and a 4S Li-Po battery charged to 15.8 V.


---

## Results

Only one test was performed due to time constraints. The data collected using the TCP script mentioned above was imported and processed using a [Julia script](/tests/thrust_response/plotting_thrust_increaser.jl), where the data is converted to the correct units by scaling along both axes, then fitted to a first order polynomial, or a linear function. The resulting plot can be seen below.

![](/images/thrust_linearization.svg)

Here we are only interested in the slope of the orange function, which is $4.436e-3$. This means that for the motor model, we have:

$$F(s)=\frac{4.436\cdot 10^{-3}}{\tau s + 1}$$