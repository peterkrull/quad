# Pitch and roll - Equations of motion

In order to describe the relationship between the thrust of each motor and the roll behaviour, we can utilize a free-body diagram go get a better understanding of the elements at play and how they affect the system. Here we are interested in the mass moment of inertia of the drone, represented by $J$, the force provided by each motor $F_1,F_2,F_3,F_4$, the distance from the motor to the center of mass $R$, and finally the air resistance modelled as a pure dampener $B$

![](/images/free_body_pitch.png)

Using this diagram, we can set up the following equations, considering the direction of rotation denoted by $\omega$.

$$J\dot{\omega} = F_2F_4R-F_1F_3R-B\omega$$

We bring it to the laplace domain:

$$J\omega(s)s = F_2(s)F_4(s)R-F_1(s)F_3(s)R-B\omega(s)$$

Next, we can simplify the motor forces. Since for a clockwise roll command, motors $F_1,F_3$ will increase in thrust, and motors $F_1,F_3$ will decrese with a similar, but negative amount of thrust, we can simplify to:

$$J\omega(s)s = 4F(s)R-B\omega(s)$$

We can now combine terms containing $\omega(s)$

$$\omega(s)(Js+B) = 4F(s)R$$

And finally cross-divide to get the transfer function. Since it might be easier to simply measure the distance between the center of two motors, we can define that $D = 2R$:

$$G(s)=\frac{\omega(s)}{F(s)} = \frac{4R}{(Js+B)} = \frac{2D}{(Js+B)}$$

This transfer function of the first order, due to the single $s$, resulting in a pole. It does assume that the propellers are a point source of thrust, and that they are on the exact same plane as the mass moment of inertia $J$. This might not be the case in reality, but the equation is likely to approximate the forces close enough.
