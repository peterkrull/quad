s = tf('s');

G_m = 4.437e-3/(50e-3*s+1) % Per-motor thrust transfer function


R = 85e-3;  % Distance from center to motor thrust vector
B = 0.00001;   % Air-resistance dapening factor (guess)
J = 0.007;    % Mass moment of inertia (so far, guess)

G_r = R/(B+J*s) % Roll transfer function

G = G_m*G_r*2

kp = 500
kd = 0.04
ki = 0

D = kp*(1+kd*s+ki/s)

delay = 10e-3;
T_d = (2-delay*s)/(2+delay*s)

direct = (T_d*D*G)/(9.82)

% margin(direct)
 
vel_CL = feedback(direct,1)

pole(direct)
zero(direct)


kp = 8
kd = 0.15
ki = 0

D = kp*(1+kd*s+ki/s)


direct = (D*vel_CL)/(s)


 
pos_CL = feedback(direct,1)

step(pos_CL)
xlim([0 2])
ylim([-0.2 1.5])

grid on


