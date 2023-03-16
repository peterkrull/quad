left_point = readtable("pitch_inertia_left_point.csv");
right_point = readtable("pitch_inertia_right_point.csv");

% Calculate angle and angular velocity
angle = atan2(right_point.y-left_point.y,right_point.x-left_point.x);
angle_v = diff(angle)./diff(left_point.t);

%% split
clf(gcf)

% Define transfer function constants
M = 26e-3;
g = 9.82;
r = (35/2)*1e-3;

J = 0.0012;
B = 0.0004;

% Define time-domain transfer function
syms s
w = M*g*r/((J*s + B)*s);
w_t = ilaplace(w);
t = 0:0.01:1.5;
Y1t = matlabFunction(w_t);
Y1 = Y1t(t);

syms s
q = M*g*r/((J*s)*s);
q_t = ilaplace(q);
t = 0:0.01:1.5;
Y2t = matlabFunction(q_t);
Y2 = Y2t(t);

% Plot against data

plot(left_point.t(1:length(left_point.t)-1),-angle_v,'x',t+0.26,Y1,t+0.26,Y2)
grid on

text(0.1,4.4,"J = 1.2e-3")
text(0.1,4.0,"B = 0.4e-3")

xlabel("Time [s]")
ylabel("Angular velocity [rad/s]")

legend('Raw tracker data','Fitted tranfer function, with dampening','Fitted tranfer function, no dampening')
