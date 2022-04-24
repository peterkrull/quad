s = tf('s');

%% Inner velocity-control loop

K = 4.437e-3;
tau = 50e-3;

G_m = K/(tau*s+1); % Per-motor int to thrust transfer function

R = 85e-3;  % Distance between center of motor thrust vector
J = 0.0012; % Moment of inertia of drone around y-axis
B = 0.0004; % Dampening for drone on test stand

G_r = 4*R/(B+J*s); % Thrust to angular velocity transfer function

G_i = G_m*G_r; % Int to angular velocity transfer function
p = pole(G_i); % Get poles for inner plant

% Total time delay
delay = 10e-3;
T_d = (2-delay*s)/(2+delay*s);
direct_i_bf = (T_d*G_i);

z_i = zero(direct_i_bf)
p_i = pole(direct_i_bf)

figure("position",[0,0,1500,1000])
margin(direct_i_bf)

%%

figure("position",[0,0,1500,1000])
step(feedback(direct_i_bf*3.5,1),1)

[k] = rlocus(direct_i_bf);
rlocus(direct_i_bf);
xlim([-21,0])

max_gain = k(2); % only one below 200 rad/s

%%
% PID controller
kp_i = 1; % Provides 20 dB of gain margin
ki_i = -p_i(3); % cancels out pole in -0.33 due to pitch model
kd_i = -1/p_i(2); % cancels out pole in -20 due to motor model
D_i = kp_i*(1+kd_i*s+ki_i/s);

direct_i = (T_d*G_i*D_i);

% Closed loop
CL_i = feedback(direct_i,1);

% Step response plot
figure("position",[0,0,1500,1000])
step(CL_i,1)
ylim([-0.2 1.5])
ylabel("Angular velocity [rad/s]")
legend "Roll anglular velocity"
grid on

%%

G = tf([-2.037e-05 0.003666 0.08133 0.02715],[6e-07 0.0001118 0.00611 0.08213 0.02715 0])

rlocus(G);


%% Outer angle-control loop

kp_o = 20; ki_o = 0; kd_o = 0.02;

% PID controller
D_o = kp_o*(1+kd_o*s+ki_o/s);
direct_o = (D_o*CL_i)/s;

% Closed loop
CL_o = feedback(direct_o,1);

% Step response plot
step(CL_o,1)
ylim([-0.2 1.5])
grid on

%%

%% Inner velocity-control loop

K = 4.437e-3;
tau = 50e-3;

G_m = K/(tau*s+1); % Per-motor int to thrust transfer function

R = 85e-3;  % Distance between center of motor thrust vector
J = 0.0012; % Moment of inertia of drone around y-axis
B = 0.0004; % Dampening for drone on test stand

G_r = 4*R/(B+J*s); % Thrust to angular velocity transfer function

G_i = G_m*G_r; % Int to angular velocity transfer function
p = pole(G_i); % Get poles for inner plant

% Total time delay
delay = 10e-3;
T_d = (2-delay*s)/(2+delay*s);
direct_i_bf = (T_d*G_i);

z_i = zero(direct_i_bf);
p_i = pole(direct_i_bf);

% figure("position",[0,0,1500,1000])
% margin(direct_i_bf)
% 
% figure("position",[0,0,1500,1000])
% step(feedback(direct_i_bf*10,1))

[rldata k] = rlocus(direct_i_bf);
rlocus(direct_i_bf);
xlim([-21,0])