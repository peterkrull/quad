s = tf('s');

%% Inner velocity-control loop

G_m = 4.437e-3/(50e-3*s+1) % Per-motor int to thrust transfer function

R = 85e-3;  % Distance from center to motor thrust vector
B = 0.0001;   % Air-resistance dapening factor (guess)
J = 0.0007;    % Mass moment of inertia (so far, guess)

G_r = R/(B+J*s) % Thrust to angular velocity transfer function

G_i = G_m*G_r*2; % Int to angular velocity transfer function

% PID controller
kp_i = 30; ki_i = 0; kd_i = 0.05;
D_i = kp_i*(1+kd_i*s+ki_i/s);

% Total time delay
delay = 10e-3;
T_d = (2-delay*s)/(2+delay*s);
direct_i = (T_d*G_i*D_i);

% Closed loop
CL_i = feedback(direct_i,1);

% Step response plot
step(CL_i)
xlim([-1 1])
ylim([-0.2 1.5])
grid on

%% Outer angle-control loop

kp_o = 0.2; ki_o = 0; kd_o = 0.02;

% PID controller
D_o = kp_o*(1+kd_o*s+ki_o/s);
direct_o = (D_o*CL_i*200)/s;

% Closed loop
CL_o = feedback(direct_o,1);

% Step response plot
step(CL_o)
xlim([-1 1])
ylim([-0.2 1.5])
grid on


%% Inner velocity-control loop 2

G_m = 4.437e-3/(50e-3*s+1) % Per-motor int to thrust transfer function

R = 85e-3;  % Distance from center to motor thrust vector
B = 0.0001;   % Air-resistance dapening factor (guess)
J = 0.07;    % Mass moment of inertia (so far, guess)

G_r = R/(B+J*s); % Thrust to angular velocity transfer function

G_i = G_m*G_r*2; % Int to angular velocity transfer function

% PID controller
kp_i = 1; ki_i = 0; kd_i = 0;
D_i = kp_i*(1+kd_i*s+ki_i/s);


% Total time delay
delay = 10e-3;
T_d = (2-delay*s)/(2+delay*s);
direct_i = (G_i*D_i);

% Closed loop
CL_i = feedback(direct_i,1);

% Step response plot
step(CL_i)
xlim([0 1])
ylim([-0.2 1.5])
grid on