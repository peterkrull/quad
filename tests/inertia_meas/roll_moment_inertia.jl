using ControlSystems,Plots,CSV,DataFrames

left_point = CSV.read("tests/inertia_meas/pitch_inertia_left_point.csv",DataFrame);
right_point = CSV.read("tests/inertia_meas/pitch_inertia_right_point.csv",DataFrame);

# Calculate angle and angular velocity
angle = atan.(right_point.y-left_point.y,right_point.x-left_point.x);
angle_v = diff(angle)./diff(left_point.t);

Ts = 0.01
T = Ts:Ts:1.5;

# Define transfer function constants
M = 26e-3; # Mass of test-weight
g = 9.82; # Gravitational acceleration
r = (35/2)*1e-3; # Radius of drum

# Fitted values
J = 1.2e-3; # Moment of inertia
B = 300e-6; # Dampening factor

s = tf('s')
G = M*g*r/(J*s+B)
F = 4.436e-3/(50e-3*s + 1)

# Plot against data
shift_time = left_point.t[1:length(left_point.t)-1] .- 0.25
p = scatter(shift_time,-angle_v,label="Measured response")
plot!(p,step(G,1.2), label = "Modeled response")
