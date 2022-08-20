from math import *
from numpy import matrix
from numpy import diag
from matplotlib import pyplot as plt
import random as rand

# rand.seed(1)

# Simulation config
time = 20 # seconds
Td = 1/500 # frequency Hz
points = int(time/Td)

pos_Hz = 5 # Frequency of position update (GPS ~ 5 Hz)
vel_Hz = 10 # Frequency of velocity update (??)
acc_Hz = 100 # frequency of acceleration update (IMU ~ 200 Hz)

pos_sigma = 0.3 # Noise distribution of position measurement
vel_sigma = 0.1 # Noise distribution of velocity measurement
acc_sigma = 0.5 # Noise distribution of acceleration measurement

acc_offset = 0

#
# GENERATE GROUND THRUTH VECTORS
#

gt_time = [i*Td for i in range(points)]

# Generate ground truth position

amplitude = 1
frequency = 2

gt_pos = []
for i in range(points):
    if i < points/10:
        gt_pos.append(amplitude)
    else:
        gt_pos.append( cos((i - points/10)/points*time*frequency )*amplitude )
    # gt_pos.append( 0 )

# Generate ground truth velocity path
gt_vel = []
for i in range(points):
    if i == 0:
        gt_vel.append(0)
    else:
        gt_vel.append( (gt_pos[i]-gt_pos[i-1])/Td )

# Generate ground truth acceleration
gt_acc = []
for i in range(points):
    if i == 0 or i == 1:
        gt_acc.append(0)
    else:
        gt_acc.append( (gt_vel[i]-gt_vel[i-1])/Td )

#
# GENERATE DUMMY SAMPLED SIGNALS
#

prev_time = 0
s_pos_noise = []
s_pos_time = []
for i in range(points):
    if gt_time[i]-prev_time > 1/pos_Hz:
        prev_time = gt_time[i]
        s_pos_noise.append(rand.gauss(gt_pos[i],pos_sigma**2))
        s_pos_time.append(gt_time[i])
        

prev_time = 0
s_vel_noise = []
s_vel_time = []
for i in range(points):
    if gt_time[i]-prev_time > 1/vel_Hz:
        prev_time = gt_time[i]
        s_vel_noise.append(rand.gauss(gt_vel[i],vel_sigma**2))
        s_vel_time.append(gt_time[i])

prev_time = 0
s_acc_noise = []
s_acc_time = []
for i in range(points):
    if gt_time[i]-prev_time > 1/acc_Hz:
        prev_time = gt_time[i]
        s_acc_noise.append(rand.gauss(gt_acc[i],acc_sigma**2) + acc_offset)
        s_acc_time.append(gt_time[i])
        
#
# Kalman filter setup
#

from KalmanFilter import KalmanFilter

F = matrix([
    [1 , Td , 1/2*Td**2],
    [0 , 1 , Td],
    [0 , 0 , 1],
])

P = diag([0.1,0,1])
Q = diag([1e-5,0,1e-2])
R = diag([pos_sigma,0,acc_sigma])

kf = KalmanFilter(F,Q,R,matrix([amplitude,0,0]).transpose(),P)

kf.add_observation_model("pos",matrix([[1,0,0],[0,0,0],[0,0,0]]))
kf.add_observation_model("vel",matrix([[0,0,0],[0,1,0],[0,0,0]]))
kf.add_observation_model("acc",matrix([[0,0,0],[0,0,0],[0,0,1]]))

#
# Run simulation of filter
#

pos_counter = 0
vel_counter = 0
acc_counter = 0

est_pos = []
est_vel = []
est_acc = []

est_time = []


for i in range(points):
    
    if gt_time[i] == s_pos_time[pos_counter]:
        # print(f"pos : {s_pos_noise[pos_counter]}")
        if gt_time[i] < 10 or gt_time[i] > 15: 
            kf.update("pos",s_pos_noise[pos_counter])  
        if pos_counter < len(s_pos_time)-1:  
            pos_counter += 1

    # Velocity measurement is not used
    # if gt_time[i] == s_vel_time[vel_counter]:
    #     # print(f"vel : {s_vel_noise[vel_counter]}")
    #     kf.update("vel",s_vel_noise[vel_counter])    
    #     if vel_counter < len(s_vel_time)-1:
    #         vel_counter += 1
        
    if gt_time[i] == s_acc_time[acc_counter]:
        # print(f"acc : {s_acc_noise[acc_counter]}")
        kf.update("acc",s_acc_noise[acc_counter])    
        if acc_counter < len(s_acc_time)-1:
            acc_counter += 1
        
    x = kf.predict(Td)
    
    est_pos.append(x[0])
    est_vel.append(x[1])
    est_acc.append(x[2])
    
    est_time.append(i*Td)
    
plt.plot(s_pos_time,s_pos_noise)
plt.plot(gt_time,gt_pos)
plt.plot(est_time,est_pos)
from controllers import control

lp = control.cascade(control.low_pass,4,tau=0.02,debug_time = Td , init_val = amplitude)

lp_pos = [lp.update(x) for x in est_pos]    
plt.plot(est_time,lp_pos)

plt.legend(["Sampled position","True position","Estimated position","Filtered estimate"])

plt.show()


plt.plot(s_vel_time,s_vel_noise)
plt.plot(gt_time,gt_vel)
plt.plot(est_time,est_vel)
plt.show()

plt.plot(s_acc_time,s_acc_noise)
plt.plot(gt_time,gt_acc)
plt.plot(est_time,est_acc)
plt.show()