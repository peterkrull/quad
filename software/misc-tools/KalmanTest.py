from math import *
from numpy import matrix
from numpy import diag
from matplotlib import pyplot as plt
import random as rand

# rand.seed(1)

# Simulation config
time = 50 # seconds
Td = 1/500 # frequency Hz
points = int(time/Td)

pos_Hz = 5 # Frequency of position update (GPS ~ 5 Hz)
vel_Hz = 10 # Frequency of velocity update (??)
acc_Hz = 100 # frequency of acceleration update (IMU ~ 200 Hz)

pos_sigma = 1 # Noise distribution of position measurement
vel_sigma = 0.2 # Noise distribution of velocity measurement
acc_sigma = 0.6 # Noise distribution of acceleration measurement

acc_offset = 0.5

#
# GENERATE GROUND THRUTH VECTORS
#

gt_time = [i*Td for i in range(points)]

# Generate ground truth position

amplitude = 2
frequency = 2

gt_pos = []
for i in range(points):
    if i < points/4:
        gt_pos.append(amplitude)
    else:
        gt_pos.append( cos((i - points/4)/points*time*frequency )*amplitude )
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
    if gt_time[i] >= prev_time + 1/pos_Hz:
        prev_time = gt_time[i]
        s_pos_noise.append(rand.gauss(gt_pos[i],pos_sigma))
        s_pos_time.append(gt_time[i])
        

prev_time = 0
s_vel_noise = []
s_vel_time = []
for i in range(points):
    if gt_time[i] >= prev_time+ 1/vel_Hz :
        prev_time = gt_time[i]
        s_vel_noise.append(rand.gauss(gt_vel[i],vel_sigma))
        s_vel_time.append(gt_time[i])

prev_time = 0
s_acc_noise = []
s_acc_time = []
for i in range(points):
    if gt_time[i] >= prev_time + 1/acc_Hz:
        prev_time = gt_time[i]
        s_acc_noise.append(rand.gauss(gt_acc[i],acc_sigma) + acc_offset)
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

P = diag([0,0,1]) # Initial "trust" in each sensor
Q = diag([1e-6,0,1e-3]) # Model noise covariance matrix

kf = KalmanFilter(F,Q,matrix([amplitude,0,0]).transpose(),P)

Q2 = diag([1e-5,0,1e-2]) # Model noise covariance matrix
kf2 = KalmanFilter(F,Q2,matrix([amplitude,0,0]).transpose(),P)

kf.add_observation_model("pos",matrix([[1,0,0]]),pos_sigma**2)
kf.add_observation_model("vel",matrix([[0,1,0]]),vel_sigma**2)
kf.add_observation_model("acc",matrix([[0,0,1]]),acc_sigma**2)

#
kf2.add_observation_model("posacc",matrix([[1,0,0],[0,0,1]]),diag([pos_sigma*10,acc_sigma]))

# Run simulation of filter
#

pos_counter = 0
vel_counter = 0
acc_counter = 0

est_pos = []
est_vel = []
est_acc = []

est_time = []

est2_pos = []
est2_vel = []
est2_acc = []

est2_time = []


from KalmanFilter import ComplementaryFilter

vel_acc_comp = ComplementaryFilter(0.9999,anti_drift_tau=0.5)
pos_vel_comp = ComplementaryFilter(0.9999,anti_drift_tau=10)

pos_vels = []
pos_vels_time = []

prev_pos = None
pos_vel = None

comp_vels = []
comp_poss = []

comp_time = []

for i in range(points):
    
    if gt_time[i] == s_pos_time[pos_counter]:
        # print(f"pos : {s_pos_noise[pos_counter]}")
        kf.update("pos",[s_pos_noise[pos_counter]])
        
        # Complementary filter
        if type(prev_pos) != type(None):
            pos_vel = (s_pos_noise[pos_counter] - prev_pos)*pos_Hz
            # print(gt_time[i],pos_vel)
            pos_vels.append(pos_vel)
            pos_vels_time.append(gt_time[i])
        prev_pos = s_pos_noise[pos_counter]
        
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
        
        if type(pos_vel) != type(None):
            comp_vel = vel_acc_comp.update(pos_vel,s_acc_noise[acc_counter],1/acc_Hz)
            # print(round(i*Td,4),s_acc_noise[acc_counter])
            comp_pos = pos_vel_comp.update(prev_pos , comp_vel , 1/acc_Hz)
            
            comp_vels.append(comp_vel)
            comp_poss.append(comp_pos)
            comp_time.append(i*Td)
            
            kf2.update("posacc",matrix([comp_pos,s_acc_noise[acc_counter]]).transpose())
            
            
        if acc_counter < len(s_acc_time)-1:
            acc_counter += 1
            
    x2 = kf2.predict(Td)
    
    est2_pos.append(x2[0])
    est2_vel.append(x2[1])
    est2_acc.append(x2[2])
    
    est2_time.append(i*Td)
        
    x = kf.predict(Td)
    
    est_pos.append(x[0])
    est_vel.append(x[1])
    est_acc.append(x[2])
    
    est_time.append(i*Td)
    
xcomp = ComplementaryFilter(0.999,anti_drift_tau=1)
    
pos_comp = [xcomp.update(est_pos[i],est_vel[i],Td) for i in range(len(est_pos))]
    
plt.plot(s_pos_time,s_pos_noise)
plt.plot(gt_time,gt_pos)
plt.plot(est_time,est_pos)
plt.plot(comp_time,comp_poss)
plt.plot(est2_time,est2_pos)
plt.plot(est_time,pos_comp)
plt.legend(["Sampled position","True position","Estimated position","Complimentary estimate","Proposed filter"])
plt.show()


plt.plot(s_vel_time,s_vel_noise)
plt.plot(gt_time,gt_vel)
plt.plot(est_time,est_vel)
plt.plot(comp_time,comp_vels)
plt.plot(est2_time,est2_vel)
plt.legend(["Sampled position","True position","Estimated position","Complimentary estimate","Proposed filter"])
plt.show()

plt.plot(s_acc_time,s_acc_noise)
plt.plot(gt_time,gt_acc)
plt.plot(est_time,est_acc)
plt.plot(est2_time,est2_acc)
plt.legend(["Sampled position","True position","Estimated position"])
plt.show()