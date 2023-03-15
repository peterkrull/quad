from math import *
from numpy import matrix
from numpy import diag
from matplotlib import pyplot as plt
import random as rand
from KalmanFilter import rotation_matrix_2d

# rand.seed(1)

# Simulation config
time = 60 # seconds
Td = 1/200 # frequency Hz
points = int(time/Td)

pos_Hz = 50 # Frequency of position update (GPS ~ 5 Hz)
vel_Hz = 2 # Frequency of velocity update (??)
acc_Hz = 10 # frequency of acceleration update (IMU ~ 200 Hz)

pos_sigma = 0.6 # Noise distribution of position measurement
vel_sigma = 0.1 # Noise distribution of velocity measurement
acc_sigma = 0.5 # Noise distribution of acceleration measurement

acc_offset_x = 0.0
acc_offset_y = 0.0

#
# GENERATE GROUND THRUTH VECTORS
#

gt_time = [i*Td for i in range(points)]

# Generate ground truth position

amplitude = 2
frequency = 1

gt_pos_x = []
gt_pos_y = []
for i in range(points):
    gt_pos_x.append(sin((i - points/10)/points*time*frequency )*amplitude)
    gt_pos_y.append(cos((i - points/10)/points*time*frequency )*amplitude)
    # gt_pos.append( 0 )

# Generate ground truth velocity path
gt_vel_x = []
gt_vel_y = []
for i in range(points):
    if i == 0:
        gt_vel_x.append(sqrt(2))
        gt_vel_y.append(sqrt(2))
    else:
        gt_vel_x.append((gt_pos_x[i]-gt_pos_x[i-1])/Td)
        gt_vel_y.append((gt_pos_y[i]-gt_pos_y[i-1])/Td)
        
# Generate ground truth acceleration
gt_acc_x = []
gt_acc_y = []
for i in range(points):
    if i == 0 or i == 1:
        gt_acc_x.append(0)
        gt_acc_y.append(-1)
    else:
        x = (gt_vel_x[i]-gt_vel_x[i-1])/Td + acc_offset_x
        y = (gt_vel_y[i]-gt_vel_y[i-1])/Td + acc_offset_y
        
        rcv = rotation_matrix_2d(x,y,0)
        
        gt_acc_x.append(rcv[0])
        gt_acc_y.append(rcv[1])

#
# GENERATE DUMMY SAMPLED SIGNALS
#

prev_time = 0
x_pos_noise = []
y_pos_noise = []
s_pos_time = []
for i in range(points):
    if gt_time[i]-prev_time > 1/pos_Hz:
        prev_time = gt_time[i]
        x_pos_noise.append(rand.gauss(gt_pos_x[i],pos_sigma**2))
        y_pos_noise.append(rand.gauss(gt_pos_y[i],pos_sigma**2))
        s_pos_time.append(gt_time[i])
        
prev_time = 0
x_vel_noise = []
y_vel_noise = []
s_vel_time = []
for i in range(points):
    if gt_time[i]-prev_time > 1/vel_Hz:
        prev_time = gt_time[i]
        x_vel_noise.append(rand.gauss(gt_vel_x[i],vel_sigma**2))
        y_vel_noise.append(rand.gauss(gt_vel_y[i],vel_sigma**2))
        s_vel_time.append(gt_time[i])

prev_time = 0
x_acc_noise = []
y_acc_noise = []
s_acc_time = []
for i in range(points):
    if gt_time[i]-prev_time > 1/acc_Hz:
        prev_time = gt_time[i]
        x_acc_noise.append(rand.gauss(gt_acc_x[i],acc_sigma**2))
        y_acc_noise.append(rand.gauss(gt_acc_y[i],acc_sigma**2))
        s_acc_time.append(gt_time[i])
        
#
# Kalman filter setup
#

from KalmanFilter import KalmanFilter

F = matrix([
    [1 , Td , 1/2*Td**2,0,0,0],
    [0 , 1 , Td,0,0,0],
    [0 , 0 , 1,0,0,0],
    [0,0,0,1 , Td , 1/2*Td**2],
    [0,0,0,0 , 1 , Td],
    [0,0,0,0 , 0 , 1],
])

P = diag([0.1,0,1,0.1,0,1])
Q = diag([0,0,1e-1,0,0,1e-1]) # Process noise
R = diag([pos_sigma**2,0,acc_sigma**2,pos_sigma**2,0,acc_sigma**2]) # Measurement noise

kf = KalmanFilter(F,Q,R,matrix([sin(0)*amplitude,0,0,cos(0)*amplitude,0,0]).transpose(),P)

kf.add_observation_model("pos",matrix([[1,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,1,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]))
kf.add_observation_model("vel",matrix([[0,0,0,0,0,0],[0,1,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,1,0],[0,0,0,0,0,0]]))
kf.add_observation_model("acc",matrix([[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,1,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,1]]))

#
# Run simulation of filter
#

pos_counter = 0
vel_counter = 0
acc_counter = 0

est_pos_x = []
est_vel_x = []
est_acc_x = []

est_pos_y = []
est_vel_y = []
est_acc_y = []

est_time = []

for i in range(points):
    
    if gt_time[i] == s_pos_time[pos_counter]:
        # print(f"pos : {s_pos_noise[pos_counter]}")
        kf.update("pos",matrix([x_pos_noise[pos_counter],0,0,y_pos_noise[pos_counter],0,0]).transpose())
        if pos_counter < len(s_pos_time)-1:  
            pos_counter += 1

    # Velocity measurement is not used
    # if gt_time[i] == s_vel_time[vel_counter]:
    #     # print(f"vel : {s_vel_noise[vel_counter]}")sss
    #     kf.update("vel",s_vel_noise[vel_counter])    
    #     if vel_counter < len(s_vel_time)-1:
    #         vel_counter += 1
        
    if gt_time[i] == s_acc_time[acc_counter]:
        # print(f"acc : {s_acc_noise[acc_counter]}")
        kf.update("acc",matrix([0,0,x_acc_noise[acc_counter],0,0,y_acc_noise[acc_counter]]).transpose())    
        if acc_counter < len(s_acc_time)-1:
            acc_counter += 1
        
    x = kf.predict(Td)
    
    est_pos_x.append(x[0])
    est_vel_x.append(x[1])
    est_acc_x.append(x[2])
    
    est_pos_y.append(x[3])
    est_vel_y.append(x[4])
    est_acc_y.append(x[5])
    
    est_time.append(i*Td)
    
    
    
from controllers import control
lp_x = control.cascade(control.low_pass,3,tau=0.02,debug_time = Td , init_val = amplitude)
lp_y = control.cascade(control.low_pass,3,tau=0.02,debug_time = Td , init_val = amplitude)
lp_pos_x = [lp_x.update(x) for x in est_pos_x]  
lp_pos_y = [lp_y.update(x) for x in est_pos_y]  
    
figure, axis = plt.subplots(2, 2,figsize=(16,9))

axis[0, 0].plot(s_pos_time,x_pos_noise)
axis[0, 0].plot(est_time,est_pos_x) 
axis[0, 0].plot(est_time,lp_pos_x)
axis[0, 0].plot(gt_time,gt_pos_x)
axis[0, 0].set_xlim([40,50])
axis[0, 0].set_ylim([-amplitude*1.4,amplitude*1.4])

axis[0, 1].plot(s_pos_time,y_pos_noise)
axis[0, 1].plot(est_time,est_pos_y)
axis[0, 1].plot(est_time,lp_pos_y)
axis[0, 1].plot(gt_time,gt_pos_y)
axis[0, 1].set_xlim([40,50])
axis[0, 1].set_ylim([-amplitude*1.4,amplitude*1.4])

axis[1, 1].plot(x_pos_noise[int(points/3):len(y_pos_noise)-1],y_pos_noise[int(points/3):len(y_pos_noise)-1])
axis[1, 1].plot(est_pos_x[int(points/3):len(est_pos_y)-1],est_pos_y[int(points/3):len(est_pos_y)-1])
axis[1, 1].plot(lp_pos_x[int(points/3):len(lp_pos_y)-1],lp_pos_y[int(points/3):len(lp_pos_y)-1])
axis[1, 1].plot(gt_pos_x[int(points/3):len(gt_pos_y)-1],gt_pos_y[int(points/3):len(gt_pos_y)-1])

plt.legend(["Sampled position","Estimated position","Filtered estimate","True position"])

axis[1, 0].plot(x_vel_noise[int(points/3):len(y_vel_noise)-1],y_vel_noise[int(points/3):len(y_vel_noise)-1])
axis[1, 0].plot(gt_vel_x[int(points/3):len(gt_vel_y)-1],gt_vel_y[int(points/3):len(gt_vel_y)-1])
axis[1, 0].plot(est_vel_x[int(points/3):len(est_vel_y)-1],est_vel_y[int(points/3):len(est_vel_y)-1])

plt.show()
