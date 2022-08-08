from math import *
import numpy as np
from matplotlib import pyplot as plt

# Config
points = 1000
Td = 0.01



# Generate correct path
pos = []
for i in range(points):
    x = cos(i/points*2*pi)
    y = sin((i/points*2*pi)*2)
    vec = np.array([x,y])
    pos.append(vec)

# Create vel and acc vectors

vel = []
prev_pos = None
for xpos in pos:
    if type(prev_pos) == type(None):
        vel.append(np.array([0,0]))
    else:
        vel.append((xpos-prev_pos)/Td)
    prev_pos = xpos

acc = []
prev_vel = None
for xvel in vel:
    if type(prev_vel) == type(None):
        acc.append(np.array([0,0]))
    else:
        acc.append((xvel-prev_vel)/Td)
    prev_vel = xvel

def xyplot(listing:list):
    x = []
    y = []

    for entry in listing:
        x.append(entry[0])
        y.append(entry[1])

    plt.plot(x,y)
    plt.show()

xyplot(acc)