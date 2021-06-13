'''
Artificial potential fields
Basic idea :
-----------
U_art = U_xd(attraction potential) + U_O(repulsive Potential)

U_att = 1/2 * kp * (x -  xd)^2

            1/2 * n * (1/rho - 1/rho_node )^2       if rho <= rho_node
U_rep = {
            0                                       if rho > rho_node

U_art = The artificial potential field, which the agent is subjected to
U_xd  = Attractive potential field created by the goal
U_O   = Repulsive potential field created by the obstacle
x     = position of agent
x_d   = goal position
k_p   = position gain
eta   = constant gain
rho   = shortest distance to the obstacle
rho_0 = limit distance of the potential field influence
'''
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import LinearLocator
import numpy as np
import math

# Initialize figure
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

#Global Variables
#Gains:
KP = 1 #Position Gain

#Map specific:
spacingResolution = 10  #defines number of points between each descrete position
mapBoundaryX = 10   # Map x length
mapBoundaryY = 10   # Map y length

xAxis = np.linspace(0, mapBoundaryX, spacingResolution)
yAxis = np.linspace(0, mapBoundaryY, spacingResolution)

#Robot specific
robotX = 0  #initial robot X position
robotY = 0  #initial robot Y position

goalX = 5   #goal X position
goalY = 5   #goal Y position

#potential field generation:
uPot = np.zeros(shape=(len(xAxis),len(yAxis)))


for i in range(len(xAxis)):
    for j in range(len(yAxis)):
        uPot[i][j] = 1 / 2 * KP * math.sqrt(((goalX-(i))**2+(goalY - (j))**2))

print(uPot.shape)
print(uPot)

xAxis, yAxis = np.meshgrid(xAxis, yAxis)

surf = ax.plot_surface(xAxis, yAxis, uPot, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)


# for i in range(5):
#     ax.scatter(math.sin(i),i)
#     plt.pause(0.001)

plt.show()