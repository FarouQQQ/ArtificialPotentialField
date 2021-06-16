"""
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
"""
import matplotlib.pyplot as plt
from matplotlib.pyplot import imshow, show, colorbar
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import LinearLocator
import numpy as np
import math
import Utilities

# Initialize figure
fig = plt.figure(figsize=(15, 8))
ax = fig.add_subplot(1, 2, 1, projection='3d')

# Global Variables:

# Map specific:
mapBoundaryX = 11  # Map x length
mapBoundaryY = 11  # Map y length
reso = 10  # Map resolution

# repulsion gains:
eta = 5.0
rho_node = 5.0

# Attraction Gains:
KP = 2  # Position Gain

spacingResolution = (mapBoundaryX * reso) + 1

xAxis = np.linspace(0, mapBoundaryX, spacingResolution)
yAxis = np.linspace(0, mapBoundaryY, spacingResolution)

# Robot specific
robotX = 2.0  # initial robot X position
robotY = 2.0  # initial robot Y position

goalX = 10.0  # goal X position
goalY = 10.0  # goal Y position

# Obstacle:
obsX = 5.0
obsY = 5.0

# Position scaling for grid:
xGoalOnGrid = goalX * reso
yGoalOnGrid = goalY * reso

xRobotOnGrid = robotX * reso
yRobotOnGrid = robotY * reso

xObsOnGrid = obsX * reso
yObsOnGrid = obsY * reso

# potential field generation:
uPot = np.zeros(shape=(len(xAxis), len(yAxis)))
uAtr = np.zeros(shape=(len(xAxis), len(yAxis)))
uRep = np.zeros(shape=(len(xAxis), len(yAxis)))

# Calculate Attraction
for i in range(len(xAxis)):
    for j in range(len(yAxis)):
        uAtr[i][j] = 1 / 2.0 * KP * math.sqrt(abs(xGoalOnGrid - i) ** 2 + abs(yGoalOnGrid - j) ** 2) / reso

# Calculate repulsion potential:
for i in range(len(xAxis)):
    for j in range(len(yAxis)):
        rho = math.sqrt(((xObsOnGrid - i) ** 2 + (yObsOnGrid - j) ** 2)) / reso
        if rho <= rho_node:
            if rho == 0:
                uRep[i][j] = np.max(uAtr) / 2
            else:
                uRep[i][j] = 0.5 * eta * ((1 / rho) - (1 / rho_node)) ** 2

            if uRep[i][j] > eta:
                uRep[i][j] = eta
        else:
            uRep[i][j] = 0

uPot = uAtr + uRep

# for debugging
# print(uPot.shape)
# print(uPot)

xAxis, yAxis = np.meshgrid(xAxis, yAxis)
surf = ax.plot_surface(xAxis, yAxis, uPot, cmap=cm.viridis, linewidth=0, antialiased=True)
ax.view_init(azim=30)

ax = fig.add_subplot(1, 2, 2)
imshow(uPot, origin='lower', extent=[0, int(mapBoundaryX), 0, int(mapBoundaryY)])
colorbar()

# Simulate motion:
distanceToGoal = math.sqrt((xRobotOnGrid - xGoalOnGrid) ** 2 + (yRobotOnGrid - yGoalOnGrid) ** 2)
distanceTolerance = 1

robotStartPoint = [robotX, robotY]
goalPoint = [goalX, goalY]
motionDirection, quadrant = Utilities.getAngleAndDirection(robotStartPoint, goalPoint)

numIterations = 0
indexX = int(xRobotOnGrid)
indexY = int(yRobotOnGrid)

while distanceToGoal >= distanceTolerance or numIterations < 100:

    minVal = 1000
    minIndex = [indexX, indexY]

    for i in range(-1, 2):
        for j in range(-1, 2):
            newindexX = indexX + i
            newindexY = indexY + j
            if newindexX < 0 or newindexX > spacingResolution or newindexY < 0 or newindexY > spacingResolution or (i == j and i == 0):
                pass
            elif uPot[newindexX][newindexY] < minVal:
                minVal = uPot[newindexX][newindexY]
                minIndex = [newindexX, newindexY]

    indexX = minIndex[0]
    indexY = minIndex[1]


    distanceToGoal = math.sqrt((indexX - xGoalOnGrid) ** 2 + (indexY - yGoalOnGrid) ** 2)

    if numIterations % 2 == 0:
        plt.scatter(indexX/reso, indexY/reso, color="m", marker=".")
    plt.pause(0.005)
    numIterations += 1

plt.show()