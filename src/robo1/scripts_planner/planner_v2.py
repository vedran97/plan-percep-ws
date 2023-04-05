# -*- coding: utf-8 -*-
"""
Created on Sat Mar 18 16:02:21 2023

@author: saksham
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 14 23:56:08 2023

@author: saksham
"""

import numpy as np
import time
import cv2
import matplotlib.pyplot as plt
from sys import exit

from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline
from scipy.interpolate import CubicHermiteSpline
import scipy.interpolate as interpolate

#%%

# inputs
startState_real = (45,225,0) #tuple(map(int, start.split(",")))
goalState = (225,225,0) #tuple(map(int, goal.split(",")))
rpm = (30*0.1047198,60*0.1047198,90*0.1047198) #tuple(map(int, rpm.split(",")))

c = 5 + 19 # clearance + R (robot radius)
dt = 0.3

r = 6.45/2
L = 11
push = 20
startState = (startState_real[0]+push*np.cos(np.deg2rad(startState_real[2])),
              startState_real[1]+push*np.sin(np.deg2rad(startState_real[2])),
              startState_real[2] )
#%%
## reading map
map_raw = cv2.bitwise_not(cv2.imread('map.jpg',cv2.IMREAD_GRAYSCALE))
plt.figure(1)
plt.imshow(map_raw)

#Clearance
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2*c+1, 2*c+1))
mapN = cv2.dilate(map_raw, kernel)

# walls
mapN[:,0:c] = 255
mapN[:,-c:] = 255
mapN[0:c,:] = 255
mapN[-c:,:] = 255

cv2.imwrite("map_bloated.jpg",cv2.bitwise_not(cv2.resize(mapN,(900,900))))

plt.figure(2)
plt.imshow(mapN)

#%%
# Checker if a node falls in the obstacle space
def checkFeasibility(node):
    try:
        a = mapN[np.shape(mapN)[1]-int(node[1]),int(node[0])]
    except:
        return False
    
    if a == 0:
        return True
    else:
        return False
#%%
nodeList={}

# shifter = [(0,rpm[0]),(rpm[0],0),(rpm[0],rpm[0]),(0,rpm[1]),
#            (rpm[1],0),(rpm[1],rpm[1]),(rpm[1],rpm[0]),(rpm[0],rpm[1])]

shifter = [(rpm[0],rpm[1]),(rpm[1],rpm[0]),(rpm[1],rpm[1]),(rpm[0],rpm[2]),
            (rpm[2],rpm[0]),(rpm[2],rpm[2]),(rpm[2],rpm[1]),(rpm[1],rpm[2])]

def costC(node,goal):
    d = np.sqrt((node[0]-goal[0])**2 + (node[1]-goal[1])**2)
    return d

def shift(node,cost):
    x,y,t = node
    t = np.deg2rad(t)
    for i in shifter:
        t_ = t + dt*r*(i[0]-i[1])/L
        if i[0] == i[1]:
            x_ = x + 0.5*(i[0]+i[1])*r*dt*np.cos(t)
            y_ = y + 0.5*(i[0]+i[1])*r*dt*np.sin(t)
        else:
            F = (L/2)*(i[0]+i[1])/(i[0]-i[1])
            x_ = x + F*( np.sin( (r*(i[0]-i[1])*dt/L) +t ) -np.sin(t))
            y_ = y - F*( np.cos( (r*(i[0]-i[1])*dt/L) +t ) -np.cos(t))
        childNode = (x_,y_,np.rad2deg(t_))
        if checkFeasibility(childNode):
            nodeList[childNode] = i
            yield childNode, cost+costC(node,childNode)
        else:
            pass
#%%
# main algorithm
    
# time calculation
startTime = time.time()

if not checkFeasibility(startState) or not checkFeasibility(goalState):
    print('Infeasable states! Check Input')
    exit()

closedNodes = {}
openNodes = {startState:( costC(startState,goalState) , costC(startState,goalState) ,0,0,0)}

# order is totalCost, cost2Goal, cost2come, parent, self
nodeVisit = 255*np.ones(np.shape(map_raw))

child = 1
repeatSkip=0

while True:
    
    # popping first node
    parent=list(openNodes.keys())[0]

    closedNodes[parent] = openNodes[parent]
    
    if costC(parent,goalState) < L/2:
        print("Goal Found after",len(closedNodes),"nodes in ",time.time()-startTime, " seconds!")
        print("overwrote nodes :",repeatSkip)
        break
        
    for node,cost in shift(parent,openNodes[parent][2]):
        try:
            aa = nodeVisit[int(round(node[0])),int(round(node[1]))]==125
        except:
            print([int(round(node[0])),int(round(node[1]))])
        if nodeVisit[int(round(node[0])),int(round(node[1]))]==125:
            repeatSkip = repeatSkip +1
            pass
        
        else:
            if nodeVisit[int(round(node[0])),int(round(node[1]))] == 255 and node != None:
                # ...... and if not, add child
                openNodes[node] = (1.5*costC(node,goalState) + cost,
                         costC(node,goalState),
                         cost,openNodes[parent][4],child)
                child = child + 1
                nodeVisit[int(round(node[0])),int(round(node[1]))]=125
   
    nodeVisit[int(round(parent[0])),int(round(parent[1]))] = 0
    del openNodes[parent]
    
    # Sort the dict before popping
    openNodes = dict(sorted(openNodes.items(), key=lambda x:x[1]))

# backtracking
backTrack = [node,parent]
child = closedNodes[parent][3]
while child >0:
    for key, value in closedNodes.items():
        
        if value[4] == child:
            node = key
            child = value[3]
            backTrack.append(node)
            
backTrack.append(startState)
backTrack.reverse()

#%%
timeRate = 0.02  #50 hertz
resolution = 100000
average_vel = 10 #cm/sec

#%%
xy_points = np.array(backTrack)

tck, u = interpolate.splprep([xy_points[:,0], xy_points[:,1]])
x_i, y_i = interpolate.splev(np.linspace(0, 1, resolution), tck, ext=0)
x_D, y_D = interpolate.splev(np.linspace(0, 1, resolution), tck, ext=0,der=1)

xy_slope = np.arctan2(y_D,x_D)

plt.figure(3)
plt.plot(x_i, y_i)
plt.scatter(xy_points[:,0], xy_points[:,1], c='r')

plt.figure(4)
plt.plot(xy_slope)

#%%

xy_points = np.column_stack((x_i,y_i))
xy_diff = np.sqrt(np.sum(np.square(xy_points[1:]-xy_points[:-1]),axis=1))

time_taken = np.sum(xy_diff)/average_vel
delta_t = time_taken/len(xy_diff)

xy_Dangle = (xy_slope[1:] - xy_slope[:-1])/delta_t

Wl = (average_vel/r) + (L/(2*r))*xy_Dangle
Wr = (average_vel/r) - (L/(2*r))*xy_Dangle

timeRange = np.arange(0,time_taken,delta_t) + (2*push/average_vel) + delta_t

ang_vel = np.column_stack((Wl,Wr,timeRange))
vel_profile = np.vstack(([0,0,0],[average_vel/r,average_vel/r,(2*push/average_vel)],ang_vel))

plt.figure(5)
plt.plot(vel_profile[:,2],vel_profile[:,np.array([True, True, False])])
#%%

f_Ul = interp1d(vel_profile[:,2], vel_profile[:,0], kind='linear')
f_Ur = interp1d(vel_profile[:,2], vel_profile[:,1], kind='linear')

new_time = np.arange(0, vel_profile[-1,2], timeRate)
# Interpolate the x,y values to the desired time steps
new_x, new_y = f_Ul(new_time).T , f_Ur(new_time).T

vel_profile_wheels = np.column_stack((new_x, new_y, new_time))

plt.figure(6)
plt.plot(np.gradient(vel_profile_wheels,axis=0))
#%%
np.savetxt("vel_traj.csv", vel_profile_wheels,delimiter = ",")