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
import scipy.interpolate as interpolate

#%%

# inputs
startState = (45,225,0) #tuple(map(int, start.split(",")))
goalState = (225,105,0) #tuple(map(int, goal.split(",")))
rpm = (30*0.1047198,60*0.1047198,90*0.1047198) #tuple(map(int, rpm.split(",")))

c = 5 + 19 # clearance + R (robot radius)
dt = 0.6

r = 6.45/2
L = 11

#%%
## reading map
map_raw = cv2.bitwise_not(cv2.imread('map.jpg',cv2.IMREAD_GRAYSCALE))
plt.figure(1)
plt.imshow(map_raw)

#Clearance
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6*c+1, 6*c+1))
mapN = cv2.dilate(map_raw, kernel)

# walls
mapN[:,0:3*c] = 255
mapN[:,-3*c:] = 255
mapN[0:3*c,:] = 255
mapN[-3*c:,:] = 255

cv2.imwrite("map_bloated.jpg",cv2.bitwise_not(cv2.resize(mapN,(900,900))))

plt.figure(2)
plt.imshow(mapN)

#%%
# Checker if a node falls in the obstacle space
def checkFeasibility(node):
    try:
        a = mapN[np.shape(mapN)[1]-int(round(3*node[1])),int(round(3*node[0]))]
    except:
        return False
    
    if a < 255:
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
xy_points = np.array(backTrack)
xy_points_orig = np.array(backTrack)

def pointSplitter(p1,p2):
    X = np.rint(np.linspace(p1[0], p2[0],num=30))
    Y = np.rint(np.linspace(p1[1], p2[1],num=30))
    P = np.column_stack((X,Y))
    
    for i in P:
        if not checkFeasibility(i):
            return False
    return True

i = 0
while True:  
    if pointSplitter(xy_points[i],xy_points[i+2]):
            xy_points = np.delete(xy_points, (i+1), axis=0)
    else:
        i=i+1    
    if i >= len(xy_points)-2:
        break
    
    
          
plt.figure(3)
plt.imshow(mapN,cmap='Greys')
plt.scatter(3*xy_points_orig[:,0], 900-3*xy_points_orig[:,1], c='g',s=2)
plt.scatter(3*xy_points[:,0], 900-3*xy_points[:,1], c='r',s=4)
xy_points = np.delete(xy_points, (2), axis=1)
plt.savefig("map_path.jpg")

ang = [np.deg2rad(startState[2])]
for i in range(len(xy_points)-1):
    ang.append(np.arctan2(xy_points[i+1][1]-xy_points[i][1],xy_points[i+1][0]-xy_points[i][0]))
    
xyt_points = np.column_stack((xy_points,ang))
print(xyt_points)
#%%
max_W = 6 # rad/sec
timeStep = 0.02 #seconds
timeWait = 0.5 # seconds


W_turn = np.array([0,0])
time = 0

for i in range(len(xyt_points)-1):

    # Turning
    angle_turn = xyt_points[i+1,2]-xyt_points[i,2]
    
    dT = (abs(angle_turn)*L)/(2*r*max_W)

    W_turn_ = max_W*np.ones(int(np.round(dT/timeStep)))
    W_turn_ = np.sign(angle_turn)*np.column_stack((-W_turn_,W_turn_))
    W_turn_ = np.vstack((W_turn_,np.tile([0,0], ( round(timeWait/timeStep) ,1)  )))
    W_turn = np.vstack((W_turn,W_turn_))
    
    # Moving
    dist = np.sqrt( (xyt_points[i+1,0]-xyt_points[i,0])**2 + (xyt_points[i+1,1]-xyt_points[i,1])**2)
    
    dT = (2*dist)/(L*max_W)
    W_turn_ = max_W*np.ones(int(np.round(dT/timeStep)))
    W_turn_ = np.column_stack((W_turn_,W_turn_))
    W_turn_ = np.vstack((W_turn_,np.tile([0,0], ( round(timeWait/timeStep) ,1)  )))
    W_turn = np.vstack((W_turn,W_turn_))

    
plt.figure(4)
plt.plot(W_turn)
#plt.show()

#%%
np.savetxt("vel_traj.csv", W_turn,delimiter = ",")
