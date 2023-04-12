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
startState = (1,150,0) #tuple(map(int, start.split(",")))
goalState = (399,50,0) #tuple(map(int, goal.split(",")))
rpm = (15*0.1047198,17.5*0.1047198,20*0.1047198) #tuple(map(int, rpm.split(",")))

c = 5 + 19 # clearance + R (robot radius)
dt = 2

r = 6.45/2
L = 11

#%%
## reading map
map_raw = cv2.bitwise_not(cv2.imread('map.png',cv2.IMREAD_GRAYSCALE))
plt.figure(1)
plt.imshow(map_raw)

#Clearance
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6*c+1, 6*c+1))
mapN = cv2.dilate(map_raw, kernel)

# walls
# mapN[:,0:3*c] = 255
# mapN[:,-3*c:] = 255
# mapN[0:3*c,:] = 255
# mapN[-3*c:,:] = 255

cv2.imwrite("map_bloated.jpg",cv2.bitwise_not(mapN))

plt.figure(2)
plt.imshow(mapN)

#%%
# Checker if a node falls in the obstacle space
def checkFeasibility(node):
    try:
        a = mapN[np.shape(mapN)[0]-int(round(3*node[1])),int(round(3*node[0]))]
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
np.savetxt("vel_coord.csv", xy_points,delimiter = ",")
xy_points_orig = np.array(backTrack)
          
plt.figure(3)
plt.imshow(mapN,cmap='Greys')
plt.scatter(3*xy_points_orig[:,0], mapN.shape[0]-3*xy_points_orig[:,1], c='g',s=2)
plt.scatter(3*xy_points[:,0], mapN.shape[0]-3*xy_points[:,1], c='r',s=4)
xy_points = np.delete(xy_points, (2), axis=1)
plt.savefig("map_path.jpg")