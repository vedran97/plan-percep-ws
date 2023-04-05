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
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d

#%%

# inputs
startState_real = (45,45,0) #tuple(map(int, start.split(",")))
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
print(np.shape(map_raw))
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

#cv2.imwrite("map_bloated.jpg",cv2.bitwise_not(cv2.resize(mapN,(900,900))))

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
def interpol(nodeP,shifter,dtime,k):
    points = []
    x,y,t = nodeP
    t = np.deg2rad(t)
    i = shifter
    dt = dtime/k
    
    for I in range(k):
        t_ = t + dt*r*(i[0]-i[1])/L
        if i[0] == i[1]:
            x_ = x + 0.5*(i[0]+i[1])*r*dt*np.cos(t)
            y_ = y + 0.5*(i[0]+i[1])*r*dt*np.sin(t)
        else:
            F = (L/2)*(i[0]+i[1])/(i[0]-i[1])
            x_ = x + F*( np.sin( (r*(i[0]-i[1])*dt/L) +t ) -np.sin(t))
            y_ = y - F*( np.cos( (r*(i[0]-i[1])*dt/L) +t ) -np.cos(t))
        x,y,t = x_,y_,t_
        points.append([x,y,np.rad2deg(t)])
    
    return np.array(points)
#%%
k=3
plane = cv2.resize(cv2.bitwise_not(mapN),(k*mapN.shape[0],k*mapN.shape[1]),interpolation = cv2.INTER_AREA)
plane = np.dstack((plane,plane,plane))

CN = np.array(list(closedNodes.keys()))*k
for i in CN:
    plane[k*mapN.shape[0]-int(round(i[1])),int(round(i[0])),:] = np.array([0,200,0])
ON = np.array(list(openNodes.keys()))*k
for i in ON:
    plane[k*mapN.shape[0]-int(round(i[1])),int(round(i[0])),:] = np.array([200,200,0])

pointL = np.array(backTrack)[0:-1,:]*k
fullPath = []

for i in range(len(pointL)-1):
    start_point = (int(round(pointL[i][0])),k*mapN.shape[1]-int(round(pointL[i][1])))
    end_point = (int(round(pointL[i+1][0])),k*mapN.shape[1]-int(round(pointL[i+1][1])))
    #plane = cv2.line(plane, start_point, end_point, (250,0,0), 2)
    k_plan = interpol(backTrack[i], nodeList[backTrack[i+1]], dt, 30)
    fullPath.append(k_plan)
    
    plane = cv2.line(plane, (k*int(round(startState_real[0])),k*mapN.shape[1]-k*int(round(startState_real[1]))),
                     (k*int(round(startState[0])),k*mapN.shape[1]-k*int(round(startState[1]))),
                     (250,0,0), 1)
    
    for j in k_plan:
        j = j*k
        plane = cv2.circle(plane,(int(round(j[0])),k*mapN.shape[1]-int(round(j[1]))),
                           0,(250,0,0), -1)

plt.figure(3)
plt.imshow(plane)
cv2.imwrite("path.jpg",plane)
fullPath = np.vstack(fullPath)

#%%

fullPath = np.apply_along_axis(lambda x: gaussian_filter1d(x, sigma=10, mode='nearest', cval=0), 
                               axis=0, arr=fullPath)

plt.figure(4)
plt.plot(fullPath)
#%%
average_rpm = 30*0.1047198
time = (2*push)/(r*average_rpm)

vel_profile = [[0,0,0,np.deg2rad(startState[2])],
               [average_rpm,average_rpm,time,np.deg2rad(startState[2])]]


for i in range(len(fullPath)-1):
    Pn = fullPath[i]
    Pn1= fullPath[i+1]
    
    #dist = np.sqrt((Pn[0]-Pn1[0])**2 + (Pn[1]-Pn1[1])**2)
    #d_time = dist/5 # 5cm/sec speed
    
    theta = np.arctan2(Pn1[1]-Pn[1],Pn1[0]-Pn[0])
    
    if abs(Pn[2]-Pn1[2]) > 0.00001:
        time_step = [(Pn1[0]-Pn[0])/(average_rpm*r*np.cos(theta)) , (Pn1[1]-Pn[1])/(average_rpm*r*np.sin(theta))]
        time_av = np.mean(time_step)
    else:
        time_av = np.sqrt((Pn[0]-Pn1[0])**2 + (Pn[1]-Pn1[1])**2)/10
    theta_dot = np.deg2rad(Pn1[2]-Pn[2])/time_av
    
    Ul = (average_rpm) + (theta_dot*L/(2*r))
    Ur = (average_rpm) - (theta_dot*L/(2*r))
    
    time = time + time_av
    
    vel_profile.append([Ul,Ur,time,theta])

vel_profile = np.array(vel_profile)
plt.figure(5)
plt.plot(vel_profile[:,2],vel_profile[:,np.array([True, True, False, True])])
plt.show()
#%%
timeRate = 0.02  #50 hertz

f_Ul = interp1d(vel_profile[:,2], vel_profile[:,0], kind='linear')
f_Ur = interp1d(vel_profile[:,2], vel_profile[:,1], kind='linear')

new_time = np.arange(0, vel_profile[-1,2], timeRate)
# Interpolate the x,y values to the desired time steps
new_x, new_y = f_Ul(new_time).T , f_Ur(new_time).T

vel_profile_wheels = np.column_stack((new_x, new_y, new_time))

#%%
np.savetxt("vel_traj.csv", vel_profile_wheels,delimiter = ",")