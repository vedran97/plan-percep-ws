#!/usr/bin/env python3

import numpy as np
import time
import cv2
import matplotlib.pyplot as plt
from sys import exit

#%%

# inputs
startState = (1,80,0) #tuple(map(int, start.split(",")))
goalState = (299 , 100 , 0) #tuple(map(int, goal.split(",")))
rpm = (15*0.1047198,20*0.1047198,25*0.1047198) #tuple(map(int, rpm.split(",")))

c = 4 + 11 # clearance + R (robot radius)
dt = 1

r = 6.6/2
L = 16

#%%
## reading map
map_raw = cv2.bitwise_not(cv2.imread('comp_map_v2.png',cv2.IMREAD_GRAYSCALE))

#Clearance
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6*c+1, 6*c+1))
mapN = cv2.dilate(map_raw, kernel)

# walls
# mapN[:,0:3*c] = 255
# mapN[:,-3*c:] = 255
# mapN[0:3*c,:] = 255
# mapN[-3*c:,:] = 255

cv2.imwrite("map_bloated.jpg",cv2.bitwise_not(mapN))


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
plt.show()


import rospy
import time
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import numpy as np
import rospkg
import csv
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray
import cv2
import copy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion

# Construct path to CSV file

data_raw = xy_points
data = data_raw - data_raw[0]

###############################################################################
# Global variables
line_idx = 0
r = 6.6/2
L = 16

global_planner = True
init_run = True
start_cond = Odometry()
ang_i = None

rospy.init_node('pose_follower')

###############################################################################
# Global planner
def global_planner(msg):
    global target_pose
    global line_idx
    global init_run
    global start_cond
    global ang_i

    if init_run:
        start_cond = msg
        angles = msg.pose.pose.orientation
        (_, _, theta) = euler_from_quaternion ([angles.x,angles.y,angles.z,angles.w])
        ang_i = theta

        init_run = False
        return
    
    X = (msg.pose.pose.position.x * 100) - (start_cond.pose.pose.position.x * 100)
    Y = (msg.pose.pose.position.y * 100) - (start_cond.pose.pose.position.y * 100)
    angles = msg.pose.pose.orientation

    (_, _, THETA) = euler_from_quaternion ([angles.x,angles.y,angles.z,angles.w])
    THETA = THETA - ang_i
    
    if line_idx > len(data)-1:
        publish_this(0,0)
        print("reached!")
        return
    
    target_pose = np.float32(data[line_idx])
    if ((target_pose[1] - Y)**2 + (target_pose[0] - X)**2)**0.5 < 4:
        line_idx = line_idx + 1
        print("temp goal ", line_idx, " reached!")

    curr_heading = THETA
    need_heading = np.arctan2(target_pose[1] - Y , target_pose[0] - X)
    angle_diff = (need_heading - curr_heading)

    vel = 0.14 if np.cos(angle_diff) > 0 else 0

    publish_this( vel , 0.9* angle_diff)


###############################################################################

def publish_this(vel,ang_vel):
    targ_vel = Twist()
    targ_vel.linear.x = vel
    targ_vel.angular.z = ang_vel
    targ_vel_pub.publish(targ_vel)

targ_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

while not rospy.is_shutdown():

    # Set up a publishers for the /TARG_VEL topic and state
    
    curr_odom_sub = rospy.Subscriber('/odom', Odometry, global_planner)

    rospy.spin()