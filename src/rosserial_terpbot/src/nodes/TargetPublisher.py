#!/usr/bin/env python

from time import sleep
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt

PUB_TOPIC = 'TARG_VEL'
START_TOPIC = 'START'
rospy.init_node('TARGET_PUBLISHER')
pose_publisher = rospy.Publisher(PUB_TOPIC, Pose2D, queue_size=10)
start_pub = rospy.Publisher(START_TOPIC, Bool, queue_size=10)
rospy.loginfo("Publishes target to arduino")

NO_OF_WAYPOINTS = 350
CONTROL_FREQ = 50
vmax =0.406 #//(m/s) actually some 0.406~

def convertLinearToRPS(linearVelocity):
    return linearVelocity*15.503875969

def convertRPSToLinear(RPS):
    return RPS*(1/15.503875969)

def getXLinearVelocity(t,T):
    return  t*(4*vmax/T)+(-(4*vmax/(T*T))*t*t)

def getT( xInitial,xFinal):
    return (6.0/(4.0*vmax))*(xFinal - xInitial)

class Trajectory:
    wayPoints = []
    noOfWayPoints=NO_OF_WAYPOINTS
    totalTime=0
    def __init__(self):
        time = 0.0
        loopRate = CONTROL_FREQ
        timeIncrement = 1/loopRate
        xInit = 0;  
        xFinal = 3.0; #// 1 rotation of the wheel //0.20263272615
        self.totalTime = getT(xInit,xFinal)
        i=0
        while (True):
            self.wayPoints.append(convertLinearToRPS(getXLinearVelocity(time,self.totalTime)))
            time += timeIncrement
            if(time>self.totalTime):
                self.wayPoints.append(0.0)
                break
            else:
                i+=1
        self.noOfWayPoints = i

wayPoints = Trajectory()

print("\r\n TOTAL_TRAJ_TIME , NO_OF_WAYPOINTS:{},{}".format(wayPoints.totalTime,wayPoints.noOfWayPoints))


# Create a new figure, plot into it, then close it so it never gets displayed

plt.ioff()

# Create a new figure, plot into it, then close it so it never gets displayed
fig = plt.figure()
plt.plot(wayPoints.wayPoints)
plt.savefig('./waypoints.png')
plt.close(fig)

pose_msg = Pose2D()
pose_msg.x =  0.0 # Left Motor Target
pose_msg.y =  0.0 # Right Motor Target
pose_msg.theta = 0 # Empty Array

rate = rospy.Rate(CONTROL_FREQ)

ctrl_msg = Bool()
ctrl_msg.data = True

# start_pub.publish(ctrl_msg)
sleep(1)
while not rospy.is_shutdown():
    for wp in wayPoints.wayPoints:
        pose_msg.x = wp
        pose_msg.y = wp
        pose_msg.theta = 1
        pose_publisher.publish(pose_msg)
        rate.sleep()
    print("STOPPING THE TRAJGEN NOW")
    # ctrl_msg.data=False
    # start_pub.publish(ctrl_msg)
    pose_msg.theta = 0
    pose_publisher.publish(pose_msg)
    sleep(1)
    break

rospy.signal_shutdown("Exiting the process")
exit()