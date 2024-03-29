#!/usr/bin/env python

from time import sleep
import rospy
from geometry_msgs.msg import Pose2D
from terpbot_msgs.msg import Gains
from std_msgs.msg import Bool
import numpy as np
# import matplotlib.pyplot as plt

PUB_TOPIC = 'TARG_VEL'
GAINS_TOPIC = 'GAINS'

rospy.init_node('TARGET_PUBLISHER')
pose_publisher = rospy.Publisher(PUB_TOPIC, Pose2D, queue_size=10)
gains_publisher = rospy.Publisher(GAINS_TOPIC, Gains, queue_size=10)
rospy.loginfo("Publishes target and gains to arduino")

NO_OF_WAYPOINTS = 350
CONTROL_FREQ = 50
TICKS_PER_REV= 495
DOUBLE_PI = 2*np.pi
DIA_WHEEL = 64.5/1000 #(converting it to meter)
MAX_RPM = 140
VMAX = (MAX_RPM/60)*DOUBLE_PI*(DIA_WHEEL/2)

print("\r\nVMAX:{}".format(VMAX))

def convertLinearToRPS(linearVelocity):
    return linearVelocity/(DIA_WHEEL/2)

def convertRPSToLinear(RPS):
    return RPS*(DIA_WHEEL/2)

def getXLinearVelocity(t,T):
    return  t*(4*VMAX/T)+(-(4*VMAX/(T*T))*t*t)

def getT( xInitial,xFinal):
    return (6.0/(4.0*VMAX))*(xFinal - xInitial)


class Trajectory:
    wayPoints = []
    noOfWayPoints=NO_OF_WAYPOINTS
    totalTime=0
    def __init__(self):
        time = 0.0
        loopRate = CONTROL_FREQ
        timeIncrement = 1/loopRate
        xInit = 0;  
        xFinal = 1.0; #// 1 rotation of the wheel //0.20263272615
        self.totalTime = getT(xInit,xFinal)
        i=0
        while (True):
            self.wayPoints.append(convertLinearToRPS(getXLinearVelocity(time,self.totalTime))*(TICKS_PER_REV/DOUBLE_PI)*(1/CONTROL_FREQ))
            time += timeIncrement
            if(time>self.totalTime):
                self.wayPoints.append(0.0)
                break
            else:
                i+=1
        self.noOfWayPoints = i

wayPoints = Trajectory()

print("\r\nTOTAL_TRAJ_TIME , NO_OF_WAYPOINTS:{},{}\r\n".format(wayPoints.totalTime,wayPoints.noOfWayPoints))


# Create a new figure, plot into it, then close it so it never gets displayed

# plt.ioff()

# Create a new figure, plot into it, then close it so it never gets displayed
# fig = plt.figure()
# plt.plot(wayPoints.wayPoints)
# plt.savefig('./waypoints.png')
# plt.close(fig)

pose_msg = Pose2D()
pose_msg.x =  0.0 # Left Motor Target
pose_msg.y =  0.0 # Right Motor Target
pose_msg.theta = 0 # Empty Array

rate = rospy.Rate(CONTROL_FREQ)

## ROTATION GAINS:
# left_KU = 35
# left_TU = 0.1481
# left_kd_coeff = 0.085

# right_KU = 42
# right_TU = 0.1481
# right_kd_coeff = 0.085

# TRANSLATION GAINS:
left_KU = 32.5
left_TU = 0.1481
left_kd_coeff = 0.075

right_KU = 35
right_TU = 0.20
right_kd_coeff = 0.085

leftGain = Gains()

leftGain.kp = 0.6*left_KU
leftGain.ki = 1.2*left_KU/left_TU
leftGain.kd = left_kd_coeff*left_KU*left_TU
leftGain.i_clamp = 100.0
leftGain.isleft = True


rightGain = Gains()
rightGain.kp =  0.6*right_KU
rightGain.ki = 1.2*right_KU/left_TU
rightGain.kd = right_kd_coeff*right_KU*right_TU
rightGain.i_clamp = 100.0
rightGain.isleft = False

# '{kp: 30.0, ki: 0.05, kd: 0.1, i_clamp: 100.0, isleft: false}

while not rospy.is_shutdown():
    rospy.loginfo("Sending Gains:")
    rospy.loginfo("Left Motor Gains:\r\n{}".format(leftGain))
    rospy.loginfo("Right Motor Gains:\r\n{}".format(rightGain))
    rospy.loginfo("WAYPOINT TIME:\r\n{}".format(wayPoints.totalTime))
    for i in range(20):
        gains_publisher.publish(leftGain)
        rate.sleep()

    for i in range(20):
        gains_publisher.publish(rightGain)
        rate.sleep()


    rospy.loginfo("Finished sending gains")
    pose_msg.x = 0
    pose_msg.y = 0
    pose_msg.theta = 1
    pose_publisher.publish(pose_msg)
    sleep(1)
    for wp in wayPoints.wayPoints:
        pose_msg.x = wp
        pose_msg.y = +wp
        pose_msg.theta = 1
        pose_publisher.publish(pose_msg)
        rate.sleep()
    print("\r\nSTOPPING THE TRAJGEN NOW\r\n")
    for i in range(20):
        pose_msg.theta = 0
        pose_publisher.publish(pose_msg)
        rate.sleep()
    sleep(1)
    break

rospy.signal_shutdown("\r\nExiting the process")
exit()