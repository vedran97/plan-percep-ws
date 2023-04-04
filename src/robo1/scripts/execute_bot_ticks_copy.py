#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np

curr_ticks = 0
target_pose = Pose2D()

flagTurn = False
flagGo = False

turnTicks = 368
goTicks = 1221

poseM = Pose2D()
poseM.theta = 0
poseM.x = 2
poseM.y = -2

poseS = Pose2D()
poseS.theta = 0
poseS.x = 0
poseS.y = 0

def curr_ticks_callback(msg):
    global flagTurn
    if flagTurn:
        return
    
    global poseM
    global poseS
    global curr_ticks

    curr_ticks = curr_ticks + (abs(msg.x) + abs(msg.y)) /2

    print(curr_ticks)

    if curr_ticks < 368:
        targ_vel_pub.publish(poseM)
    else:
        targ_vel_pub.publish(poseS)
        curr_ticks = 0
        flagTurn =  True
        rospy.sleep(1) 
        flagTurn = False
        print("YO")


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pose_follower')

    # Set up a publishers for the /TARG_VEL topic and state
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=1)

    curr_odom_sub = rospy.Subscriber('/CURR_VEL', Pose2D, curr_ticks_callback)
    

    rospy.spin()