#!/usr/bin/env python3

import csv
import rospy
from std_msgs.msg import Float64
import rospkg
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np


curr_pose = Pose2D()
target_pose = Pose2D()
state = 0
flag1 = False
flag2 = False


# STATE is running variable, for keeping track of operations
# -1 -> Next position coordinates given (this will be changed by other node to 1)
# 0  -> Home/start state
# 1  -> Waiting for next position coordinates, reached last position
# 2  -> Rotating 
# 3  -> Done rotating, will start translation next
# 4  -> Translating
# 5  -> Done Translating, will start waiting for next coordinate



def curr_odom_callback(msg):
    global curr_pose
    curr_pose = msg

# Callback function for the /NEXT_POS topic
def next_pos_callback(msg):

    global target_pose
    global curr_pose
    global flag1
    global flag2

    if flag2 and msg==target_pose:
        return

    flag1 = True
    flag2 = False
    target_pose = msg

    dx = target_pose.x - curr_pose.x
    dy = target_pose.y - curr_pose.y
    distance = ((dx ** 2) + (dy ** 2)) ** 0.5

    angle = np.arctan2(dy,dx)

    if abs(angle-curr_pose.theta) > 0.04:
        dirxn = np.sign(angle-curr_pose.theta)
        runBot_turn(dirxn)
        state_pub.publish(2)

    elif distance > 1:
        runBot_forward()
        state_pub.publish(4)

    else:
        print("reached cord")
        pose = Pose2D()
        pose.x = 0
        pose.y = 0
        pose.theta = 0

        seconds = rospy.get_time()
        while (rospy.get_time() - seconds) <4:
            pub1.publish(pose.x)
            pub2.publish(pose.y)
            flag2 = True
            state_pub.publish(5)


def runBot_turn( sign ):
    pose = Pose2D()
    pose.x = 3 * sign
    pose.y = 3 * sign
    pose.theta = 0

    pub1.publish(pose.x)
    pub2.publish(pose.y)

def runBot_forward():

    pose = Pose2D()
    pose.x = -6
    pose.y = 6
    pose.theta = 0

    pub1.publish(pose.x)
    pub2.publish(pose.y)


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pose_follower')

    # Set up a publisher for the /STATE topic
    state_pub = rospy.Publisher('/STATE', Int32, queue_size=10)

    # Set up publishers for the two topics
    pub1 = rospy.Publisher('/robo1/driveL_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/robo1/driveR_controller/command', Float64, queue_size=10)

    # Subscribe

    while not rospy.is_shutdown():
        curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, curr_odom_callback)
        next_pos_sub = rospy.Subscriber('/NEXT_POS', Pose2D, next_pos_callback)

        if not flag1:
            state_pub.publish(0)

    rospy.spin()