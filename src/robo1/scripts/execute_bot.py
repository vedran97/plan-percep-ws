#!/usr/bin/env python3


# for battery voltage 16.7V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.6 radians per sec  9.52 reverse
# for battery voltage 16.25V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.25 radians per sec
# for batt volt 16.7V left to right ratio  of 1.206 and additional turn ratio of 1.1, pwm 75,98  turning angular vel = 5 right turn
# for batt volt 16.7V , pwm 85,90.45  turning angular vel = 5 left turn

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np

curr_pose = Pose2D()
target_pose = Pose2D()
state = 0
flag1 = False
flag2 = False


# STATE is running variable, for keeping track of operations
# -1 -> Next position coordinates given (this will be changed by some other node to 1)
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
            targ_vel_pub.publish(pose)
            flag2 = True
            state_pub.publish(5)


def runBot_turn( sign ):
    pose = Pose2D()
    pose.x = 3 * sign
    pose.y = 3 * sign
    pose.theta = 0

    targ_vel_pub.publish(pose)

def runBot_forward():

    pose = Pose2D()
    pose.x = -6
    pose.y = 6
    pose.theta = 0

    targ_vel_pub.publish(pose)


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pose_follower')

    # Set up a publishers for the /TARG_VEL topic and state
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)
    state_pub = rospy.Publisher('/STATE', Int32, queue_size=10)

    while not rospy.is_shutdown():
        curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, curr_odom_callback)
        next_pos_sub = rospy.Subscriber('/NEXT_POS', Pose2D, next_pos_callback)

        if not flag1:
            state_pub.publish(0)

    rospy.spin()