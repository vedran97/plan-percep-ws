#!/usr/bin/env python3


# for battery voltage 16.7V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.6 radians per sec  9.52 reverse
# for battery voltage 16.25V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.25 radians per sec
# for batt volt 16.7V left to right ratio  of 1.206 and additional turn ratio of 1.1, pwm 75,98  turning angular vel = 5 right turn
# for batt volt 16.7V , pwm 85,90.45  turning angular vel = 5 left turn

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import csv
import numpy as np

curr_pose = Pose2D()
target_pose = Pose2D()
state = 0


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
    target_pose = msg

def state_callback(msg):
    global state
    state = msg.data

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pose_follower')

    # Subscribe
    curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, curr_odom_callback)
    next_pos_sub = rospy.Subscriber('/NEXT_POS', Pose2D, next_pos_callback)
    state_sub = rospy.Subscriber('/STATE', Int32, state_callback)

    # Set up a publisher for the /TARG_VEL topic
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)
    state_pub = rospy.Publisher('/STATE', Int32, queue_size=10)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        if state == -1:
            state_pub.publish(2)
            continue

        if state == 2:
            if abs(curr_pose.theta - target_pose.theta) > 0.1:

                pose = Pose2D()
                pose.x = -5 * np.sign((curr_pose.theta - target_pose.theta))
                pose.y = 5 * np.sign((curr_pose.theta - target_pose.theta))
                pose.theta = 0

                targ_vel_pub.publish(pose)
            else:
                pose = Pose2D()
                pose.x = 0
                pose.y = 0
                pose.theta = 0
                targ_vel_pub.publish(pose)

                state_pub.publish(4)
                rospy.sleep(1)

        if state == 4:
            dx = target_pose.x - curr_pose.x
            dy = target_pose.y - curr_pose.y
            distance = ((dx ** 2) + (dy ** 2)) ** 0.5

            if abs(distance) > 1:

                pose = Pose2D()
                pose.x = 5
                pose.y = 5
                pose.theta = 0

                targ_vel_pub.publish(pose)
            else:
                pose = Pose2D()
                pose.x = 0
                pose.y = 0
                pose.theta = 0
                targ_vel_pub.publish(pose)

                state_pub.publish(1)
                rospy.sleep(1)

        # Sleep to maintain the loop rate
        rate.sleep()