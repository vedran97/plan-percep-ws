#!/usr/bin/env python3


# for battery voltage 16.7V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.6 radians per sec  9.52 reverse
# for battery voltage 16.25V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.25 radians per sec
# for batt volt 16.7V left to right ratio  of 1.206 and additional turn ratio of 1.1, pwm 75,98  turning angular vel = 5 right turn
# for batt volt 16.7V , pwm 85,90.45  turning angular vel = 5 left turn

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np

curr_ticks = 0
target_pose = Pose2D()
state = 0

r = 6.45/2
L = 19.2

flagState = False

rospy.init_node('pose_follower')

# STATE is running variable, for keeping track of operations
# -1 -> Next position coordinates given (this will be changed by some other node to 1)
# 0  -> Home/start state
# 1  -> Waiting for next position coordinates, reached last position
# 2  -> Rotating 
# 3  -> Done rotating, will start translation next
# 4  -> Translating
# 5  -> Done Translating, will start waiting for next coordinate


def curr_odom_callback(msg):
    global target_pose

    curr_heading = msg.theta
    need_heading = np.arctan2(target_pose.y - msg.y , target_pose.x - msg.x)

    targ_vel = Pose2D()
    targ_vel.x = 4
    targ_vel.y = 0.5 * (need_heading - curr_heading)

    targ_vel_pub.publish(targ_vel)
                
# Callback function for the /NEXT_POS topic
def next_pos_callback(msg):
    global target_pose
    global flagState

    if not flagState:
        target_pose = msg

        flagState = True

if __name__ == '__main__':

    # Set up a publishers for the /TARG_VEL topic and state
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)
    state_pub = rospy.Publisher('/STATE', Int32, queue_size=10)

    next_pos_sub = rospy.Subscriber('/NEXT_POS', Pose2D, next_pos_callback)
    curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, curr_odom_callback)

    rospy.spin()