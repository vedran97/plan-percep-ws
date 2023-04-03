#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import numpy as np
import math

r = 6.45/2
L = 11
flag = True

# global variable to store the current odometry values
curr_odom = Pose2D()

curr_odom.x = 0.0
curr_odom.y = 0.0
curr_odom.theta = 0.0

def curr_vel_callback(data):
    global timeGap
    vel = data

    # Some wierd rounding method by me
    velX = vel.x #round(vel.x*5)/5
    velY = vel.y #round(vel.y*5)/5

    rate = rospy.get_time() - timeGap

    curr_odom.x = curr_odom.x + (((6.45/2)/2)* (velX + velY)*np.cos(curr_odom.theta))* rate
    curr_odom.y = curr_odom.y + (((6.45/2)/2)* (velX + velY)*np.sin(curr_odom.theta))* rate
    curr_odom.theta = curr_odom.theta + (((6.45/2)/11)*(velX - velY))* rate

    curr_odom.theta = np.arctan2(np.sin(curr_odom.theta),np.cos(curr_odom.theta))

    timeGap = rospy.get_time()

    # Publish updated Pose2D message to '/CURR_ODOM' topic
    curr_odom_publisher.publish(curr_odom)



if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node('odom_vel_addition', anonymous=True)

    if flag:
        timeGap = rospy.get_time()
        flag = False

    # Create publisher for '/CURR_ODOM' topic with 'Pose2D' message type
    curr_odom_publisher = rospy.Publisher('/CURR_ODOM', Pose2D, queue_size=10)

    # Subscribe to '/CURR_VEL' topic with 'Pose2D' message type
    rospy.Subscriber('/CURR_VEL', Pose2D, curr_vel_callback)

    # Spin until node is stopped
    rospy.spin()
