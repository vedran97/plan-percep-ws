#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import numpy as np
import math

r = 6.45/2
L = 11
flag = True
flagVel = True

velX_old = 0
velY_old = 0

# global variable to store the current odometry values
curr_odom = Pose2D()

curr_odom.x = 0.0
curr_odom.y = 0.0
curr_odom.theta = 0.0

def curr_vel_callback(data):
    global timeGap
    global velX_old
    global velY_old
    global flagVel

    # Old velocity at first run is not important
    if flagVel:
        velX_old = data.x
        velY_old = data.y
        flagVel = False

    # Read velocity data from '/CURR_VEL' topic
    vel = data

    # Some wierd rounding method by me
    velX = vel.x #round(vel.x*5)/5
    velY = vel.y #round(vel.y*5)/5

    if abs(velX-velX_old) > abs(velX) and velX!=0 and velX_old!=0:
        velX = velX_old
    if abs(velY-velY_old) > abs(velY) and velY!=0 and velY_old!=0:
        velY = velY_old

    # Add velocity values to current odometry values
    rate = rospy.get_time() - timeGap

    curr_odom.x = curr_odom.x + (((6.45/2)/2)* (velX + velY)*np.cos(curr_odom.theta))* rate
    curr_odom.y = curr_odom.y + (((6.45/2)/2)* (velX + velY)*np.sin(curr_odom.theta))* rate
    curr_odom.theta = curr_odom.theta + (((6.45/2)/11)*(velX - velY))* rate

    #curr_odom.theta = ( curr_odom.theta % 2*math.pi )

    timeGap = rospy.get_time()

    # Publish updated Pose2D message to '/CURR_ODOM' topic
    curr_odom_publisher.publish(curr_odom)

    velX_old = velX
    velY_old = velY


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
