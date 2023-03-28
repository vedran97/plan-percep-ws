#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import numpy as np
import math

timeStep = 0.02 #50 Hz
r = 6.45/2
L = 11

# global variable to store the current odometry values
curr_odom = Pose2D()

curr_odom.x = 0.0
curr_odom.y = 0.0
curr_odom.theta = 0.0

def curr_vel_callback(data):
    # Read velocity data from '/CURR_VEL' topic
    vel = data

    # Add velocity values to current odometry values
    curr_odom.x = curr_odom.x + ((6.45/2)/2)* (vel.x + vel.y)*np.cos(curr_odom.theta)
    curr_odom.y = curr_odom.y + ((6.45/2)/2)* (vel.x + vel.y)*np.sin(curr_odom.theta)
    curr_odom.theta = curr_odom.theta + ((6.45/2)/11)*(vel.x - vel.y)

    curr_odom.theta = ((curr_odom.theta + math.pi)%(2*math.pi)) - math.pi

    # Publish updated Pose2D message to '/CURR_ODOM' topic
    curr_odom_publisher.publish(curr_odom)

def curr_odom_callback(data):
    # Read current odometry values from '/CURR_ODOM' topic
    global curr_odom
    curr_odom = data

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('odom_vel_addition', anonymous=True)

    # Create publisher for '/CURR_ODOM' topic with 'Pose2D' message type
    curr_odom_publisher = rospy.Publisher('/CURR_ODOM', Pose2D, queue_size=10)

    # Subscribe to '/CURR_VEL' topic with 'Pose2D' message type
    rospy.Subscriber('/CURR_VEL', Pose2D, curr_vel_callback)

    # Subscribe to '/CURR_ODOM' topic with 'Pose2D' message type
    rospy.Subscriber('/CURR_ODOM', Pose2D, curr_odom_callback)

    # Spin until node is stopped
    rospy.spin()
