#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState
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
    vel = data.velocity

    velX = round(vel[0]*5)/5
    velY = round(vel[1]*5)/5

    # Add velocity values to current odometry values
    curr_odom.x = curr_odom.x - (((6.45/2)/2)* (velX - velY)*np.cos(curr_odom.theta))*0.02
    curr_odom.y = curr_odom.y - (((6.45/2)/2)* (velX - velY)*np.sin(curr_odom.theta))*0.02
    curr_odom.theta = curr_odom.theta + (((6.45/2)/11)*(velX + velY))*0.02

    curr_odom.theta = ((curr_odom.theta + math.pi)%(2*math.pi)) - math.pi

    # Publish updated Pose2D message to '/CURR_ODOM' topic
    curr_odom_publisher.publish(curr_odom)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('odom_vel_addition', anonymous=True)

    # Create publisher for '/CURR_ODOM' topic with 'Pose2D' message type
    curr_odom_publisher = rospy.Publisher('/CURR_ODOM', Pose2D, queue_size=10)

    # Subscribe to '/CURR_VEL' topic with 'Pose2D' message type
    rospy.Subscriber('/robo1/joint_states', JointState, curr_vel_callback)

    # Spin until node is stopped
    rospy.spin()
