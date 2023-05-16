#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import numpy as np
import math
from std_msgs.msg import Float64

r = 6.45/2
L = 19.2
firstRun_flag = True

# global variable to store the current odometry values

def curr_vel_callback(data):
    global timeGap
    global curr_odom
    global firstRun_flag

    firstRun_flag = False

    ticks = data

    rate = rospy.get_time() - timeGap

    velX = - ticks.x * (2 * 3.141596 / 495) / rate
    velY = ticks.y * (2 * 3.141596 / 495) / rate

    timeGap = rospy.get_time()

    # Publish updated Pose2D message to '/CURR_ODOM' topic
    pub1.publish(float(velX))
    pub2.publish(float(velY))

if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node('driver_gazebo', anonymous=True)

    if firstRun_flag:
        timeGap = rospy.get_time()

    # Create publisher for '/Drive' topics for Gazebo
    pub1 = rospy.Publisher('/robo1/driveL_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/robo1/driveR_controller/command', Float64, queue_size=10)

    # Subscribe to '/CURR_VEL' topic with 'Pose2D' message type
    rospy.Subscriber('/CURR_VEL', Pose2D, curr_vel_callback)

    # Spin until node is stopped
    rospy.spin()
