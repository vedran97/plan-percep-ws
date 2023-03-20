#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

rospy.init_node('TARGET_PUBLISHER')
pose_publisher = rospy.Publisher('TARG_VEL', Pose2D, queue_size=10)

pose_msg = Pose2D()
pose_msg.x = 0.1 # Left Motor Targer
pose_msg.y = 0.1 # Right Motor Target
pose_msg.theta = 0 # Empty Array

rate = rospy.Rate(50) # 50 Hz
while not rospy.is_shutdown():
    pose_publisher.publish(pose_msg)
    rate.sleep()