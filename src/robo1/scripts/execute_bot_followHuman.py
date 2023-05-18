#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import numpy as np
import rospkg
import csv
from std_msgs.msg import Float32MultiArray
import cv2
import copy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion



###############################################################################
# Global variables
line_idx = 0
r = 6.45/2
L = 19.2

obst_radius = 10 + 19
obst = None
goal = None

stopFlag = False

global_planner = True
init_local = True

rospy.init_node('pose_follower')

timeNow = rospy.Time.now()
###############################################################################

def obst_callback(msg):
    global obst

    obst = np.array(msg).reshape(-1,2)

    draw_obst(obst)
    return

def goal_callback(msg):
    global goal
    global stopFlag
    global timeNow

    if stopFlag:
        if rospy.Time.now() - timeNow > rospy.Duration.from_sec(1):
            stopFlag = False
            return
        else:
            return

    goal = [msg.x , msg.y]

    angle_diff = np.arctan2( msg.y , msg.x )
    vel = 15 if np.cos(angle_diff) > 0 else 0

    if msg.theta != 0:
        publish_this( vel , -1.2* angle_diff)
        pass
    else:
        stopFlag = True
        timeNow = rospy.Time.now()
        publish_this( 0,0 )
    
    draw_goal(msg)
    return

###############################################################################  

###### Supporting Functions


def draw_obst(obstList):
    marker_msg = Marker()
    marker_msg.ns = "point_ns"
    marker_msg.type = Marker.CYLINDER
    marker_msg.action = Marker.ADD
    marker_msg.pose.orientation.w = 1.0

    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.3

    marker_msg.color.a = 1.0
    marker_msg.color.r = 0.9
    marker_msg.color.g = 0.1
    marker_msg.color.b = 0.0

    marker_array_msg = MarkerArray()

    for i, p in enumerate(obstList):
        point_msg = Point()
        point_msg.x = p[0] / 100
        point_msg.y = -p[1] / 100
        point_msg.z = 0
        
        marker_msg.header.frame_id = "dummy_base_link"
        marker_msg.header.stamp = rospy.Time.now()
        
        marker_msg.points.append(point_msg)
        marker_msg.id = i
        marker_array_msg.markers.append(marker_msg)
    
    obst_marker_pub.publish(marker_array_msg)

def draw_goal(msg):
    x = msg.x
    y = msg.y

    marker = Marker()
    marker.header.frame_id = "dummy_base_link"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = -y / 100
    marker.pose.position.y = x / 100
    marker.pose.position.z = 0.2
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.4
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime.secs = 0

    goal_marker_pub.publish(marker)

###############################################################################

def publish_this(vel,ang_vel):
    targ_vel = Pose2D()
    targ_vel.x = vel
    targ_vel.y = ang_vel
    targ_vel_pub.publish(targ_vel)


if __name__ == '__main__':

    # Set up a publishers for the /TARG_VEL topic and state
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)
    goal_marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1, latch=False)
    obst_marker_pub = rospy.Publisher('/obst_marker', MarkerArray, queue_size=1, latch=True)

    goal_sub = rospy.Subscriber('/goal_cone', Pose2D, goal_callback)
    obstacle_sub = rospy.Subscriber('/obst_cone', Float32MultiArray, obst_callback)

    rospy.spin()
