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

# Function to calculate force at any position
def forceCalc(Q,PointSet):
    
    R = np.sum(np.square(PointSet),axis=1)**1.5
    R = np.hstack((R[:,np.newaxis],R[:,np.newaxis]))
    F = Q * np.divide( PointSet , R)
    F = np.sum(F,axis=0)
    
    return F

# Function to get point list
def circlify(obst):
    detObs_x = obst[0] + obst_radius*np.cos(np.linspace(0, 2*np.pi,150))
    detObs_y = obst[1] + obst_radius*np.sin(np.linspace(0, 2*np.pi,150))
    detObs = np.hstack( (detObs_x[:,np.newaxis],detObs_y[:,np.newaxis]) )
    
    return detObs

# Function to convert x,y to pixels (to plot)
def coord2pixel(point,mapN):
    pixel = (int(3*point[0]), mapN.shape[0]-int(3*point[1]))
    return pixel
def pixel2point(pixel,mapN):
    point = (pixel[0]/3 , (pixel[1]-mapN.shape[0])/3)
    return point
# Function to get force vector end point on image (for plotting)
def dirxPlotter(force,odom):
    scale = np.sqrt(np.sum(force**2))
    force = force/np.sqrt(np.sum(force**2))
    end_point = ( int(coord2pixel(odom + data_raw[0],map_)[0]+ scale*1000*force[0]),
                 int(coord2pixel(odom + data_raw[0],map_)[1]- scale*1000*force[1]) )
    
    return end_point

###############################################################################
# Global variables
line_idx = 0
r = 6.45/2
L = 19.2

obst_radius = 10 + 19
obst = None
goal = None

global_planner = True
init_local = True

rospy.init_node('pose_follower')

###############################################################################

def obst_callback(msg):
    global obst
    obst = np.array(msg).reshape(-1,2)

    draw_obst(obst)
    return

def goal_callback(msg):
    global goal
    goal = [msg.x , msg.y]

    draw_goal(msg)
    return
###############################################################################  
# local planner

def local_planner():
    global obst
    global goal

    forceList = []

    for i in obst:
        forceList.append( forceCalc ( -0.3 , circlify(i) ))
    forceList.append( forceCalc ( 10 , circlify(goal) ))

    forceList = np.array(forceList)
    Force = np.sum(forceList , axis = 0)
    
    angle = np.arctan2(Force[2],Force[1])


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
        point_msg.y = p[1] / 100
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
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime.secs = 1

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
    goal_marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1, latch=True)
    obst_marker_pub = rospy.Publisher('/obst_marker', MarkerArray, queue_size=1, latch=True)

    goal_sub = rospy.Subscriber('/goal_cone', Pose2D, goal_callback)
    obstacle_sub = rospy.Subscriber('/obst_cone', Float32MultiArray, obst_callback)

    rospy.spin()