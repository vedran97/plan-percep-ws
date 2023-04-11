#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
import tf
import numpy as np

curr_x = None
curr_y = None

r = 6.45/2
L = 19.2

def model_states_callback(msg):
    global curr_x, curr_y
    idx = msg.name.index("my_robot")

    orientation = msg.pose[idx].orientation
    rotation_matrix = tf.transformations.quaternion_matrix([orientation.x,orientation.y,orientation.z,orientation.w])
    rpy = tf.transformations.euler_from_matrix(rotation_matrix,'sxyz')[2] + (np.pi/2)
    rpy = np.arctan2(np.sin(rpy),np.cos(rpy))

    pose = Pose2D()
    pose.x = msg.pose[idx].position.x * 100
    pose.y = msg.pose[idx].position.y * 100
    pose.theta = rpy
    
    pub_curr_odom.publish(pose)

def targ_vel_callback(msg):

    vel = msg.x 
    ang_vel = -msg.y
    Ul = 0.5* ( (2*vel/r) + (L*ang_vel/r))
    Ur = 0.5* ( (2*vel/r) - (L*ang_vel/r))

    pub_wheel_right.publish(Ul)
    pub_wheel_left.publish(-Ur)

rospy.init_node("robot_controller")

sub_model_states = rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
pub_curr_odom = rospy.Publisher("/CURR_ODOM", Pose2D, queue_size=10)

sub_targ_vel = rospy.Subscriber("/TARG_VEL", Pose2D, targ_vel_callback)

pub_wheel_right = rospy.Publisher("/robo1/driveR_controller/command", Float64, queue_size=10)
pub_wheel_left = rospy.Publisher("/robo1/driveL_controller/command", Float64, queue_size=10)

rospy.spin()