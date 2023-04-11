#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np
import rospkg
import csv
from std_msgs.msg import Float32MultiArray

# Construct path to CSV file
rospack = rospkg.RosPack()
csv_path = rospack.get_path('robo1') + '/scripts_planner/vel_coord.csv'
csv_file_path = open(csv_path,'r')
data = list(csv.reader(csv_file_path, delimiter=","))

line_idx = 0
r = 6.45/2
L = 19.2

global_planner = True

rospy.init_node('pose_follower')

def global_planner(msg):
    global global_planner
    global target_pose
    global line_idx

    if not global_planner:
        local_planner(msg)
        return
    
    target_pose = np.float32(data[line_idx])
    if ((target_pose[1] - msg.y)**2 + (target_pose[0] - msg.x)**2)**0.5 < 1:
        line_idx = line_idx + 1
        print("temp goal reached!")

    curr_heading = msg.theta
    need_heading = np.arctan2(target_pose[1] - msg.y , target_pose[0] - msg.x)

    publish_this( 10 , -0.1*(need_heading - curr_heading) )

    

def local_planner(msg):
    publish_this(0,0)

def publish_this(vel,ang_vel):
    targ_vel = Pose2D()
    targ_vel.x = vel
    targ_vel.y = ang_vel
    targ_vel_pub.publish(targ_vel)

if __name__ == '__main__':

    # Set up a publishers for the /TARG_VEL topic and state
    targ_vel_pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)

    curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, global_planner)

    rospy.spin()