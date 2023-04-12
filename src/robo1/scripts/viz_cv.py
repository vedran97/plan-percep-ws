#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import numpy as np
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import csv

# Image 
rospack = rospkg.RosPack()
img_path = rospack.get_path('robo1') + '/scripts_planner/map.jpg'
img = cv2.imread(img_path)
robot_dim = [22,22]
bridge = CvBridge()

# Start point
rospack = rospkg.RosPack()
csv_path = rospack.get_path('robo1') + '/scripts_planner/vel_coord.csv'
csv_file_path = open(csv_path,'r')
start = np.array(list(csv.reader(csv_file_path, delimiter=",")),dtype=float)[0]


rospy.init_node('odom_viz')

# Supporting functions

def plot_rect(odom_msg):
    global img
    x = odom_msg.x
    y = odom_msg.y
    theta = odom_msg.theta

    center = ( img.shape[1] -3*(int(start[0] + x + 15.5*np.cos(theta))) , 3*int(start[1] + y + 15.5*np.sin(theta)) )
    rect = ((center[0], center[1]), (22*3,22*3), theta*180/np.pi)
    box = cv2.boxPoints(rect)
    box = np.intp(box)

    # Draw the rectangle on the image
    img = cv2.circle(img,center,3,(0,0,255),-1)
    img_copy = np.copy(img)
    img_copy = cv2.drawContours(img_copy, [box], 0, (255,0,0), 2)

    image_msg = bridge.cv2_to_imgmsg(img_copy, encoding="bgr8")
    image_pub.publish(image_msg)

# Main loop
if __name__ == '__main__':
    # Image pub
    image_pub = rospy.Publisher('/odom_viz', Image, queue_size=1)

    # Odom subscriber
    curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, plot_rect)

    rospy.spin()