#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import csv

line_idx = 0
state = 0

def state_callback(msg):
    global state
    global line_idx
    if state != msg.data and msg.data == 5:
        line_idx = line_idx + 1
        print("Changed!")
    state = msg.data


# Construct path to CSV file
csv_file_path = open('vel_coord.csv','r')
data = list(csv.reader(csv_file_path, delimiter=","))


if __name__ == '__main__':
    rospy.init_node('csv_to_topic_node')

    while not rospy.is_shutdown():
        state_sub = rospy.Subscriber('/STATE', Int32, state_callback)
        next_pos_pub = rospy.Publisher('/NEXT_POS', Pose2D, queue_size=10)

        try:
            x, y, theta = data[line_idx]
        except:
            pass
        x = float(x)
        y = float(y)
        theta = float(theta)

        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = theta
            
        next_pos_pub.publish(pose_msg)

        rospy.sleep(0.1)
    
    rospy.spin()