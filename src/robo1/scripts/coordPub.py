#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D

def state_callback(msg):
    global state
    state = msg.data

def csv_reader(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        return lines

def parse_csv_line(line):
    values = line.split(',')
    x = float(values[0])
    y = float(values[1])
    theta = float(values[2])
    return x, y, theta

if __name__ == '__main__':
    rospy.init_node('csv_to_topic_node')

    state = -1
    file_path = 'vel_coord.csv'
    lines = csv_reader(file_path)
    line_idx = 0

    state_sub = rospy.Subscriber('/STATE', Int32, state_callback)
    next_pos_pub = rospy.Publisher('/NEXT_POS', Pose2D, queue_size=10)
    state_pub = rospy.Publisher('/STATE', Int32, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if state == 1:

            x, y, theta = parse_csv_line(lines[line_idx])
            pose_msg = Pose2D()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = theta
            next_pos_pub.publish(pose_msg)

            line_idx = (line_idx + 1) % len(lines)
            
            state = -1
            state_pub.publish(-1)

        rate.sleep()

    rospy.spin()
