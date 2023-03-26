#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import csv

def drive_bot(vel_data):
    # Create a Pose message
    pose_msg = Pose2D()

    # Set the linear and angular velocities based on the angular velocities of the left and right wheels
    pose_msg.x = vel_data[0]
    pose_msg.y = vel_data[1]

    if pose_msg.x > 0 and pose_msg.y < 0:
        pose_msg.theta = 1
    elif pose_msg.x < 0 and pose_msg.y > 0:
        pose_msg.theta = -1
    else:
        pose_msg.theta = 0
    # Publish the Pose message to the cmd_vel topic

    pub.publish(pose_msg)


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('turtlebot_driver')

    # Create a publisher to the cmd_vel topic
    pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)

    # Load the angular velocities from the CSV file
    with open('vel_traj.csv', 'r') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        for row in csv_reader:
            # Convert the values to floats
            Left = float(row[0])
            Right = float(row[1])

            # Call the drive_turtlebot function with the velocity data
            drive_bot([Left,Right])

            # Sleep for a fixed amount of time to simulate a fixed time step
            rospy.sleep(0.01)

