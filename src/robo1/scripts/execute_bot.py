#!/usr/bin/env python3


# for battery voltage 16.7V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.6 radians per sec  9.52 reverse
# for battery voltage 16.25V , left to right ratio of 1.21, at pwm 100,120, angular velocity = 9.25 radians per sec
# for batt volt 16.7V left to right ratio  of 1.206 and additional turn ratio of 1.1, pwm 75,98  turning angular vel = 5 right turn
# for batt volt 16.7V , pwm 85,90.45  turning angular vel = 5 left turn

import rospy
from geometry_msgs.msg import Pose2D
import csv

def drive_bot(vel_data,type_):
    # Create a Pose message
    pose_msg = Pose2D()

    # Set the linear and angular velocities based on the angular velocities of the left and right wheels
    pose_msg.x = vel_data[0]
    pose_msg.y = vel_data[1]

    # Publish the Pose message to the cmd_vel topic

    pub.publish(pose_msg)


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('turtlebot_driver')

    # Create a publisher to the cmd_vel topic
    pub = rospy.Publisher('/TARG_VEL', Pose2D, queue_size=10)

    # Load the angular velocities from the CSV file
    with open('vel_coord.csv', 'r') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        for row in csv_reader:

            X = float(row[0])
            Y = float(row[1])
            THETA = float(row[2])


            drive_bot([Left,Right])

            # Sleep for a fixed amount of time to simulate a fixed time step
            rospy.sleep(0.02)