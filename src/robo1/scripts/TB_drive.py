#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import csv

def drive_turtlebot(vel_data):
    # Create a Twist message
    twist_msg = Twist()

    # Set the linear and angular velocities based on the angular velocities of the left and right wheels
    twist_msg.linear.x = ((vel_data['vel_left'] + vel_data['vel_right']) / 2.0)*0.033
    twist_msg.angular.z = (vel_data['vel_right'] - vel_data['vel_left']) * (0.33/0.16)

    # Publish the Twist message to the cmd_vel topic
    pub.publish(twist_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('turtlebot_driver')

    # Create a publisher to the cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Load the angular velocities from the CSV file
    with open('vel_traj.csv', 'r') as csvfile:
        vel_reader = csv.DictReader(csvfile)
        for vel_data in vel_reader:
            # Convert the values to floats
            vel_data['vel_left'] = float(vel_data['vel_left'])
            vel_data['vel_right'] = float(vel_data['vel_right'])

            # Call the drive_turtlebot function with the velocity data
            drive_turtlebot(vel_data)

            # Sleep for a fixed amount of time to simulate a fixed time step
            rospy.sleep(0.02)

