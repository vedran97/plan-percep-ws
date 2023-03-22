#!/usr/bin/env python3

import csv
import rospy
from std_msgs.msg import Float64
import rospkg

# Set up ROS node
rospy.init_node('csv_reader')

# Set up publishers for the two topics
pub1 = rospy.Publisher('/robo1/driveL_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/robo1/driveR_controller/command', Float64, queue_size=10)

# Set time rate in Hz
k = 100  # Change this to the desired time rate

rp = rospkg.RosPack()
pkg_path = rp.get_path('robo1')

# Construct path to CSV file
csv_file_path = pkg_path + '/scripts/vel_traj.csv'

# Read in data from CSV file
with open(csv_file_path) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    next(csv_reader)  # Skip header row
    rate = rospy.Rate(k)  # Set target publishing rate to k Hz
    dt = 1.0 / k  # Compute fixed time step
    for row in csv_reader:
        x = float(row[0])
        y = float(row[1])

        # Publish X and Y values to appropriate topic
        pub1.publish(x)
        pub2.publish(y)

        # Wait for fixed time step
        rate.sleep()

pub1.publish(0)
pub2.publish(0)
rospy.sleep(1)
