#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D

class PosePlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-400, 400)
        self.ax.set_ylim(-400, 400)
        self.arrow_length = 4
        self.arrow_color = 'r'
        self.pose_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, self.pose_callback)

    def pose_callback(self, pose_msg):
        # Clear the axis and plot the arrow
        self.ax.clear()
        self.ax.set_xlim(-400, 400)
        self.ax.set_ylim(-400, 400)
        self.ax.arrow(pose_msg.x, pose_msg.y, self.arrow_length * cos(pose_msg.theta),
                      self.arrow_length * sin(pose_msg.theta), head_width=1, head_length=1,
                      fc=self.arrow_color, ec=self.arrow_color)
        self.fig.canvas.draw()

if __name__ == '__main__':
    rospy.init_node('pose_plotter')
    pose_plotter = PosePlotter()
    plt.show()
