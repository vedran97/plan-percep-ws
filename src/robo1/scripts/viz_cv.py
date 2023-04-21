#!/usr/bin/env python3

import rospy


import tf_conversions
import tf2_ros
from geometry_msgs.msg import Pose2D, PoseStamped
import geometry_msgs.msg
import csv

import numpy as np
import cv2
import rospkg
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

last_pose = None

rospy.init_node('tf2_TerpBot_broadcaster')




################################# Publishing Map
rospack = rospkg.RosPack()
map_path = rospack.get_path('robo1') + '/scripts_planner/map.png'
image_data = cv2.bitwise_not(cv2.imread(map_path,cv2.IMREAD_GRAYSCALE))
image_data = np.transpose(image_data)

csv_path = rospack.get_path('robo1') + '/scripts_planner/vel_coord.csv'
csv_file_path = open(csv_path,'r')
data_raw = np.array(list(csv.reader(csv_file_path, delimiter=",")),dtype=float)
start_point = data_raw[0]

image_data[image_data < 128] = 0
image_data[image_data >= 128] = 100

header = Header(stamp=rospy.Time.now(), frame_id='world_frame')
resolution = 0.00333333  # 5 cm per pixel
width = image_data.shape[1]
height = image_data.shape[0]

origin_x = - resolution * start_point[1]
origin_y = - resolution * start_point[0]
data = list(image_data.reshape((width * height,)))
occupancy_grid = OccupancyGrid(header=header, info=MapMetaData(resolution=resolution, width=width, height=height,
                                                               origin=Pose(Point(x=origin_x, y=origin_y), 
                                                                           Quaternion(0, 0, 0, 1))), data=data)

################################## Publishing path

marker_msg = Marker()
marker_msg.ns = "point_ns"
marker_msg.type = Marker.LINE_STRIP
marker_msg.action = Marker.ADD
marker_msg.pose.orientation.w = 1.0
marker_msg.scale.x = 0.01
marker_msg.color.a = 1.0
marker_msg.color.r = 0.0
marker_msg.color.g = 1.0
marker_msg.color.b = 0.0

marker_array_msg = MarkerArray()

for i, p in enumerate(data_raw - start_point):
    point_msg = Point()
    point_msg.x = - p[1] / 100
    point_msg.y = p[0] / 100
    point_msg.z = 0
    
    marker_msg.header.frame_id = "world_frame"
    marker_msg.header.stamp = rospy.Time.now()
    
    marker_msg.points.append(point_msg)
    marker_msg.id = i
    marker_array_msg.markers.append(marker_msg)

################################## publish odom the transform


def handle_robot_pose(msg):
    global last_pose
    marker_pub.publish(marker_array_msg)

    if last_pose is None or last_pose.x != msg.x or last_pose.y != msg.y or last_pose.theta != msg.theta:
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world_frame"
        t.child_frame_id = "dummy_base_link"
        t.transform.translation.x = - msg.y / 100
        t.transform.translation.y = msg.x / 100
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # pose_stamped_msg = PoseStamped()
        # pose_stamped_msg.header.stamp = rospy.Time.now()
        # pose_stamped_msg.header.frame_id = "dummy_base_link"
        # pose_stamped_msg.pose.position.x = 0
        # pose_stamped_msg.pose.position.y = 0
        # pose_stamped_msg.pose.position.z = 0.0
        # pub2.publish(pose_stamped_msg)

        br.sendTransform(t)
        
        last_pose = msg

curr_odom_sub = rospy.Subscriber('/CURR_ODOM', Pose2D, handle_robot_pose)

pub1 = rospy.Publisher('/map_disp', OccupancyGrid, queue_size=1, latch=True)
marker_pub = rospy.Publisher('point_marker', MarkerArray, queue_size=10)

while not rospy.is_shutdown():
    pub1.publish(occupancy_grid)
    marker_pub.publish(marker_array_msg)
    rospy.sleep(1)