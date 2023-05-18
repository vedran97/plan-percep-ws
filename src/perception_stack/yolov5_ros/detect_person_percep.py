import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2, PointField
# import ros_numpy

from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray, Header
from std_msgs.msg import struct
# from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Pose2D

import argparse
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.dataloaders import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_boxes, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box, dominant_color
from utils.torch_utils import select_device, time_sync
from utils.augmentations import letterbox
import os

import time
import numpy as np
from imutils.video import VideoStream
from midas.model_loader import default_models, load_model
from runmidas import process, get_depth_map

os.chdir('/workspaces/plan-percep-ws/src/perception_stack/yolov5_ros')

# Define global constants
img_w = 410
img_h = 308
sensor_w = 3.68
sensor_h = 2.76
cam2base = 16
f = 3.04
cone_h = 177.8
pose = None

closest_dist = 21
previous_goal = None

# weights = "weights/best.pt"
weights = "yolov5s.pt"
# weights_d = "midas_weights/dpt_swin2_large_384.pt"
weights_d = "midas_weights/dpt_swin2_tiny_256.pt"
# d_model_type = "dpt_swin2_large_384"
d_model_type = "dpt_swin2_tiny_256"

imgsz = 640
device = ''
conf = 0.7
set_logging()
half = True
device = select_device(device)
print(device)
view_img = True

# cone_xyz = []
# goal_xyz = None

# Load model
model = attempt_load(weights, device=device)  # load FP32 model
d_model, transform, net_w, net_h = load_model(device, weights_d, model_type = d_model_type, optimize=False)

stride = int(model.stride.max())  # model stride
imgsz = check_img_size(imgsz, s=stride)  # check img_size
if half:
    model.half()  # to FP16

names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

# Run inference
if device.type != 'cpu':
    model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

count = 0
conf_count = 0

# Function to convert point data to pointcloud data and publish to ROS
# def publish_pointcloud(pc):

#     pc_array = np.zeros(len(pc), dtype=[
#     ('x', np.float32),
#     ('y', np.float32),
#     ('z', np.float32),
#     ('intensity', np.float32),])
#     pc_array['x'] = pc[:, 0]
#     pc_array['y'] = pc[:, 1]
#     pc_array['z'] = 0.1 * np.ones_like(pc[:,0])
#     pc_array['intensity'] = np.ones_like(pc[:,0])
#     # msg.header.stamp = rospy.Time.now()

#     pc_msg = ros_numpy.msgify(PointCloud2, pc_array, stamp=rospy.Time.now(), frame_id='dummy_base_link' )
#     pc_pub.publish(pc_msg)

def point_cloud(points, parent_frame):
    dummy = np.ones((points.shape[0],5))
    points = np.hstack((points, np.array([0.1,1,0,0,1])*dummy))
    """ Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]

    header = Header(frame_id=parent_frame, stamp=rospy.Time.now())

    pc_msg = PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )

    pc_pub.publish(pc_msg)

# Callback function to process the image
def process_image(image_msg):
    global count
    global pose
    global model
    global imgsz
    global stride
    global view_img
    global d_model
    global transform
    global net_h
    global net_w
    global d_model_type
    global conf_count
    global previous_goal
    # global goal_xyz
    # global cone_xyz
    goal_xyz = None

    count += 1
    if count % 1 != 0:
        return

    conf_thres = 0.3
    iou_thres = 0.45

    # Convert the ROS CompressedImage message to a CV image
    np_arr = np.frombuffer(image_msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

    og_img = cv_image.copy()
    proc_img = og_img.copy()
    d_img = og_img.copy()

    # Process image for monocular depth estimation
    d_img_rgb = cv2.cvtColor(d_img, cv2.COLOR_BGR2RGB) / 255.0
    d_img = transform({"image": d_img_rgb})["image"]

    # Process image for YOLO cone detection
    proc_img = letterbox(proc_img, imgsz, stride=stride)[0]
    proc_img = proc_img.transpose((2, 0, 1))[::-1]
    proc_img = np.ascontiguousarray(proc_img)

    t0 = time.time()
    # for path, img, im0s, vid_cap,s in dataset:
    proc_img = torch.from_numpy(proc_img).to(device)
    proc_img = proc_img.half() if half else proc_img.float()  # uint8 to fp16/32
    proc_img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if proc_img.ndimension() == 3:
        proc_img = proc_img.unsqueeze(0)

    t1 = time_sync()

    # Inference for YOLO model
    pred = model(proc_img, augment=False)[0]

     # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=None, agnostic=False)

    t2 = time_sync()

    # print(f"\nTime taken for yolo inference = {t2-t1}\n")

    t3 = time_sync()

    # Inference for monocular depth estimation model
    with torch.no_grad():
        prediction_d = process(device, d_model, d_model_type, d_img, (net_w, net_h), d_img_rgb.shape[1::-1],
                                False, False)
        
    prediction_d = prediction_d + 0.00000001
    depth_map = get_depth_map(prediction_d)

    t4 = time_sync()

    if t4 - t3 > 0.1:
        print(f"Time taken for midas inference = {t4-t3}\n")

    potential_goals = []

    xyxy_list = []

    # Process detections
    for i, det in enumerate(pred):  # detections per image
        s = '%gx%g ' % proc_img.shape[2:]  # print string
        gn = torch.tensor(og_img.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_boxes(proc_img.shape[2:], det[:, :4], og_img.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det):
                color, color_val, h = dominant_color(xyxy, og_img)

                if names[int(cls)] == 'person':
                    xyxy_norm = [int(xyxy[0].cpu().numpy()), int(xyxy[1].cpu().numpy()), int(xyxy[2].cpu().numpy()), int(xyxy[3].cpu().numpy())]
                    xy = [int((xyxy[0].cpu().numpy() + xyxy[2].cpu().numpy())/2), int((xyxy[1].cpu().numpy() + xyxy[3].cpu().numpy())/2 )]
                    depths = (closest_dist * prediction_d[img_h-1, int(img_w/2)])/ (prediction_d)
                    depths = depths[xyxy_norm[1]:xyxy_norm[3], xyxy_norm[0]:xyxy_norm[2]]
                    depths = np.abs(depths)
                    person_depth = np.min(depths)
                    goal_xyz = [xy[0], (np.abs(xyxy[1].cpu().numpy() - xyxy[3].cpu().numpy())), person_depth]
                    xyxy_list.append(xyxy)
                    potential_goals.append(goal_xyz)

                    # if view_img:  # Add bbox to image
                    #     label = f'{names[int(cls)]}'
                    #     plot_one_box(xyxy, og_img, label=label, color=color_val, line_thickness=3)

    # Reinitialize leader everytime
    leader = None

    # print(potential_goals)

    # If this is the first time humans are getting detected, choose the one nearest to the center as the leader and initialize previous goal
    if previous_goal is None and len(potential_goals) != 0:
        x_pot_goals = np.array(potential_goals)[:,0]
        x_pot_goals = np.abs(x_pot_goals - img_w/2)
        leader = potential_goals[np.argmin(x_pot_goals)]

    # Check for current goal by comparing with the previous goal if the previous goal is not None
    if previous_goal is not None and len(potential_goals) != 0:
        x_pot_goals = np.array(potential_goals)[:,0]
        x_pot_goals = np.abs(x_pot_goals - previous_goal[0])
        leader = potential_goals[np.argmin(x_pot_goals)]

    if leader is not None:
        # print(f"Distance of the leader = {leader[2]}")
        pass

    # Color code the right leader
    if view_img:
        for i, pot_goal in enumerate(potential_goals):
            if pot_goal == leader:
                label = "Leader"
                plot_one_box(xyxy_list[i], og_img, label=label, color= [0,255,0], line_thickness=3)
            else:
                label = "Unused"
                plot_one_box(xyxy_list[i], og_img, label=label, color= [0,0,255], line_thickness=3)

    # Stream results
    if view_img:
        cv2.imshow("Yolo_img", og_img)
        cv2.imshow("Depth_map", depth_map)
        cv2.waitKey(1)  # 1 millisecond

    # obs_positions = []
    goal_position = None
    # norm_z = 0.81

    # Create data for point cloud
    depths = np.abs((closest_dist * prediction_d[img_h-1, int(img_w/2)])/ (prediction_d))
    centerline_z = np.abs(depths[int(img_h/2), :])
    centerline_z[centerline_z > 100] = 100
    centerline_x = np.arange(img_w)
    centerline_x = ((centerline_x - img_w/2)/img_w)*3.68
    centerline_x = centerline_x * (centerline_z)/(f)
    pc_data = np.vstack((centerline_x/100, centerline_z/100)).T

    point_cloud(pc_data, 'dummy_base_link')

    # if np.min(centerline_z) < 30:
    #     print(f"Closest point distance = {np.min(centerline_z)}")

    # Goal position
    if leader is not None:
        z = leader[2] + 5
        x_img = ((leader[0] - img_w/2)/img_w)*3.68
        x = x_img * (z)/(f)
        goal_position = [z + cam2base, x]

    if goal_position is not None:
        goal_msg = Pose2D()
        goal_msg.x = goal_position[0]
        goal_msg.y = -goal_position[1]
        if goal_position[0] > 50:
            if np.min(centerline_z) < 30:
                goal_msg.theta = 0
                print("OH IT STOPPED")
            else:
                goal_msg.theta = 1
        else:
            goal_msg.theta = 0
        pub_goal.publish(goal_msg)
    
    else:
        goal_msg = Pose2D()
        goal_msg.theta = 0
        pub_goal.publish(goal_msg)
    
    # Initialize previous goal after it publishes the goal
    previous_goal = leader


    # Obstacle positions
    # for i, xyz in enumerate(cone_xyz):
    #     z = xyz[2] + 5
    #     x_img = ((xyz[0] - img_w/2)/img_w)*3.68
    #     x = x_img * (z)/(f)

    #     if z > 80:
    #         return
        
    #     if z < 80:
    #         conf_count += 1

        # alpha = pose[2] - np.arctan2(x, z + cam2base)
        # D = np.sqrt(x**2 + (z + cam2base)**2)
        # x_g = pose[0] + D * np.cos(alpha)
        # y_g = pose[1] + D * np.sin(alpha)
        # obs_positions.append([x_g,y_g])
        # obs_positions.append([z + cam2base, x])

    # Publish the goal message
    # if goal_position is not None:
    #     print("hey")
        # goal_msg = Pose2D()
        # goal_msg.x = goal_position[0]
        # goal_msg.y = goal_position[1]
        # if goal_position > 20:
        #     goal_msg.theta = 1
        # else:
        #     goal_msg.theta = 0
        # pub_goal.publish(goal_msg)

    # Publish the processed obstacle message
    # if len(obs_positions) > 0 :
        # msg = numpy_msg(Float64MultiArray, positions)
        # pub.publish(msg)
        # position_msg = Float32MultiArray()
        # position_msg = Pose2D()
        # position_msg.x = obs_positions[0][0]
        # position_msg.y = obs_positions[0][1]
        # obs_positions = np.array(obs_positions).flatten()
        # position_msg.data = list(obs_positions)

        # if conf_count > 5:
        #     position_msg.theta = 1

        # print(position_msg)
        # pub_obs.publish(position_msg)
    # else:
        # print("No cone!")
        # print("")


def get_odom(pose_msg):
    global pose
    # print("In odom callback!")
    pose = [pose_msg.x, pose_msg.y, pose_msg.theta]

if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('obstacle_map')
    rate = rospy.Rate(10) #10Hz
    # pub_obs = rospy.Publisher('/obst_cone', Float32MultiArray, queue_size=1)
    pub_goal = rospy.Publisher('/goal_cone', Pose2D, queue_size=1)

    # Create a CV bridge object
    cv_bridge = CvBridge()

    # Subscribe to the TerpBot's odom
    rospy.Subscriber('/CURR_ODOM', Pose2D, get_odom, queue_size=1)

    # Subscribe to the Raspberry Pi camera topic
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, process_image, queue_size=1)

    # Publisher for point cloud
    pc_pub = rospy.Publisher('pointcloud', PointCloud2, queue_size=1)

    # Start the main ROS loop
    rospy.spin()