import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
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

weights = "weights/best.pt"
weights_d = "midas_weights/dpt_swin2_large_384.pt"
d_model_type = "dpt_swin2_large_384"

imgsz = 640
device = ''
conf = 0.4
set_logging()
half = True
device = select_device(device)
print(device)
view_img = True

cone_xyz = []
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
    # global goal_xyz
    global cone_xyz
    goal_xyz = None

    count += 1
    if count % 5 != 0:
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

    # Inference for monocular depth estimation model
    with torch.no_grad():
        prediction_d = process(device, d_model, d_model_type, d_img, (net_w, net_h), d_img_rgb.shape[1::-1],
                                False, False)
        
    depth_map = get_depth_map(prediction_d)

    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=None, agnostic=False)

    t2 = time_sync()

    # Print time Depth detection
    # print(f'Depth Inference and YOLO done in ({t2 - t1:.3f}s)')
    # print(len(pred))

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

                if color == 'red':
                    xy = [int((xyxy[0].cpu().numpy() + xyxy[2].cpu().numpy())/2), int((xyxy[1].cpu().numpy() + xyxy[3].cpu().numpy())/2 )]
                    d_cone = (closest_dist * prediction_d[img_h-1, int(img_w/2)])/ (prediction_d[xy[1], xy[0]])
                    goal_xyz = [xy[0], (np.abs(xyxy[1].cpu().numpy() - xyxy[3].cpu().numpy())), d_cone]
                    print(f"Depth of the goal cone = {d_cone}")

                # if color == 'red':
                #     xy = [int((xyxy[0].cpu().numpy() + xyxy[2].cpu().numpy())/2), int((xyxy[1].cpu().numpy() + xyxy[3].cpu().numpy())/2 )]
                #     d_cone = (closest_dist * prediction_d[img_h-1, int(img_w/2)])/ (prediction_d[xy[1], xy[0]])
                #     cone_xyz.append([xy[0], (np.abs(xyxy[1].cpu().numpy() - xyxy[3].cpu().numpy())), d_cone])
                #     # print(f"Depth of the cone = {d_cone}")

                if view_img:  # Add bbox to image
                    # print(color)
                    label = f'{color} {names[int(cls)]}'
                    # print(label)
                    plot_one_box(xyxy, og_img, label=label, color=color_val, line_thickness=3)

    

    # Stream results
    if view_img:
        cv2.imshow("Yolo_img", og_img)
        cv2.imshow("Depth_map", depth_map)
        cv2.waitKey(1)  # 1 millisecond

    # if pose is None:
    #     return
    
    obs_positions = []
    goal_position = None
    norm_z = 0.81

    # if goal_xyz is not None:
    #     print(goal_xyz)

    # Goal position
    if goal_xyz is not None:
        z = goal_xyz[2] + 5
        x_img = ((goal_xyz[0] - img_w/2)/img_w)*3.68
        x = x_img * (z)/(f)
        goal_position = [z + cam2base, x]

    if goal_xyz is not None:
        print('hey')
        # print(goal_position)
        goal_msg = Pose2D()
        goal_msg.x = goal_position[0]
        goal_msg.y = goal_position[1]
        if goal_position[0] > 40:
            goal_msg.theta = 1
        else:
            goal_msg.theta = 0
        pub_goal.publish(goal_msg)
    
    else:
        goal_msg = Pose2D()
        goal_msg.theta = 0
        pub_goal.publish(goal_msg)

    # Obstacle positions
    for i, xyz in enumerate(cone_xyz):
        z = xyz[2] + 5
        x_img = ((xyz[0] - img_w/2)/img_w)*3.68
        x = x_img * (z)/(f)

        if z > 80:
            return
        
        if z < 80:
            conf_count += 1

        # alpha = pose[2] - np.arctan2(x, z + cam2base)
        # D = np.sqrt(x**2 + (z + cam2base)**2)
        # x_g = pose[0] + D * np.cos(alpha)
        # y_g = pose[1] + D * np.sin(alpha)
        # obs_positions.append([x_g,y_g])
        obs_positions.append([z + cam2base, x])

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
    if len(obs_positions) > 0 :
        # msg = numpy_msg(Float64MultiArray, positions)
        # pub.publish(msg)
        position_msg = Float32MultiArray()
        # position_msg = Pose2D()
        # position_msg.x = obs_positions[0][0]
        # position_msg.y = obs_positions[0][1]
        obs_positions = np.array(obs_positions).flatten()
        position_msg.data = list(obs_positions)

        # if conf_count > 5:
        #     position_msg.theta = 1

        # print(position_msg)
        pub_obs.publish(position_msg)
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
    pub_obs = rospy.Publisher('/obst_cone', Float32MultiArray, queue_size=1)
    pub_goal = rospy.Publisher('/goal_cone', Pose2D, queue_size=1)

    # Create a CV bridge object
    cv_bridge = CvBridge()

    # Subscribe to the TerpBot's odom
    rospy.Subscriber('/CURR_ODOM', Pose2D, get_odom, queue_size=1)

    # Subscribe to the Raspberry Pi camera topic
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, process_image, queue_size=1)

    # Start the main ROS loop
    rospy.spin()