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

    count += 1
    if count % 5 != 0:
        return

    conf_thres = 0.7
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

    cone_xyz = []
    goal_xyz = None

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

                if color == 'green':
                    xy = [int((xyxy[0].cpu().numpy() + xyxy[2].cpu().numpy())/2), int((xyxy[1].cpu().numpy() + xyxy[3].cpu().numpy())/2 )]
                    # print(depth_map.shape)
                    # d_cone = (21 * prediction_d[307,204])/(prediction_d[xy[1], xy[0]])
                    d_cone = (closest_dist * prediction_d[img_h-1, int(img_w/2)])/ (prediction_d[xy[1], xy[0]])
                    # print(f"Depth far of the cone = {depth_map[xy[1], xy[0]]}")
                    # print(f"Depth close of the cone = {depth_map[307,204]}")
                    goal_xyz = [xy[0], (np.abs(xyxy[1].cpu().numpy() - xyxy[3].cpu().numpy())), d_cone]
                    print(f"Depth of the goal cone = {d_cone}")

                if color == 'red':
                    # cone_xy.append([(xyxy[0].cpu().numpy() + xyxy[2].cpu().numpy())/2, (np.abs(xyxy[1].cpu().numpy() - xyxy[3].cpu().numpy()))])
                    xy = [int((xyxy[0].cpu().numpy() + xyxy[2].cpu().numpy())/2), int((xyxy[1].cpu().numpy() + xyxy[3].cpu().numpy())/2 )]
                    # print(depth_map.shape)
                    # d_cone = (21 * prediction_d[307,204])/(prediction_d[xy[1], xy[0]])
                    d_cone = (closest_dist * prediction_d[img_h-1, int(img_w/2)])/ (prediction_d[xy[1], xy[0]])
                    # print(f"Depth far of the cone = {depth_map[xy[1], xy[0]]}")
                    # print(f"Depth close of the cone = {depth_map[307,204]}")
                    cone_xyz.append([xy[0], (np.abs(xyxy[1].cpu().numpy() - xyxy[3].cpu().numpy())), d_cone])
                    print(f"Depth of the cone = {d_cone}")

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

    if pose is None:
        return
    
    positions = []
    norm_z = 0.81

    # for i, xy in enumerate(cone_xy):
    #     # max_y = np.max(temp[:,1])
    #     # min_y = np.min(temp[:,1])
    #     # print(max_y,min_y)
    #     y_img = (xy[1]/img_h)*2.76
    #     z = f * (cone_h)/(y_img*norm_z)
    #     # x_img = ((np.mean(temp[:,0]) - img_w/2)/img_w)*3.68
    #     x_img = ((xy[0] - img_w/2)/img_w)*3.68
    #     x = x_img * (cone_h)/(y_img*10)
    #     z = (z - 160)/10
    #     print(z)
    #     # print(z)
    #     if z > 70:
    #         return
    #     # print(z)

    #     alpha = pose[2] - np.arctan2(x, z + cam2base)
    #     D = np.sqrt(x**2 + (z + cam2base)**2)
    #     x_g = pose[0] + D * np.cos(alpha)
    #     y_g = pose[1] + D * np.sin(alpha)
    #     positions.append([x_g,y_g])
    #     # filtered_contours.append(contour)

    for i, xyz in enumerate(cone_xyz):
        # max_y = np.max(temp[:,1])
        # min_y = np.min(temp[:,1])
        # print(max_y,min_y)
        y_img = (xyz[1]/img_h)*2.76
        # z = f * (cone_h)/(y_img*norm_z)
        z = xyz[2] + 5
        # x_img = ((np.mean(temp[:,0]) - img_w/2)/img_w)*3.68
        x_img = ((xy[0] - img_w/2)/img_w)*3.68
        # x = x_img * (cone_h)/(y_img*10)
        x = x_img * (z)/(f)
        # z = (z - 16)
        # print(z)
        # print(z)
        if z > 60:
            return
        
        if z < 60:
            conf_count += 1
        # print(z)

        alpha = pose[2] - np.arctan2(x, z + cam2base)
        D = np.sqrt(x**2 + (z + cam2base)**2)
        x_g = pose[0] + D * np.cos(alpha)
        y_g = pose[1] + D * np.sin(alpha)
        positions.append([x_g,y_g])
        # filtered_contours.append(contour)

    # cv_image = cv2.GaussianBlur(cv_image,(17,17),0)
    
    # # Perform some image processing on the CV image
    # # cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
    # cv_image = cv2.GaussianBlur(cv_image, (9,9), 0)
    # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # # Define lower and upper bounds for the fluorescent orange color
    # # lower_orange = np.array([50,50,200])
    # # upper_orange = np.array([110, 100, 255])
    # lower_orange = np.array([50,30,180])
    # upper_orange = np.array([110, 120, 255])

    # # Threshold the HSV image to get only fluorescent orange colors
    # mask = cv2.inRange(cv_image, lower_orange, upper_orange)

    # img = cv_image.copy()

    # # Perform closing
    # kernel = np.ones((5,5),np.uint8)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # # Find the contours in the binary image
    # contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # filtered_contours = []
    # positions = []

    # for i, contour in enumerate(contours):
    #     if cv2.contourArea(contour) > 1000:
    #         temp = np.squeeze(contour)
    #         max_y = np.max(temp[:,1])
    #         min_y = np.min(temp[:,1])
    #         # print(max_y,min_y)
    #         y_img = (np.abs(max_y - min_y)/img_h)*2.76
    #         z = f * (cone_h)/y_img
    #         # x_img = ((np.mean(temp[:,0]) - img_w/2)/img_w)*3.68
    #         x_img = ((cone_x[i] - img_w/2)/img_w)*3.68
    #         x = x_img * (cone_h)/(y_img*10)
    #         z = (z - 160)/10

    #         alpha = pose[2] - np.arctan2(x, z + cam2base)
    #         D = np.sqrt(x**2 + (z + cam2base)**2)
    #         x_g = pose[0] + D * np.cos(alpha)
    #         y_g = pose[1] + D * np.sin(alpha)
    #         positions.append([x_g,y_g])
    #         filtered_contours.append(contour)
    
    # # positions = np.array(positions)
    # # print(positions)

    # # Draw the contours on the original image
    # cv2.drawContours(img, filtered_contours, -1, (0, 255, 0), 2)

    # # Display the image with contours
    # # cv2.imshow('Contours', img)

    # res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # # Display the processed image
    # # cv2.imshow('Processed Image', cv_image)
    # # cv2.imshow('Fluorescent Orange Only', res)
    # cv2.waitKey(1)
    # print(conf_count)
    
    # # Publish the processed message
    if len(positions) >0 :
        # msg = numpy_msg(Float64MultiArray, positions)
        # pub.publish(msg)
        position_msg = Pose2D()
        position_msg.x = positions[0][0]
        position_msg.y = positions[0][1]

        if conf_count > 5:
            position_msg.theta = 1

        # print(position_msg)
        pub.publish(position_msg)
    # else:
        # print("No cone!")
        # print("")


def get_odom(pose_msg):
    global pose
    # print("In odom callback!")
    pose = [pose_msg.x, pose_msg.y, pose_msg.theta]

if __name__ == '__main__':

    # parser = argparse.ArgumentParser()
    # parser.add_argument('--weights', nargs='+', type=str, default='yolov5s.pt', help='model.pt path(s)')
    # parser.add_argument('--source', type=str, default='data/images', help='source')  # file/folder, 0 for webcam
    # parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    # parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    # parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    # parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    # parser.add_argument('--view-img', action='store_true', help='display results')
    # parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    # parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    # parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    # parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    # parser.add_argument('--augment', action='store_true', help='augmented inference')
    # parser.add_argument('--update', action='store_true', help='update all models')
    # parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    # parser.add_argument('--name', default='exp', help='save results to project/name')
    # parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    # opt = parser.parse_args()
    # print(opt)
    # check_requirements()

    # with torch.no_grad():
    #     if opt.update:  # update all models (to fix SourceChangeWarning)
    #         for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
    #             detect()
    #             strip_optimizer(opt.weights)
    #     else:
    #         detect()

    # Initialize the ROS node
    rospy.init_node('obstacle_map')
    rate = rospy.Rate(10) #10Hz
    pub = rospy.Publisher('/positions', Pose2D, queue_size=10)

    # Create a CV bridge object
    cv_bridge = CvBridge()

    # Subscribe to the TerpBot's odom
    rospy.Subscriber('/CURR_ODOM', Pose2D, get_odom, queue_size=1)

    # Subscribe to the Raspberry Pi camera topic
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, process_image, queue_size=1)

    # Start the main ROS loop
    rospy.spin()