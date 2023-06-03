import torch
import cv2

# Model
model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom

# Images
# img = "https://ultralytics.com/images/zidane.jpg"  # or file, Path, PIL, OpenCV, numpy, list
img = "/home/shyam-pi/Github/plan-percep-ws/src/yolov5/data/images/cones.png"

# Inference
results = model(img)

# Results
results.show() 