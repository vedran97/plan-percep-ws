import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Initialize the ROS node
rospy.init_node('image_processing_node')

# Create a CV bridge object
cv_bridge = CvBridge()

# Define a function to process the image
def process_image(image_msg):

    # Convert the ROS CompressedImage message to a CV image
    np_arr = np.frombuffer(image_msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # cv_image = cv2.GaussianBlur(cv_image,(17,17),0)
    
    # Perform some image processing on the CV image
    cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
    cv_image = cv2.GaussianBlur(cv_image, (9,9), 0)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for the fluorescent orange color
    lower_orange = np.array([50,50,200])
    upper_orange = np.array([110, 100, 255])

    # Threshold the HSV image to get only fluorescent orange colors
    mask = cv2.inRange(cv_image, lower_orange, upper_orange)

    img = cv_image.copy()

    # Find the contours in the binary image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = []

    for contour in contours:
        if cv2.contourArea(contour) > 500:
            filtered_contours.append(contour)

    # Draw the contours on the original image
    cv2.drawContours(img, filtered_contours, -1, (0, 255, 0), 2)

    # Display the image with contours
    cv2.imshow('Contours', img)

    res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Display the processed image
    cv2.imshow('Processed Image', cv_image)
    cv2.imshow('Fluorescent Orange Only', res)
    cv2.waitKey(1)

# Subscribe to the Raspberry Pi camera topic
rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, process_image)

# Start the main ROS loop
rospy.spin()