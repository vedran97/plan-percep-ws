import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
from rospy.numpy_msg import numpy_msg

# Define global constants
img_w = 410
img_h = 308
sensor_w = 3.68
sensor_h = 2.76
f = 3.04
cone_h = 177.8

# Callback function to process the image
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
    # lower_orange = np.array([50,50,200])
    # upper_orange = np.array([110, 100, 255])
    lower_orange = np.array([50,50,200])
    upper_orange = np.array([110, 120, 255])

    # Threshold the HSV image to get only fluorescent orange colors
    mask = cv2.inRange(cv_image, lower_orange, upper_orange)

    img = cv_image.copy()

    # Perform closing
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find the contours in the binary image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = []
    positions = []

    for contour in contours:
        if cv2.contourArea(contour) > 1800:
            temp = np.squeeze(contour)
            max_y = np.max(temp[:,1])
            min_y = np.min(temp[:,1])
            # print(max_y,min_y)
            y_img = (np.abs(max_y - min_y)/img_h)*2.76
            z = f * (cone_h)/y_img
            x_img = ((np.mean(temp[:,0]) - img_w/2)/img_w)*3.68
            x = x_img * (cone_h)/y_img
            print([x,z])
            positions.append([x,z])
            filtered_contours.append(contour)
    
    # positions = np.array(positions)
    # print(positions)

    # Draw the contours on the original image
    cv2.drawContours(img, filtered_contours, -1, (0, 255, 0), 2)

    # Display the image with contours
    cv2.imshow('Contours', img)

    res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Display the processed image
    cv2.imshow('Processed Image', cv_image)
    cv2.imshow('Fluorescent Orange Only', res)
    cv2.waitKey(1)
    
    # Publish the processed message
    # msg = numpy_msg(Float64MultiArray, positions)
    # pub.publish(msg)
    
if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('obstacle_map')
    rate = rospy.Rate(10) #10Hz
    # pub = rospy.Publisher('/positions', numpy_msg(Float64MultiArray), queue_size=10)

    # Create a CV bridge object
    cv_bridge = CvBridge()

    # Subscribe to the Raspberry Pi camera topic
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, process_image)

    # Start the main ROS loop
    rospy.spin()
