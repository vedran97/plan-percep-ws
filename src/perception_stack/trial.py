import cv2
import numpy as np

# Load the image
cv_image = cv2.imread('src/perception_stack/image.png')

# Define constants
img_w = cv_image.shape[1]
img_h = cv_image.shape[0]
sensor_w = 3.68
sensor_h = 2.76
f = 3.04
cone_h = 177.8

# Perform some image processing on the CV image
cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
cv_image = cv2.GaussianBlur(cv_image, (9,9), 0)
hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

# Define lower and upper bounds for the fluorescent orange color
lower_orange = np.array([50,50,200])
upper_orange = np.array([110, 120, 255])

# Threshold the HSV image to get only fluorescent orange colors
mask = cv2.inRange(cv_image, lower_orange, upper_orange)

img = cv_image.copy()

# Find the contours in the binary image
contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

filtered_contours = []
positions = []

for contour in contours:
    if cv2.contourArea(contour) > 500:
        temp = np.squeeze(contour)
        max_y = np.max(temp[:,1])
        min_y = np.min(temp[:,1])
        y_img = (np.abs(max_y - min_y)/img_h)*2.76
        z = f * (cone_h)/y_img
        x_img = ((np.mean(temp[:,0]) - img_w/2)/img_w)*3.68
        x = x_img * (cone_h)/y_img
        positions.append([x,z])
        filtered_contours.append(contour)

print(positions)

# Draw the contours on the original image
cv2.drawContours(img, filtered_contours, -1, (0, 255, 0), 2)

# Display the image with contours
cv2.imshow('Contours', img)

res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

# Display the processed image
cv2.imshow('Processed Image', cv_image)
cv2.imshow('Fluorescent Orange Only', res)

# Wait for any key to be pressed and close all windows if 'q' is pressed
# key = cv2.waitKey(1) & 0xFF
# if key == ord('q'):

cv2.waitKey(0)
# Release the capture and destroy all windows
cv2.destroyAllWindows()
