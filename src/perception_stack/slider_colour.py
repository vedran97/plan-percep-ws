import cv2
import numpy as np

def nothing(x):
    pass

# Load the image
img = cv2.imread('src/perception_stack/image2.png')

print(img.shape)

# Create a window for the trackbars
cv2.namedWindow('Trackbars')

# Create trackbars for adjusting the ranges
cv2.createTrackbar('LH', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('LS', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('LV', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('UH', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('US', 'Trackbars', 255, 255, nothing)
# cv2.createTrackbar('UV', 'Trackbars', 255, 255, nothing)

while True:
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Get trackbar values
    lh = cv2.getTrackbarPos('LH', 'Trackbars')
    ls = cv2.getTrackbarPos('LS', 'Trackbars')
    lv = cv2.getTrackbarPos('LV', 'Trackbars')
    uh = cv2.getTrackbarPos('UH', 'Trackbars')
    us = cv2.getTrackbarPos('US', 'Trackbars')
    uv = cv2.getTrackbarPos('UV', 'Trackbars')

    # Define lower and upper bounds for the fluorescent orange color
    lower_orange = np.array([lh, ls, lv])
    upper_orange = np.array([uh, us, uv])

    # Threshold the HSV image to get only fluorescent orange colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask=mask)

    # Display the original image and the masked image
    cv2.imshow('Original', img)
    cv2.imshow('Fluorescent Orange Only', res)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

# Release the capture and destroy all windows
cv2.destroyAllWindows()
