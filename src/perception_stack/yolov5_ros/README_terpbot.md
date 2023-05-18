Steps:

1. Install all the dependencies required for the project in your docker by running 'pip install -r requirements.txt' in the current folder.
2. The previous step installs all the required GPU dependencies to run Yolo as well.
3. Start the node to read image using cv_bridge, followed by processing using the Yolo model by running the 'detect_cones_ros.py' script file by using the command 'python3 detect_cones_ros.py'
4. If the raspicam_node is up and running on the raspberry pi, you can see various openCV windows showing the feed from the camera.

Notes:

1. In the current state of the code, it can only publish data regarding one 'red' cone.
2. The message published is of the type Pose2D and can be found in the topic '/positions'.
3. In the current state of the code, it publishes data to the topic if and only if the cone detected is within 70 cms from the base of the terpbot. (To modify this value, change the condition value in line 160 of the detect_cones_ros.py file)
4. The x and y coordinates reported in the Pose2D message are with respect to the Odom origin.