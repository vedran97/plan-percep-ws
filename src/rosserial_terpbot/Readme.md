## This package contains the following nodes:
1. Curr_Vel_node : This node opens a serial tty port and publishes velocity information on the topic /CURR_VEL and
wheel odometry info on /CURR_ODOM. This runs in an isolated core of the RPI for real time performance.
2. Target_Publisher_Node: This node subscribes to /TARG_VEL topic, and in the callback, publishes the target velocity to an serial tty port talking to the arduino. This runs in an isolated core of the RPI for real time performance.
3. Both of these nodes are written in C++, any other nodes present have been deprecated.

## Instructions to start publishing trajectories:

1. In the workspace > execute ```sudo ./cpuset.sh```
2. Start roscore in 1 terminal
3. In another terminal: ``` rosrun rosserial_terpbot Curr_Vel_node ```
4. In another terminal: ``` rosrun rosserial_terpbot Target_Publisher_Node ```
