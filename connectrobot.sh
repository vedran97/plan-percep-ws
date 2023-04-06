#! /bin/sh

alias myip="ip addr show wlp0s20f3 | grep -Eo 'inet ([0-9]{1,3}\.){3}[0-9]{1,3}' | awk '{print \$2}'"

export ROS_MASTER_URI=http://10.104.119.60:11311

export ROS_IP=$(myip)
export ROS_HOSTNAME=$(myip)