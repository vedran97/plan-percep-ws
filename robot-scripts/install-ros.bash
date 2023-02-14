#! /bin/bash
set -e

SCRIPT_LOCATION=$(dirname $0)
SCRIPT_NAME=$(basename $0)

USERNAME=$1
WORKSPACE_PATH=$2

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt install ros-noetic-ros-base


sudo apt-get update
sudo apt-get -y install --no-install-recommends \
    $(echo $(cat $SCRIPT_LOCATION/../dev-tools/common.txt))

# Source ROS
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

# Source workspace
echo "source ${WORKSPACE_PATH}/devel/setup.bash" >> /home/$USERNAME/.bashrc

# Define Python version for ros
echo "export ROS_PYTHON_VERSION=3" >> /home/$USERNAME/.bashrc

source /home/$USERNAME/.bashrc

sudo rosdep init
rosdep update
sudo apt-get update
rosdep install --from-paths $SCRIPT_LOCATION/../src --ignore-src -r -y

