#! /bin/bash
set -e

SCRIPT_LOCATION=$(dirname $0)
SCRIPT_NAME=$(basename $0)

USERNAME=$USER
WORKSPACE_PATH=$1

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt install -y ros-noetic-ros-base
sudo apt-get install -y python3-rosdep

sudo apt-get update
sudo apt-get -y install --no-install-recommends \
    $(echo $(cat $SCRIPT_LOCATION/../dev-tools/common.txt))

ROS_DISTRO="noetic"

# Source ROS
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

# Source workspace
echo "source ${WORKSPACE_PATH}/devel/setup.bash" >> /home/$USERNAME/.bashrc

# Define Python version for ros
echo "export ROS_PYTHON_VERSION=3" >> /home/$USERNAME/.bashrc

# These have been taken from Full ROS installation to avoid issues later on
echo "export CMAKE_PREFIX_PATH=/opt/ros/noetic" >> /home/$USERNAME/.bashrc
echo "export ROS_ROOT=/opt/ros/noetic/share/ros" >> /home/$USERNAME/.bashrc
echo "export ROS_DISTRO=noetic" >> /home/$USERNAME/.bashrc
echo "export ROS_ETC_DIR=/opt/ros/noetic/etc/ros" >> /home/$USERNAME/.bashrc
echo "export ROS_VERSION=1 >>" /home/$USERNAME/.bashrc 
echo "export PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig" >> /home/$USERNAME/.bashrc
echo "export ROS_PACKAGE_PATH=/opt/ros/noetic/share" >> /home/$USERNAME/.bashrc
echo "export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages" >> /home/$USERNAME/.bashrc
echo "export PATH=/opt/ros/noetic/bin:$PATH" >> /home/$USERNAME/.bashrc

source /home/$USERNAME/.bashrc

sudo rosdep init
rosdep update
sudo apt-get update
rosdep install --from-paths $SCRIPT_LOCATION/../src --ignore-src -r -y

