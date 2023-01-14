#!/bin/bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Installing Python tools 1/2"
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

echo "Installing Python devtools tools 2/2"
python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   pytest-repeat \
   pytest-rerunfailurescd

echo "Creating ros humble install location"
sudo mkdir -p /home/vscode/ros2_humble/src && cd /home/vscode/ros2_humble

echo "Downloading ROS repos"
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

echo "Installing dependencies"
sudo apt upgrade -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

echo "Building ROS2 Humble"
colcon build --symlink-install