#!/bin/bash
set -e

GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}Installing Base Packages${NC}"
apt update
apt install software-properties-common apt-utils -y
add-apt-repository universe
apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "${GREEN}Installing Python tools 1/2${NC}"
apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

echo -e "${GREEN}Installing Python devtools tools 2/2${NC}"
python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   pytest-repeat \
   pytest-rerunfailures

echo -e "${GREEN}Creating ros humble install location${NC}"
mkdir -p /ros2_humble/src && cd /ros2_humble

echo -e "${GREEN}Downloading ROS repos${NC}"
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

echo -e "${GREEN}Installing dependencies${NC}"
apt upgrade -y
rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

echo -e "${GREEN}Building ROS2 Humble${NC}"
colcon build --symlink-install