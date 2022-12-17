#!/bin/bash
ISAAC_VERSION="2022.2.0"
basekit_file="/home/${USER}/.local/share/ov/pkg/isaac_sim-${ISAAC_VERSION}/apps/omni.isaac.sim.base.kit"

has_ros2_humble=$(grep ros2_bridge-humble $basekit_file)

if [ -z "$has_ros2_humble" ];
then
    echo "Setting ROS2 Humble as default bridge"
    sed -i 's/ros_bridge/ros2_bridge-humble/' $basekit_file
else
    echo "ROS2 Humble Already Default"
fi
