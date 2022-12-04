#!/bin/bash
ISAAC_VERSION="2022.1.1"
basekit_file="/home/${USER}/.local/share/ov/pkg/isaac_sim-${ISAAC_VERSION}/apps/omni.isaac.sim.base.kit"

has_ros2=$(grep ros2 $basekit_file)

if [ -z "$has_ros2" ];
then
    echo "Setting ROS2 as default bridge"
    sed -i 's/ros/ros2/' $basekit_file
else
    echo "ROS2 Already Default"
fi
