#!/bin/bash
echo 'alias isaac=/isaac-sim/isaac-sim.sh' >> ~/.zshrc
echo 'source /ros2_humble/install/setup.zsh' >> ~/.zshrc
basekit_file="/isaac-sim/apps/omni.isaac.sim.base.kit"
sed -i 's/ros_bridge/ros2_bridge-humble/' $basekit_file