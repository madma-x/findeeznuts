#!/usr/bin/env bash
set -euo pipefail

source /home/ovoleur/ros2_jazzy/install/setup.bash
source /home/ovoleur/findeeznuts/install/setup.bash
export AMENT_PREFIX_PATH=/home/ovoleur/findeeznuts/install/aruco_picker:${AMENT_PREFIX_PATH:-}

# Network/discovery settings for cross-machine ROS2 communication
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec ros2 launch aruco_picker aruco_with_camera.launch.py
