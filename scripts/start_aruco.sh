#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts may reference unset vars; disable nounset while sourcing.
set +u
source /home/ovoleur/ros2_jazzy/install/setup.bash
source /home/ovoleur/holonomic_robot/install/setup.bash
set -u

export AMENT_PREFIX_PATH=/home/ovoleur/holonomic_robot/install/aruco_picker:${AMENT_PREFIX_PATH:-}

# Network/discovery settings for cross-machine ROS2 communication
export ROS_DOMAIN_ID=0



exec ros2 launch aruco_picker aruco_with_camera.launch.py
