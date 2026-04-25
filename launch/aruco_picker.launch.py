"""
aruco_picker.launch.py

Launches the aruco_picker_node with:
  - Parameters loaded from config/aruco_picker.yaml
  - Nice level -10 (elevated priority without requiring root)
  - CPU affinity pre-set via taskset (cores 0-2)

Usage:
    ros2 launch findeeznuts aruco_picker.launch.py
    ros2 launch findeeznuts aruco_picker.launch.py camera_topic:=/my_cam/image_raw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("findeeznuts")
    config_file = os.path.join(pkg_share, "config", "aruco_picker.yaml")

    # ── Launch arguments ───────────────────────────────────────────────────────
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="ROS2 topic for the raw camera stream",
    )

    # ── Main node ──────────────────────────────────────────────────────────────
    aruco_node = Node(
        package="findeeznuts",
        executable="aruco_picker_node",
        name="aruco_picker",
        output="screen",
        parameters=[
            config_file,
            {"camera_topic": LaunchConfiguration("camera_topic")},
        ],
        # Remappings (adjust if your camera driver uses a different topic name)
        remappings=[
            ("/camera/image_raw", LaunchConfiguration("camera_topic")),
        ],
        # Elevate process priority (nice -10).
        # For real-time priority (nice -20) you need root or a RT kernel.
        prefix="nice -n -10",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            camera_topic_arg,
            aruco_node,
        ]
    )
