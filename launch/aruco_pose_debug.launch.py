"""
aruco_pose_debug.launch.py

Launches the aruco_pose_debug_node alongside the camera and aruco_picker nodes,
or standalone if the rest of the pipeline is already running.

Arguments:
  camera_topic  – raw image topic (default: /camera/image_raw)
  tags_topic    – DetectedTagArray topic (default: /aruco_picker/detected_tags)
  show_window   – open a local cv2 window, set true only on a desktop
                  (default: false)

Usage:
  # Debug only (pipeline already running)
    ros2 launch findeeznuts aruco_pose_debug.launch.py

  # Full stack + debug
    ros2 launch findeeznuts aruco_pose_debug.launch.py \\
      camera_topic:=/camera/image_raw

  # With local window (desktop/X11)
    ros2 launch findeeznuts aruco_pose_debug.launch.py show_window:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share  = get_package_share_directory("findeeznuts")
    config_file = os.path.join(pkg_share, "config", "aruco_picker.yaml")

    # ── Launch arguments ───────────────────────────────────────────────────────
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="Raw camera image topic",
    )
    tags_topic_arg = DeclareLaunchArgument(
        "tags_topic",
        default_value="/findeeznuts/detected_tags",
        description="DetectedTagArray topic published by aruco_picker_node",
    )
    show_window_arg = DeclareLaunchArgument(
        "show_window",
        default_value="false",
        description="Open a local OpenCV window (requires display / X11)",
    )

    # ── Debug node ─────────────────────────────────────────────────────────────
    debug_node = Node(
        package="findeeznuts",
        executable="aruco_pose_debug_node.py",
        name="aruco_pose_debug",
        output="screen",
        emulate_tty=True,
        # Load camera_matrix + dist_coeffs from the same YAML as the main node,
        # then override the topics via launch arguments.
        parameters=[
            config_file,
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "tags_topic":   LaunchConfiguration("tags_topic"),
                "show_window":  LaunchConfiguration("show_window"),
            },
        ],
    )

    return LaunchDescription([
        camera_topic_arg,
        tags_topic_arg,
        show_window_arg,
        debug_node,
    ])
