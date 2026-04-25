import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("findeeznuts")
    config_file = os.path.join(pkg_share, "config", "aruco_picker.yaml")

    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="ROS2 camera topic"
    )

    device_id_arg = DeclareLaunchArgument(
        "device_id",
        default_value="0",
        description="V4L2 camera device id (/dev/video<id>)"
    )

    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="gstreamer",
        description="Camera backend: gstreamer or v4l2"
    )

    width_arg = DeclareLaunchArgument(
        "width",
        default_value="640",
        description="Capture width"
    )

    height_arg = DeclareLaunchArgument(
        "height",
        default_value="480",
        description="Capture height"
    )

    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="30.0",
        description="Capture FPS"
    )

    gst_pipeline_arg = DeclareLaunchArgument(
        "gst_pipeline",
        default_value=(
            "libcamerasrc ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        ),
        description="GStreamer pipeline used when backend=gstreamer"
    )

    camera_node = Node(
        package="findeeznuts",
        executable="camera_capture_node",
        name="camera_capture",
        output="screen",
        parameters=[{
            "backend": LaunchConfiguration("backend"),
            "device_id": LaunchConfiguration("device_id"),
            "camera_topic": LaunchConfiguration("camera_topic"),
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fps": LaunchConfiguration("fps"),
            "gst_pipeline": LaunchConfiguration("gst_pipeline"),
            "frame_id": "camera_optical_frame",
        }],
        emulate_tty=True,
    )

    aruco_node = Node(
        package="findeeznuts",
        executable="aruco_picker_node",
        name="findeeznuts",
        output="screen",
        parameters=[
            config_file,
            {"camera_topic": LaunchConfiguration("camera_topic")},
        ],
        prefix="nice -n -10",
        emulate_tty=True,
    )

    return LaunchDescription([
        camera_topic_arg,
        device_id_arg,
        backend_arg,
        width_arg,
        height_arg,
        fps_arg,
        gst_pipeline_arg,
        camera_node,
        aruco_node,
    ])
