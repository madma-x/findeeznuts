from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def _include(package_name: str, launch_file: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', launch_file)
        )
    )


def generate_launch_description():
    return LaunchDescription([
        _include('i2c_node', 'i2c.launch.py'),
        _include('robot_actuators', 'sequencer.launch.py'),
        _include('findeeznuts', 'aruco_with_camera.launch.py'),
    ])
