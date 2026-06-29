import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    this_dir = get_package_share_directory('vectornav')

    # Utility to set / dump the GPS Antenna Offset and GPS Compass Baseline
    # registers on a VectorNav sensor. Edit config/antenna_offset.yaml first.
    start_vn_set_antenna_config_cmd = Node(
        package='vectornav',
        executable='vn_set_antenna_config',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'antenna_offset.yaml')])

    ld = LaunchDescription()
    ld.add_action(start_vn_set_antenna_config_cmd)

    return ld
