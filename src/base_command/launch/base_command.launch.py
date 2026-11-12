"""Robot Pi: starts kobuki_node and the base_command relay.

    ros2 launch base_command base_command.launch.py
    ros2 launch base_command base_command.launch.py start_kobuki:=false   # driver already running
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('base_command'), 'config', 'base_command.yaml')
    start_kobuki = LaunchConfiguration('start_kobuki')

    kobuki = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kobuki_node'), 'launch', 'kobuki_node-launch.py')),
        condition=IfCondition(start_kobuki),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_kobuki', default_value='true',
                              description='Also start kobuki_node'),
        kobuki,
        Node(package='base_command', executable='base_command_node', name='base_command',
             parameters=[params], output='screen'),
    ])
