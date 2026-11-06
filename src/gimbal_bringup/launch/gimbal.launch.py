"""Starts all gimbal nodes, each in its own process."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('gimbal_bringup'), 'config', 'gimbal.yaml')
    return LaunchDescription([
        Node(package='gimbal_predictor', executable='predictor_node', name='predictor',
             parameters=[params], output='screen'),
        Node(package='gimbal_control', executable='control_node', name='gimbal_control',
             parameters=[params], output='screen'),
        Node(package='gimbal_vision', executable='vision_node', name='vision',
             parameters=[params], output='screen'),
    ])
