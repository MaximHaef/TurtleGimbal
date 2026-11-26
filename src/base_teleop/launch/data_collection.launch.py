"""Gimbal Pi: data-collection run (F9).

Records everything to an MCAP bag and plays an excitation sequence on
/cmd_vel. Optionally starts the gimbal nodes too. The base_command relay must
already be running on the robot Pi.

    ros2 launch base_teleop data_collection.launch.py sequence_file:=/path/to/steps.yaml \
        [bag:=DIR] [with_gimbal:=true] [with_imu:=false] [stop_on_kobuki_events:=false]

with_imu (default true) starts the standalone IMU A publisher (/gimbal/imu_a,
/gimbal/imu_a/raw). Turn it off once gimbal_control reads IMU A itself.
stop_on_kobuki_events (default true) is passed to the excitation node: stop on
Kobuki bumper, cliff, wheel-drop and button events. Needs
kobuki_ros_interfaces on this machine.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    example = os.path.join(get_package_share_directory('base_teleop'), 'config', 'steps_example.yaml')
    gimbal = os.path.join(get_package_share_directory('gimbal_bringup'), 'launch', 'gimbal.launch.py')
    params = os.path.join(get_package_share_directory('gimbal_bringup'), 'config', 'gimbal.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('sequence_file', default_value=example),
        DeclareLaunchArgument('bag', default_value='data_collection',
                              description='Output directory of the rosbag'),
        DeclareLaunchArgument('with_gimbal', default_value='false',
                              description='Also start gimbal.launch.py (needs the TBD periods set)'),
        DeclareLaunchArgument('with_imu', default_value='true',
                              description='Start the standalone IMU A publisher (gimbal_control imu_node)'),
        DeclareLaunchArgument('stop_on_kobuki_events', default_value='true',
                              description='Excitation stops on Kobuki bumper/cliff/wheel-drop/button events'),
        Node(package='gimbal_control', executable='imu_node', name='imu_a',
             parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('with_imu'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gimbal),
                                 condition=IfCondition(LaunchConfiguration('with_gimbal'))),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-s', 'mcap', '-o', LaunchConfiguration('bag')],
            output='screen'),
        Node(package='base_teleop', executable='excitation', name='base_excitation',
             parameters=[{'sequence_file': LaunchConfiguration('sequence_file'),
                          'stop_on_kobuki_events': ParameterValue(LaunchConfiguration('stop_on_kobuki_events'),
                                                                  value_type=bool)}],
             output='screen'),
    ])
