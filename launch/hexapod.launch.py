"""
Bring up the hexapod controller, with the camera as an opt-in extra.

    ros2 launch hexapod_controller hexapod.launch.py
    ros2 launch hexapod_controller hexapod.launch.py camera:=true
    ros2 launch hexapod_controller hexapod.launch.py camera:=true enable_on_start:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('hexapod_controller')

    args = [
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='also start the camera_ros node'),
        DeclareLaunchArgument(
            'config_path', default_value=os.path.join(share, 'config', 'config.yml')),
        DeclareLaunchArgument(
            'port', default_value='',
            description='overrides serial.port from the config when set'),
        DeclareLaunchArgument('update_rate', default_value='100'),
        DeclareLaunchArgument('reply_timeout', default_value='0.05'),
        DeclareLaunchArgument('enable_on_start', default_value='false'),
    ]

    controller = Node(
        package='hexapod_controller',
        executable='hexapod_controller',
        name='hexapod_controller_node',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'port': LaunchConfiguration('port'),
            'update_rate': ParameterValue(LaunchConfiguration('update_rate'), value_type=int),
            'reply_timeout': ParameterValue(LaunchConfiguration('reply_timeout'), value_type=float),
            'enable_on_start': ParameterValue(LaunchConfiguration('enable_on_start'), value_type=bool),
        }],
        output='screen',
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    return LaunchDescription(args + [controller, camera])
