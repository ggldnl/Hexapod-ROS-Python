"""
Camera bring-up for the hexapod (Raspberry Pi Camera Module 2 / NoIR, IMX219).

Starts camera_ros with the libcamera overrides that pi_camera_setup.sh wrote to
~/.config/pi-camera-env.sh on the Pi. Nothing runs unless this file is launched,
so the sensor stays free the rest of the time.

    ros2 launch hexapod_controller camera.launch.py
    ros2 launch hexapod_controller camera.launch.py camera_width:=1280 camera_height:=720
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ENV_FILE = os.environ.get('PI_CAMERA_ENV',
                          os.path.expanduser('~/.config/pi-camera-env.sh'))


def read_camera_env(path=ENV_FILE):
    """Parse the KEY=value file written by pi_camera_setup.sh, empty dict if it is absent"""
    env = {}
    if not os.path.isfile(path):
        return env
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, value = line.split('=', 1)
            env[key.strip()] = value.strip()
    return env


def libcamera_overrides(env):
    """
    Environment the camera_node process needs to load the source-built libcamera
    instead of the packaged one. LD_LIBRARY_PATH is prepended to what ROS already
    set, otherwise /opt/ros/<distro>/lib wins the search order and the packaged
    libcamera (which cannot enumerate the sensor on a Pi 5) is used again
    """
    overrides = {}
    prefix = env.get('PI_CAMERA_LD_PREFIX')
    if prefix:
        current = os.environ.get('LD_LIBRARY_PATH', '')
        overrides['LD_LIBRARY_PATH'] = f'{prefix}:{current}' if current else prefix
    for key in ('LIBCAMERA_IPA_MODULE_PATH', 'LIBCAMERA_RPI_TUNING_FILE'):
        if env.get(key):
            overrides[key] = env[key]
    return overrides


def generate_launch_description():
    env = read_camera_env()

    args = [
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='node name, topics are published under /<camera_name>'),
        DeclareLaunchArgument(
            'camera_width', default_value=env.get('PI_CAMERA_WIDTH', '640')),
        DeclareLaunchArgument(
            'camera_height', default_value=env.get('PI_CAMERA_HEIGHT', '480')),
        # XRGB8888, the pipeline default, carries an unused alpha byte that some viewers
        # render as fully transparent, and costs a third more bandwidth for nothing
        DeclareLaunchArgument(
            'camera_format', default_value=env.get('PI_CAMERA_FORMAT', 'RGB888'),
            description='libcamera pixel format, RGB888 keeps the image three-channel'),
    ]

    camera = Node(
        package='camera_ros',
        executable='camera_node',
        name=LaunchConfiguration('camera_name'),
        parameters=[{
            'width': ParameterValue(LaunchConfiguration('camera_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('camera_height'), value_type=int),
            'format': ParameterValue(LaunchConfiguration('camera_format'), value_type=str),
        }],
        additional_env=libcamera_overrides(env),
        output='screen',
    )

    return LaunchDescription(args + [camera])
