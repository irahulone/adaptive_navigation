from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# TODO:
"""
Edit the following:

- my_package
- my_node

- robot_speed
- use_sensors
- controller_gain

"""

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('rf_scalar_field'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('freq', default_value='1.0'),

        Node(
            package='rf_scalar_field',
            executable='rf_source',
            name='rf_source',
            parameters=[
                config_file,
                {
                    'port': LaunchConfiguration('port'),
                    'freq': LaunchConfiguration('freq'),
                }
            ]
        )
    ])
