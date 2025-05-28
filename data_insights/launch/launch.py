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

package_name = 'data_insights'

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_file),

        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[
                config_file,
                {
                    'config': LaunchConfiguration('config')
                }
            ]
        )
    ])
