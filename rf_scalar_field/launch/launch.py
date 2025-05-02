from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    port1 = LaunchConfiguration('port1')
    freq1 = LaunchConfiguration('freq1')
    port2 = LaunchConfiguration('port2')
    freq2 = LaunchConfiguration('freq2')

    config_file = os.path.join(
        get_package_share_directory('rf_scalar_field'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('port1', default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('freq1', default_value='1.0'),
        DeclareLaunchArgument('port2', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('freq2', default_value='1.0'),

        Node(
            package='rf_scalar_field',
            executable='rf_source',
            name='rf_source',
            parameters=[
                config_file,
                {'port': port1, 'freq': freq1}
            ]
        ),
        Node(
            package='rf_scalar_field',
            executable='rf_receiver',
            name='rf_receiver',
            parameters=[
                config_file,
                {'port': port2, 'freq': freq2}
            ]
        ),
    ])

