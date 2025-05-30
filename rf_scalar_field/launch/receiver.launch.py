from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from typing import List

from adaptive_navigation_utilities.namespace_builder import HardwareNamespace

# Namespace builder
hns: HardwareNamespace = HardwareNamespace()


def add_nodes_based_on_namespace(context, *args, **kwargs):
    # Resolve all LaunchConfigurations
    port1 = LaunchConfiguration('port1').perform(context)
    freq1 = LaunchConfiguration('freq1').perform(context)
    port2 = LaunchConfiguration('port2').perform(context)
    freq2 = LaunchConfiguration('freq2').perform(context)
    freq3 = LaunchConfiguration('freq3').perform(context)
    ns = LaunchConfiguration('ns').perform(context)

    # Update hardware namespace
    hns.sub(ns)
    print(f"[Namespace] {ns}")

    config_file = os.path.join(
        get_package_share_directory('rf_scalar_field'),
        'config',
        'params.yaml'
    )

    rf_receiver = Node(
        package='rf_scalar_field',
        executable='rf_receiver',
        name='rf_receiver',
        namespace=ns,
        parameters=[
            config_file,
            {'port': port2, 'freq': float(freq2)}
        ]
    )

    actions = [rf_receiver]

    # If sim mode, add sim node
    if hns.is_simulation:
        sim_lat_long = Node(
            package='rf_scalar_field',
            executable='sim_lat_long',
            name='sim_lat_long',
            namespace=ns,
            parameters=[
                config_file,
                {'freq': float(freq3)}
            ]
        )
        actions.append(sim_lat_long)

    return actions


def generate_launch_description():
    # Declare arguments
    declare_launch_args = [
        DeclareLaunchArgument('port1', default_value='/dev/ttyUSB2'),
        DeclareLaunchArgument('freq1', default_value='1.0'),
        DeclareLaunchArgument('port2', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('freq2', default_value='1.0'),
        DeclareLaunchArgument('freq3', default_value='2.0'),
        DeclareLaunchArgument('ns', default_value='')
    ]

    return LaunchDescription(declare_launch_args + [
        OpaqueFunction(function=add_nodes_based_on_namespace)
    ])
