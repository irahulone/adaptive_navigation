from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from typing import List
import yaml


from adaptive_navigation_utilities.namespace_builder import HardwareNamespace

# Namespace builder
hns: HardwareNamespace = HardwareNamespace()

package_name: str = 'data_insights'


def add_nodes_based_on_namespace(context, *args, **kwargs):

    # Create list of actions
    actions = list()

    # Resolve all LaunchConfigurations
    ns = LaunchConfiguration('ns').perform(context)

    # Get all robot id list
    robot_id_list = yaml.safe_load(
        LaunchConfiguration('robot_id_list').perform(context)
    )
    print(f"Robot ID list: {robot_id_list}")

    # See if plot shows latest
    # NOTE: Use YAML to safely typecast LaunchArgument a    s a string to a bool
    show_latest = yaml.safe_load(
        LaunchConfiguration('show_latest').perform(context)
    )
    print(f"Show latest: {show_latest} of type {type(show_latest)}")

    # Update hardware namespace
    hns.sub(ns)
    print(f"[Namespace] {ns}")

    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )

    # Define nodes
    live_contour_plotter = Node(
        package=package_name,
        executable='live_contour_plotter',
        name='live_contour_plotter',
        namespace=ns,
        parameters=[
            config_file,
            {
                'show_latest': show_latest
            }
        ]
    )
    actions.append(live_contour_plotter)

    # For each robot
    # Create synchronizer collect_gps_rssi nodes
    
    for robot in robot_id_list:
        print(robot)
        collect_gps_rssi = Node(
            package='synchronizer',
            executable='collect_gps_rssi',
            name= robot + '_collect_gps_rssi',
            namespace=ns,
            parameters=[
                config_file,
                {
                    'robot_id': robot
                }
            ]
        )
        actions.append(collect_gps_rssi)

    # Universal csv logger
    live_data_to_csv = Node(
        package='adaptive_navigation_utilities',
        executable='live_data_to_csv',
        parameters=[{
            "record_all_topics": True
        }]
    )
    actions.append(live_data_to_csv)

    # If sim mode, add sim node
    if hns.is_simulation:
        pass

    return actions


def generate_launch_description():
    # Declare arguments
    declare_launch_args = [
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('robot_id_list', default_value="['p1','p2']"),
        DeclareLaunchArgument('show_latest', default_value="false")
    ]

    return LaunchDescription(declare_launch_args + [
        OpaqueFunction(function=add_nodes_based_on_namespace)
    ])
