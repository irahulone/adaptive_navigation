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

package_name: str = 'data_insights'


def add_nodes_based_on_namespace(context, *args, **kwargs):
    # Resolve all LaunchConfigurations
    ns = LaunchConfiguration('ns').perform(context)

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
        ]
    )

    live_read_to_csv = Node(
        package=package_name,
        executable='live_read_to_csv',
        namespace=ns,
        parameters=[
            config_file,
        ]
    )

    actions = [live_contour_plotter, live_read_to_csv] 

    # If sim mode, add sim node
    if hns.is_simulation:
        pass

    return actions


def generate_launch_description():
    # Declare arguments
    declare_launch_args = [
        DeclareLaunchArgument('ns', default_value='')
    ]

    return LaunchDescription(declare_launch_args + [
        OpaqueFunction(function=add_nodes_based_on_namespace)
    ])
