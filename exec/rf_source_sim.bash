#!/bin/bash

# Variables
WORKSPACE_DIR=robot_ws
REPO=adaptive_navigation
PACKAGE=rf_scalar_field
NODE=rf_source

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source $HOME/$WORKSPACE_DIR/install/local_setup.sh

# Run the Node
ros2 run $PACKAGE $NODE --ros-args -r __ns:=/sim