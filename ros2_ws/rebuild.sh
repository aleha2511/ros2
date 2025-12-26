#!/bin/bash
set -e

WORKSPACE=~/ros2/ros2_ws
LAUNCH_FILE=robot_bringup/bringup.launch.py

echo "Cleaning previous builds..."
rm -rf $WORKSPACE/build/ $WORKSPACE/install/ $WORKSPACE/log/

echo "Building workspace..."
colcon build --symlink-install

echo "Sourcing ROS2 and workspace..."
source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash

echo "Launching robot_bringup..."
ros2 launch robot_bringup bringup.launch.py
