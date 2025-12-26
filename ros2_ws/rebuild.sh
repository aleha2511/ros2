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

# Если запущен старый процесс, убиваем его
OLD_PID=$(pgrep -f "ros2 launch $LAUNCH_FILE")
if [ ! -z "$OLD_PID" ]; then
    echo "Killing old launch process $OLD_PID"
    kill -9 $OLD_PID || true
fi

echo "Launching robot_bringup..."
ros2 launch robot_bringup bringup.launch.py
