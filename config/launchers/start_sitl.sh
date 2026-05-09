#!/bin/bash

echo "Waiting for Gazebo to be ready..."

sleep 8

until gz topic -l > /dev/null 2>&1
do
    sleep 1
done

echo "Re-building package..."

cd ~/ros2_ws/ 

colcon build --packages-select control_scripts

sleep 20

source /opt/ros/humble/setup.bash

source ~/ros2_ws/install/setup.bash

echo "Starting Root Controller..."

ros2 run control_scripts root_controller

exec bash
