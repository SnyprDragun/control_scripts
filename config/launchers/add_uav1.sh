#!/bin/bash

echo "Waiting for Gazebo to be ready..."

sleep 8

until gz topic -l > /dev/null 2>&1
do
    sleep 1
done

echo "Gazebo detected. Starting UAV 1..."

cd ~/PX4-Autopilot || exit

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 PX4_UXRCE_DDS_NS=px4_2 ./build/px4_sitl_default/bin/px4 -i 2
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=baylands PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 PX4_UXRCE_DDS_NS=px4_2 ./build/px4_sitl_default/bin/px4 -i 2

exec bash
