#!/bin/bash

echo "Waiting for Gazebo to be ready..."

sleep 8

until gz topic -l > /dev/null 2>&1
do
    sleep 1
done

echo "Gazebo detected. Starting UAV 2..."

cd ~/PX4-Autopilot || exit

PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=baylands PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3

exec bash
