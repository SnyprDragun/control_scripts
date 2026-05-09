#!/bin/bash

echo "Launching PX4..."

cd ~/PX4-Autopilot || exit

PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_UXRCE_DDS_NS=px4_1 ./build/px4_sitl_default/bin/px4 -i 1
# PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=baylands PX4_SIM_MODEL=gz_x500 PX4_UXRCE_DDS_NS=px4_1 ./build/px4_sitl_default/bin/px4 -i 1

exec bash
