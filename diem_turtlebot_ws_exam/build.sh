#!/bin/bash

WORK_DIR=$(pwd)

# Window for building
gnome-terminal -- bash -c "
cd $WORK_DIR;
source /opt/ros/humble/setup.bash;
colcon build;
exit"