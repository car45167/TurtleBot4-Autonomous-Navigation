#!/bin/bash

WORK_DIR=$(pwd)

gnome-terminal \
--tab --title="Launch map" -- bash -c "
cd $WORK_DIR;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
ros2 launch turtlebot4_navigation localization.launch.py map:=src/map/diem_map.yaml;
exec bash" &

gnome-terminal \
--tab --title="Nav2" -- bash -c "
cd $WORK_DIR;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
ros2 launch turtlebot4_navigation nav2.launch.py;
exec bash" &

gnome-terminal \
--tab --title="Visualization" -- bash -c "
cd $WORK_DIR;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
ros2 launch turtlebot4_viz view_robot.launch.py;
exec bash" &

gnome-terminal \
--tab --title="Cone Detector node" -- bash -c "
cd $WORK_DIR;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
ros2 run exam_pkg cone_detector;
exec bash" &

gnome-terminal \
--tab --title="Waypoint Manager node" -- bash -c "
cd $WORK_DIR;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
ros2 run exam_pkg waypoint_manager;
exec bash" &

gnome-terminal \
--tab --title="Navigation node" -- bash -c "
cd $WORK_DIR;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
ros2 run exam_pkg navigation;
exec bash" &
