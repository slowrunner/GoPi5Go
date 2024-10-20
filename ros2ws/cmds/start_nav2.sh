#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash


echo -e "\n*** STARTING NAV2 NAVIGATION ***"
# Starts Nav2 with all defaults
# echo -e "\n*** ros2 launch nav2_bringup navigation_launch.py"
# ros2 launch nav2_bringup navigation_launch.py

# Start Nav2 with local params_file
echo -e "\n*** ros2 launch nav2_bringup navigation_launch.py params_file:=gopigo3_nav2_params.yaml"
ros2 launch nav2_bringup navigation_launch.py params_file:=params/gopigo3_nav2_params.yaml
