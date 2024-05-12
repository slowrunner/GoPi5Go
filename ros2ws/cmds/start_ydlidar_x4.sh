#!/bin/bash 

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Start YDLidar X4 node"
echo "*** ros2 launch ydlidar_ros2_driver ydlidar_launch.py"
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
