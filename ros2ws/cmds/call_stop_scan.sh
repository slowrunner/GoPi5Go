#!/bin/bash

# FILE: call_stop_scan.sh

# PURPOSE:  call the ydlidar node's /stop_scan service
#           Saves 0.7W

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Calling ydlidar_ros_driver's /stop_scan service"
echo -e "ros2 service call /stop_scan std_srvs/Empty"
ros2 service call /stop_scan std_srvs/Empty
