#!/bin/bash 

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** List active ROS2 services"
echo "*** ros2 service list"
ros2 service list


echo -e "\n\n\n*** List active ROS2 services with service type"
echo "*** ros2 service list -t"
ros2 service list -t


