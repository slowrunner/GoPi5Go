#!/bin/bash

# FILE: call_undock.sh

# PURPOSE:  call the docking_node  /undock service

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\nNOTE: IF dave_node is running, dave_node will also call /undock"
echo -e "\n*** Calling docking_node /undock service"
echo -e "ros2 service call /undock dave_interfaces/srv/Undock"
ros2 service call /undock dave_interfaces/srv/Undock
