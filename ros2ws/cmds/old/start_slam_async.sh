#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** STARTING ROS2 SLAM-TOOLBOX "
echo "*** Drive GoPiGo3 around room, generating /map topics"
echo "*** (Async: Best-effort processing, online - navigating on limited CPU)"
echo "*** ros2 launch slam_toolbox online_async_launch.py"
ros2 launch slam_toolbox online_async_launch.py 'slam_params_file:=./params/my_mapper_params_online_async.yaml'

