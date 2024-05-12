#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

trap '[[ $BASH_COMMAND != echo* ]] && echo $BASH_COMMAND' DEBUG


echo -e "\n*** Killing slam-toolbox ***"
# killall slam-toolbox


# echo -e "\n*** Try killing any teleop_node and joy_node ***"
killall ros2

# Use when these are set up as lifecycle nodes
# ros2 lifecycle set slam_toolbox_node shutdown
# ros2 lifecycle set ydlidar_ros2_driver_node shutdown
