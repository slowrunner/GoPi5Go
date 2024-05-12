#!/bin/bash 

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Start GoPiGo3 ultrasonic ranger node"
echo "*** ros2 run ros2_gopigo3_node ultrasonic_ranger &"
ros2 run ros2_gopigo3_node ultrasonic_ranger &

