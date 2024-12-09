#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\nPublishing servo one right 45 degree position (-0.707 radians)"
ros2 topic pub --once /servo/position/S1 std_msgs/msg/Float64 '{data: -0.707}'
