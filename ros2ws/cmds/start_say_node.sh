#!/bin/bash

basedir=GoPi5Go

echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Start gopi5go_dave.say_node"
echo '*** ros2 run gopi5go_dave say_node &'
ros2 run gopi5go_dave say_node &

