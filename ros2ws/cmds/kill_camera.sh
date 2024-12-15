#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

trap '[[ $BASH_COMMAND != echo* ]] && echo $BASH_COMMAND' DEBUG

echo -e "\n*** STARTING kill_joy.sh ***"
killall joy_node
killall teleop_node
echo -e  "\n*** DONE KILLING JOY ***\n"
