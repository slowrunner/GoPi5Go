#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Capturing one /map topic (flow-style) from slam-toolbox:"
echo "*** ros2 topic echo --once --flow-style /map"
ros2 topic echo --once --flow-style /map
ros2 topic hz /map


