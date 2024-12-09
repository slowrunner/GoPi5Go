#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

# Note: slam_toolbox issues one /pose topic every half second
# echo -e "\n*** Capturing one /pose topic (flow-style)"
# echo "*** ros2 topic echo --once --flow-style /pose"
# ros2 topic echo --once --flow-style /pose
# ros2 topic hz /pose
ros2 topic echo --flow-style --once /amcl_pose

