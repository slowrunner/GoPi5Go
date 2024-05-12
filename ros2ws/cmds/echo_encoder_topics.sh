#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Capturing left encoder topic (flow-style):"
echo "*** ros2 topic echo --once --flow-style /motor/encoder/left"
ros2 topic echo --once --flow-style /motor/encoder/left

echo -e "\n*** Capturing right encoder topic (flow-style):"
echo "*** ros2 topic echo --once --flow-style /motor/encoder/right"
ros2 topic echo --once --flow-style /motor/encoder/right

