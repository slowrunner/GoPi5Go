#!/bin/bash

# FILE: run_teleop_gopigo3_keyboard.sh
# PURPOSE: Start GoPiGo3 robot version of teleop_twist_keyboard

echo -e "\n\n******** STARTING TELEOP GOPIGO3 KEYBOARD ********"

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Start teleop_gopigo3_keyboard node"
echo "*** ros2 run teleop_gopigo3_keyboard teleop_gopigo3_keyboard"
ros2 run teleop_gopigo3_keyboard teleop_gopigo3_keyboard

