#!/bin/bash

# FILE:  start_dave_robot_and_joint_state.sh

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Starting Robot_State and Joint_State Publishers"
echo "*** with URDF file: gpg3_gopi5go_dave.urdf"
ros2 launch ros2_gopigo3_node ros2_gopi5go_dave_state_and_joint.launch.py &
