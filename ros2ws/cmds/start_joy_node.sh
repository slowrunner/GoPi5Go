#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** sudo chgrp input /dev/input/event*"
sudo chgrp input /dev/input/event*


# echo -e "\n*** Start SNES gamepad node"
# echo '*** ros2 launch teleop_twist_joy teleop-launch.py joy_config:="snes_slow" '
# ros2 launch teleop_twist_joy teleop-launch.py joy_config:="snes_slow" 

echo -e "\n*** Start F710 game controller node"
echo '*** ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" '
ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" &
