#!/bin/bash

<< ////
    FILE:  /home/pi/GoPi5Go/ros2ws/start_GoPi5Go-Dave.sh

This file starts the needed GoPi5Go-Dave nodes:
- ros2_gopigo3_node  GoPiGo3 ROS2 publishes odom, offers /cmd_vel etc
- battery_node  publishes /batter_state
- docking_node  publishes /dock_status, offers /dock and /undock services
- dave_node     calls /dock when vBatt<10v, and calls /undock when charge current < -175mA
- odometer      records ROS all /cmd_vel movement (does not record dock/undock movement)
- joy_node      handles wireless F710 joy controller to publish /cmd_vel
- say_node      TTS speech server offers /say {"phrase"} service

////

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash


echo -e "\n*** Start ROS2 GoPiGo3 node"
echo "*** ros2 run ros2_gopigo3_node gopigo3_node --ros-args -p S1LPW:=2094 -p S1RPW:=750 -p S1SECTOR:=2.443 &"
ros2 run ros2_gopigo3_node gopigo3_node --ros-args -p S1LPW:=2094 -p S1RPW:=750 -p S1SECTOR:=2.443 &

# Example starting with a yaml file for parameters
# ros2 run ros2_gopigo3_node gopigo3_node --ros-args --params-file ./src/ros2_gopigo3_node/gopigo3_node_params.yaml &

sleep 5

echo -e "\n*** Starting Robot_State and Joint_State Publishers"
echo "*** with URDF file: gopi5go_dave.urdf"
ros2 launch ros2_gopigo3_node ros2_gopi5go_dave_state_and_joint.launch.py &

sleep 5

echo -e "\n*** Starting Battery Node"
echo -e "*** ros2 run gopi5go_dave battery_node &"
ros2 run gopi5go_dave battery_node &

sleep 5

echo -e "\n*** Start Docking Node"
echo -e "*** ros2 run gopi5go_dave docking_node & "
ros2 run gopi5go_dave docking_node &

sleep 5 

echo -e "\n*** Starting Odometer Node to log movements"
echo "*** ros2 run ros2_gopigo3_node odometer"
ros2 run ros2_gopigo3_node odometer &

sleep 5

echo -e "\n*** Start gopi5go_dave.say_node"
echo '*** ros2 run gopi5go_dave say_node & '
ros2 run gopi5go_dave say_node &

sleep 5

# could not do this is container build for some reason...
echo -e "\n*** sudo chgrp input /dev/input/event*"
sudo chgrp input /dev/input/event*

# echo -e "\n*** Start SNES gamepad node"
# echo '*** ros2 launch teleop_twist_joy teleop-launch.py joy_config:="snes_slow" & '
# ros2 launch teleop_twist_joy teleop-launch.py joy_config:="snes_slow"  &

echo -e "\n*** Start F710 game controller node"
echo '*** ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" & '
ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" &

sleep 5

# DEBUG - sleep to keep detached ROS Docker container alive
# echo -e "\n*** DEBUG: NOT STARTING DAVE NODE ***"
# sleep 36000

# Start the last node in foreground to keep detached docker terminal running:
echo -e "\n*** Start Dave Node"
echo -e "*** ros2 run gopi5go_dave dave_node "
ros2 run gopi5go_dave dave_node  
