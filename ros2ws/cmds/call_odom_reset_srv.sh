#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Before /odom/reset, Check current /odom"
echo -e "*** Capturing one odometer topic (flow-style):"
echo "*** ros2 topic echo --once --flow-style /odom"
ros2 topic echo --once --flow-style /odom


echo -e "\n*** Calling /odom/reset service"
echo "*** ros2 service call /odom/reset std_srvs/srv/Trigger"
echo "*** (note no data required for Trigger type)"
ros2 service call /odom/reset std_srvs/srv/Trigger


echo -e "\n*** Now check /odom topic to see reset effect"
echo -e "*** Capturing one odometer topic (flow-style):"
echo "*** ros2 topic echo --once --flow-style /odom"
ros2 topic echo --once --flow-style /odom

# Log reset
dt=$(date +"%Y-%m-%d %H:%M:%S")
echo -e "${dt}|RESET **** -  x:  0.000 y:  0.000 z:  0.000 heading:    0 - Manual RESET" >> /home/pi/GoPi5Go/logs/odometer.log
