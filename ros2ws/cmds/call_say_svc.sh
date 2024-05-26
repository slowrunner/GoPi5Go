#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash


if [ "$#" -ne 1 ] ;
	then echo 'Usage:  ./call_say_svc.sh "string to speak" '
	exit
fi

echo -e "ros2 service call /say dave_interfaces/srv/Say \"${1}\""

ros2 service call /say dave_interfaces/srv/Say "saystring:  '${1}'"
