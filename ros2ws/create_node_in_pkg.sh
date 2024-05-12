#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash


if [ "$#" -ne 2 ]; then
  echo "USAGE: ./create_node_in_pkg.sh node_name pkg_name"
  exit
fi
echo -e "\n*** CREATE ROS 2 NODE $1 IN PACKAGE $2"
echo -e "ros2 pkg create --build-type ament_python --node-name $1 $2"
pushd src
ros2 pkg create --build-type ament_python --node-name $1 $2
popd

