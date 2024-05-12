#!/bin/bash 

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\nStart MPU9250 IMU sensor node"
echo "ros2 run mpu9250_driver mpu9250driver_launch.py"
ros2 launch mpu9250driver mpu9250driver_launch.py



