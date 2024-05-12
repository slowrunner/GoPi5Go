#!/bin/bash
basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

# for mpu9250driver package
echo -e "\n*** Capturing one MPU9250 IMU topic (flow-style):"
echo "*** ros2 topic echo --once --flow-style /imu"
ros2 topic echo --once --flow-style /imu

