#!/bin/bash

echo -e "\n*** REBUILDING ROS2 GoPi5Go ***"

echo "*** Executing rebuild.sh ***"

echo "*** cd ~/GoPi5Go/ros2ws"
cd ~/GoPi5Go/ros2ws

echo "*** rosdep install -i --from-path src"
rosdep install -i --from-path src


echo "*** colcon build --packages-select ros2_gopigo3_msg "
colcon build --packages-select ros2_gopigo3_msg

echo "*** colcon build --packages-select ros2_gopigo3_node"
colcon build --symlink-install --packages-select ros2_gopigo3_node

echo "*** colcon build --packages-select teleop_gopigo3_keyboard"
colcon build --symlink-install --packages-select teleop_gopigo3_keyboard

# echo "*** colcon build --packages-select ros2_libcamera_pub"
# colcon build --packages-select ros2_libcamera_pub

# echo "** colcon build --symlink-install --packages-select ydlidar_ros2_driver"
# colcon build --symlink-install --packages-select ydlidar_ros2_driver

# echo "** colcon build --symlink-install --packages-select scan_client"
# colcon build --symlink-install --packages-select scan_client

# echo "** colcon build --symlink-install --packages-select mpu9250driver"
# colcon build --symlink-install --packages-select mpu9250driver

echo -e "\n*** . install/setup.bash"
. install/setup.bash

echo "*** REBUILD DONE"
