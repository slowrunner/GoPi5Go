#!/bin/bash

# echo -e "\n*** THIS NODE DOES NOT FUNCTION YET - USE start_image_pub.sh"
# exit

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** Start Camera Node"
# echo "*** ros2 run v4l2_camera v4l2_camera_node"
# ros2 run v4l2_camera v4l2_camera_node
echo "*** ros2 run v4l2_camera v4l2_camera_node -ros-args --params-file /home/ubuntu/ros2ws/v4l2_camera_params.yaml"
ros2 run v4l2_camera v4l2_camera_node -ros-args --params-file /home/ubuntu/ros2ws/v4l2_camera_params.yaml
