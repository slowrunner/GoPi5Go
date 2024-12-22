#!/bin/bash


dt=`(uptime)`
echo -e "\n ${dt}"

echo -e "Running apriltag_node to detect tag36h11 type tags (Dave's dock is tag 05 with frame_id dock"
echo -e "ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/oak/rgb/image_raw -r camera_info:=/oak/rgb/camera_info \ "
echo -e "    --param-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/dave_tags_36h11.yaml"

ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/oak/rgb/image_raw \
    -r camera_info:=/oak/rgb/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/dave_tags_36h11.yaml


dt=`(uptime)`
echo -e "${dt}\n"
