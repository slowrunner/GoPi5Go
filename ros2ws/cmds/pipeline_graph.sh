#!/bin/bash

echo -e "/n*** RUNNING DepthAI pipeline_graph"
echo -e "(TigerVNC into GoPi5Go-Dave)"
echo -e "(./display_to_goPi5GoROS2.sh)"
echo -e "pipeline_graph run \"ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-W params_file:=/home/pi/GoPi5Go/ros2ws/params/camera.yaml\" "
pipeline_graph run "ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-W params_file:=/home/pi/GoPi5Go/ros2ws/params/camera.yaml"

