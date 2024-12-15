#!/bin/bash


# ros2 topic echo --once --flow-style /color/mobilenet_detections 

# 0            "background",
# 1           "aeroplane",
# 2           "bicycle",
# 3           "bird",
# 4           "boat",
# 5           "bottle",
# 6           "bus",
# 7           "car",
# 8           "cat",
# 9           "chair",
# 10           "cow",
# 11           "diningtable",
# 12           "dog",
# 13           "horse",
# 14           "motorbike",
# 15           "person",
# 16           "pottedplant",
# 17           "sheep",
# 18           "sofa",
# 19           "train",
# 20          "tvmonitor"


dt=`(uptime)`
echo -e "\n ${dt}"
echo -e "LAUNCHING OAK-D-W97 camera.launch.py WITH params/camera.yaml (mobilenet + RGBD)"
echo -e "ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-W params_file:=/home/pi/GoPi5Go/ros2ws/params/camera.yaml\n"
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-W params_file:=/home/pi/GoPi5Go/ros2ws/params/camera.yaml


dt=`(uptime)`
echo -e "${dt}\n"
