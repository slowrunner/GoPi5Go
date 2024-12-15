#!/bin/bash


echo -e `date +"%A %D %H:%M:%S"`

ros2 launch depthai_ros_driver rtabmap.launch.py params_file:=/home/pi/GoPi5Go/ros2ws/params/rtabmap.yaml

echo -e `date +"%A %D %H:%M:%S"`

