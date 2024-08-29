#!/bin/bash

# FILE: run_gopi5goROS2.sh

# Use to start the gopi5gor2hdp image as gopi5goROS2 named container

# Can be used manually to start, or automatically via service
#   sudo systemctl enable gopi5goROS2
#   sudo systemctl start gopi5goROS2

cd ~/GoPi5Go/ros2ws
# --rm    remove container after running

docker run -it --net=host \
 -v /dev/snd:/dev/snd \
 -v /dev/input:/dev/input \
 -v /home/pi:/home/pi \
 -v /dev/bus/usb:/dev/bus/usb \
 -v /var/lock:/var/lock \
 -e TZ=America/New_York \
 -w /home/pi/GoPi5Go/ros2ws \
 --privileged \
 --rm \
 -- name gopi5goROS2
 gopi5gor2hdp
