#!/bin/bash

# FILE:  run_docker_detached_gopi5goROS2.sh


cd /home/pi/GoPi5Go/ros2ws
# --rm \    remove container after running
# Prob not need
# --device /dev/gpiochip4 \

docker run -dt --net=host \
 -v /dev/snd:/dev/snd \
 -v /dev/input:/dev/input \
 -v /home/pi:/home/pi \
 -v /dev/bus/usb:/dev/bus/usb \
 -v /dev/ttyUSB0:/dev/ttyUSB0 \
 -v /var/lock:/var/lock \
 -e TZ=America/New_York \
 -w /home/pi/GoPi5Go/ros2ws \
 --privileged \
 --name gopi5goROS2 \
 gopi5gor2hdp
