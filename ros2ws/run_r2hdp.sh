#!/bin/bash

cd ~/GoPi5Go/ros2ws
# --rm    remove container after running
docker run -it --net=host \
 -v ~/GoPi5Go/ros2ws:/ros2ws \
 -v /dev/snd:/dev/snd \
 -v /dev/input:/dev/input \
 -v /home/pi:/home/pi \
 -v /dev/bus/usb:/dev/bus/usb \
 -e TZ=America/New_York \
 -w /ros2ws \
 --privileged \
 --rm \
 r2hdp
