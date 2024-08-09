#!/bin/bash

cd ~/GoPi5Go/systests/legacy
# --rm    remove container after running
# Prob not need
# --device /dev/gpiochip4 \

docker run -it --net=host \
 -v /dev/snd:/dev/snd \
 -v /dev/input:/dev/input \
 -v /home/pi:/home/pi \
 -v /dev/bus/usb:/dev/bus/usb \
 -e TZ=America/New_York \
 -w /home/pi/GoPi5Go/sysests/legacy \
 --privileged \
 --rm \
 gpg3buster
