#!/bin/bash

# FILE: 8_build_ydlidar.sh

# This is an extention of 7_build_gopi5gor2hdp.sh that adds the GoPi5Go API and the YDLidar-SDK to the base r2hdp container

cd ~/GoPi5Go/config/docker
if [ -d YDLidar-SDK ]; then
    echo -e "Removing existing docker/YDLidar-SDK/" 
    rm -rf YDLidar-SDK
fi
echo -e "Bring down clean YDLidar-SDK"
git clone https://github.com/YDLIDAR/YDLidar-SDK

echo -e "copy in my ydlidar_test_interactive.cpp example"
cp ~/GoPi5Go/systests/myYDLidar-SDK/ydlidar_test_interactive.cpp YDLidar-SDK/examples


# build a ros humble desktop plus navigation, slam-toolbox, localization, with ydlidar sdk installed, tagged ydlidar
sudo docker build --no-cache . -t ydlidar -f docker_files/ydlidar_dockerfile
