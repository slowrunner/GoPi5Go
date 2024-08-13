#!/bin/bash

# FILE: 7_build_gopi5gor2hdp.sh

# This adds the GoPi5Go API, GoP5Go easy_ina219,  and YDLidar-SDK to the base r2hdp container

cd ~/GoPi5Go/config/docker
if [ -d YDLidar-SDK ]; then
    echo -e "Removing existing docker/YDLidar-SDK/" 
    rm -rf YDLidar-SDK
fi
echo -e "Bring down clean YDLidar-SDK"
git clone https://github.com/YDLIDAR/YDLidar-SDK

echo -e "copy in my ydlidar_test_interactive.cpp example"
cp ~/GoPi5Go/systests/myYDLidar-SDK/ydlidar_test_interactive.cpp YDLidar-SDK/examples


# build a ros humble desktop plus navigation, slam-toolbox, localization tagged r2hdp
sudo docker build --no-cache . -t gopi5gor2hdp -f docker_files/gopi5go_r2hdp_dockerfile
