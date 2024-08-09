#!/bin/bash

# FILE: build_gopigo3_buster.sh

# This adds the pi user and GoPi5Go API to the base python:3.7-buster image

cd ~/GoPi5Go/systests/legacy
if [ -d ~/GoPi5Go/systests/legacy/GoPiGo3  ]; then
    echo -e "Removing existing legacy/GoPiGo3/ " 
    rm -rf GoPiGo3
fi
echo -e "Bring down clean GoPiGo3 repository"
git clone http://www.github.com/DexterInd/GoPiGo3.git

# build a legacy OS with GoPiGo3 API installed tagged gpg3buster
sudo docker build --no-cache . -t gpg3buster -f gopigo3_buster_dockerfile  2>&1 | tee build.log
