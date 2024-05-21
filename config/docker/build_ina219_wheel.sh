#!/bin/bash

pushd .
# Create a place for the GoPi5Go wheels available to the docker build
mkdir -p /home/pi/config/docker/gopigo3

echo -e "ina219  Wheel"
cd /home/pi/GoPi5Go/systests/ina219/gopigo3-pi5-ina219
sudo python3 setup.py bdist_wheel
cp dist/*.whl /home/pi/GoPi5Go/config/docker/gopigo3

popd

echo -e "Now add gopigo3_pi5_ina219 to docker/gopigo3/requirements.txt"
echo -e "Python Wheels for ina219 on GoPiGo3 Pi5 ready for docker build"

