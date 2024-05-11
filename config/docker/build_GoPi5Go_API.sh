#!/bin/bash

pushd .
# Create a place for the GoPi5Go wheels available to the docker build
mkdir -p /home/pi/config/docker/gopigo3

echo -e "Build RFR_TOOLS Wheel"
cd /home/pi/Dexter/lib/Dexter//RFR_Tools/miscellaneous/
sudo python3 setup.py bdist_wheel
cp dist/*.whl /home/pi/GoPi5Go/config/docker/gopigo3


echo -e "Build GoPiGo3 Python Wheel"
cd /home/pi/Dexter/GoPiGo3/Software/Python
sudo python3 setup.py bdist_wheel
cp dist/*.whl /home/pi/GoPi5Go/config/docker/gopigo3

echo -e "Build di_sensors Wheel"
cd /home/pi/Dexter/DI_Sensors/Python/
sudo python3 setup.py bdist_wheel
cp dist/*.whl /home/pi/GoPi5Go/config/docker/gopigo3

popd

#echo -e "Python Wheels for Modified GoPiGo3 API ready for docker build"

