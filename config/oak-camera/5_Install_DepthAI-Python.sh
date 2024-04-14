#!/bin/bash

cd ~/GoPi5Go/systests/Oak-D-W-97/vdepthai
source bin/activate
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python/examples
python3 install_requirements.py

echo -e "\nCreating depthai model directory and copying yolo v4 tiny tf blob from depthai-python"
mkdir -p ~/GoPi5Go/models/depthai
cp /home/pi/GoPi5Go/systests/Oak-D-W-97/vdepthai/depthai-python/examples/models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob ~/GoPi5Go/models/depthai
