#!/bin/bash

cd ~/GoPi5Go/systests/Oak-D-W-97/vdepthai/
git clone https://github.com/luxonis/depthai.git
source bin/activate
pip3 install opencv-python opencv-contrib-python
cd depthai
python3 install_requirements.py 


