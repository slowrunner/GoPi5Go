#!/bin/bash

cd ~/pi5desk/systests/vdepthai/
git clone https://github.com/luxonis/depthai.git
source bin/activate
pip3 install opencv-python opencv-contrib-python
cd depthai
python3 install_requirements.py 


