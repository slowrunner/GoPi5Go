#!/bin/bash

echo -e "\n*** INSTALL gopigo3_nav2_params.yaml FILE to default Nav2 Bringup node location"
echo -e "sudo cp gopigo3_nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/"
sudo cp gopigo3_nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/
echo -e "gopigo3 nav2 parameters file copied to nav2 bringup params/ directory"

