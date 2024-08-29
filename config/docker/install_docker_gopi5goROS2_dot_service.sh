#!/bin/bash

cd ~/GoPi5Go/config/docker
sudo cp etc_systemd_system-docker.gopi5goROS2.service /etc/systemd/system/docker.gopi5goROS2.service
sudo systemctl enable docker.gopi5goROS2
echo -e "\n*** Starting docker GoPi5Go-Dave ROS 2 Humble Desktop Plus - 60 second wait"
sudo systemctl start docker.gopi5goROS2
