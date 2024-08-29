#!/bin/bash

echo -e "Must be outside Docker (In PiOS)"
echo -e "Restarting docker.gopi5goROS2 Service"
sudo systemctl restart docker.gopi5goROS2
