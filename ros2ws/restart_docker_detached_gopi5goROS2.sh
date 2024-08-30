#!/bin/bash

if [ ! -f /usr/bin/docker ]; then
    echo -e "Must be outside Docker (In PiOS)"
else
    echo -e "Restarting docker.gopi5goROS2 Service"
    sudo systemctl restart docker.gopi5goROS2
fi
