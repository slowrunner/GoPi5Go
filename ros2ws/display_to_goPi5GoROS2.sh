#!/bin/bash

# start a term (with DISPLAY) to already running ROS 2 Humble Desktop Plus Docker container named gopi5goROS2
docker ps
echo -e "\nStarting BASH TERMINAL (with DISPLAY available) to gopi5goROS2 DOCKER container"
echo -e "docker exec -it -e DISPLAY gopi5goROS2 bash"
echo -e "To exit:  exit (will not terminate container)"
docker exec -it -e DISPLAY gopi5goROS2 bash
echo -e "Exited Docker Terminal"
