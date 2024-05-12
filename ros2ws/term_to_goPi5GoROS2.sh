#!/bin/bash

# start a term to already running ROS 2 Humble Desktop Plus Docker container named gopi5goROS2
docker ps
echo -e "\nStarting BASH TERMINAL to gopi5goROS2 DOCKER container"
echo -e "To exit:  exit (will not terminate container)"
docker exec -it gopi5goROS2 bash
echo -e "Exited Docker Terminal"
