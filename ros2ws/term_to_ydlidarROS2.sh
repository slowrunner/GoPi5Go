#!/bin/bash

# start a term to already running ROS 2 Humble Desktop Plus Docker container named ydlidarROS2
docker ps
echo -e "\nStarting BASH TERMINAL to ydlidarROS2 DOCKER container"
echo -e "To exit:  exit (will not terminate container)"
docker exec -it ydlidarROS2 bash
echo -e "Exited Docker Terminal"
