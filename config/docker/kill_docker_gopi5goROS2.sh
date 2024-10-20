#!/bin/bash

echo -e "Killing Detached Docker Container gopi5goROS2"
docker ps
docker stop gopi5goROS2
docker ps
echo -e "Pruning Docker Container gopi5goROS2"
docker container list -a
docker container prune -f
docker container list -a
echo -e "Done"
