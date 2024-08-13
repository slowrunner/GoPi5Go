#!/bin/bash

echo -e "Killing Detached Docker Container ydlidarROS2"
docker ps
docker stop ydlidarROS2
docker ps
echo -e "Done"
