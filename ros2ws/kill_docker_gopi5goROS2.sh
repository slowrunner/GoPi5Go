#!/bin/bash

echo -e "Killing Detached Docker Container gopi5goDave"
docker ps
docker stop gopi5goROS2
docker ps
echo -e "Done"
