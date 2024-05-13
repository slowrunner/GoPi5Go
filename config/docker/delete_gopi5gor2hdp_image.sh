#!/bin/bash

echo -e "Deleting Docker Image gopi5gor2hdp"
docker image list

echo -e "docker image rm gopi5gor2hdp"

docker image rm gopi5gor2hdp

docker image list
