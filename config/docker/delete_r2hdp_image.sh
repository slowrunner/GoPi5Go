#!/bin/bash

echo -e "Deleting Base Docker Image r2hdp"
docker image list

echo -e "docker image rm r2hdp"

docker image rm r2hdp

docker image list
