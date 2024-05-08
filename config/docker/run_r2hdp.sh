#!/bin/bash

cd ~/GoPi5Go/ros2ws
# --rm    remove container after running
docker run -it --net=host  -v ~/GoPi5Go/ros2ws:/ros2ws -w /ros2ws --rm r2hdp
