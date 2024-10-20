#!/bin/bash

cd ~/GoPi5Go/ros2ws

echo -e "Launching map_server (with params/map_server_params.yaml which points to floorplan.map.yaml)"
echo -e "ros2 run nav2_map_server map_server __params:=params/map_server_parms.yaml"
ros2 run nav2_map_server map_server __params:=params/map_server_params.yaml
