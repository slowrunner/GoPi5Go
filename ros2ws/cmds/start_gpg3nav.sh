#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** export GPG3_MODEL=gopi5go_dave"
export GPG3_MODEL=gopi5go_dave

# echo -e "\n*** STARTING gpg3_navigation with house_tile_cart.yaml and gpg3_navigation/params/gopi5go_dave.yaml"

echo -e "\n*** STARTING gpg3_navigation with floorplan.map.yaml"
echo -e "      and gpg3_navigation/params/gopi5go_dave.yaml"
echo "*** ros2 launch gpg3_navigation2 navigation2.launch.py map:=maps/floorplan.map.yaml"

ros2 launch gpg3_navigation2 navigation2.launch.py map:=maps/floorplan.map.yaml
