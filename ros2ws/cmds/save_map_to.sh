#!/bin/bash

# REQ:  mogrify  sudo apt install -y imagemagick

# USAGE:  cmds/save_map_to.sh "maps/square_world.map"

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** SAVING MAP TO $1"
# also could run:  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: data: 'map_name'"
echo '*** ros2 run nav2_map_server map_saver_cli -f ' \"$1\"
ros2 run nav2_map_server map_saver_cli -f "${1}"
echo -e "\n*** WAITING 10s FOR MAP SAVER"
sleep 10



# echo -e "\n*** NOW RUN export_jpg_map.sh ***"
echo -e "\n*** Exporting $1.pgm to jpg format: $1.jpg"
echo -e "***  mogrify -format jpg $1.pgm"
mogrify -format jpg $1.pgm
echo -e "*** ${1}.jpg created"
