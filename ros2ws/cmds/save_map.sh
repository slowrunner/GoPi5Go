#!/bin/bash

basedir=GoPi5Go
echo -e "\n*** Switching to ~/${basedir}/ros2ws"
cd ~/$basedir/ros2ws

echo -e "\n*** Sourcing /opt/ros/humble/setup.bash"
. /opt/ros/humble/setup.bash

echo -e "\n*** Sourcing install/setup.bash"
. ~/$basedir/ros2ws/install/setup.bash

echo -e "\n*** SAVING MAP TO xyzzy.map"
# also could run:  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: data: 'map_name'"
echo '*** ros2 run nav2_map_server map_saver_cli -f "xyzzy.map" '
ros2 run nav2_map_server map_saver_cli -f "xyzzy.map"
echo -e "\n*** WAITING 10s FOR MAP SAVER"
sleep 10



# echo -e "\n*** NOW RUN export_jpg_map.sh ***"
echo -e "\n*** Exporting xyzzy.map.pgm to jpg format: xyzzy.map.jpg"
echo "***  mogrify -format jpg xyzzy.map.pgm"
mogrify -format jpg xyzzy.map.pgm
echo "*** xyzzy.map.jpg created"
