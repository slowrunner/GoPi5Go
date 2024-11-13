#!/bin/bash


echo -e "\n********** ROS2 GoPiGo3 Status ******************************"; 
echo -n `date +"%A %D"`; 
echo ""; 
uptime; 
if [ -f /usr/bin/docker ]; then
    vcgencmd measure_temp && vcgencmd measure_clock arm && vcgencmd get_throttled; \
    python3 /home/pi/GoPi5Go/plib/battery.py;

fi; 
free -h; 
python3 /home/pi/GoPi5Go/ros2ws/gopigo3_battery.py; 
echo -e "\n"
echo -e "ROS 2 NODES"
ps -ef | grep "ros2 run" | grep -v "grep"  && ps -ef | grep "ros2 launch" | grep -v "grep"
echo -e "\n"
